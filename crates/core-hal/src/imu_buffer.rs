//! IMU sample buffering — DMA-ready ring + deterministic mock.
//!
//! Two independent pieces live here, both implementing
//! [`crate::traits::ImuSource`]:
//!
//! 1. [`ImuRingBuffer`] — a [`heapless::spsc::Queue`] wrapper sized for
//!    interrupt-driven DMA hand-off. The producer half lives in an ISR /
//!    DMA-complete callback; the consumer half is pulled by the rate-loop
//!    task. Lock-free, `no_std`, `alloc`-free.
//! 2. [`MockImuSource`] — replays a fixed script of samples in order, used
//!    for SITL and unit tests. Also `no_std` (uses [`heapless::Vec`]).

use core::convert::Infallible;
use heapless::spsc::{Consumer, Producer, Queue};

use crate::traits::{ImuSample, ImuSource};

/// Default IMU ring-buffer capacity. At 1 kHz sampling this is 16 ms of
/// slack — enough to absorb occasional consumer stalls while keeping the
/// queue small enough to fit on any FMU stack.
pub const DEFAULT_IMU_BUFFER_LEN: usize = 16;

/// Convenience alias for the 16-slot IMU queue.
pub type DefaultImuQueue = Queue<ImuSample, DEFAULT_IMU_BUFFER_LEN>;

// --- heapless Consumer → ImuSource blanket impl ---------------------------

impl<'a, const N: usize> ImuSource for Consumer<'a, ImuSample, N> {
    type Error = Infallible;

    fn try_read(&mut self) -> Result<Option<ImuSample>, Self::Error> {
        Ok(self.dequeue())
    }
}

/// Create a zero-initialised `N`-slot queue and immediately split it into
/// (producer, consumer) halves sharing the same backing storage.
///
/// The caller owns the storage — in production this is typically a
/// `static` inside a `RefCell` / `critical-section::Mutex`, or a
/// field in an application struct.
///
/// ```
/// use core_hal::imu_buffer::{make_buffer, DefaultImuQueue};
///
/// let mut queue = DefaultImuQueue::new();
/// let (_producer, _consumer) = make_buffer(&mut queue);
/// ```
pub fn make_buffer<const N: usize>(
    queue: &mut Queue<ImuSample, N>,
) -> (Producer<'_, ImuSample, N>, Consumer<'_, ImuSample, N>) {
    queue.split()
}

// --- Mock source ----------------------------------------------------------

/// Deterministic IMU source backed by an in-memory sample script.
///
/// The cursor advances by one on each successful `try_read`; once exhausted
/// the source returns `Ok(None)` forever unless [`Self::rewind`] is called.
#[derive(Debug)]
pub struct MockImuSource<const CAPACITY: usize> {
    samples: heapless::Vec<ImuSample, CAPACITY>,
    cursor: usize,
}

impl<const CAPACITY: usize> Default for MockImuSource<CAPACITY> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAPACITY: usize> MockImuSource<CAPACITY> {
    /// Construct an empty source. Use [`Self::push`] to append samples.
    #[must_use]
    pub const fn new() -> Self {
        Self { samples: heapless::Vec::new(), cursor: 0 }
    }

    /// Append a sample to the replay script. Returns the sample back as
    /// `Err` if the backing vector is full.
    pub fn push(&mut self, sample: ImuSample) -> Result<(), ImuSample> {
        self.samples.push(sample)
    }

    /// Reset the read cursor to 0 so the same script replays from the start.
    pub fn rewind(&mut self) {
        self.cursor = 0;
    }

    /// Number of samples remaining before the source is exhausted.
    #[must_use]
    pub fn remaining(&self) -> usize {
        self.samples.len().saturating_sub(self.cursor)
    }

    /// Total number of samples scripted (unchanged by read cursor position).
    #[must_use]
    pub fn len(&self) -> usize {
        self.samples.len()
    }

    /// `true` when no samples have been pushed.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }
}

impl<const CAPACITY: usize> ImuSource for MockImuSource<CAPACITY> {
    type Error = Infallible;

    fn try_read(&mut self) -> Result<Option<ImuSample>, Self::Error> {
        // Avoid the `indexing_slicing`-denied `self.samples[self.cursor]`
        // by reaching through `.get()`.
        let out = self.samples.get(self.cursor).copied();
        if out.is_some() {
            self.cursor = self.cursor.saturating_add(1);
        }
        Ok(out)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    use proptest::prelude::*;

    fn make_sample(ts: u64, seed: f32) -> ImuSample {
        ImuSample {
            timestamp_us: ts,
            gyro_rad_s: Vector3::new(seed, seed + 0.1, seed + 0.2),
            accel_m_s2: Vector3::new(seed + 1.0, seed + 1.1, seed + 1.2),
            temperature_c: 20.0 + seed,
        }
    }

    fn samples_equal(a: &ImuSample, b: &ImuSample) -> bool {
        a.timestamp_us == b.timestamp_us
            && a.gyro_rad_s == b.gyro_rad_s
            && a.accel_m_s2 == b.accel_m_s2
            && a.temperature_c == b.temperature_c
    }

    #[test]
    fn ring_buffer_is_fifo() {
        let mut q: Queue<ImuSample, 4> = Queue::new();
        let (mut p, mut c) = q.split();

        let s0 = make_sample(0, 0.0);
        let s1 = make_sample(1, 1.0);
        let s2 = make_sample(2, 2.0);

        p.enqueue(s0).unwrap();
        p.enqueue(s1).unwrap();
        p.enqueue(s2).unwrap();

        assert!(samples_equal(&c.try_read().unwrap().unwrap(), &s0));
        assert!(samples_equal(&c.try_read().unwrap().unwrap(), &s1));
        assert!(samples_equal(&c.try_read().unwrap().unwrap(), &s2));
        assert!(c.try_read().unwrap().is_none());
    }

    #[test]
    fn ring_buffer_refuses_overflow() {
        // Queue<T, 4> actually holds 3 items — heapless keeps one slot
        // free to disambiguate head == tail.
        let mut q: Queue<ImuSample, 4> = Queue::new();
        let (mut p, _c) = q.split();
        assert!(p.enqueue(make_sample(0, 0.0)).is_ok());
        assert!(p.enqueue(make_sample(1, 1.0)).is_ok());
        assert!(p.enqueue(make_sample(2, 2.0)).is_ok());
        // Fourth enqueue must fail because the ring is "full" (3 free slots).
        assert!(p.enqueue(make_sample(3, 3.0)).is_err());
    }

    #[test]
    fn mock_source_yields_then_exhausts() {
        let mut src: MockImuSource<4> = MockImuSource::new();
        assert!(src.is_empty());
        src.push(make_sample(10, 0.0)).unwrap();
        src.push(make_sample(20, 1.0)).unwrap();
        assert_eq!(src.len(), 2);
        assert_eq!(src.remaining(), 2);

        let first = src.try_read().unwrap().unwrap();
        assert_eq!(first.timestamp_us, 10);
        assert_eq!(src.remaining(), 1);

        let second = src.try_read().unwrap().unwrap();
        assert_eq!(second.timestamp_us, 20);
        assert_eq!(src.remaining(), 0);

        assert!(src.try_read().unwrap().is_none());
    }

    #[test]
    fn mock_source_rewinds() {
        let mut src: MockImuSource<2> = MockImuSource::new();
        src.push(make_sample(1, 0.0)).unwrap();
        src.try_read().unwrap();
        assert!(src.try_read().unwrap().is_none());
        src.rewind();
        assert!(src.try_read().unwrap().is_some());
    }

    proptest! {
        /// FIFO: for any sequence of samples that fits the queue, dequeueing
        /// yields them in the original order.
        #[test]
        fn fifo_over_random_sequences(seeds in proptest::collection::vec(-100.0f32..100.0, 0..10)) {
            let mut q: Queue<ImuSample, 16> = Queue::new();
            let (mut p, mut c) = q.split();

            let inputs: heapless::Vec<ImuSample, 10> =
                seeds.iter().enumerate()
                    .map(|(i, &s)| make_sample(u64::try_from(i).unwrap_or(0), s))
                    .collect();
            for s in &inputs {
                p.enqueue(*s).unwrap();
            }
            for expected in &inputs {
                let got = c.try_read().unwrap().unwrap();
                prop_assert!(samples_equal(&got, expected));
            }
            prop_assert!(c.try_read().unwrap().is_none());
        }
    }
}
