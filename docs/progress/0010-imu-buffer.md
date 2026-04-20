# 0010 — IMU 环形缓冲 + Mock 生产者

**Commit**: `68fa83d`  **Date**: 2026-04-20  **Milestone**: M1.6

## What / 做了什么

在 `core-hal` 加了 `imu_buffer` 模块，提供两种都实现 `ImuSource` trait 的 IMU 采样传递机制：

- **Production path** —— 对 `heapless::spsc::Queue<ImuSample, N>` 的 `Consumer` 做 `ImuSource` blanket impl。生产端（ISR / DMA 完成回调）`enqueue`，消费端（rate loop）通过 trait 读。**无锁、零堆、no_std**。
- **Test/SITL path** —— `MockImuSource<CAPACITY>` 从内存中 replay 预置样本。cursor 到末尾后 `Ok(None)` 或调 `rewind()` 从头再读。

`make_buffer(&mut Queue)` 返回 `(Producer, Consumer)` —— 把 heapless split 的调用糖化。

5 个单元测试 + 1 个 doctest + 1 个 proptest（256 样本）。

## Why / 为什么这么做

### 为什么是 SPSC 而不是 MPMC / 带锁队列

1. **硬实时要求**：IMU 采样是**中断驱动**的 —— ICM-42688-P 触发 DRDY 中断，ISR 读 SPI DMA，然后得**确定性 O(1)** 推一个样本给消费端。任何锁争用都可能让 ISR 挤占 rate loop 的时序预算。
2. **单一生产者**：整个 FMU 只有一个 IMU 物理中断在往这个 queue 写，所以 SPSC 语义天然契合。
3. **`heapless::spsc`** 是基于 `Atomic*` 的无锁实现，数十行汇编即完成 enqueue/dequeue，无需 critical section。在 Cortex-M7 上实测是飞控社区的黄金方案。

### 为什么给 `Consumer` 直接 impl 我们自己的 `ImuSource`

**孤儿规则**允许在本地 trait 上为外部类型实现 —— 这里 `ImuSource` 在 `core-hal`（本地），`Consumer` 来自 `heapless`（外部）。合法且惯用。

这避免了写一个"dataful wrapper"（包装 Consumer 的 struct）—— 那会带来冗余字段、额外析构、`Deref` 糖。直接 blanket impl 让调用点**一步**从 heapless 对象拿到 ImuSource 能力。

### Queue `<T, 4>` 只能放 3 个：为什么

heapless 的 SPSC queue 用 **环形缓冲 + 头尾指针**的经典实现。为了区分"空"（head == tail）和"满"（head == tail 绕一圈），**永远保留一个空槽**。所以 `Queue<T, N>` 有效容量是 `N - 1`。

`ring_buffer_refuses_overflow` 测试里 `Queue<_, 4>` 只允许 3 次 enqueue —— 这是 heapless 的合同，不是 bug。实际生产用 `DEFAULT_IMU_BUFFER_LEN = 16` 对应 15 个有效槽（1 kHz 采样 → 15 ms 消费延迟余量）。

### 为什么 mock 不用 `Vec<_, 1024>` 直接 heap

`MockImuSource` 故意放在 no_std 下（用 `heapless::Vec`）。这样：
- 能在 FMU 上直接跑（比如 HITL 烧到实机重放历史数据）
- 和 production path 共用同一套类型，测试逻辑 = 飞行逻辑
- 不绑到 `std` 的 alloc 才能用

代价是容量写死在类型参数里，但 SITL 场景知道规模。

## How it's verified / 怎么验证的

```bash
$ cargo test -p core-hal
test imu_buffer::tests::mock_source_yields_then_exhausts ... ok
test imu_buffer::tests::ring_buffer_is_fifo ... ok
test imu_buffer::tests::ring_buffer_refuses_overflow ... ok
test imu_buffer::tests::mock_source_rewinds ... ok
test imu_buffer::tests::fifo_over_random_sequences ... ok
test result: ok. 5 passed

Doc-tests core_hal
running 1 test
test crates/core-hal/src/imu_buffer.rs - imu_buffer::make_buffer ... ok

$ cargo clippy --workspace --all-targets -- -D warnings
Finished
```

## Follow-ups / 遗留

- **真实驱动**：ICM-42688-P 的 SPI + DMA 配置要在 `core-hal` 里做 STM32H7 specific backend，`#[cfg(feature = "stm32h753")]` 包着。**M1.7 或 M2 着手**。
- **PPS 对齐**：多传感器时间戳硬件同步。这需要 TIMx input capture，留给 GPS 驱动 step。
- **采样率检测**：对着 queue 的到达率做统计，偏离目标频率 5% 以上就 `HealthLevel::Degraded`。接 `algo-fdir` 里的状态机。
