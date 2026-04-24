//! Opens a `.rrd` recording in the rerun native viewer, in-process.
//!
//! Run:
//!   cargo run -p sim-hil --example open_rrd --features rerun-viz --release -- PATH.rrd
//!
//! Companion to `sitl_rerun` which produces the `.rrd` file. Splitting
//! capture and viewing into two binaries means:
//!
//!   * the SITL run finishes fast (no GUI event loop to slow it)
//!   * the recording can be shared / replayed / diff'd offline
//!   * the viewer itself is the trivial binary — shows up in
//!     `target/release/examples` and doesn't need a separate
//!     `cargo install rerun-cli`

#![allow(clippy::unwrap_used, clippy::expect_used)]

use rerun::external::re_log_encoding::decoder::Decoder;
use rerun::external::re_log_encoding::VersionPolicy;
use rerun::external::re_log_types::LogMsg;

fn main() -> anyhow::Result<()> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "auto_pilot_figure_eight.rrd".to_string());
    println!("[open_rrd] reading {path}");

    // Decode the .rrd file into a Vec<LogMsg>. Rerun's `show()` takes
    // an in-memory message batch — this is one of the "load a recording
    // from disk" recipes in their integration tests.
    let file = std::fs::File::open(&path)?;
    let reader = std::io::BufReader::new(file);
    let decoder = Decoder::new(VersionPolicy::Warn, reader)?;
    let msgs: Vec<LogMsg> = decoder.collect::<Result<Vec<_>, _>>()?;
    println!("[open_rrd] decoded {} messages", msgs.len());

    // `i_promise_i_am_on_the_main_thread` — we're in main(), so this
    // is trivially true. The token is a marker the viewer needs to
    // confirm that egui / winit calls happen on the main thread.
    let main_thread_token =
        rerun::MainThreadToken::i_promise_i_am_on_the_main_thread();

    println!("[open_rrd] launching native viewer…");
    rerun::native_viewer::show(main_thread_token, msgs)
        .map_err(|e| anyhow::anyhow!("viewer failed: {e}"))?;
    Ok(())
}
