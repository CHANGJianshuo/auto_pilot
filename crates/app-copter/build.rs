// Wire up `memory.x` into the linker search path when cross-compiling
// for a bare-metal Arm target. Host builds are a no-op.

use std::env;
use std::error::Error;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn Error>> {
    let target = env::var("TARGET").unwrap_or_default();
    if !target.starts_with("thumb") {
        return Ok(());
    }

    let out_dir = PathBuf::from(env::var("OUT_DIR")?);
    let memory_x = include_bytes!("memory.x");
    File::create(out_dir.join("memory.x"))?.write_all(memory_x)?;

    // cortex-m-rt's `link.x` imports `memory.x`; telling the linker to
    // search OUT_DIR first makes it pick ours up. cortex-m-rt's own
    // build.rs already emits `-Tlink.x`, so don't duplicate that flag
    // here — if we do, memory.x gets parsed twice and the linker fails
    // with "region 'FLASH' already defined".
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=memory.x");
    Ok(())
}
