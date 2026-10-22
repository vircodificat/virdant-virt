//! `virt` — build and run simulators for the virtual FPGA platforms
//! (`virt::Virt`, `virt::VirtVga`).
//!
//! Usage:
//!   virt build [--top=<TOP>] [--build-dir=<DIR>]
//!       Runs `vir build`, copies ext/*.sv, then Verilator + link, producing
//!       the simulator binary at `build/${TOP}`.
//!   virt run [--top=<TOP>] [--build-dir=<DIR>] [binary flags...]
//!       Builds (if the binary is missing or stale) and runs it.
//!
//! The top module defaults to `[prog] top` from Virdant.toml (e.g.
//! `top::Top` → the simple name `Top`); `--top` overrides it.
//!
//! Which board the simulator emulates is decided by the platform the top
//! module implements (its `for` clause), read back from the manifest
//! `vir build` emits at `build/virmanifest.json`:
//!
//!   - `virt::Virt`   → assets/sim.cpp    — UART, SPI flash, exit.
//!   - `virt::VirtVga` → assets/sim_vga.cpp — Virt + a VGA monitor (SDL2).

use std::env;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::SystemTime;

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    let Some(sub) = args.first() else {
        usage();
    };
    match sub.as_str() {
        "build" => build(&args[1..]),
        "run" => run(&args[1..]),
        "-h" | "--help" | "help" => usage(),
        other => {
            eprintln!("virt: unknown command `{other}`");
            usage();
        }
    }
}

fn usage() -> ! {
    eprintln!("usage: virt build [--top=<TOP>] [--build-dir=<DIR>]");
    eprintln!("       virt run  [--top=<TOP>] [--build-dir=<DIR>] [binary flags...]");
    std::process::exit(2);
}

// ─────────────────────────────────────────────────────────────────────────
// `virt build`
// ─────────────────────────────────────────────────────────────────────────

fn build(args: &[String]) {
    let mut top: Option<String> = None;
    let mut build_dir = PathBuf::from("build");

    for a in args {
        if let Some(v) = a.strip_prefix("--top=") {
            top = Some(v.to_string());
        } else if let Some(v) = a.strip_prefix("--build-dir=") {
            build_dir = PathBuf::from(v);
        } else {
            eprintln!("virt: unknown flag `{a}`");
            std::process::exit(2);
        }
    }

    let top = resolve_top(top);
    build_with(&top, &build_dir);
}

// ─────────────────────────────────────────────────────────────────────
// Platform manifest (`vir build` → build/virmanifest.json)
// ─────────────────────────────────────────────────────────────────────

/// The platform metadata `vir build` emits that this tool needs: which
/// `virt::` platform the top module implements (this selects the board
/// the simulator emulates) and the platform clock frequency (the UART
/// baud and VGA timing base).  A std-only scan of the JSON this tool
/// itself consumes — the manifest is written by `vir build`, so simple
/// field extraction suffices (no nested objects in these fields).
struct Manifest {
    /// Simple platform name, e.g. `"VirtVga"`.
    platform: String,
    /// FPGA family string, e.g. `"virt"`.
    fpga: String,
    /// Primary clock frequency in Hz, e.g. `50000000`.
    clock_hz: u64,
}

/// The virtual boards, and the fixed harness that emulates each.  Both
/// harnesses compile against the same Verilator model class `Vvirt`
/// (`virt build` pins the `--prefix`), so the top module's name is free.
#[derive(Clone, Copy)]
enum Board {
    /// `virt::Virt` — UART (stdin/stdout), 16 MiB SPI flash, exit port.
    /// Emulated by `assets/sim.cpp` (overridable via $VIRT_SIM_CPP).
    Virt,
    /// `virt::VirtVga` — `Virt` plus a Nexys4-style VGA connector
    /// (RGB444 + HSYNC/VSYNC, either polarity) rendered with SDL2.
    /// Emulated by `assets/sim_vga.cpp` (overridable via $VIRT_SIM_VGA_CPP).
    VirtVga,
}

impl Board {
    /// The fixed C++ harness source for this board.
    fn harness_path(self) -> PathBuf {
        let (name, override_env) = match self {
            Board::Virt => ("sim.cpp", "VIRT_SIM_CPP"),
            Board::VirtVga => ("sim_vga.cpp", "VIRT_SIM_VGA_CPP"),
        };
        match env::var(override_env) {
            Ok(p) => PathBuf::from(p),
            Err(_) => PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("assets").join(name),
        }
    }
}

/// Read the manifest, or exit with a diagnostic.
fn read_manifest(build_dir: &Path) -> Manifest {
    match try_read_manifest(build_dir) {
        Some(manifest) => manifest,
        None => {
            let path = build_dir.join("virmanifest.json");
            if !path.is_file() {
                eprintln!(
                    "virt: no {} — was `vir build` run in a project whose [prog] top implements a platform?",
                    path.display()
                );
            } else {
                eprintln!("virt: cannot parse {}", path.display());
            }
            eprintln!(
                "virt: this tool simulates the virt FPGA family: `for virt::Virt` (harness sim.cpp) or `for virt::VirtVga` (harness sim_vga.cpp)"
            );
            std::process::exit(1);
        }
    }
}

/// Read the manifest, or `None` if missing/unparseable.
fn try_read_manifest(build_dir: &Path) -> Option<Manifest> {
    let text = std::fs::read_to_string(build_dir.join("virmanifest.json")).ok()?;
    Some(Manifest {
        platform: manifest_str_field(&text, "platform_name")?,
        fpga: manifest_str_field(&text, "fpga")?,
        clock_hz: manifest_num_field(&text, "clock_hz")?,
    })
}

/// The value of a string field, scanned from the manifest text.
fn manifest_str_field(text: &str, key: &str) -> Option<String> {
    let rest = manifest_value_of(text, key)?;
    let mut chars = rest.chars();
    (chars.next() == Some('"'))
        .then(|| chars.take_while(|&c| c != '"').collect())
}

/// The value of a natural-number field, scanned from the manifest text.
fn manifest_num_field(text: &str, key: &str) -> Option<u64> {
    let rest = manifest_value_of(text, key)?;
    rest.chars()
        .take_while(char::is_ascii_digit)
        .collect::<String>()
        .parse()
        .ok()
}

/// The text just after `"<key>" :`, trimmed.  (Scans the first match;
/// the manifest has no repeated keys at this depth.)
fn manifest_value_of<'a>(text: &'a str, key: &str) -> Option<&'a str> {
    let key_pattern = format!("\"{key}\"");
    let key_pos = text.find(&key_pattern)?;
    let rest = text[key_pos + key_pattern.len()..].trim_start();
    let rest = rest.strip_prefix(':')?.trim_start();
    Some(rest)
}

/// Which board to emulate, from the manifest — or exit: this tool only
/// simulates the `virt` FPGA family, and only the boards it has
/// harnesses for.
fn board_for(manifest: &Manifest) -> Board {
    if manifest.fpga != "virt" {
        eprintln!(
            "virt: platform `{}` targets FPGA family `{}`, which this tool cannot simulate",
            manifest.platform, manifest.fpga
        );
        eprintln!("virt: this tool simulates the virt FPGA family only");
        std::process::exit(1);
    }
    match manifest.platform.as_str() {
        "Virt" => Board::Virt,
        "VirtVga" => Board::VirtVga,
        other => {
            eprintln!(
                "virt: platform `{other}` is not a simulated virtual board (expected virt::Virt or virt::VirtVga)"
            );
            std::process::exit(1);
        }
    }
}

/// Flags from `sdl2-config`, whitespace-split (its output is plain
/// `-I…`/`-L… -l…` flags; no quoting).
fn sdl2_flags(which: &str) -> Vec<String> {
    let output = Command::new("sdl2-config")
        .arg(which)
        .output()
        .unwrap_or_else(|e| {
            eprintln!("virt: cannot run sdl2-config: {e}");
            eprintln!("virt: the virt::VirtVga harness renders with SDL2 — install it (e.g. `libsdl2-dev` on Debian)");
            std::process::exit(1);
        });
    if !output.status.success() {
        eprintln!("virt: sdl2-config {which} failed");
        std::process::exit(1);
    }
    String::from_utf8_lossy(&output.stdout)
        .split_whitespace()
        .map(String::from)
        .collect()
}

fn build_with(top: &str, build_dir: &Path) {
    // ── Step 1: regenerate the Verilog with `vir build` ──
    eprintln!("virt: running vir build...");
    let mut vir = Command::new("vir");
    vir.arg("build");
    run_tool(vir, "vir");

    // ── Step 2: copy external Verilog (ext/*.sv) into the build dir ──
    // ext mods are declared in .vir but their Verilog lives in ext/*.sv
    // (same flow as virdant-riscv).
    copy_ext("ext", build_dir);

    // Collect the generated SystemVerilog sources.
    let sv_files: Vec<PathBuf> = match std::fs::read_dir(build_dir) {
        Ok(entries) => {
            let mut files: Vec<PathBuf> = entries
                .filter_map(|e| e.ok())
                .map(|e| e.path())
                .filter(|p| p.extension().map(|x| x == "sv").unwrap_or(false))
                .collect();
            files.sort();
            files
        }
        Err(e) => {
            eprintln!("virt: cannot read build directory {}: {e}", build_dir.display());
            eprintln!("virt: run `vir build` first");
            std::process::exit(1);
        }
    };
    if sv_files.is_empty() {
        eprintln!("virt: no .sv files in {} — run `vir build` first", build_dir.display());
        std::process::exit(1);
    }
    eprintln!(
        "virt: {} .sv files from {}",
        sv_files.len(),
        build_dir.display()
    );

    // ── Step 3: read the platform manifest and pick the board harness ──
    // The top module's `for` clause (resolved by `vir build` into the
    // manifest) decides which board this build simulates.
    let manifest = read_manifest(build_dir);
    let board = board_for(&manifest);
    let sim_cpp = board.harness_path();
    let harness_name = |p: &Path| {
        p.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default()
    };
    if !sim_cpp.is_file() {
        eprintln!(
            "virt: {} harness not found at {}",
            harness_name(&sim_cpp),
            sim_cpp.display()
        );
        std::process::exit(1);
    }
    eprintln!(
        "virt: platform virt::{} ({} MHz) — harness {}",
        manifest.platform,
        manifest.clock_hz / 1_000_000,
        sim_cpp.display()
    );

    // Find Verilator's include directory.
    let verilator_root = verilator_root();

    // Verilator: --prefix Vvirt pins the generated class name to `Vvirt`,
    // regardless of the top module's name, so the fixed harnesses
    // (sim.cpp / sim_vga.cpp) work for any design.
    let mdir = build_dir.join("virt");
    let mut cmd = Command::new("verilator");
    // --trace emits VerilatedVcdC support so the harness can record VCDs
    // with --vcd=; verilated_vcd_c.o is left loose in the Mdir alongside the
    // other runtime objects.
    cmd.arg("--sv")
        .arg("--cc")
        .args(&sv_files)
        .arg("--Mdir").arg(&mdir)
        .arg("--prefix").arg("Vvirt")
        .arg("--top-module").arg(top)
        .arg("--build")
        .arg("--trace")
        .arg("-Wno-UNOPTFLAT");
    eprintln!("virt: running verilator...");
    run_tool(cmd, "verilator");

    // Link the harness with the generated model library (libVvirt.a) and
    // the Verilator runtime objects (verilated.o, verilated_threads.o),
    // which verilator --build leaves loose in the Mdir.
    let lib = mdir.join("libVvirt.a");
    if !lib.is_file() {
        eprintln!("virt: expected {} after verilator --build", lib.display());
        std::process::exit(1);
    }
    let runtime_objs: Vec<PathBuf> = ["verilated.o", "verilated_threads.o", "verilated_vcd_c.o"]
        .iter()
        .map(|o| mdir.join(o))
        .filter(|p| {
            if !p.is_file() {
                eprintln!("virt: missing Verilator runtime object {}", p.display());
                std::process::exit(1);
            }
            true
        })
        .collect();
    let out = build_dir.join(top);
    let mut cmd = Command::new("g++");
    cmd.arg("-O2")
        .arg("-I").arg(&mdir)
        .arg("-I").arg(verilator_root.join("include"))
        .arg("-I").arg(verilator_root.join("include/vltstd"));
    if let Board::VirtVga = board {
        // The harness renders the VGA output with SDL2 and derives its
        // UART baud and sync-timing expectations from the platform
        // clock (the manifest's clock_hz from the platform's
        // @period_ns).
        cmd.arg(format!("-DVIRT_CLOCK_HZ={}ull", manifest.clock_hz));
        for flag in sdl2_flags("--cflags") {
            cmd.arg(&flag);
        }
    }
    cmd.arg(&sim_cpp);
    if let Board::VirtVga = board {
        for flag in sdl2_flags("--libs") {
            cmd.arg(&flag);
        }
    }
    cmd.arg(&lib)
        .args(&runtime_objs)
        .arg("-o").arg(&out);
    eprintln!("virt: linking {}...", out.display());
    run_tool(cmd, "g++");

    eprintln!("virt: built {}", out.display());
}

// ─────────────────────────────────────────────────────────────────────────
// `virt run`
// ─────────────────────────────────────────────────────────────────────────

fn run(args: &[String]) {
    let mut top: Option<String> = None;
    let mut build_dir = PathBuf::from("build");
    let mut passthrough: Vec<String> = vec![];

    for a in args {
        if let Some(v) = a.strip_prefix("--top=") {
            top = Some(v.to_string());
        } else if let Some(v) = a.strip_prefix("--build-dir=") {
            build_dir = PathBuf::from(v);
        } else {
            // Unknown flags pass through to the simulator binary
            // (e.g. --flash=..., --cycles=...).
            passthrough.push(a.clone());
        }
    }

    let top = resolve_top(top);
    let bin = build_dir.join(&top);

    if needs_rebuild(&bin, &build_dir) {
        build_with(&top, &build_dir);
    } else {
        eprintln!("virt: {} is up to date", bin.display());
    }

    if !bin.is_file() {
        eprintln!("virt: {} not found", bin.display());
        std::process::exit(1);
    }

    let status = Command::new(&bin)
        .args(&passthrough)
        .status()
        .unwrap_or_else(|e| {
            eprintln!("virt: cannot run {}: {e}", bin.display());
            std::process::exit(1);
        });
    std::process::exit(status.code().unwrap_or(1));
}

/// True when the simulator binary is missing or older than any design
/// source (src/*.vir, ext/*.sv, and the board harness chosen by the
/// manifest).
fn needs_rebuild(bin: &Path, build_dir: &Path) -> bool {
    if !bin.is_file() {
        return true;
    }

    // Which harness this binary was built from (which board it
    // emulates): a stale or missing manifest forces a rebuild, so the
    // next build regenerates it.
    let Some(manifest) = try_read_manifest(build_dir) else {
        return true;
    };
    let board = board_for(&manifest);
    let bin_mtime = mtime(bin);

    let mut sources: Vec<PathBuf> = vec![];
    for dir in [Path::new("src"), Path::new("ext")] {
        if let Ok(entries) = std::fs::read_dir(dir) {
            for entry in entries.filter_map(|e| e.ok()) {
                let p = entry.path();
                let is_src = p.extension().map(|x| x == "vir").unwrap_or(false);
                let is_sv = p.extension().map(|x| x == "sv").unwrap_or(false);
                if is_src || is_sv {
                    sources.push(p);
                }
            }
        }
    }
    // The fixed harness is a source too: a sim.cpp/sim_vga.cpp change
    // must trigger a rebuild even when no .vir/.sv file changed.  The
    // manifest itself is deliberately NOT a rebuild source: `vir build`
    // rewrites it every run, so its mtime would force a rebuild every
    // time (its content only changes when a source does).
    sources.push(board.harness_path());

    sources.iter().any(|p| mtime(p) > bin_mtime)
}

fn mtime(p: &Path) -> SystemTime {
    std::fs::metadata(p)
        .and_then(|m| m.modified())
        .unwrap_or(SystemTime::UNIX_EPOCH)
}

// ─────────────────────────────────────────────────────────────────────────
// Shared helpers
// ─────────────────────────────────────────────────────────────────────────

/// Resolve the top module's simple (Verilog) name: `--top` overrides;
/// otherwise `[prog] top` from Virdant.toml (FQN `top::Top` → `Top`).
fn resolve_top(override_top: Option<String>) -> String {
    let fqn = match override_top {
        Some(t) => t,
        None => match read_prog_top(Path::new(".")) {
            Some(t) => t,
            None => {
                eprintln!(
                    "virt: no --top given and Virdant.toml is missing [prog] top"
                );
                std::process::exit(2);
            }
        },
    };
    simple_name(&fqn)
}

fn simple_name(fqn: &str) -> String {
    fqn.rsplit("::").next().unwrap_or(fqn).to_string()
}

/// Minimal TOML scan for `[prog] top = "..."` (std-only; no toml crate).
fn read_prog_top(cwd: &Path) -> Option<String> {
    let text = std::fs::read_to_string(cwd.join("Virdant.toml")).ok()?;
    let mut in_prog = false;
    for line in text.lines() {
        let t = line.trim();
        if t.starts_with('[') {
            in_prog = t == "[prog]";
            continue;
        }
        if !in_prog {
            continue;
        }
        if let Some(rest) = t.strip_prefix("top") {
            let rest = rest.trim_start();
            if let Some(rest) = rest.strip_prefix('=') {
                let val = rest.trim().trim_matches('"');
                if !val.is_empty() {
                    return Some(val.to_string());
                }
            }
        }
    }
    None
}

fn copy_ext(ext_dir: &str, build_dir: &Path) {
    let dir = Path::new(ext_dir);
    if !dir.is_dir() {
        return;
    }
    let mut copied = 0;
    for entry in std::fs::read_dir(dir).unwrap_or_else(|e| {
        eprintln!("virt: cannot read {}: {e}", dir.display());
        std::process::exit(1);
    }) {
        let path = match entry {
            Ok(e) => e.path(),
            Err(e) => {
                eprintln!("virt: {e}");
                std::process::exit(1);
            }
        };
        if path.extension().map(|x| x == "sv").unwrap_or(false) {
            let Some(name) = path.file_name() else { continue };
            let dest = build_dir.join(name);
            std::fs::copy(&path, &dest).unwrap_or_else(|e| {
                eprintln!(
                    "virt: cannot copy {} to {}: {e}",
                    path.display(),
                    dest.display()
                );
                std::process::exit(1);
            });
            copied += 1;
        }
    }
    if copied > 0 {
        eprintln!(
            "virt: copied {copied} ext .sv file(s) into {}",
            build_dir.display()
        );
    }
}

fn verilator_root() -> PathBuf {
    if let Ok(root) = env::var("VERILATOR_ROOT") {
        return PathBuf::from(root);
    }
    let out = Command::new("verilator")
        .arg("-V")
        .output()
        .unwrap_or_else(|e| {
            eprintln!("virt: cannot run verilator: {e}");
            std::process::exit(1);
        });
    let text = String::from_utf8_lossy(&out.stdout);
    for line in text.lines() {
        if let Some(rest) = line.trim().strip_prefix("VERILATOR_ROOT") {
            if let Some(path) = rest.trim().strip_prefix('=') {
                return PathBuf::from(path.trim());
            }
        }
    }
    eprintln!("virt: cannot determine VERILATOR_ROOT from `verilator -V`");
    std::process::exit(1);
}

fn run_tool(mut cmd: Command, what: &str) {
    let status = cmd.status().unwrap_or_else(|e| {
        eprintln!("virt: cannot run {what}: {e}");
        std::process::exit(1);
    });
    if !status.success() {
        eprintln!("virt: {what} failed ({status})");
        std::process::exit(1);
    }
}
