//! `virt` — build and run simulators for the virtual FPGA platform (`virt::Virt`).
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

    // Locate the fixed harness.  Prefer $VIRT_SIM_CPP (override); otherwise
    // use the sim.cpp shipped next to this crate's source.
    let sim_cpp: PathBuf = match env::var("VIRT_SIM_CPP") {
        Ok(p) => PathBuf::from(p),
        Err(_) => PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("sim.cpp"),
    };
    if !sim_cpp.is_file() {
        eprintln!("virt: sim.cpp not found at {}", sim_cpp.display());
        std::process::exit(1);
    }

    // Find Verilator's include directory.
    let verilator_root = verilator_root();

    // Verilator: --prefix Vvirt pins the generated class name to `Vvirt`,
    // regardless of the top module's name, so the fixed sim.cpp works for
    // any design.
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
        .arg("-I").arg(verilator_root.join("include/vltstd"))
        .arg(&sim_cpp)
        .arg(&lib)
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
/// source (src/*.vir, ext/*.sv).
fn needs_rebuild(bin: &Path, build_dir: &Path) -> bool {
    if !bin.is_file() {
        return true;
    }
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
    // The fixed harness is a source too: a sim.cpp change must trigger a
    // rebuild even when no .vir/.sv file changed.
    let harness: PathBuf = match env::var("VIRT_SIM_CPP") {
        Ok(p) => PathBuf::from(p),
        Err(_) => PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("assets").join("sim.cpp"),
    };
    sources.push(harness);
    let _ = build_dir;

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
