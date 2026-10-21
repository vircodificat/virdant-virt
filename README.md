# virdant-virt

Simulator tooling for the **virtual FPGA** platform `virt::Virt`
(defined in `virdant/lib/virt.vir`).

`virt` is not a physical board.  Designs written `for Virt` are compiled
to Verilog with `vir build` as usual, then wrapped in a Verilator-based
simulator by this tool.  The simulated board emulates:

- **UART** at 115200 8N1, wired to stdin/stdout.
- **SPI flash**: 16 MiB, initially all `0xFF`, readable with the standard
  `0x03` (READ) command — continuously, with auto-incrementing address,
  until `spi_cs` deasserts.

## Usage

Inside a Virdant project whose top module is `for Virt`:

```sh
virt build --top=MyTop    # runs `vir build`, then Verilator → build/MyTop
virt run                  # build (if stale) and run; flags pass through
./build/MyTop             # or run the binary directly
```

`--top` defaults to `[prog] top` from Virdant.toml (e.g. `top::Top` → the
simple name `Top`); the flag overrides it.  `virt run` rebuilds when the
binary is missing or older than any `src/*.vir` / `ext/*.sv`; unknown flags
are passed to the simulator binary.

### `virt build`

| Flag | Meaning |
|------|---------|
| `--top=<TOP>` | Name of the top module (required). |
| `--build-dir=<DIR>` | Where the `.sv` files live (default `build`). |

The produced binary is named `build/${TOP}`.

### Simulator binary flags

| Flag | Meaning |
|------|---------|
| `--flash=<FILE>` | Preload the emulated flash from FILE (up to 16 MiB). |
| `--cycles=<N>` | Stop after N emulated clock cycles. |
| `--vcd=<FILE>` | Record a VCD waveform trace. |
| `--trace` | Log device traffic (SPI transactions, UART bytes) to stderr. |

### Exit status

The platform's `exit` port drives termination: while `exit` is `@Invalid()`
the simulation runs; when it reads `@Valid(n)` on a clock tick the
simulator exits with code `n`.  Hitting the `--cycles` cap without an exit
request exits with code **124** (so test harnesses can distinguish a hang
from a pass).  Interactive runs end with Ctrl+C (code 0).

## How it works

- `virt build` runs Verilator with `--prefix Vvirt`, so the generated C++
  class is always `Vvirt` regardless of the top module's name.  This lets
  the fixed harness `sim.cpp` (the "virtual board") work for any design.
- `sim.cpp` implements the board side of the `virt::Virt` port contract:
  it decodes UART frames on `tx`, shifts keystrokes into `rx`, and models
  the SPI flash device on `spi_ck`/`spi_cs`/`spi_mosi`/`spi_miso`.

## Port contract

Designs must use exactly the `virt::Virt` platform ports:

| Port | Dir | Meaning |
|------|-----|---------|
| `clock` | incoming | 48 MHz primary clock |
| `tx` | outgoing | UART transmit → host stdout |
| `rx` | incoming | UART receive ← host stdin |
| `exit` | outgoing | exit request: `@Valid(n)` ends the simulation with code n |
| `spi_ck` | outgoing | SPI flash clock |
| `spi_cs` | outgoing | SPI flash chip select (active low) |
| `spi_mosi` | outgoing | SPI data out of the FPGA |
| `spi_miso` | incoming | SPI data into the FPGA |
