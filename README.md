# virdant-virt

Simulator tooling for the **virtual FPGA** platforms `virt::Virt` and
`virt::VirtVga` (defined in `virdant/lib/virt.vir`).

`virt` is not a physical board.  Designs written `for Virt` are compiled
to Verilog with `vir build` as usual, then wrapped in a Verilator-based
simulator by this tool.  The simulated board emulates:

- **UART** at 115200 8N1, wired to stdin/stdout.
- **SPI flash**: 16 MiB, initially all `0xFF`, readable with the standard
  `0x03` (READ) command — continuously, with auto-incrementing address,
  until `spi_cs` deasserts.

Designs written `for VirtVga` additionally get a **VGA monitor**,
modelled on the Digilent Nexys4's VGA connector (RGB444 + HSYNC/VSYNC,
50 MHz board clock), rendered into an SDL2 window.

## Usage

Inside a Virdant project whose top module is `for Virt` (or `VirtVga`):

```sh
virt build --top=MyTop    # runs `vir build`, then Verilator → build/MyTop
virt run                  # build (if stale) and run; flags pass through
./build/MyTop             # or run the binary directly
```

`--top` defaults to `[prog] top` from Virdant.toml (e.g. `top::Top` → the
simple name `Top`); the flag overrides it.  `virt run` rebuilds when the
binary is missing or older than any `src/*.vir` / `ext/*.sv` or the
harness; unknown flags are passed to the simulator binary.

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
from a pass).  Interactive runs end with Ctrl+C (code 0), or — on the
`VirtVga` board — closing the SDL window / pressing ESC or `q`.

## How it works

- `vir build` resolves the top module's `for` clause and emits the
  platform metadata to **`build/virmanifest.json`**: the platform name,
  FPGA family, clock frequencies, and port list.  `virt build` reads that
  manifest to decide which board this build simulates:
  - `virt::Virt` → the `sim.cpp` harness (`$VIRT_SIM_CPP` overrides).
  - `virt::VirtVga` → the `sim_vga.cpp` harness (`$VIRT_SIM_VGA_CPP`
    overrides), compiled with `-DVIRT_CLOCK_HZ=<manifest clock>` and
    linked against **SDL2** (`sdl2-config`; e.g. `libsdl2-dev` on Debian).
- `virt build` runs Verilator with `--prefix Vvirt`, so the generated C++
  class is always `Vvirt` regardless of the top module's name.  This lets
  the fixed harnesses work for any design.
- `sim.cpp` implements the board side of the `virt::Virt` port contract:
  it decodes UART frames on `tx`, shifts keystrokes into `rx`, and models
  the SPI flash device on `spi_ck`/`spi_cs`/`spi_mosi`/`spi_miso`.
- `sim_vga.cpp` implements `virt::VirtVga`: everything `sim.cpp` emulates
  plus a **sync-driven VGA monitor**.  Nothing about the design's internal
  pixel-clock divider or sync polarity is assumed: the HSYNC/VSYNC
  periods, pulse widths, and polarities are measured over the first
  frames and matched against a **VESA timing table** (see
  `assets/sim_vga.cpp`, `VESA_MODES`) to recover the frame geometry;
  pixels are then sampled from the RGB buses at the design's own
  measured pixel rate into an SDL2 window.  Any table mode works —
  including the two most popular, 640x480@60 (25 MHz pixel clock = the
  board clock /2) and 800x600@72 (50 MHz = the board clock), and a dozen
  more; new resolutions are one table entry each (see the
  `ADD NEW RESOLUTIONS HERE` block comment in `sim_vga.cpp`).  Headless
  runs work with `SDL_VIDEODRIVER=dummy`.

## Port contract

Designs must use exactly the platform's ports.  `Virt`:

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

`VirtVga` has the same ports (clock at 50 MHz) plus:

| Port | Dir | Meaning |
|------|-----|---------|
| `vga_r` | outgoing | VGA red, 4 bits (RGB444) |
| `vga_g` | outgoing | VGA green, 4 bits (RGB444) |
| `vga_b` | outgoing | VGA blue, 4 bits (RGB444) |
| `vga_hs` | outgoing | horizontal sync (either polarity; recovered from duty cycle) |
| `vga_vs` | outgoing | vertical sync (either polarity; recovered from duty cycle) |