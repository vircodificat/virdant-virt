// sim_vga.cpp — the virtual FPGA board with a VGA monitor, emulated
// with Verilator + SDL2.
//
// This harness is part of the `virt` tool (virdant-virt/).  It
// implements the "board" side of the `virt::VirtVga` platform
// (virdant/lib/virt.vir): everything `sim.cpp` emulates for `Virt` —
//
//   - UART at 115200 8N1, wired to stdin/stdout.
//   - A 16 MiB SPI flash device (initially all 0xFF), readable with the
//     standard `0x03` READ command, continuously, with auto-incrementing
//     address until CS deasserts.  Preload with --flash=<FILE>.
//
// — plus a VGA monitor modelled on the Digilent Nexys4's VGA connector:
// 4 bits per RGB channel (RGB444) and active-high HSYNC/VSYNC.  The
// output is rendered into an SDL2 window.
//
// The compiled design is always instantiated as class `Vvirt` because
// `virt build` passes Verilator `--prefix Vvirt`.  The harness therefore
// works for any top module name, as long as the top conforms to the
// virt::VirtVga port contract:
//
//   incoming clock    : Clock          -> top->clock
//   outgoing tx       : Bit            -> top->tx
//   incoming rx       : Bit            -> top->rx
//   outgoing exit     : Valid[Word[8]] -> top->exit (10 bits: [1:0] tag,
//     [9:2] payload; tag 2'b10 = @Valid(n), 2'b01 = @Invalid())
//   outgoing spi_ck   : Bit            -> top->spi_ck
//   outgoing spi_cs   : Bit            -> top->spi_cs
//   outgoing spi_mosi : Bit            -> top->spi_mosi
//   incoming spi_miso : Bit            -> top->spi_miso
//   outgoing vga_r    : Word[4]        -> top->vga_r   (4 bits each)
//   outgoing vga_g    : Word[4]        -> top->vga_g
//   outgoing vga_b    : Word[4]        -> top->vga_b
//   outgoing vga_hs   : Bit            -> top->vga_hs
//   outgoing vga_vs   : Bit            -> top->vga_vs
//
// The monitor is *sync-driven*: nothing about the design's internal
// pixel-clock divider or sync polarity is assumed (the Nexys4 connector
// is active-high, VESA modes vary; the monitor recovers polarity from
// duty cycle).  Instead the HSYNC/VSYNC periods, pulse widths, and
// polarities are measured over the first frames and matched against the
// VESA timing table below to recover the frame geometry (visible area
// and porches); pixels are then sampled from the RGB buses at the
// design's own measured pixel rate.  A design can use any clock division
// (or none) and any of the table's resolutions.
//
// Compile-time configuration (set by `virt build`):
//   -DVIRT_CLOCK_HZ=<N>  The virt::VirtVga clock frequency (the
//                       platform's @period_ns, as an integer Hz).
//                       Defaults to the platform's 50 MHz.

#include "Vvirt.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <SDL2/SDL.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <signal.h>

// ─────────────────────────────────────────────────────────────────────────
// Clocking and baud
// ─────────────────────────────────────────────────────────────────────────

// virt::VirtVga clock: 50 MHz (the Nexys4's 100 MHz oscillator / 2, a
// clean source for common VGA pixel clocks: /2 → 25 MHz (640x480@60),
// /1 → 50 MHz (800x600@72)).  May be overridden at compile time.
#ifndef VIRT_CLOCK_HZ
#define VIRT_CLOCK_HZ 50000000ull
#endif
static const uint64_t CLOCK_HZ = VIRT_CLOCK_HZ;
static const uint64_t BAUD     = 115200;

// Baud timing, exact:  time is counted in *ticks* of 1/36 cycle.  One
// clock cycle advances the tick counter by 36; one bit-time is
// CLOCK_HZ * 36 / 115200 ticks, which is an integer for every standard
// virt clock (48 MHz → 15000, 50 MHz → 15625, 100 MHz → 31250), so all
// UART sample points are exact — no long-run drift.
static const uint64_t TICKS_PER_CYCLE = 36;
static const uint64_t TICKS_PER_BIT   = CLOCK_HZ * TICKS_PER_CYCLE / BAUD;
static_assert(CLOCK_HZ * TICKS_PER_CYCLE % BAUD == 0,
              "115200 baud is not cycle-exact at this VIRT_CLOCK_HZ");

static const uint64_t FLASH_SIZE = 1ull << 24; // 16 MiB

// Simulation time reported to Verilator.  The VCD is declared in
// picoseconds ($timescale 1ps), so advance one 20000 ps clock period
// (10000 ps per half cycle = 50 MHz) per emulated cycle.
static uint64_t sim_time_ps = 0;
double sc_time_stamp() { return (double)sim_time_ps; }

// ─────────────────────────────────────────────────────────────────────────
// VESA timing table
// ─────────────────────────────────────────────────────────────────────────
//
// One entry per video mode, in VESA/DMT order (front porch → sync →
// back porch, relative to the *sync pulse start* line/pixel origin).
//
// ─┬──────────────────────────────────────────────────────────────────────
//  │ ADD NEW RESOLUTIONS HERE
//  │
//  │ Append to VESA_MODES below.  All four quantities must be the
//  │ standard's exact values, because mode identification matches the
//  │ measured sync timings against them (within MATCH_TOLERANCE):
//  │
//  │   { "<name>", <pixel_clock_hz>,
//  │     { <visible>, <front>, <sync>, <back>, <hsync active low?> },
//  │     { <visible>, <front>, <sync>, <back>, <vsync active low?> } },
//  │
//  │ The two most popular modes (640x480@60, 800x600@60) are first.
// ─┴──────────────────────────────────────────────────────────────────────

/// One axis of a video mode.  Porches run back-to-sync-pulse-start:
/// the `front` porch precedes the sync pulse, the `back` porch follows
/// it, and the visible area starts after `back`.
struct VesaTiming {
    uint32_t visible;        // visible pixels (H) or lines (V)
    uint32_t front;          // front porch
    uint32_t sync;           // sync pulse width
    uint32_t back;           // back porch
    bool     active_low;     // sync polarity: true = active low

    uint32_t total() const { return visible + front + sync + back; }
};

/// One VESA video mode.
struct VesaMode {
    const char* name;        // e.g. "640x480@60"
    uint64_t    pixel_clock_hz;
    VesaTiming  h;
    VesaTiming  v;
};

static const VesaMode VESA_MODES[] = {
    //                mode          pixel clock  H: vis/f/sync/back, pol    V: vis/f/sync/back, pol
    { "640x480@60",  25175000ull, { 640, 16,  96,  48, true  }, { 480, 10,  2,  33, true  } },
    { "800x600@60",  40000000ull, { 800, 40, 128,  88, false }, { 600,  1,  4,  23, false } },
    { "640x480@72",  31500000ull, { 640, 24,  40, 128, true  }, { 480,  9,  3,  28, true  } },
    { "640x480@75",  31500000ull, { 640, 16,  64, 120, true  }, { 480,  1,  3,  16, false } },
    { "800x600@56",  36000000ull, { 800, 24, 128,  72, false }, { 600,  1,  4,  20, false } },
    { "800x600@72",  50000000ull, { 800, 56, 120,  64, false }, { 600, 37,  6,  23, false } },
    { "800x600@75",  49500000ull, { 800, 16,  80, 160, false }, { 600,  1,  3,  21, false } },
    { "1024x768@60", 65000000ull, {1024, 24, 136, 160, true  }, { 768,  3,  6,  29, true  } },
    { "1024x768@75", 78750000ull, {1024, 16,  96, 176, false }, { 768,  1,  3,  28, false } },
    { "1280x720@60", 74250000ull, {1280,110,  40, 220, false }, { 720,  5,  5,  20, false } },
    { "1280x1024@60",108000000ull,{1280, 48, 112, 248, false }, {1024,  1,  3,  38, false } },
    { "1600x1200@60",162000000ull,{1600, 64, 192, 304, false }, {1200,  1,  3,  46, false } },
    { "1920x1080@60",148500000ull,{1920, 88,  44, 148, false }, {1080,  4,  5,  36, false } },
};
static const size_t VESA_MODE_COUNT = sizeof VESA_MODES / sizeof VESA_MODES[0];

// Relative error tolerated when matching measured sync timings against
// the table.  Generous enough for clock-divider rounding (640x480@60
// from a 25 MHz /2 pixel clock is 0.8% off the 25.175 MHz standard),
// tight enough that no two table modes can cross-match.
static const double MATCH_TOLERANCE = 0.04;

// ─────────────────────────────────────────────────────────────────────────
// Terminal raw mode (so keystrokes go straight to the simulated UART)
// ─────────────────────────────────────────────────────────────────────────

static struct termios saved_term;
static bool term_saved = false;

static void restore_term() {
    if (term_saved) {
        tcsetattr(STDIN_FILENO, TCSANOW, &saved_term);
        term_saved = false;
    }
}

extern "C" void sigint_handler(int) {
    restore_term();
    _exit(0);
}

// ─────────────────────────────────────────────────────────────────────────
// SPI flash model
// ─────────────────────────────────────────────────────────────────────────
//
// (Identical to sim.cpp: SPI mode 0, READ (0x03) only, framed by CS.)

struct SpiFlash {
    std::vector<uint8_t> mem; // FLASH_SIZE bytes, initially 0xFF

    enum State { IDLE, CMD, ADDR, DATA, IGNORE };
    State state = IDLE;

    bool prev_cs = true;
    bool prev_ck = false;
    bool prev_mosi = false;

    uint8_t  cmd       = 0;
    int      cmd_bits  = 0;
    uint32_t addr      = 0;
    int      addr_bits = 0;

    uint8_t tx_byte = 0;
    int     tx_bits = 0;
    bool    miso    = false;

    bool    trace = false;
    uint64_t cycle = 0;

    SpiFlash() : mem(FLASH_SIZE, 0xFF) {}

    void load(const std::string& path) {
        FILE* f = fopen(path.c_str(), "rb");
        if (!f) {
            fprintf(stderr, "virt: cannot open flash image: %s\n", path.c_str());
            exit(1);
        }
        size_t n = fread(mem.data(), 1, FLASH_SIZE, f);
        fclose(f);
        fprintf(stderr, "virt: loaded %zu bytes from %s\n", n, path.c_str());
    }

    // See sim.cpp for the NBA-semantics commentary: a ck edge is only
    // processed when cs was LOW *before* the edge; a rising ck samples
    // the PRE-edge mosi; miso changes after a falling ck (SPI mode 0).
    void step(bool cs, bool ck, bool mosi) {
        bool cs_fell = prev_cs && !cs;
        bool cs_rose = !prev_cs && cs;
        bool ck_rose = !prev_ck && ck;
        bool ck_fell = prev_ck && !ck;
        bool active = prev_cs == false;

        if (cs_fell) {
            state = CMD;
            cmd = 0; cmd_bits = 0;
            addr = 0; addr_bits = 0;
            tx_bits = 0;
            miso = false;
        }
        if (cs_rose) {
            state = IDLE;
            miso = false;
        }

        if (active) {
            if (ck_rose) {
                bool bit = prev_mosi;
                switch (state) {
                case CMD:
                    cmd = (uint8_t)((cmd << 1) | (bit ? 1 : 0));
                    if (++cmd_bits == 8) {
                        if (trace) fprintf(stderr, "[virt spi %llu] cmd 0x%02X\n", (unsigned long long)cycle, cmd);
                        if (cmd == 0x03) {
                            state = ADDR;
                            addr = 0; addr_bits = 0;
                        } else {
                            state = IGNORE;
                        }
                    }
                    break;
                case ADDR:
                    addr = (addr << 1) | (bit ? 1 : 0);
                    if (++addr_bits == 24) {
                        uint32_t a = addr & (FLASH_SIZE - 1);
                        if (trace) fprintf(stderr, "[virt spi %llu] READ addr 0x%06X\n", (unsigned long long)cycle, a);
                        tx_byte = mem[a];
                        tx_bits = 0;
                        state = DATA;
                    }
                    break;
                default:
                    break;
                }
            }
            if (ck_fell && state == DATA) {
                miso = (tx_byte >> (7 - tx_bits)) & 1;
                if (++tx_bits == 8) {
                    tx_bits = 0;
                    uint32_t a = (addr + 1) & (FLASH_SIZE - 1);
                    addr = a;
                    tx_byte = mem[a];
                }
            }
        }

        prev_cs = cs;
        prev_ck = ck;
        prev_mosi = mosi;
    }

    bool miso_active() const { return state == DATA; }
    bool miso_value()  const { return miso; }
};

// ─────────────────────────────────────────────────────────────────────────
// UART emulation
// ─────────────────────────────────────────────────────────────────────────
//
// (Identical to sim.cpp modulo the generalized TICKS_PER_BIT above.)

struct UartTx {
    enum State { IDLE, RECEIVING } state = IDLE;
    uint64_t t = 0;
    uint64_t next = 0;
    int      bit = 0;
    uint8_t  data = 0;
    bool     last = true;

    void cycle(bool tx) {
        switch (state) {
        case IDLE:
            if (last && !tx) {
                state = RECEIVING;
                bit = 0;
                data = 0;
                t = 0;
                next = TICKS_PER_BIT / 2;
            }
            break;
        case RECEIVING:
            t += TICKS_PER_CYCLE;
            if (t >= next) {
                next += TICKS_PER_BIT;
                if (bit == 0) {
                    // start bit: expected low
                } else if (bit >= 1 && bit <= 8) {
                    data |= (tx ? 1u : 0u) << (bit - 1);
                } else {
                    if (tx) {
                        uint8_t b = data;
                        ssize_t n = write(STDOUT_FILENO, &b, 1);
                        (void)n;
                    }
                    state = IDLE;
                }
                bit++;
            }
            break;
        }
        last = tx;
    }
};

struct UartRx {
    enum State { IDLE, SENDING } state = IDLE;
    uint64_t t = 0;
    uint64_t next = 0;
    int      bit = 0;
    uint8_t  data = 0;
    bool     pin = true;
    bool     trace = false;
    std::deque<uint8_t> pending;
    bool     eof = false;

    void poll_stdin() {
        uint8_t buf[256];
        for (;;) {
            ssize_t n = read(STDIN_FILENO, buf, sizeof buf);
            if (n == 0) { eof = true; break; }
            if (n < 0) break;
            for (ssize_t i = 0; i < n; i++) pending.push_back(buf[i]);
        }
    }

    void cycle() {
        switch (state) {
        case IDLE:
            pin = true;
            if (!pending.empty()) {
                data = pending.front();
                pending.pop_front();
                if (trace) fprintf(stderr, "[virt uart %p] rx byte 0x%02X\n", (void*)this, data);
                state = SENDING;
                bit = 0;
                t = 0;
                next = TICKS_PER_BIT;
            }
            break;
        case SENDING:
            t += TICKS_PER_CYCLE;
            if (t >= next) {
                next += TICKS_PER_BIT;
                bit++;
                if (bit == 10) {
                    state = IDLE;
                }
            }
            break;
        }
        if (state == SENDING) {
            if (bit == 0)                  pin = false;
            else if (bit >= 1 && bit <= 8) pin = (data >> (bit - 1)) & 1;
            else                           pin = true;
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────
// VGA monitor emulation (SDL2)
// ─────────────────────────────────────────────────────────────────────────
//
// Fixed-point: all "fp" quantities below are in units of
// 1/FP_SCALE simulated clock cycles.

static const int      FP_SHIFT = 16;
static const uint64_t FP_SCALE = 1ull << FP_SHIFT;

/// Period/pulse-width measurement for one sync signal, from pin edges.
/// All quantities are whole simulated cycles; `t` is the global cycle
/// counter at each edge.
struct SyncMeasure {
    bool have_prev = false;
    bool prev = false;
    uint64_t falls = 0, rises = 0;   // edge counts
    uint64_t last_fall = 0, last_rise = 0;
    uint64_t period_fall = 0, period_rise = 0; // start-to-start periods
    uint64_t gap_fr = 0, gap_rf = 0; // low/high pulse durations

    void step(bool pin, uint64_t t) {
        if (!have_prev) { prev = pin; have_prev = true; return; }
        if (pin == prev) return;
        if (!pin) {                      // falling edge
            if (falls > 0) period_fall = t - last_fall;
            if (rises > 0) gap_rf = t - last_rise;
            last_fall = t;
            falls++;
        } else {                         // rising edge
            if (rises > 0) period_rise = t - last_rise;
            if (falls > 0) gap_fr = t - last_fall;
            last_rise = t;
            rises++;
        }
        prev = pin;
    }

    /// Two full pulses seen: periods and widths are measured.
    bool ready() const { return falls >= 2 && rises >= 2; }

    /// Pulse-start-to-pulse-start period, in cycles (fall/rise edges
    /// agree in steady state; average for noise immunity).
    double period() const {
        double p = 0; int n = 0;
        if (period_fall) { p += (double)period_fall; n++; }
        if (period_rise) { p += (double)period_rise; n++; }
        return n ? p / n : 0.0;
    }

    /// Time the pin spends low / high, in cycles (the shorter of the
    /// two is the sync pulse; the other is the inter-pulse gap).
    double pulse_low()  const { return (double)gap_fr; }
    double pulse_high() const { return (double)gap_rf; }
};

struct VgaMonitor {
    // Raw pin history
    bool prev_hs = true, prev_vs = true;
    bool have_prev = false;
    uint64_t t = 0;                    // global sim cycle counter

    SyncMeasure hs, vs;

    // Identification
    const VesaMode* mode = nullptr;
    uint64_t unrecognized_warns = 0;

    // Sync polarity as driven by the design (recovered from duty cycle:
    // the pulse is the <20% duty level), set at adoption time.
    bool hs_pulse_low = false, vs_pulse_low = false;

    // Capture geometry (fixed point cycles)
    uint64_t pixel_period_fp = 0;      // cycles per pixel
    uint64_t sync_width_fp   = 0;      // hsync pulse width, cycles
    int64_t  v_origin_line   = 0;      // first visible line, counted from vsync pulse start
    int64_t  line_index      = 0;      // lines since vsync pulse start (-1 = pulse line not yet ended... see cycle())
    uint64_t line_time_fp    = 0;      // cycles since line (hsync pulse) start
    uint64_t next_capture_fp = 0;      // line time of the next pixel capture
    uint32_t capture_x       = 0;      // next visible pixel x within the line
    bool     frame_armed     = false;  // capture starts at the next vsync pulse
    bool     frame_active    = false;   // capturing a frame now
    bool     line_capture    = false;   // this line is in the visible band

    // SDL
    bool sdl_init     = false;
    SDL_Window*   window   = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* texture  = nullptr;
    uint32_t tex_w = 0, tex_h = 0;
    std::vector<uint8_t> fb;           // RGB24, w*h*3
    uint64_t frames_presented = 0;
    bool quit = false;
    bool trace = false;

    // ── mode matching ────────────────────────────────────────────────────

    /// Expected HSYNC period in cycles for `m`.
    double exp_line_period(const VesaMode* m) const {
        return (double)m->h.total() * CLOCK_HZ / (double)m->pixel_clock_hz;
    }

    /// Worst-case relative error between the measured sync timings and
    /// `m`, or a value > MATCH_TOLERANCE if any quantity is off.
    ///
    /// Pulse *polarity* is deliberately not scored: sync pulses are
    /// <20% duty in every table mode, so the pulse is whichever level
    /// has the lower duty cycle, whatever the design's polarity.  (The
    /// Nexys4 reference drives active-high syncs; VESA modes specify
    /// varying polarity — a monitor recovers it, and so does this.)
    /// With polarity ignored, no two table modes share period/pulse
    /// timings, so identification stays unambiguous.
    double match_error(const VesaMode* m) const {
        double line  = hs.period();
        double frame = vs.period();
        double exp_line  = exp_line_period(m);
        double exp_frame = (double)m->v.total() * exp_line;
        double exp_hs = (double)m->h.sync * CLOCK_HZ / (double)m->pixel_clock_hz;

        // Vertical sync pulses are a couple of *lines* wide: scale the
        // expectation with the measured line period rather than the
        // table's own pixel clock, so a design running the mode at a
        // slightly-off pixel rate (25 MHz vs 25.175) still matches.
        // (v.sync is in *lines*; lines × cycles/line = cycles.)
        double exp_vs = (double)m->v.sync * line;

        double hs_pulse = hs.pulse_low() < hs.pulse_high()
            ? hs.pulse_low() : hs.pulse_high();
        double vs_pulse = vs.pulse_low() < vs.pulse_high()
            ? vs.pulse_low() : vs.pulse_high();

        double errs[4] = {
            std::fabs(line  - exp_line ) / exp_line,
            std::fabs(frame - exp_frame) / exp_frame,
            std::fabs(hs_pulse - exp_hs) / exp_hs,
            std::fabs(vs_pulse - exp_vs) / exp_vs,
        };
        double worst = 0;
        for (double e : errs) worst = worst < e ? e : worst;
        return worst;
    }

    /// Adopt the best-matching mode, or nullptr if none is close.
    const VesaMode* identify() const {
        const VesaMode* best = nullptr;
        double best_err = MATCH_TOLERANCE;
        for (size_t i = 0; i < VESA_MODE_COUNT; i++) {
            double e = match_error(&VESA_MODES[i]);
            if (e < best_err) { best = &VESA_MODES[i]; best_err = e; }
        }
        return best;
    }

    // ── SDL plumbing ────────────────────────────────────────────────────

    bool sdl_setup(uint32_t w, uint32_t h) {
        if (!sdl_init) {
            if (SDL_Init(SDL_INIT_VIDEO) != 0) {
                fprintf(stderr, "virt: SDL_Init failed: %s\n", SDL_GetError());
                return false;
            }
            sdl_init = true;
        }
        if (window && tex_w == w && tex_h == h) return true;

        if (texture) { SDL_DestroyTexture(texture); texture = nullptr; }
        if (renderer) { SDL_DestroyRenderer(renderer); renderer = nullptr; }
        if (window) { SDL_DestroyWindow(window); window = nullptr; }

        window = SDL_CreateWindow(
            "virt: virt::VirtVga", 0, 0, (int)w, (int)h,
            SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
        if (!window) {
            fprintf(stderr, "virt: SDL_CreateWindow failed: %s\n", SDL_GetError());
            return false;
        }
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
        if (!renderer) renderer = SDL_CreateRenderer(window, -1, 0);
        if (!renderer) {
            fprintf(stderr, "virt: SDL_CreateRenderer failed: %s\n", SDL_GetError());
            return false;
        }
        SDL_RenderSetLogicalSize(renderer, (int)w, (int)h);
        texture = SDL_CreateTexture(
            renderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING,
            (int)w, (int)h);
        if (!texture) {
            fprintf(stderr, "virt: SDL_CreateTexture failed: %s\n", SDL_GetError());
            return false;
        }
        tex_w = w; tex_h = h;
        fb.assign((size_t)w * h * 3, 0);
        return true;
    }

    /// Drain the SDL event queue; flags window-close / ESC / q.
    void poll_events() {
        if (!sdl_init) return;
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) quit = true;
            if (ev.type == SDL_KEYDOWN) {
                SDL_Keycode k = ev.key.keysym.sym;
                if (k == SDLK_ESCAPE || k == SDLK_q) quit = true;
            }
        }
    }

    /// Blit the framebuffer and show it.
    void present() {
        if (!texture || quit) return;
        SDL_UpdateTexture(texture, nullptr, fb.data(), (int)tex_w * 3);
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
        frames_presented++;
        if (trace) {
            fprintf(stderr, "[virt vga %llu] presented frame %llu (%ux%u)\n",
                    (unsigned long long)t, (unsigned long long)frames_presented,
                    tex_w, tex_h);
        }
        poll_events();
    }

    /// Tear the identification down (e.g. on drift) to re-measure.
    void drop_mode(const char* why) {
        fprintf(stderr, "virt: vga: sync timings drifted from %s (%s); re-measuring\n",
                mode ? mode->name : "?", why);
        mode = nullptr;
        frame_armed = frame_active = line_capture = false;
    }

    /// Adopt `m`: derive the capture geometry from the *measured*
    /// timings (so the monitor tracks the design's actual pixel rate,
    /// whatever divider it uses) and open the window.
    void adopt(const VesaMode* m) {
        mode = m;

        // Polarity: whichever level has the lower duty cycle is the
        // pulse (see match_error).
        hs_pulse_low = hs.pulse_low() < hs.pulse_high();
        vs_pulse_low = vs.pulse_low() < vs.pulse_high();

        // Pixel rate: measured line period / total pixels per line.
        double line = hs.period();                 // cycles
        double px   = line / m->h.total();          // cycles per pixel
        pixel_period_fp = (uint64_t)llround(px * (double)FP_SCALE);

        // HSYNC pulse width as the design drives it.
        double hsw = hs_pulse_low ? hs.pulse_low() : hs.pulse_high();
        sync_width_fp = (uint64_t)llround(hsw * (double)FP_SCALE);

        // Vertical: the vsync pulse is vsync_lines lines long, then the
        // back porch, then the visible lines.
        double vsw = vs_pulse_low ? vs.pulse_low() : vs.pulse_high();
        double vsync_lines = vsw / line;
        v_origin_line = (int64_t)llround(vsync_lines) + m->v.back;

        if (!sdl_setup(m->h.visible, m->v.visible)) {
            // Unrecoverable in a headless environment: keep simulating
            // (UART/SPI/exit still work); the picture just cannot show.
            mode = nullptr;
            return;
        }
        frame_armed = true;   // start capturing at the next vsync pulse
        frame_active = false;
        if (trace) {
            fprintf(stderr, "[virt vga %llu] identified %s (%.1f MHz pixel clock; "
                    "%.2f cycles/pixel measured, %s-sync)\n",
                    (unsigned long long)t, m->name,
                    m->pixel_clock_hz / 1e6, px,
                    hs_pulse_low ? "active-low" : "active-high");
        }
    }

    /// Re-check the measured timings against the adopted mode, at each
    /// frame boundary.
    void check_drift() {
        if (!mode) return;
        double e = match_error(mode);
        if (e > MATCH_TOLERANCE) drop_mode("mode timings no longer match");
    }

    /// One simulated clock cycle: `hs`/`vs` are the sync pins, `r`/`g`/`b`
    /// the 4-bit RGB buses, all sampled after the posedge eval.
    void cycle(bool hs_pin, bool vs_pin, uint8_t r, uint8_t g, uint8_t b) {
        t++;
        // Advance line time first: the line-start cycle below (the
        // cycle whose posedge begins the sync pulse) is time 0, so the
        // first visible pixel is captured a whole number of pixel
        // periods after it, aligned with the design's own counters.
        line_time_fp += FP_SCALE;
        hs.step(hs_pin, t);
        vs.step(vs_pin, t);

        // Sync *pulse starts*: edges into the design's pulse level
        // (polarity recovered at adoption; hs_pulse_low ? falling : rising).
        bool hs_start = false, vs_start = false;
        if (have_prev) {
            hs_start = hs_pin != prev_hs && (hs_pulse_low ? !hs_pin : hs_pin);
            vs_start = vs_pin != prev_vs && (vs_pulse_low ? !vs_pin : vs_pin);
        }
        have_prev = true;
        prev_hs = hs_pin;
        prev_vs = vs_pin;

        // ── identification (before capture, so mode is fresh) ──
        if (!mode) {
            if (hs.ready() && vs.ready()) {
                const VesaMode* m = identify();
                if (m) {
                    adopt(m);
                } else if (unrecognized_warns < 3) {
                    unrecognized_warns++;
                    fprintf(stderr,
                        "virt: vga: unrecognized sync timings after %llu cycles "
                        "(line %.0f, frame %.0f, hs pulse %.0f, vs pulse %.0f); "
                        "add the mode to the VESA_MODES table in sim_vga.cpp?\n",
                        (unsigned long long)t,
                        hs.period(), vs.period(),
                        hs.pulse_low() < hs.pulse_high() ? hs.pulse_low() : hs.pulse_high(),
                        vs.pulse_low() < vs.pulse_high() ? vs.pulse_low() : vs.pulse_high());
                }
            }
        }

        // ── frame boundary ──
        if (vs_start) {
            if (frame_active) present();
            check_drift();
            if (mode && frame_armed) frame_active = true;
            line_index = -1;   // the vsync pulse start line; line 0 is the
                               // next hsync pulse start (usually this cycle)
        }

        // ── line boundary ──
        if (mode && hs_start) {
            line_index++;
            line_time_fp = 0;
            capture_x = 0;
            line_capture = frame_active
                && line_index >= v_origin_line
                && line_index <  v_origin_line + (int64_t)mode->v.visible;
            // First visible pixel: after the hsync pulse and the back
            // porch, both as the *design* runs them (measured pulse
            // width; porch in pixels at the measured pixel rate).
            next_capture_fp = sync_width_fp
                + (uint64_t)mode->h.back * pixel_period_fp;
        }

        // ── pixel capture ──
        if (mode && line_capture) {
            const uint32_t w = mode->h.visible;
            while (capture_x < mode->h.visible && line_time_fp >= next_capture_fp) {
                int64_t y = line_index - v_origin_line;
                if (y >= 0 && y < (int64_t)mode->v.visible) {
                    uint8_t* p = &fb[((size_t)y * w + capture_x) * 3];
                    p[0] = (uint8_t)((r << 4) | (r & 0xF));   // 4-bit → 8-bit
                    p[1] = (uint8_t)((g << 4) | (g & 0xF));
                    p[2] = (uint8_t)((b << 4) | (b & 0xF));
                }
                capture_x++;
                next_capture_fp += pixel_period_fp;
            }
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    std::string flash_path;
    std::string vcd_path;
    uint64_t max_cycles = ~0ull;
    bool trace = false;

    auto value_of = [&](const std::string& name, int& i) -> std::string {
        std::string a = argv[i];
        size_t eq = a.find('=');
        if (eq != std::string::npos) return a.substr(eq + 1);
        if (i + 1 < argc) return argv[++i];
        fprintf(stderr, "virt: missing value for %s\n", name.c_str());
        exit(2);
    };
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a.rfind("--flash=", 0) == 0 || a == "--flash") {
            flash_path = value_of("--flash", i);
        } else if (a.rfind("--vcd=", 0) == 0 || a == "--vcd") {
            vcd_path = value_of("--vcd", i);
        } else if (a.rfind("--cycles=", 0) == 0 || a == "--cycles") {
            max_cycles = strtoull(value_of("--cycles", i).c_str(), nullptr, 0);
        } else if (a == "--trace") {
            trace = true;
        } else if (a == "-h" || a == "--help") {
            printf("usage: %s [--flash=<FILE>] [--vcd=<FILE>] [--trace] [--cycles=<N>]\n", argv[0]);
            return 0;
        }
        // Unknown flags are ignored (Verilator also consumes +args).
    }

    SpiFlash flash;
    flash.trace = trace;
    if (!flash_path.empty()) flash.load(flash_path);

    // Interactive stdin: raw mode so characters reach the simulated UART
    // as they are typed.  In both cases stdin is made non-blocking: the
    // main loop polls it periodically and must never stall the simulation.
    bool stdin_is_tty = isatty(STDIN_FILENO) == 1;
    int stdin_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (stdin_flags != -1) fcntl(STDIN_FILENO, F_SETFL, stdin_flags | O_NONBLOCK);
    if (stdin_is_tty) {
        if (tcgetattr(STDIN_FILENO, &saved_term) == 0) {
            term_saved = true;
            struct termios raw = saved_term;
            cfmakeraw(&raw);
            raw.c_oflag |= OPOST | ONLCR;
            raw.c_iflag |= ICRNL;
            raw.c_lflag |= ISIG;
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        }
        signal(SIGINT, sigint_handler);
    }

    Vvirt* top = new Vvirt;

    // Start inputs in a sane state.
    top->rx       = 1; // UART idle high
    top->spi_miso = 0;

    UartTx uart_tx;
    UartRx uart_rx;
    uart_rx.trace = trace;

    VgaMonitor vga;
    vga.trace = trace;

    // Optional VCD tracing (requires the model to be built with verilator
    // --trace, which `virt build` always does).
    VerilatedVcdC* tfp = nullptr;
    if (!vcd_path.empty()) {
        tfp = new VerilatedVcdC;
        top->trace(tfp, 99);
        tfp->open(vcd_path.c_str());
    }

    uint64_t cycles = 0;
    const uint64_t stdin_poll_interval = 1000;  // ~20 µs of simulated time
    const uint64_t event_poll_interval  = 500000;
    int exit_code = 0;
    bool exit_requested = false;

    // Termination: the `exit` port (@Valid(n) exits with code n), the
    // --cycles cap (124), $finish, closing the SDL window, or Ctrl+C
    // (interactive).  There is no stdin-EOF exit: per the platform
    // contract the simulation runs for as long as `exit` is @Invalid().

    for (;;) {
        // ── Low phase ──
        top->clock = 0;
        top->rx       = uart_rx.pin;
        top->spi_miso = flash.miso_active() ? flash.miso_value() : 0;
        top->eval();
        flash.step(top->spi_cs, top->spi_ck, top->spi_mosi);
        sim_time_ps += 10000;
        if (tfp) tfp->dump(sim_time_ps);

        // ── High phase (posedge: the design samples rx / spi_miso here) ──
        top->clock = 1;
        top->eval();
        flash.step(top->spi_cs, top->spi_ck, top->spi_mosi);
        sim_time_ps += 10000;
        if (tfp) tfp->dump(sim_time_ps);

        // ── Per-cycle emulation ──
        flash.cycle = cycles;
        uart_tx.cycle(top->tx);
        if (cycles % stdin_poll_interval == 0) uart_rx.poll_stdin();
        uart_rx.cycle(); // computes uart_rx.pin for the next cycle
        vga.cycle(top->vga_hs != 0, top->vga_vs != 0,
                  (uint8_t)(top->vga_r & 0xF),
                  (uint8_t)(top->vga_g & 0xF),
                  (uint8_t)(top->vga_b & 0xF));
        if (cycles % event_poll_interval == 0) vga.poll_events(); // window stays responsive
                                                             // even without vsync
        // ── Exit request (virt::VirtVga `exit` port) ──
        // Valid[Word[8]] packs as tag [1:0] + payload [9:2]; tag 2'b10
        // is @Valid.  A @Valid(n) on a clock tick ends the simulation
        // with exit code n.
        if ((top->exit & 0x3) == 0x2) {
            exit_code = (int)((top->exit >> 2) & 0xFF);
            exit_requested = true;
            if (trace) {
                fprintf(stderr, "[virt] exit request @%llu cycles: code %d\n",
                        (unsigned long long)cycles, exit_code);
            }
            break;
        }

        if (vga.quit) break; // SDL window closed / ESC
        if (++cycles >= max_cycles) break;
        if (Verilated::gotFinish()) break;
    }

    restore_term();
    if (tfp) {
        tfp->close();
        delete tfp;
    }
    top->final();
    delete top;

    // Distinct status for hitting the cycle cap without an exit request,
    // so test drivers can distinguish a hang (124) from a clean run (0).
    if (!exit_requested && cycles >= max_cycles) return 124;
    return exit_code;
}