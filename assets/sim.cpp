// sim.cpp — the virtual FPGA board, emulated with Verilator.
//
// This harness is part of the `virt` tool (virdant-virt/).  It implements
// the "board" side of the `virt::Virt` platform (virdant/lib/virt.vir):
//
//   - UART at 115200 8N1, wired to stdin/stdout.
//   - A 16 MiB SPI flash device (initially all 0xFF), readable with the
//     standard `0x03` READ command, continuously, with auto-incrementing
//     address until CS deasserts.  Preload with --flash=<FILE>.
//
// The compiled design is always instantiated as class `Vvirt` because
// `virt build` passes Verilator `--prefix Vvirt`.  The harness therefore
// works for any top module name, as long as the top conforms to the
// virt::Virt port contract:
//
//   incoming clock  : Clock -> top->clock
//   outgoing tx     : Bit   -> top->tx
//   incoming rx     : Bit   -> top->rx
//   outgoing exit    : Valid[Word[8]] -> top->exit (10 bits: [1:0] tag,
//     [9:2] payload; tag 2'b10 = @Valid(n), 2'b01 = @Invalid())
//   outgoing spi_ck : Bit   -> top->spi_ck
//   outgoing spi_cs : Bit   -> top->spi_cs
//   outgoing spi_mosi : Bit -> top->spi_mosi
//   incoming spi_miso : Bit -> top->spi_miso
//
// Exit semantics: while `exit` is @Invalid(), the simulation runs.  When
// it reads @Valid(n) on a clock tick, the simulator exits with exit code
// n.  Reaching the --cycles cap without an exit request exits with code
// 124 (so test harnesses can tell a hang from a pass).

#include "Vvirt.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
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

static const uint64_t CLOCK_HZ   = 48'000'000; // virt::Virt clock
static const uint64_t BAUD       = 115'200;    // fixed 115200 8N1
static const uint64_t FLASH_SIZE = 1ull << 24; // 16 MiB

// Baud timing, exact:
//   cycles per bit = CLOCK_HZ / BAUD = 48000000 / 115200 = 1250 / 3.
// We count time in *ticks* of 1/3 cycle.  One clock cycle advances the
// tick counter by 3; one bit-time is exactly 1250 ticks.  All sample
// points are therefore exact integers — no long-run drift.
static const uint64_t TICKS_PER_CYCLE = 3;
static const uint64_t TICKS_PER_BIT   = 1250;

// Simulation time reported to Verilator.  The VCD is declared in
// picoseconds ($timescale 1ps), so advance one 20834 ps clock period
// (10417 ps per half cycle ≈ 48 MHz) per emulated cycle.
static uint64_t sim_time_ps = 0;
double sc_time_stamp() { return (double)sim_time_ps; }

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
// SPI mode 0: the device samples MOSI on rising edges of CK and updates
// MISO on falling edges.  Transactions are framed by CS (active low).

struct SpiFlash {
    std::vector<uint8_t> mem; // FLASH_SIZE bytes, initially 0xFF

    enum State { IDLE, CMD, ADDR, DATA, IGNORE };
    State state = IDLE;

    // Previous pin values, for edge detection.
    bool prev_cs = true;
    bool prev_ck = false;
    bool prev_mosi = false;

    uint8_t  cmd       = 0;
    int      cmd_bits  = 0;
    uint32_t addr      = 0;
    int      addr_bits = 0;

    uint8_t tx_byte = 0;
    int     tx_bits = 0;
    bool    miso    = false; // value driven on spi_miso while in DATA

    bool trace = false; // set by main(); logs device traffic to stderr
    uint64_t cycle = 0;  // current sim cycle (for trace timestamps)

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

    // Called once after every eval() with the current (post-edge) pin
    // values.  To match Verilog non-blocking-assignment semantics — the
    // contract under which the controller designs are written and tested
    // (see virdant-riscv/tb/spi_tb.sv) — a device's always-block at a
    // posedge reads the PRE-edge values of its inputs.  So:
    //
    //   - a ck edge is processed only if cs was LOW *before* the edge
    //     (an edge coinciding with the cs falling edge is ignored),
    //   - a rising ck edge samples the PRE-edge mosi (latched prev_mosi),
    //   - the device's miso changes after a falling ck edge (i.e. is
    //     stable through the following rising edge, SPI mode 0).
    //
    void step(bool cs, bool ck, bool mosi) {
        bool cs_fell = prev_cs && !cs;
        bool cs_rose = !prev_cs && cs;
        bool ck_rose = !prev_ck && ck;
        bool ck_fell = prev_ck && !ck;
        bool active = prev_cs == false; // cs was low before this edge

        if (cs_fell) {
            // New transaction: expect a command byte.  Any ck edge at this
            // same instant is ignored (cs was high pre-edge).
            state = CMD;
            cmd = 0; cmd_bits = 0;
            addr = 0; addr_bits = 0;
            tx_bits = 0;
            miso = false;
        }
        if (cs_rose) {
            // Transaction complete.
            state = IDLE;
            miso = false;
        }

        if (active) {
            if (ck_rose) {
                bool bit = prev_mosi; // sample pre-edge mosi (NBA semantics)
                switch (state) {
                case CMD:
                    cmd = (uint8_t)((cmd << 1) | (bit ? 1 : 0));
                    if (++cmd_bits == 8) {
                        if (trace) fprintf(stderr, "[virt spi %llu] cmd 0x%02X\n", (unsigned long long)cycle, cmd);
                        if (cmd == 0x03) { // READ
                            state = ADDR;
                            addr = 0; addr_bits = 0;
                        } else {
                            // Only READ is emulated; ignore until CS releases.
                            state = IGNORE;
                        }
                    }
                    break;
                case ADDR:
                    addr = (addr << 1) | (bit ? 1 : 0);
                    if (++addr_bits == 24) {
                        // Address complete: begin streaming data.  The first
                        // data bit goes out on the *next falling edge* of CK.
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
                    uint32_t a = (addr + 1) & (FLASH_SIZE - 1); // auto-increment, wraps
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

// TX: decode the design's transmission on `tx` (design → host stdout).
struct UartTx {
    enum State { IDLE, RECEIVING } state = IDLE;
    uint64_t t = 0;      // tick counter (1 tick = 1/3 cycle)
    uint64_t next = 0;   // tick count of the next sample point
    int      bit = 0;    // sample index: 0 = start, 1..8 = data, 9 = stop
    uint8_t  data = 0;
    bool     last = true;

    // One clock cycle.  `tx` is the current pin value.
    void cycle(bool tx) {
        switch (state) {
        case IDLE:
            // Detect the start-bit falling edge.
            if (last && !tx) {
                state = RECEIVING;
                bit = 0;
                data = 0;
                t = 0;
                // First sample at 0.5 bit-times (mid-start-bit).  Samples
                // k = 0..9 then land at 0.5 + k bit-times: k=0 validates
                // the start bit, k=1..8 are the 8 data bits (LSB first),
                // k=9 is the stop bit.
                next = TICKS_PER_BIT / 2;
            }
            break;
        case RECEIVING:
            t += TICKS_PER_CYCLE;
            if (t >= next) {
                next += TICKS_PER_BIT;
                // Sample index k happens at (1.5 + k) bit-times.
                if (bit == 0) {
                    // start bit: expected low; nothing to store
                } else if (bit >= 1 && bit <= 8) {
                    data |= (tx ? 1u : 0u) << (bit - 1); // LSB first
                } else { // bit == 9: stop bit
                    if (tx) {
                        uint8_t b = data;
                        ssize_t n = write(STDOUT_FILENO, &b, 1);
                        (void)n;
                    }
                    // Framing error (stop low) → discard silently.
                    state = IDLE;
                }
                bit++;
            }
            break;
        }
        last = tx;
    }
};

// RX: shift bytes from stdin into the design on `rx` (host → design).
struct UartRx {
    enum State { IDLE, SENDING } state = IDLE;
    uint64_t t = 0;    // tick counter
    uint64_t next = 0; // tick count of the next bit change
    int      bit = 0;  // 0 = start, 1..8 = data, 9 = stop
    uint8_t  data = 0;
    bool     pin = true; // value to drive on rx (true = idle)
    bool     trace = false; // set by main()
    std::deque<uint8_t> pending;
    bool     eof = false; // set once stdin reaches end-of-file

    void poll_stdin() {
        uint8_t buf[256];
        for (;;) {
            ssize_t n = read(STDIN_FILENO, buf, sizeof buf);
            if (n == 0) { eof = true; break; } // EOF: no more data ever
            if (n < 0) break;                  // EAGAIN: no data right now
            for (ssize_t i = 0; i < n; i++) pending.push_back(buf[i]);
        }
    }

    // One clock cycle: update the state machine and the pin value.
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
                if (bit == 10) { // start + 8 data + stop all done
                    state = IDLE;
                }
            }
            break;
        }
        if (state == SENDING) {
            if (bit == 0)                  pin = false;                    // start bit
            else if (bit >= 1 && bit <= 8) pin = (data >> (bit - 1)) & 1;  // LSB first
            else                           pin = true;                    // stop bit
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

    // Flag values may be given either as `--flag=value` or `--flag value`
    // (the latter is what `virt run` forwards when invoked as
    // `virt run --flash echo.flash`).
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
            // Re-enable output processing so \n still maps to \r\n.
            raw.c_oflag |= OPOST | ONLCR;
            // Re-enable ICRNL so Enter sends \n instead of raw \r.
            raw.c_iflag |= ICRNL;
            // Keep Ctrl+C working: ISIG generates SIGINT (restores term).
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

    // Optional VCD tracing (requires the model to be built with verilator
    // --trace, which `virt build` always does).
    VerilatedVcdC* tfp = nullptr;
    if (!vcd_path.empty()) {
        tfp = new VerilatedVcdC;
        top->trace(tfp, 99);
        tfp->open(vcd_path.c_str());
    }

    uint64_t cycles = 0;
    const uint64_t stdin_poll_interval = 1000; // ~20.8 µs of simulated time
    int exit_code = 0;
    bool exit_requested = false;

    // Termination: the `exit` port (@Valid(n) exits with code n), the
    // --cycles cap (124), $finish, or Ctrl+C (interactive).  There is no
    // stdin-EOF exit: per the platform contract the simulation runs for
    // as long as `exit` is @Invalid().

    for (;;) {
        // ── Low phase ──
        top->clock = 0;
        top->rx       = uart_rx.pin;
        top->spi_miso = flash.miso_active() ? flash.miso_value() : 0;
        top->eval();
        flash.step(top->spi_cs, top->spi_ck, top->spi_mosi);
        sim_time_ps += 10417;
        if (tfp) tfp->dump(sim_time_ps);

        // ── High phase (posedge: the design samples rx / spi_miso here) ──
        top->clock = 1;
        top->eval();
        flash.step(top->spi_cs, top->spi_ck, top->spi_mosi);
        sim_time_ps += 10417;
        if (tfp) tfp->dump(sim_time_ps);

        // ── Per-cycle emulation ──
        flash.cycle = cycles;
        uart_tx.cycle(top->tx);
        if (cycles % stdin_poll_interval == 0) uart_rx.poll_stdin();
        uart_rx.cycle(); // computes uart_rx.pin for the next cycle

        // ── Exit request (virt::Virt `exit` port) ──
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
