// Lockstep verification harness: m68020_core (Verilator) vs Musashi (68020 mode).
//
//   ./lockstep random [--seed S] [--count N] [--post P] [--show M] [--nowait] [--vcd f.vcd] [-v]
//   ./lockstep prog file.bin [--max K] [--nowait] [--vcd f.vcd] [-v]
//   ./lockstep --musashi-only [--seed S]
//
// See README.md in this directory.
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <memory>

#include "Vtb.h"
#include "verilated.h"
#if VM_TRACE
#include "verilated_vcd_c.h"
#endif

extern "C" {
#include "m68k.h"
}

// ---------------------------------------------------------------- utilities
static inline uint64_t splitmix64(uint64_t x) {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
}

struct Rng {
    uint64_t s;
    explicit Rng(uint64_t seed = 1) : s(seed) {}
    uint32_t u32() { s = splitmix64(s); return (uint32_t)(s >> 16); }
    uint32_t below(uint32_t n) { return u32() % n; }
};

// ---------------------------------------------------------------- memory model
// Sparse byte memory. Unwritten bytes read as a hash of (address, seed), or 0 in zero-fill mode.
struct Mem {
    uint64_t seed = 0;
    bool zero_fill = false;
    std::unordered_map<uint32_t, uint8_t> w;
    void clear(uint64_t s, bool zf) { seed = s; zero_fill = zf; w.clear(); }
    uint8_t rd8(uint32_t a) const {
        auto it = w.find(a);
        if (it != w.end()) return it->second;
        if (zero_fill) return 0;
        return (uint8_t)splitmix64(((uint64_t)a << 8) ^ (seed * 0x100000001b3ULL));
    }
    void wr8(uint32_t a, uint8_t v) { w[a] = v; }
    uint32_t rd(uint32_t a, int n) const {
        uint32_t v = 0;
        for (int i = 0; i < n; i++) v = (v << 8) | rd8(a + i);
        return v;
    }
    void wr(uint32_t a, int n, uint32_t v) {
        for (int i = n - 1; i >= 0; i--) { wr8(a + i, v & 0xff); v >>= 8; }
    }
};

struct WriteEntry { uint32_t addr; int n; uint32_t data; };
struct WriteLog {
    std::vector<WriteEntry> raw;
    std::map<uint32_t, uint8_t> bytes; // last-write-wins byte image
    void clear() { raw.clear(); bytes.clear(); }
    void add(uint32_t a, int n, uint32_t v) {
        raw.push_back({a, n, v});
        for (int i = n - 1; i >= 0; i--) { bytes[a + i] = v & 0xff; v >>= 8; }
    }
};

static Mem mus_mem, rtl_mem;
static WriteLog mus_log, rtl_log;

// ---------------------------------------------------------------- Musashi glue
static unsigned long mus_instr_count = 0;
extern "C" {
void lockstep_instr_hook(unsigned int) { mus_instr_count++; }
unsigned int m68k_read_memory_8(unsigned int a) { return mus_mem.rd(a, 1); }
unsigned int m68k_read_memory_16(unsigned int a) { return mus_mem.rd(a, 2); }
unsigned int m68k_read_memory_32(unsigned int a) { return mus_mem.rd(a, 4); }
unsigned int m68k_read_disassembler_8(unsigned int a) { return mus_mem.rd(a, 1); }
unsigned int m68k_read_disassembler_16(unsigned int a) { return mus_mem.rd(a, 2); }
unsigned int m68k_read_disassembler_32(unsigned int a) { return mus_mem.rd(a, 4); }
// prog-mode test hook: a byte write to IRQ_PORT sets the IPL level seen by both models
static const uint32_t IRQ_PORT = 0x00DFF0F0;
static int irq_level_req = -1;          // set by either model's write, applied by the harness
void m68k_write_memory_8(unsigned int a, unsigned int v) {
    if (a == IRQ_PORT) { m68k_set_irq(v & 7); }
    mus_mem.wr(a, 1, v); mus_log.add(a, 1, v & 0xff);
}
void m68k_write_memory_16(unsigned int a, unsigned int v) { mus_mem.wr(a, 2, v); mus_log.add(a, 2, v & 0xffff); }
void m68k_write_memory_32(unsigned int a, unsigned int v) { mus_mem.wr(a, 4, v); mus_log.add(a, 4, v); }
}

struct CpuState {
    uint32_t d[8], a[8], pc, usp, isp, msp, vbr;
    uint16_t sr;
};

static CpuState mus_state() {
    CpuState s;
    for (int i = 0; i < 8; i++) s.d[i] = m68k_get_reg(NULL, (m68k_register_t)(M68K_REG_D0 + i));
    for (int i = 0; i < 8; i++) s.a[i] = m68k_get_reg(NULL, (m68k_register_t)(M68K_REG_A0 + i));
    s.pc = m68k_get_reg(NULL, M68K_REG_PC);
    s.sr = m68k_get_reg(NULL, M68K_REG_SR);
    s.usp = m68k_get_reg(NULL, M68K_REG_USP);
    s.isp = m68k_get_reg(NULL, M68K_REG_ISP);
    s.msp = m68k_get_reg(NULL, M68K_REG_MSP);
    s.vbr = m68k_get_reg(NULL, M68K_REG_VBR);
    return s;
}

// Execute exactly one instruction (incl. exception processing). Returns false if Musashi made no progress
// (e.g. stopped).
static bool mus_step() {
    unsigned long before = mus_instr_count;
    for (int tries = 0; tries < 4 && mus_instr_count == before; tries++) m68k_execute(1);
    return mus_instr_count == before + 1;
}

static std::string disasm(uint32_t pc) {
    char buf[256];
    m68k_disassemble(buf, pc, M68K_CPU_TYPE_68020);
    return buf;
}

// mnemonic class: first token, size suffix stripped, lower-case
static std::string mclass(const std::string &dis) {
    if (dis.find("ILLEGAL") != std::string::npos) return "illegal";
    if (dis.find("opcode 1010") != std::string::npos) return "linea";
    if (dis.find("opcode 1111") != std::string::npos) return "linef";
    std::string m = dis.substr(0, dis.find_first_of(" \t"));
    size_t dot = m.find('.');
    if (dot != std::string::npos && m.rfind("dc", 0) != 0) m = m.substr(0, dot);
    for (auto &c : m) c = (char)tolower(c);
    if (m.rfind("dc", 0) == 0) m = "invalid";
    return m;
}

// ---------------------------------------------------------------- RTL driver
struct Rtl {
    std::unique_ptr<VerilatedContext> ctx;
    std::unique_ptr<Vtb> t;
#if VM_TRACE
    VerilatedVcdC *vcd = nullptr;
#endif
    Rng rng{1};
    bool waits = true;
    int wait_left = -1;
    uint64_t cycles = 0;
    // protocol check: values latched at start of a transfer
    bool in_xfer = false;
    uint32_t x_addr = 0, x_wdata = 0; uint8_t x_size = 0, x_we = 0, x_fc = 0;
    std::vector<std::string> proto_errs;

    Rtl() {
        ctx.reset(new VerilatedContext);
        t.reset(new Vtb(ctx.get()));
        t->clk = 0; t->rst_n = 0; t->bus_ack = 0; t->bus_berr = 0; t->bus_avec = 0; t->bus_rdata = 0; t->ipl = 0;
        t->eval();
    }
    void open_vcd(const char *fn) {
#if VM_TRACE
        Verilated::traceEverOn(true);
        vcd = new VerilatedVcdC;
        t->trace(vcd, 99);
        vcd->open(fn);
#else
        (void)fn;
        fprintf(stderr, "VCD requested but built without TRACE=1\n");
#endif
    }
    void dump() {
#if VM_TRACE
        if (vcd) vcd->dump(ctx->time());
#endif
        ctx->timeInc(1);
    }
    void tick() {
        t->clk = 0;
        t->eval();
        dump();
        // ---- bus slave (decides on the low phase, from settled outputs) ----
        t->bus_ack = 0; t->bus_berr = 0; t->bus_avec = 0;
        if (t->rst_n && t->bus_req) {
            if (!in_xfer) {
                in_xfer = true;
                x_addr = t->bus_addr; x_size = t->bus_size; x_we = t->bus_we; x_fc = t->bus_fc; x_wdata = t->bus_wdata;
                wait_left = waits ? (int)rng.below(4) : 0;
            } else if (t->bus_addr != x_addr || t->bus_size != x_size || t->bus_we != x_we || t->bus_fc != x_fc ||
                       (x_we && t->bus_wdata != x_wdata)) {
                char b[160];
                snprintf(b, sizeof b, "cycle %llu: bus outputs changed while transfer pending (addr %08x->%08x)",
                         (unsigned long long)cycles, x_addr, (unsigned)t->bus_addr);
                proto_errs.push_back(b);
            }
            if (wait_left == 0) {
                int n = x_size == 0 ? 1 : x_size == 1 ? 2 : x_size == 2 ? 4 : 0;
                if (n == 0) {
                    proto_errs.push_back("bus_size == 3");
                    n = 4;
                }
                if (x_fc == 7 && !x_we) {               // CPU space: interrupt ack / bkpt ack -> autovector
                    t->bus_avec = 1;
                    t->bus_rdata = 0xffffffff;
                } else if (x_we) {
                    uint32_t v = n == 4 ? x_wdata : (x_wdata & ((1u << (8 * n)) - 1));
                    rtl_mem.wr(x_addr, n, v);
                    if (x_addr == IRQ_PORT && n == 1) t->ipl = v & 7;
                    rtl_log.add(x_addr, n, v);
                } else {
                    t->bus_rdata = rtl_mem.rd(x_addr, n);
                }
                t->bus_ack = 1;
                in_xfer = false;
                wait_left = -1;
            } else {
                wait_left--;
            }
        } else if (in_xfer) {
            char b[120];
            snprintf(b, sizeof b, "cycle %llu: bus_req dropped before ack (addr %08x)", (unsigned long long)cycles, x_addr);
            proto_errs.push_back(b);
            in_xfer = false;
        }
        t->eval();
        t->clk = 1;
        t->eval();
        dump();
        cycles++;
    }
    void reset() {
        t->rst_n = 0;
        in_xfer = false;
        for (int i = 0; i < 8; i++) tick();
        t->rst_n = 1;
    }
    // run until dbg_retire; returns false on timeout/halt
    bool step(uint64_t max_cycles, std::string &why) {
        for (uint64_t i = 0; i < max_cycles; i++) {
            tick();
            if (t->dbg_retire) return true;
            if (t->halted) { why = "core halted (double bus fault)"; return false; }
        }
        why = "timeout waiting for dbg_retire";
        return false;
    }
    CpuState state() {
        CpuState s;
        for (int i = 0; i < 8; i++) { s.d[i] = t->dbg_d[i]; s.a[i] = t->dbg_a[i]; }
        s.pc = t->dbg_pc; s.sr = t->dbg_sr; s.usp = t->dbg_usp; s.isp = t->dbg_isp; s.msp = t->dbg_msp; s.vbr = t->dbg_vbr;
        return s;
    }
    ~Rtl() {
#if VM_TRACE
        if (vcd) { vcd->close(); delete vcd; }
#endif
        t->final();
    }
};

// ---------------------------------------------------------------- comparison
static const uint16_t SR_IMPL = 0xF71F; // T1 T0 S M . I2 I1 I0 . . . X N Z V C
enum { CC_C = 1, CC_V = 2, CC_Z = 4, CC_N = 8, CC_X = 16 };

static const uint32_t HANDLER_BASE = 0x00F00000; // handler for vector v at HANDLER_BASE + v*16

static int handler_vector(uint32_t pc) {
    if (pc >= HANDLER_BASE && pc < HANDLER_BASE + 256 * 16 && (pc & 15) == 0) return (pc - HANDLER_BASE) / 16;
    return -1;
}

struct FlagMask {
    uint8_t live = 0;     // CCR bits to ignore in the live SR
    uint8_t stacked = 0;  // CCR bits to ignore in a stacked SR (exception frame), if an exception was taken
};

static FlagMask undefined_flags(const std::string &cls, const CpuState &m, bool exception) {
    FlagMask f;
    if (cls.rfind("div", 0) == 0) {
        if (exception) { f.live = f.stacked = CC_N | CC_Z | CC_V; }   // divide by zero
        else if (m.sr & CC_V) f.live = CC_N | CC_Z;                      // overflow
    } else if (cls == "abcd" || cls == "sbcd" || cls == "nbcd") {
        f.live = CC_N | CC_V;
    } else if (cls == "chk") {
        if (exception) f.live = f.stacked = CC_Z | CC_V | CC_C;
        else f.live = CC_N | CC_Z | CC_V | CC_C;
    } else if (cls == "chk2" || cls == "cmp2") {
        f.live = f.stacked = CC_N | CC_V;
    }
    return f;
}

static std::string compare(const CpuState &m, const CpuState &r, const FlagMask &fm, bool exception) {
    std::string out;
    char b[200];
    for (int i = 0; i < 8; i++)
        if (m.d[i] != r.d[i]) { snprintf(b, sizeof b, "  D%d: musashi %08x rtl %08x\n", i, m.d[i], r.d[i]); out += b; }
    for (int i = 0; i < 8; i++)
        if (m.a[i] != r.a[i]) { snprintf(b, sizeof b, "  A%d: musashi %08x rtl %08x\n", i, m.a[i], r.a[i]); out += b; }
    if (m.pc != r.pc) { snprintf(b, sizeof b, "  PC: musashi %08x rtl %08x\n", m.pc, r.pc); out += b; }
    uint16_t srm = SR_IMPL & ~(uint16_t)fm.live;
    if ((m.sr & srm) != (r.sr & srm)) { snprintf(b, sizeof b, "  SR: musashi %04x rtl %04x (mask %04x)\n", m.sr, r.sr, srm); out += b; }
    if (m.usp != r.usp) { snprintf(b, sizeof b, "  USP: musashi %08x rtl %08x\n", m.usp, r.usp); out += b; }
    if (m.isp != r.isp) { snprintf(b, sizeof b, "  ISP: musashi %08x rtl %08x\n", m.isp, r.isp); out += b; }
    if (m.msp != r.msp) { snprintf(b, sizeof b, "  MSP: musashi %08x rtl %08x\n", m.msp, r.msp); out += b; }
    if (m.vbr != r.vbr) { snprintf(b, sizeof b, "  VBR: musashi %08x rtl %08x\n", m.vbr, r.vbr); out += b; }
    // writes (byte images). Stacked-SR low byte lives at new A7+1 when an exception was taken.
    uint32_t sr_lo = m.a[7] + 1;
    std::map<uint32_t, uint8_t> all;
    for (auto &kv : mus_log.bytes) all[kv.first] = 1;
    for (auto &kv : rtl_log.bytes) all[kv.first] = 1;
    int shown = 0;
    for (auto &kv : all) {
        uint32_t a = kv.first;
        auto mi = mus_log.bytes.find(a), ri = rtl_log.bytes.find(a);
        bool mh = mi != mus_log.bytes.end(), rh = ri != rtl_log.bytes.end();
        uint8_t mask = 0xff;
        if (exception && a == sr_lo) mask &= ~fm.stacked;
        if (mh && rh && ((mi->second ^ ri->second) & mask) == 0) continue;
        if (shown++ < 12) {
            char ms[8] = "--", rs[8] = "--";
            if (mh) snprintf(ms, sizeof ms, "%02x", mi->second);
            if (rh) snprintf(rs, sizeof rs, "%02x", ri->second);
            snprintf(b, sizeof b, "  MEM[%08x]: musashi %s rtl %s\n", a, ms, rs);
            out += b;
        }
    }
    if (shown > 12) { snprintf(b, sizeof b, "  ... %d differing bytes in write log\n", shown); out += b; }
    return out;
}

static std::string fmt_state(const CpuState &s) {
    char b[600];
    snprintf(b, sizeof b,
             "  D: %08x %08x %08x %08x %08x %08x %08x %08x\n"
             "  A: %08x %08x %08x %08x %08x %08x %08x %08x\n"
             "  PC %08x SR %04x USP %08x ISP %08x MSP %08x VBR %08x\n",
             s.d[0], s.d[1], s.d[2], s.d[3], s.d[4], s.d[5], s.d[6], s.d[7], s.a[0], s.a[1], s.a[2], s.a[3], s.a[4],
             s.a[5], s.a[6], s.a[7], s.pc, s.sr, s.usp, s.isp, s.msp, s.vbr);
    return b;
}

static std::string fmt_writes(const char *who, const WriteLog &l) {
    std::string o;
    char b[80];
    for (auto &e : l.raw) {
        snprintf(b, sizeof b, "    %s W%d [%08x] = %0*x\n", who, e.n, e.addr, e.n * 2, e.data);
        o += b;
    }
    return o;
}

// ---------------------------------------------------------------- test images
static void put16(std::vector<uint16_t> &v, uint16_t w) { v.push_back(w); }
static void put32(std::vector<uint16_t> &v, uint32_t l) { v.push_back(l >> 16); v.push_back(l & 0xffff); }

static const uint32_t PREAMBLE_ADDR = 0x400;
static const uint32_t RESET_SSP = 0x0007fff0;

struct RandomTest {
    uint32_t d[8], a[7], usp, isp, msp, pc;
    uint16_t sr;
    uint16_t code[11];
};

// preamble: loads all registers then JMPs to the test PC. Returns instruction count.
static int build_preamble(const RandomTest &t, std::vector<uint16_t> &p) {
    int n = 0;
    put16(p, 0x207C); put32(p, t.usp); put16(p, 0x4E60); n += 2;                    // movea.l #usp,a0 ; move a0,usp
    put16(p, 0x207C); put32(p, t.msp); put16(p, 0x4E7B); put16(p, 0x8803); n += 2;  // movec a0,msp
    put16(p, 0x207C); put32(p, t.isp); put16(p, 0x4E7B); put16(p, 0x8804); n += 2;  // movec a0,isp
    for (int i = 0; i < 8; i++) { put16(p, 0x203C | (i << 9)); put32(p, t.d[i]); n++; }   // move.l #,dn
    for (int i = 0; i < 7; i++) { put16(p, 0x207C | (i << 9)); put32(p, t.a[i]); n++; }   // movea.l #,an
    put16(p, 0x46FC); put16(p, t.sr); n++;                                            // move #sr,sr
    put16(p, 0x4EF9); put32(p, t.pc); n++;                                            // jmp (pc).l
    return n;
}

static void load_both(uint32_t addr, const std::vector<uint16_t> &w) {
    for (size_t i = 0; i < w.size(); i++) { mus_mem.wr(addr + 2 * i, 2, w[i]); rtl_mem.wr(addr + 2 * i, 2, w[i]); }
}
static void load32_both(uint32_t addr, uint32_t v) { mus_mem.wr(addr, 4, v); rtl_mem.wr(addr, 4, v); }

static RandomTest gen_test(Rng &r) {
    RandomTest t;
    auto addrish = [&](void) -> uint32_t {
        switch (r.below(4)) {
            case 0: return r.u32();
            case 1: return r.u32() & 0x00fffffe;
            case 2: return 0x00100000 + (r.u32() & 0xffff);
            default: return r.u32() & 0x7fffffff;
        }
    };
    for (int i = 0; i < 8; i++) {
        switch (r.below(5)) {
            case 0: t.d[i] = r.below(64); break;                          // small (shift counts, bit numbers)
            case 1: t.d[i] = (uint32_t)(int32_t)(int8_t)r.u32(); break;   // sign-extended byte
            default: t.d[i] = r.u32(); break;
        }
    }
    for (int i = 0; i < 7; i++) t.a[i] = addrish();
    t.usp = addrish() & ~1u;
    t.isp = (0x00200000 + (r.u32() & 0x3fff0)) & ~1u;
    t.msp = (0x00300000 + (r.u32() & 0x3fff0)) & ~1u;
    t.sr = 0x2700 | (r.below(2) << 12) | (r.u32() & 0x1f);
    t.pc = (0x00002000 + (r.u32() % (0x00E00000 - 0x2000))) & ~1u;
    for (int i = 0; i < 11; i++) t.code[i] = r.u32() & 0xffff;
    return t;
}

static void setup_random_memory(uint64_t seed, const RandomTest &t, int &preamble_n) {
    mus_mem.clear(seed, false);
    rtl_mem.clear(seed, false);
    load32_both(0, RESET_SSP);
    load32_both(4, PREAMBLE_ADDR);
    for (int v = 2; v < 256; v++) {
        load32_both(v * 4, HANDLER_BASE + v * 16);
        std::vector<uint16_t> h = {0x4E71, 0x4E71, 0x4E71, 0x4E71};
        load_both(HANDLER_BASE + v * 16, h);
    }
    std::vector<uint16_t> pre;
    preamble_n = build_preamble(t, pre);
    load_both(PREAMBLE_ADDR, pre);
    std::vector<uint16_t> code(t.code, t.code + 11);
    load_both(t.pc, code);
}

// ---------------------------------------------------------------- options
struct Opts {
    std::string mode;
    std::string file;
    uint64_t seed = 1;
    long count = 1000;
    long max_instr = 100000;
    int post = 1;
    int show = 20;
    bool waits = true;
    bool verbose = false;
    std::string vcd;
    uint64_t timeout = 20000;
};

static void usage() {
    fprintf(stderr,
            "usage:\n"
            "  lockstep random [--seed S] [--count N] [--post P] [--show M] [--nowait] [--timeout C] [--vcd f] [-v]\n"
            "  lockstep prog file.bin [--max K] [--nowait] [--timeout C] [--vcd f] [-v]\n"
            "  lockstep --musashi-only [--seed S]\n");
    exit(2);
}


// reset Musashi and zero the registers that the 68020 does not define at reset,
// so both models start identical (the RTL zeroes them in its reset too)
static void mus_reset() {
    m68k_pulse_reset();
    for (int r = M68K_REG_D0; r <= M68K_REG_A6; r++) m68k_set_reg((m68k_register_t)r, 0);
    m68k_set_reg(M68K_REG_USP, 0);
    m68k_set_reg(M68K_REG_MSP, 0);
}

// ---------------------------------------------------------------- musashi self-check
static int musashi_only(const Opts &o) {
    Rng r(o.seed);
    RandomTest t = gen_test(r);
    t.code[0] = 0xD081;  // add.l d1,d0
    int pre_n;
    setup_random_memory(o.seed, t, pre_n);
    mus_reset();
    int fails = 0;
    for (int i = 0; i < pre_n; i++) {
        uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
        std::string d = disasm(pc);
        if (!mus_step()) { printf("musashi did not step at %08x\n", pc); return 1; }
        if (o.verbose) printf("%08x  %s\n", pc, d.c_str());
    }
    CpuState s = mus_state();
    for (int i = 0; i < 8; i++) if (s.d[i] != t.d[i]) { printf("D%d wrong\n", i); fails++; }
    for (int i = 0; i < 7; i++) if (s.a[i] != t.a[i]) { printf("A%d wrong\n", i); fails++; }
    if (s.pc != t.pc) { printf("PC wrong %08x\n", s.pc); fails++; }
    if ((s.sr & SR_IMPL) != t.sr) { printf("SR wrong %04x vs %04x\n", s.sr, t.sr); fails++; }
    if (s.usp != t.usp) { printf("USP wrong\n"); fails++; }
    if (s.isp != t.isp) { printf("ISP wrong %08x vs %08x\n", s.isp, t.isp); fails++; }
    if (s.msp != t.msp) { printf("MSP wrong %08x vs %08x\n", s.msp, t.msp); fails++; }
    if (s.a[7] != ((t.sr & 0x1000) ? t.msp : t.isp)) { printf("A7 not active SP\n"); fails++; }
    printf("after preamble (%d instructions):\n%s", pre_n, fmt_state(s).c_str());
    printf("%08x  %s\n", t.pc, disasm(t.pc).c_str());
    mus_step();
    CpuState s2 = mus_state();
    if (s2.d[0] != t.d[0] + t.d[1]) { printf("add.l d1,d0 wrong\n"); fails++; }
    // exception check: illegal instruction -> vector 4 handler, format 0 frame
    mus_mem.wr(s2.pc, 2, 0x4AFC);
    uint32_t sp_before = s2.a[7];
    mus_log.clear();
    mus_step();
    CpuState s3 = mus_state();
    if (handler_vector(s3.pc) != 4) { printf("ILLEGAL did not reach vector 4 (pc %08x)\n", s3.pc); fails++; }
    if (s3.a[7] != sp_before - 8) { printf("ILLEGAL frame size wrong\n"); fails++; }
    printf("%s", fmt_writes("musashi", mus_log).c_str());
    printf("musashi self-check: %s\n", fails ? "FAIL" : "PASS");
    return fails ? 1 : 0;
}

// ---------------------------------------------------------------- random mode
struct ClassStat { long run = 0, fail = 0; };

static int run_random(const Opts &o) {
    Rtl rtl;
    rtl.waits = o.waits;
    if (!o.vcd.empty()) rtl.open_vcd(o.vcd.c_str());
    std::map<std::string, ClassStat> stats;
    std::map<std::string, long> skips;
    long shown = 0, total = 0, failed = 0;

    for (long ti = 0; ti < o.count; ti++) {
        uint64_t tseed = splitmix64(o.seed * 1000003ULL + ti);
        Rng r(tseed);
        RandomTest t = gen_test(r);
        int pre_n;
        setup_random_memory(tseed, t, pre_n);
        std::string test_dis = disasm(t.pc);
        std::string test_cls = mclass(test_dis);
        static const char *skip_cls[] = {"stop", "reset", "bkpt", "callm", "rtm", nullptr};
        bool skip = false;
        for (int k = 0; skip_cls[k]; k++) if (test_cls == skip_cls[k]) skip = true;
        if (skip) { skips[test_cls]++; continue; }

        rtl.rng = Rng(tseed ^ 0x5555);
        rtl.proto_errs.clear();
        mus_reset();
        rtl.reset();

        int n_total = pre_n + 1 + o.post;
        bool test_failed = false, preamble_fail = false;
        std::string report;
        for (int k = 0; k < n_total; k++) {
            CpuState pre = mus_state();
            std::string dis = disasm(pre.pc);
            std::string cls = mclass(dis);
            bool is_test = (k == pre_n);
            if (k > pre_n) {
                bool sk = false;
                for (int q = 0; skip_cls[q]; q++) if (cls == skip_cls[q]) sk = true;
                if (sk) break;
            }
            mus_log.clear();
            rtl_log.clear();
            bool mok = mus_step();
            CpuState m = mus_state();
            if (!mok) { if (is_test) skips["musashi-no-step"]++; break; }
            if (m.pc & 1) { skips["odd-pc (address error not modelled by musashi)"]++; break; }
            std::string why;
            bool rok = rtl.step(o.timeout, why);
            bool exception = handler_vector(m.pc) >= 0 && handler_vector(pre.pc) < 0;
            std::string diff;
            CpuState rs{};
            if (!rok) diff = "  " + why + "\n";
            else {
                rs = rtl.state();
                FlagMask fm = undefined_flags(cls, m, exception);
                if (k < pre_n - 1) fm.live |= 0x1F;  // CCR is undefined after reset until MOVE #sr,SR
                diff = compare(m, rs, fm, exception);
            }
            for (auto &e : rtl.proto_errs) diff += "  protocol: " + e + "\n";
            rtl.proto_errs.clear();
            if (!diff.empty()) {
                test_failed = true;
                char hdr[400];
                const char *phase = k < pre_n ? "preamble" : is_test ? "TEST" : "post";
                snprintf(hdr, sizeof hdr, "test %ld (seed %llu) %s instr #%d @%08x: %s  [ops %04x %04x %04x]\n", ti,
                         (unsigned long long)tseed, phase, k, pre.pc, dis.c_str(), mus_mem.rd(pre.pc, 2),
                         mus_mem.rd(pre.pc + 2, 2), mus_mem.rd(pre.pc + 4, 2));
                report = hdr;
                if (exception) {
                    char b[64];
                    snprintf(b, sizeof b, "  (musashi took exception vector %d)\n", handler_vector(m.pc));
                    report += b;
                }
                report += diff;
                report += " before:\n" + fmt_state(pre);
                report += " musashi after:\n" + fmt_state(m);
                if (rok) report += " rtl after:\n" + fmt_state(rs);
                report += fmt_writes("musashi", mus_log) + fmt_writes("rtl    ", rtl_log);
                if (k < pre_n) { stats["(preamble)"].run++; stats["(preamble)"].fail++; preamble_fail = true; }
                break;
            }
            if (o.verbose && is_test) printf("test %ld: %08x %s ok\n", ti, pre.pc, dis.c_str());
            if (!rok) break;
        }
        total++;
        if (!preamble_fail) stats[test_cls].run++;
        if (test_failed) {
            failed++;
            if (!preamble_fail) stats[test_cls].fail++;
            if (shown++ < o.show) printf("---- MISMATCH (class %s)\n%s", test_cls.c_str(), report.c_str());
        }
    }
    printf("\n==== summary: %ld tests, %ld passed, %ld failed\n", total, total - failed, failed);
    std::vector<std::pair<std::string, ClassStat>> v(stats.begin(), stats.end());
    std::sort(v.begin(), v.end(), [](auto &a, auto &b) { return a.second.fail > b.second.fail; });
    printf("%-14s %8s %8s\n", "class", "run", "fail");
    for (auto &kv : v) printf("%-14s %8ld %8ld%s\n", kv.first.c_str(), kv.second.run, kv.second.fail, kv.second.fail ? "  <--" : "");
    if (!skips.empty()) {
        printf("skipped:\n");
        for (auto &kv : skips) printf("  %-50s %ld\n", kv.first.c_str(), kv.second);
    }
    return failed ? 1 : 0;
}

// ---------------------------------------------------------------- prog mode
static int run_prog(const Opts &o, bool mus_only = false) {
    FILE *f = fopen(o.file.c_str(), "rb");
    if (!f) { perror(o.file.c_str()); return 2; }
    std::vector<uint8_t> img;
    int c;
    while ((c = fgetc(f)) != EOF) img.push_back((uint8_t)c);
    fclose(f);
    mus_mem.clear(0, true);
    rtl_mem.clear(0, true);
    for (size_t i = 0; i < img.size(); i++) { mus_mem.wr8(i, img[i]); rtl_mem.wr8(i, img[i]); }

    Rtl rtl;
    rtl.waits = o.waits;
    rtl.rng = Rng(o.seed);
    if (!o.vcd.empty()) rtl.open_vcd(o.vcd.c_str());
    mus_reset();
    if (!mus_only) rtl.reset();
    for (long k = 0; k < o.max_instr; k++) {
        CpuState pre = mus_state();
        uint16_t op = mus_mem.rd(pre.pc, 2);
        std::string dis = disasm(pre.pc);
        if (op == 0x4E72 && mus_only) {
            printf("musashi-only %s: %ld instructions, reached STOP at %08x\n%s", o.file.c_str(), k, pre.pc,
                   fmt_state(pre).c_str());
            return 0;
        }
        if (mus_only) {
            if (o.verbose) printf("%6ld %08x  %s\n", k, pre.pc, dis.c_str());
            if (!mus_step()) { printf("musashi stopped at %08x\n", pre.pc); return 1; }
            continue;
        }
        if (op == 0x4E72) {  // STOP = end-of-test marker
            printf("PASS %s: %ld instructions in lockstep, %llu RTL cycles, reached STOP at %08x (D0=%08x)\n",
                   o.file.c_str(), k, (unsigned long long)rtl.cycles, pre.pc, pre.d[0]);
            return 0;
        }
        mus_log.clear();
        rtl_log.clear();
        if (!mus_step()) { printf("musashi stopped at %08x\n", pre.pc); return 1; }
        CpuState m = mus_state();
        std::string why;
        uint64_t c0 = rtl.cycles;
        bool rok = rtl.step(o.timeout, why);
        uint64_t dc = rtl.cycles - c0;
        FlagMask fm = undefined_flags(mclass(dis), m, false);
        if (k == 0) fm.live |= 0x1F;  // CCR undefined after reset: tests must start with move #xxxx,sr
        std::string diff = rok ? compare(m, rtl.state(), fm, false) : "  " + why + "\n";
        for (auto &e : rtl.proto_errs) diff += "  protocol: " + e + "\n";
        if (o.verbose) printf("%6ld %08x %4llu  %s\n", k, pre.pc, (unsigned long long)dc, dis.c_str());
        if (!diff.empty()) {
            printf("FAIL %s at instruction %ld @%08x: %s\n%s before:\n%s musashi after:\n%s", o.file.c_str(), k, pre.pc,
                   dis.c_str(), diff.c_str(), fmt_state(pre).c_str(), fmt_state(m).c_str());
            if (rok) printf(" rtl after:\n%s", fmt_state(rtl.state()).c_str());
            printf("%s%s", fmt_writes("musashi", mus_log).c_str(), fmt_writes("rtl    ", rtl_log).c_str());
            return 1;
        }
    }
    printf("PASS %s: %ld instructions (limit reached)\n", o.file.c_str(), o.max_instr);
    return 0;
}

int main(int argc, char **argv) {
    Opts o;
    bool mus_only = false;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() -> const char * { if (i + 1 >= argc) usage(); return argv[++i]; };
        if (a == "random" || a == "prog") o.mode = a;
        else if (a == "--musashi-only") mus_only = true;
        else if (a == "--seed") o.seed = strtoull(next(), 0, 0);
        else if (a == "--count") o.count = strtol(next(), 0, 0);
        else if (a == "--max") o.max_instr = strtol(next(), 0, 0);
        else if (a == "--post") o.post = atoi(next());
        else if (a == "--show") o.show = atoi(next());
        else if (a == "--timeout") o.timeout = strtoull(next(), 0, 0);
        else if (a == "--nowait") o.waits = false;
        else if (a == "--vcd") o.vcd = next();
        else if (a == "-v") o.verbose = true;
        else if (o.mode == "prog" && o.file.empty() && a[0] != '-') o.file = a;
        else usage();
    }
    m68k_init();
    m68k_set_cpu_type(M68K_CPU_TYPE_68020);
    if (mus_only && o.mode == "prog" && !o.file.empty()) return run_prog(o, true);
    if (mus_only) return musashi_only(o);
    if (o.mode == "random") return run_random(o);
    if (o.mode == "prog" && !o.file.empty()) return run_prog(o);
    usage();
}
