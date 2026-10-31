/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026 Gerold Bausch, Eric Fischer */
/* crc_bench.c - Hardware vs. software CRC on the STM32L432KC
 *
 * Three algorithms ("suites"):
 *   CRC-32        Ethernet/zlib: poly 0x04C11DB7, init FFFFFFFF, reflected, XorOut FFFFFFFF,
 *                 check value("123456789") = 0xCBF43926
 *   CRC-16/MODBUS poly 0x8005, init FFFF, reflected, XorOut 0, check value 0x4B37
 *   CRC-8         poly 0x07 (SMBus PEC), init 00, not reflected, XorOut 0, check value 0xF4
 *
 * Variants per suite:
 *   bitwise   software, bit-by-bit loop
 *   table     software, byte table (CRC-32: 1 KB, CRC-16: 512 B, CRC-8: 256 B, in RAM)
 *   slice4    software, slicing-by-4 (CRC-32: 4 KB, CRC-16: 2 KB, CRC-8: 1 KB RAM)
 *   hw_cpu    CRC peripheral, CPU writes word by word into CRC->DR
 *   hw_x4     like hw_cpu, loop unrolled 4x
 *   hw_dma    CRC peripheral, DMA1 channel 1 (memory-to-memory)
 *   hal       (CRC-32 only, optional -DBENCH_WITH_HAL) HAL_CRC_Calculate()
 *
 * Note on CRC-8 (not reflected): The hardware processes a 32-bit word MSB first, but a
 * little-endian word holds the first byte in its lowest byte. Without input bit reversal the
 * CPU therefore has to swap every word with __REV() (hw_cpu/hw_x4), and DMA can only use
 * byte transfers (one byte per transfer, hw_dma).
 *
 * Timing uses the DWT cycle counter, per call including the reset of the CRC unit. For each size
 * and variant the minimum of BENCH_REPS runs counts (warm cache); the measurement overhead is
 * subtracted. CRC polynomial, init value and polynomial size are set once before each suite and
 * are not part of the measured time.
 *
 * Build options:
 *   -DBENCH_RETARGET_UART2  printf over huart2 (HAL, Nucleo VCP: PA2/PA15) via _write()
 *   -DBENCH_WITH_HAL        additionally measure HAL_CRC_Calculate() (extern CRC_HandleTypeDef hcrc)
 *
 * Compiler optimization: -O2 recommended. With -O0 the software numbers are meaningless.
 */
#include "crc_bench.h"
#include "crc_sw.h"

#if defined(BENCH_WITH_HAL) || defined(BENCH_RETARGET_UART2)
#include "stm32l4xx_hal.h"
#else
#include "stm32l4xx.h"
#endif

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

#define BENCH_REPS      16
#define BENCH_MAX_LEN   4096u
#define NA              0xFFFFFFFFu     /* "not applicable" in the result table */

/* CRC->CR Bits (RM0394): RESET [0], POLYSIZE [4:3], REV_IN [6:5], REV_OUT [7] */
#define CR_RESET         (1u << 0)
#define CR_PS32          (0u << 3)
#define CR_PS16          (1u << 3)
#define CR_PS8           (2u << 3)
#define CR_REV_IN_BYTE   (1u << 5)
#define CR_REV_IN_WORD   (3u << 5)
#define CR_REV_OUT       (1u << 7)

/* *_W: configuration for word accesses, *_T: for the tail bytes (n % 4) */
#define CFG32_W   (CR_PS32 | CR_REV_IN_WORD | CR_REV_OUT)
#define CFG32_T   (CR_PS32 | CR_REV_IN_BYTE | CR_REV_OUT)
#define CFG16_W   (CR_PS16 | CR_REV_IN_WORD | CR_REV_OUT)
#define CFG16_T   (CR_PS16 | CR_REV_IN_BYTE | CR_REV_OUT)
#define CFG8_W    (CR_PS8)
#define CFG8_T    (CR_PS8)

/* ------------------------------------------------------------------ */
/* Output                                                              */
/* ------------------------------------------------------------------ */
#ifdef BENCH_RETARGET_UART2
extern UART_HandleTypeDef huart2;
int _write(int fd, char *p, int len)
{
    (void)fd;
    HAL_UART_Transmit(&huart2, (uint8_t *)p, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}
#endif

/* ------------------------------------------------------------------ */
/* Hardware CRC                                                       */
/* ------------------------------------------------------------------ */
static void hw_enable_clocks(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN | RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR;                 /* read-back barrier after clock enable */
}

/* Set the polynomial size first, then polynomial and init, then RESET (loads INIT into DR). */
static void hw_setup(uint32_t cfg, uint32_t pol, uint32_t init)
{
    CRC->CR   = cfg | CR_RESET;
    CRC->POL  = pol;
    CRC->INIT = init;
    CRC->CR   = cfg | CR_RESET;
}

#define NOSWAP(v)  (v)
#define BSWAP(v)   __REV(v)
#define POST32(x)  (~(x))
#define POST16(x)  ((x) & 0xFFFFu)
#define POST8(x)   ((x) & 0xFFu)

/* CPU writes words (d must be 4-byte aligned), tail bytes one by one. */
#define HW_CPU(NAME, CFGW, CFGT, SWAP, POST)                                   \
static uint32_t NAME(const uint8_t *d, size_t n)                               \
{                                                                              \
    CRC->CR = (CFGW) | CR_RESET;                                               \
    const uint32_t *w = (const uint32_t *)d;                                   \
    size_t words = n >> 2;                                                     \
    while (words--) { uint32_t v = *w++; CRC->DR = SWAP(v); }                  \
    size_t tail = n & 3u;                                                      \
    if (tail) {                                                                \
        const uint8_t *b = (const uint8_t *)w;                                 \
        CRC->CR = (CFGT);              /* no RESET: the running value is kept */ \
        while (tail--) { *(volatile uint8_t *)&CRC->DR = *b++; }               \
    }                                                                          \
    return POST(CRC->DR);                                                      \
}

/* like HW_CPU, 4 words per loop iteration */
#define HW_X4(NAME, CFGW, CFGT, SWAP, POST)                                    \
static uint32_t NAME(const uint8_t *d, size_t n)                               \
{                                                                              \
    CRC->CR = (CFGW) | CR_RESET;                                               \
    const uint32_t *w = (const uint32_t *)d;                                   \
    size_t words = n >> 2;                                                     \
    while (words >= 4) {                                                       \
        uint32_t a = w[0], b = w[1], c = w[2], e = w[3];                       \
        CRC->DR = SWAP(a);                                                     \
        CRC->DR = SWAP(b);                                                     \
        CRC->DR = SWAP(c);                                                     \
        CRC->DR = SWAP(e);                                                     \
        w += 4;                                                                \
        words -= 4;                                                            \
    }                                                                          \
    while (words--) { uint32_t v = *w++; CRC->DR = SWAP(v); }                  \
    size_t tail = n & 3u;                                                      \
    if (tail) {                                                                \
        const uint8_t *b = (const uint8_t *)w;                                 \
        CRC->CR = (CFGT);                                                      \
        while (tail--) { *(volatile uint8_t *)&CRC->DR = *b++; }               \
    }                                                                          \
    return POST(CRC->DR);                                                      \
}

/* DMA1 channel 1, memory-to-memory: source = CPAR (incrementing), destination = CMAR (= CRC->DR, fixed).
 * SZ = transfer size (word: PSIZE_1|MSIZE_1, byte: 0), SHIFT = log2(bytes per transfer).
 * TCIF1 is polled: this measures wall-clock time, not CPU load. */
#define HW_DMA(NAME, CFGW, CFGT, SZ, SHIFT, POST)                              \
static uint32_t NAME(const uint8_t *d, size_t n)                               \
{                                                                              \
    size_t units = n >> (SHIFT);                                               \
    CRC->CR = (CFGW) | CR_RESET;                                               \
    if (units) {                                                               \
        DMA1_Channel1->CCR   = 0;                                              \
        DMA1->IFCR           = DMA_IFCR_CGIF1;                                 \
        DMA1_Channel1->CPAR  = (uint32_t)(uintptr_t)d;                         \
        DMA1_Channel1->CMAR  = (uint32_t)(uintptr_t)&CRC->DR;                  \
        DMA1_Channel1->CNDTR = (uint32_t)units;                                \
        DMA1_Channel1->CCR   = DMA_CCR_MEM2MEM | (SZ) | DMA_CCR_PINC |         \
                               DMA_CCR_EN;                                     \
        while (!(DMA1->ISR & DMA_ISR_TCIF1)) { }                               \
        DMA1_Channel1->CCR = 0;                                                \
    }                                                                          \
    size_t tail = n & ((1u << (SHIFT)) - 1u);                                  \
    if (tail) {                                                                \
        const uint8_t *b = d + (units << (SHIFT));                             \
        CRC->CR = (CFGT);                                                      \
        while (tail--) { *(volatile uint8_t *)&CRC->DR = *b++; }               \
    }                                                                          \
    return POST(CRC->DR);                                                      \
}

#define DMA_WORD  (DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1)
#define DMA_BYTE  (0u)

/* CRC-32: reflected, word path with bit reversal in hardware, XorOut in software (~) */
HW_CPU(crc32_hw_cpu, CFG32_W, CFG32_T, NOSWAP, POST32)
HW_X4 (crc32_hw_x4,  CFG32_W, CFG32_T, NOSWAP, POST32)
HW_DMA(crc32_hw_dma, CFG32_W, CFG32_T, DMA_WORD, 2, POST32)

/* CRC-16/MODBUS: reflected, polynomial size 16, 32-bit data accesses */
HW_CPU(crc16_hw_cpu, CFG16_W, CFG16_T, NOSWAP, POST16)
HW_X4 (crc16_hw_x4,  CFG16_W, CFG16_T, NOSWAP, POST16)
HW_DMA(crc16_hw_dma, CFG16_W, CFG16_T, DMA_WORD, 2, POST16)

/* CRC-8: not reflected -> swap words with __REV(), DMA byte-wise only */
HW_CPU(crc8_hw_cpu,  CFG8_W,  CFG8_T,  BSWAP,  POST8)
HW_X4 (crc8_hw_x4,   CFG8_W,  CFG8_T,  BSWAP,  POST8)
HW_DMA(crc8_hw_dma,  CFG8_W,  CFG8_T,  DMA_BYTE, 0, POST8)

#ifdef BENCH_WITH_HAL
/* Expected CubeMX configuration of hcrc (CRC-32):
 *   DefaultPolynomialUse = ENABLE, DefaultInitValueUse = ENABLE,
 *   InputDataInversionMode = CRC_INPUTDATA_INVERSION_WORD,
 *   OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE,
 *   InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS
 * Only meaningful for lengths with n % 4 == 0. */
extern CRC_HandleTypeDef hcrc;
static uint32_t crc32_hal(const uint8_t *d, size_t n)
{
    return ~HAL_CRC_Calculate(&hcrc, (uint32_t *)(uintptr_t)d, (uint32_t)(n >> 2));
}
#endif

/* ------------------------------------------------------------------ */
/* Suites                                                              */
/* ------------------------------------------------------------------ */
typedef uint32_t (*crc_fn)(const uint8_t *, size_t);

typedef struct {
    const char *name;
    crc_fn      fn;
    uint8_t     is_hw;       /* 1: hardware variant (for the summary) */
    uint8_t     word_only;   /* 1: only lengths with n % 4 == 0 */
} method_t;

typedef struct {
    const char     *name;
    const char     *col;     /* short column name for the summary */
    uint32_t        check;   /* check value for "123456789" */
    const method_t *m;
    size_t          nm;
    uint32_t        cfg, pol, init;   /* HW configuration (without RESET) */
} suite_t;

static uint32_t crc32_null(const uint8_t *d, size_t n) { (void)d; (void)n; return 0; }

static const method_t m32[] = {
    { "bitwise", crc32_bitwise, 0, 0 },
    { "table",   crc32_table,   0, 0 },
    { "slice4",  crc32_slice4,  0, 0 },
    { "hw_cpu",  crc32_hw_cpu,  1, 0 },
    { "hw_x4",   crc32_hw_x4,   1, 0 },
    { "hw_dma",  crc32_hw_dma,  1, 0 },
#ifdef BENCH_WITH_HAL
    { "hal",     crc32_hal,     1, 1 },
#endif
};
static const method_t m16[] = {
    { "bitwise", crc16m_bitwise, 0, 0 },
    { "table",   crc16m_table,   0, 0 },
    { "slice4",  crc16m_slice4,  0, 0 },
    { "hw_cpu",  crc16_hw_cpu,   1, 0 },
    { "hw_x4",   crc16_hw_x4,    1, 0 },
    { "hw_dma",  crc16_hw_dma,   1, 0 },
};
static const method_t m8[] = {
    { "bitwise", crc8_bitwise,   0, 0 },
    { "table",   crc8_table,     0, 0 },
    { "slice4",  crc8_slice4,    0, 0 },
    { "hw_cpu",  crc8_hw_cpu,    1, 0 },
    { "hw_x4",   crc8_hw_x4,     1, 0 },
    { "hw_dma",  crc8_hw_dma,    1, 0 },
};

#define NELEM(a) (sizeof(a) / sizeof((a)[0]))

static const suite_t suites[] = {
    { "CRC-32",        "CRC-32",   0xCBF43926u, m32, NELEM(m32), CFG32_W, 0x04C11DB7u, 0xFFFFFFFFu },
    { "CRC-16/MODBUS", "CRC-16",   0x4B37u,     m16, NELEM(m16), CFG16_W, 0x00008005u, 0x0000FFFFu },
    { "CRC-8 (0x07)",  "CRC-8",    0xF4u,       m8,  NELEM(m8),  CFG8_W,  0x00000007u, 0x00000000u },
};
#define NSUITES NELEM(suites)
#define MAXM    7

static const uint16_t sizes[] = { 4, 6, 8, 13, 16, 32, 64, 128, 256, 512, 1024, 4096 };
#define NSIZES NELEM(sizes)

static uint8_t  testbuf[BENCH_MAX_LEN] __attribute__((aligned(4)));
static uint32_t cyc[NSUITES][NSIZES][MAXM];
static uint8_t  suite_ok[NSUITES];

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Minimum of BENCH_REPS runs, interrupts disabled during the measurement. */
static uint32_t measure(crc_fn f, const uint8_t *d, size_t n)
{
    uint32_t best = 0xFFFFFFFEu;
    for (int i = 0; i < BENCH_REPS; i++) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uint32_t t0 = DWT->CYCCNT;
        volatile uint32_t r = f(d, n);
        uint32_t t1 = DWT->CYCCNT;
        __set_PRIMASK(primask);
        (void)r;
        uint32_t dt = t1 - t0;
        if (dt < best) best = dt;
    }
    return best;
}

/* ------------------------------------------------------------------ */
/* Self-test                                                           */
/* ------------------------------------------------------------------ */
static int self_test(const suite_t *s)
{
    static uint8_t vec[12] __attribute__((aligned(4)));
    static const uint16_t lens[] = { 4, 16, 64, 1024, 4096 };
    int all_ok = 1;

    memcpy(vec, "123456789", 9);
    printf("Self-test %s (reference: bitwise software, check value 0x%lX):\r\n",
           s->name, (unsigned long)s->check);

    for (size_t m = 0; m < s->nm; m++) {
        int ok = 1;
        if (!s->m[m].word_only) {
            /* "123456789": word part + 1 tail byte -> also exercises the hardware byte tail */
            if (s->m[m].fn(vec, 9) != s->check) ok = 0;
            for (size_t l = 5; l <= 7; l++) {
                if (s->m[m].fn(testbuf, l) != s->m[0].fn(testbuf, l)) ok = 0;
            }
        }
        for (size_t i = 0; i < NELEM(lens); i++) {
            if (s->m[m].fn(testbuf, lens[i]) != s->m[0].fn(testbuf, lens[i])) ok = 0;
        }
        printf("  %-8s %s\r\n", s->m[m].name, ok ? "PASS" : "FAIL");
        if (!ok) all_ok = 0;
    }
    return all_ok;
}

/* ------------------------------------------------------------------ */
/* Printing the tables                                                */
/* ------------------------------------------------------------------ */
static void print_suite(size_t si)
{
    const suite_t *s = &suites[si];

    printf("\r\n--- %s%s ---\r\nCycles per call (including CRC reset):\r\n%6s",
           s->name, suite_ok[si] ? "" : "  [self-test FAIL - values unreliable]", "Bytes");
    for (size_t m = 0; m < s->nm; m++) printf(" %8s", s->m[m].name);
    printf("  fastest\r\n");
    for (size_t k = 0; k < NSIZES; k++) {
        size_t best = MAXM;
        printf("%6u", (unsigned)sizes[k]);
        for (size_t m = 0; m < s->nm; m++) {
            if (cyc[si][k][m] == NA) {
                printf(" %8s", "-");
            } else {
                printf(" %8lu", (unsigned long)cyc[si][k][m]);
                if (best == MAXM || cyc[si][k][m] < cyc[si][k][best]) best = m;
            }
        }
        printf("  %s\r\n", best == MAXM ? "-" : s->m[best].name);
    }

    printf("Cycles per byte:\r\n%6s", "Bytes");
    for (size_t m = 0; m < s->nm; m++) printf(" %8s", s->m[m].name);
    printf("\r\n");
    for (size_t k = 0; k < NSIZES; k++) {
        printf("%6u", (unsigned)sizes[k]);
        for (size_t m = 0; m < s->nm; m++) {
            if (cyc[si][k][m] == NA) {
                printf(" %8s", "-");
            } else {
                unsigned long x10 = ((unsigned long)cyc[si][k][m] * 10UL + sizes[k] / 2u) / sizes[k];
                printf(" %5lu.%lu ", x10 / 10UL, x10 % 10UL);
            }
        }
        printf("\r\n");
    }
}

/* Summary: fastest software / fastest hardware per size and algorithm.
 * Values > 1.0: hardware faster. Values < 1.0: software faster. */
static void print_summary(void)
{
    printf("\r\n=== Summary: fastest SW / fastest HW (>1.0 = HW faster) ===\r\n%6s", "Bytes");
    for (size_t si = 0; si < NSUITES; si++) printf(" %8s", suites[si].col);
    printf("\r\n");

    for (size_t k = 0; k < NSIZES; k++) {
        printf("%6u", (unsigned)sizes[k]);
        for (size_t si = 0; si < NSUITES; si++) {
            uint32_t sw = NA, hw = NA;
            for (size_t m = 0; m < suites[si].nm; m++) {
                uint32_t c = cyc[si][k][m];
                if (c == NA || c == 0) continue;
                if (suites[si].m[m].is_hw) { if (c < hw) hw = c; }
                else                       { if (c < sw) sw = c; }
            }
            if (sw == NA || hw == NA) {
                printf(" %8s", "-");
            } else {
                unsigned long r10 = ((unsigned long)sw * 10UL + hw / 2u) / hw;
                printf(" %5lu.%lux", r10 / 10UL, r10 % 10UL);
            }
        }
        printf("\r\n");
    }
}

/* ------------------------------------------------------------------ */
/* Main function                                                      */
/* ------------------------------------------------------------------ */
void crc_bench_run(void)
{
    crc_sw_init_tables();
    hw_enable_clocks();
    dwt_init();

    for (size_t i = 0; i < sizeof(testbuf); i++) {          /* pseudo-random, xorshift */
        static uint32_t x = 0x12345678u;
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;
        testbuf[i] = (uint8_t)x;
    }

    printf("\r\n=== CRC Benchmark STM32L432KC ===\r\n");
    printf("SystemCoreClock = %lu Hz, FLASH->ACR = 0x%08lX, reps = %d (minimum)\r\n",
           (unsigned long)SystemCoreClock, (unsigned long)FLASH->ACR, BENCH_REPS);

    uint32_t overhead = measure(crc32_null, testbuf, 0);
    printf("Measurement overhead (subtracted): %lu cycles\r\n\r\n", (unsigned long)overhead);

    for (size_t si = 0; si < NSUITES; si++) {
        const suite_t *s = &suites[si];

        hw_setup(s->cfg, s->pol, s->init);
        suite_ok[si] = (uint8_t)self_test(s);

        for (size_t k = 0; k < NSIZES; k++) {
            for (size_t m = 0; m < MAXM; m++) cyc[si][k][m] = NA;
            for (size_t m = 0; m < s->nm; m++) {
                if (s->m[m].word_only && (sizes[k] & 3u)) continue;   /* stays NA */
                uint32_t c = measure(s->m[m].fn, testbuf, sizes[k]);
                cyc[si][k][m] = (c > overhead) ? (c - overhead) : 0;
            }
        }
    }

    for (size_t si = 0; si < NSUITES; si++) print_suite(si);
    print_summary();
    printf("\r\n=== End ===\r\n");
}
