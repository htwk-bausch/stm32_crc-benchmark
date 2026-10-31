/* crc_bench.h - Hardware vs. software CRC benchmark for STM32L432KC (Cortex-M4F) */
#ifndef CRC_BENCH_H
#define CRC_BENCH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Call once from main() (after SystemClock_Config() and UART init).
 * Prints the self-test and the result tables via printf. */
void crc_bench_run(void);

#ifdef __cplusplus
}
#endif

#endif /* CRC_BENCH_H */
