/* crc_sw.h - Software CRCs: CRC-32 (Ethernet/zlib, reflected, init 0xFFFFFFFF, XorOut 0xFFFFFFFF),
 * CRC-16/MODBUS and CRC-8.
 * Portable C code without any hardware dependency, so it can also be tested on a host.
 * Check value: CRC32("123456789") = 0xCBF43926
 */
#ifndef CRC_SW_H
#define CRC_SW_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Must be called once before any *_table() or *_slice4() function (the tables live in RAM). */
void crc_sw_init_tables(void);

uint32_t crc32_bitwise(const uint8_t *d, size_t n);  /* no memory, slow             */
uint32_t crc32_table(const uint8_t *d, size_t n);    /* 1 KB table, 1 byte/step     */
uint32_t crc32_slice4(const uint8_t *d, size_t n);   /* 4 KB tables, 4 bytes/step   */

/* CRC-16/MODBUS: poly 0x8005 (reflected 0xA001), init 0xFFFF, refin/refout, XorOut 0.
 * Check value CRC16("123456789") = 0x4B37. Result in the lower 16 bits. */
uint32_t crc16m_bitwise(const uint8_t *d, size_t n);
uint32_t crc16m_table(const uint8_t *d, size_t n);   /* 512 byte table */
uint32_t crc16m_slice4(const uint8_t *d, size_t n);  /* 2 KB tables, 4 bytes/step */

/* CRC-8 (SMBus PEC): poly 0x07, init 0x00, not reflected, XorOut 0.
 * Check value CRC8("123456789") = 0xF4. Result in the lower 8 bits. */
uint32_t crc8_bitwise(const uint8_t *d, size_t n);
uint32_t crc8_table(const uint8_t *d, size_t n);     /* 256 byte table */
uint32_t crc8_slice4(const uint8_t *d, size_t n);    /* 1 KB tables, 4 bytes/step */

#ifdef __cplusplus
}
#endif

#endif /* CRC_SW_H */
