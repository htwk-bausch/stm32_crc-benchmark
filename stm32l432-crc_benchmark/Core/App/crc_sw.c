#include "crc_sw.h"
#include <string.h>

#define POLY_REFLECTED 0xEDB88320u

/* t[0] = classic byte table, t[1..3] = tables for slicing-by-4 */
static uint32_t t[4][256];
static uint16_t t16[256];    /* CRC-16/MODBUS, reflected */
static uint8_t  t8[256];     /* CRC-8 poly 0x07, not reflected */
static uint16_t s16[4][256]; /* slicing-by-4 CRC-16/MODBUS (2 KB) */
static uint8_t  s8[4][256];  /* slicing-by-4 CRC-8 (1 KB)         */

void crc_sw_init_tables(void)
{
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c16 = i;
        for (int k = 0; k < 8; k++) {
            c16 = (c16 & 1u) ? (c16 >> 1) ^ 0xA001u : (c16 >> 1);
        }
        t16[i] = (uint16_t)c16;

        uint32_t c8 = i;
        for (int k = 0; k < 8; k++) {
            c8 = (c8 & 0x80u) ? ((c8 << 1) ^ 0x07u) : (c8 << 1);
            c8 &= 0xFFu;
        }
        t8[i] = (uint8_t)c8;
    }

    /* Slicing tables: Tk[i] = CRC state after byte i followed by k zero bytes */
    for (uint32_t i = 0; i < 256; i++) {
        s16[0][i] = t16[i];
        s8[0][i]  = t8[i];
        for (int k = 1; k < 4; k++) {
            uint16_t p16 = s16[k - 1][i];
            s16[k][i] = (uint16_t)((p16 >> 8) ^ t16[p16 & 0xFFu]);
            s8[k][i]  = t8[s8[k - 1][i]];
        }
    }

    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = (c & 1u) ? (c >> 1) ^ POLY_REFLECTED : (c >> 1);
        }
        t[0][i] = c;
    }
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = t[0][i];
        for (int s = 1; s < 4; s++) {
            c = t[0][c & 0xFFu] ^ (c >> 8);
            t[s][i] = c;
        }
    }
}

uint32_t crc32_bitwise(const uint8_t *d, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    while (n--) {
        c ^= *d++;
        for (int k = 0; k < 8; k++) {
            c = (c & 1u) ? (c >> 1) ^ POLY_REFLECTED : (c >> 1);
        }
    }
    return ~c;
}

uint32_t crc32_table(const uint8_t *d, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    while (n--) {
        c = t[0][(c ^ *d++) & 0xFFu] ^ (c >> 8);
    }
    return ~c;
}

/* Assumes little-endian (Cortex-M and x86). memcpy allows unaligned accesses
 * without aliasing problems; the compiler turns it into a single LDR. */
uint32_t crc32_slice4(const uint8_t *d, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    while (n >= 4) {
        uint32_t v;
        memcpy(&v, d, 4);
        v ^= c;
        c = t[3][v & 0xFFu] ^ t[2][(v >> 8) & 0xFFu] ^
            t[1][(v >> 16) & 0xFFu] ^ t[0][v >> 24];
        d += 4;
        n -= 4;
    }
    while (n--) {
        c = t[0][(c ^ *d++) & 0xFFu] ^ (c >> 8);
    }
    return ~c;
}

/* ---------------- CRC-16/MODBUS ---------------- */
uint32_t crc16m_bitwise(const uint8_t *d, size_t n)
{
    uint32_t c = 0xFFFFu;
    while (n--) {
        c ^= *d++;
        for (int k = 0; k < 8; k++) {
            c = (c & 1u) ? (c >> 1) ^ 0xA001u : (c >> 1);
        }
    }
    return c;
}

uint32_t crc16m_table(const uint8_t *d, size_t n)
{
    uint32_t c = 0xFFFFu;
    while (n--) {
        c = (c >> 8) ^ t16[(c ^ *d++) & 0xFFu];
    }
    return c;
}

uint32_t crc16m_slice4(const uint8_t *d, size_t n)
{
    uint32_t c = 0xFFFFu;
    while (n >= 4) {
        uint32_t v;
        memcpy(&v, d, 4);
        v ^= c;                        /* the state affects the lower two bytes */
        c = (uint32_t)(s16[3][v & 0xFFu] ^ s16[2][(v >> 8) & 0xFFu] ^
                       s16[1][(v >> 16) & 0xFFu] ^ s16[0][v >> 24]);
        d += 4;
        n -= 4;
    }
    while (n--) {
        c = (c >> 8) ^ t16[(c ^ *d++) & 0xFFu];
    }
    return c;
}

/* ---------------- CRC-8 (Poly 0x07) ---------------- */
uint32_t crc8_bitwise(const uint8_t *d, size_t n)
{
    uint32_t c = 0;
    while (n--) {
        c ^= *d++;
        for (int k = 0; k < 8; k++) {
            c = (c & 0x80u) ? ((c << 1) ^ 0x07u) : (c << 1);
            c &= 0xFFu;
        }
    }
    return c;
}

uint32_t crc8_table(const uint8_t *d, size_t n)
{
    uint32_t c = 0;
    while (n--) {
        c = t8[(c ^ *d++) & 0xFFu];
    }
    return c;
}

/* Not reflected: the first byte is in the lowest byte of the little-endian word. */
uint32_t crc8_slice4(const uint8_t *d, size_t n)
{
    uint32_t c = 0;
    while (n >= 4) {
        uint32_t v;
        memcpy(&v, d, 4);
        c = (uint32_t)(s8[3][(v ^ c) & 0xFFu] ^ s8[2][(v >> 8) & 0xFFu] ^
                       s8[1][(v >> 16) & 0xFFu] ^ s8[0][v >> 24]);
        d += 4;
        n -= 4;
    }
    while (n--) {
        c = t8[(c ^ *d++) & 0xFFu];
    }
    return c;
}
