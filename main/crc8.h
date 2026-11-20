/*
 * CRC-8 profiles for the 36-bit PHY frame (spec section 4).
 *
 * The wire default is CRC-8/SMBUS.  The other three catalogue profiles are
 * selectable at build time, but both boards must use the same one: there is
 * no on-wire negotiation.
 */
#ifndef PHY_CRC8_H
#define PHY_CRC8_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
    CRC8_PROFILE_SMBUS = 0,     /* poly 0x07, init 0x00, xorout 0x00, check 0xF4 */
    CRC8_PROFILE_AUTOSAR,       /* poly 0x2F, init 0xFF, xorout 0xFF, check 0xDF */
    CRC8_PROFILE_SAE_J1850,     /* poly 0x1D, init 0xFF, xorout 0xFF, check 0x4B */
    CRC8_PROFILE_MAXIM_DOW,     /* poly 0x31, init 0x00, refin/refout, check 0xA1 */
    CRC8_PROFILE_COUNT
} crc8_profile_t;

/* Generic implementation driven by the profile parameter table. */
uint8_t crc8_compute(crc8_profile_t profile, const uint8_t *data, size_t len);

/* Profile compiled in via protocol_config.h (PHY_CRC_PROFILE). */
crc8_profile_t crc8_active_profile(void);

/* CRC-8 over the protected frame region: CONTROL low nibble, then DATA. */
uint8_t crc8_frame_checksum(uint8_t control_nibble, uint8_t data);

const char *crc8_profile_name(crc8_profile_t profile);

/* Catalogue check value for ASCII "123456789". */
uint8_t crc8_catalogue_check(crc8_profile_t profile);

#endif /* PHY_CRC8_H */
