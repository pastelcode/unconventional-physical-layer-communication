#include "crc8.h"

#include <stdbool.h>
#include <string.h>

#include "protocol_config.h"

typedef struct {
    uint8_t poly;
    uint8_t init;
    uint8_t xorout;
    bool refin;
    bool refout;
    uint8_t check; /* CRC of ASCII "123456789" */
} crc8_params_t;

static const crc8_params_t s_params[CRC8_PROFILE_COUNT] = {
    [CRC8_PROFILE_SMBUS]     = {0x07, 0x00, 0x00, false, false, 0xF4},
    [CRC8_PROFILE_AUTOSAR]   = {0x2F, 0xFF, 0xFF, false, false, 0xDF},
    [CRC8_PROFILE_SAE_J1850] = {0x1D, 0xFF, 0xFF, false, false, 0x4B},
    [CRC8_PROFILE_MAXIM_DOW] = {0x31, 0x00, 0x00, true,  true,  0xA1},
};

static uint8_t reflect8(uint8_t v)
{
    v = (uint8_t)(((v & 0xF0) >> 4) | ((v & 0x0F) << 4));
    v = (uint8_t)(((v & 0xCC) >> 2) | ((v & 0x33) << 2));
    v = (uint8_t)(((v & 0xAA) >> 1) | ((v & 0x55) << 1));
    return v;
}

uint8_t crc8_compute(crc8_profile_t profile, const uint8_t *data, size_t len)
{
    if (profile < 0 || profile >= CRC8_PROFILE_COUNT || (data == NULL && len > 0)) {
        return 0;
    }

    const crc8_params_t *p = &s_params[profile];
    uint8_t crc = p->init;

    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        if (p->refin) {
            byte = reflect8(byte);
        }
        crc ^= byte;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80u) {
                crc = (uint8_t)((crc << 1) ^ p->poly);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    if (p->refout) {
        crc = reflect8(crc);
    }
    return (uint8_t)(crc ^ p->xorout);
}

crc8_profile_t crc8_active_profile(void)
{
    return (crc8_profile_t)PHY_CRC_PROFILE;
}

uint8_t crc8_frame_checksum(uint8_t control_nibble, uint8_t data)
{
    /* CONTROL in the low nibble with a zero high nibble, then DATA, and
     * nothing else (SYN, SFD and the CRC byte itself are excluded). */
    const uint8_t payload[2] = {(uint8_t)(control_nibble & 0x0Fu), data};
    return crc8_compute(crc8_active_profile(), payload, sizeof(payload));
}

const char *crc8_profile_name(crc8_profile_t profile)
{
    switch (profile) {
    case CRC8_PROFILE_SMBUS:     return "CRC-8/SMBUS";
    case CRC8_PROFILE_AUTOSAR:   return "CRC-8/AUTOSAR";
    case CRC8_PROFILE_SAE_J1850: return "CRC-8/SAE-J1850";
    case CRC8_PROFILE_MAXIM_DOW: return "CRC-8/MAXIM-DOW";
    default:                     return "CRC-8/unknown";
    }
}

uint8_t crc8_catalogue_check(crc8_profile_t profile)
{
    if (profile < 0 || profile >= CRC8_PROFILE_COUNT) {
        return 0;
    }
    return s_params[profile].check;
}
