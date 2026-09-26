#include "iv_protocol.h"
#include <string.h>

void iv_bow_packet_init(iv_bow_packet_t *p, uint16_t seq, uint32_t t_us)
{
    memset(p, 0, sizeof(*p));
    p->magic   = IV_PROTO_MAGIC;
    p->version = IV_PROTO_VERSION;
    p->seq     = seq;
    p->t_us    = t_us;
}

void iv_neck_packet_init(iv_neck_packet_t *p, uint8_t type, uint32_t echo_t_us,
                         uint32_t proc_us)
{
    memset(p, 0, sizeof(*p));
    p->magic     = IV_PROTO_MAGIC;
    p->version   = IV_PROTO_VERSION;
    p->type      = type;
    p->echo_t_us = echo_t_us;
    p->proc_us   = proc_us > UINT16_MAX ? UINT16_MAX : (uint16_t)proc_us;
}

/* Копируем, а не приводим указатель: буфер приёмника не обязан быть выровнен. */
static bool header_valid(const void *data, size_t len, size_t expect)
{
    struct { uint16_t magic; uint8_t version; } h;
    if (data == NULL || len != expect) {
        return false;
    }
    memcpy(&h, data, sizeof(h));
    return h.magic == IV_PROTO_MAGIC && h.version == IV_PROTO_VERSION;
}

bool iv_bow_packet_valid(const void *data, size_t len)
{
    return header_valid(data, len, sizeof(iv_bow_packet_t));
}

bool iv_neck_packet_valid(const void *data, size_t len)
{
    return header_valid(data, len, sizeof(iv_neck_packet_t));
}

uint16_t iv_seq_gap(uint16_t prev, uint16_t cur)
{
    uint16_t diff = (uint16_t)(cur - prev);
    return diff ? (uint16_t)(diff - 1) : 0;
}

/* Округление к ближайшему без libm: в протоколе математике не место, а на
 * смычке это делается 200 раз в секунду. */
static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static int32_t round_to_int(float v)
{
    return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

int16_t iv_pack_angle(float deg)
{
    return (int16_t)round_to_int(clampf(deg, -180.0f, 180.0f) * IV_ANGLE_SCALE);
}

float iv_unpack_angle(int16_t v)
{
    return (float)v / IV_ANGLE_SCALE;
}

int16_t iv_pack_vel(float norm)
{
    return (int16_t)round_to_int(clampf(norm, -1.0f, 1.0f) * IV_VEL_SCALE);
}

float iv_unpack_vel(int16_t v)
{
    return (float)v / IV_VEL_SCALE;
}

uint16_t iv_pack_accel(float g)
{
    return (uint16_t)round_to_int(clampf(g, 0.0f, 65535.0f / IV_ACCEL_SCALE) * IV_ACCEL_SCALE);
}

float iv_unpack_accel(uint16_t v)
{
    return (float)v / IV_ACCEL_SCALE;
}
