#include "iv_link_stats.h"
#include <string.h>

void iv_link_stats_reset(iv_link_stats_t *s)
{
    memset(s, 0, sizeof(*s));
}

bool iv_link_stats_on_packet(iv_link_stats_t *s, uint16_t seq, int64_t now_us)
{
    const bool was_offline = !s->online;

    if (s->have_seq && s->online) {
        const uint16_t gap = iv_seq_gap(s->last_seq, seq);
        if (gap > IV_SEQ_GAP_MAX) {
            s->resets++;
        } else {
            s->lost += gap;
        }
    }

    s->last_seq   = seq;
    s->have_seq   = true;
    s->last_rx_us = now_us;
    s->online     = true;
    s->received++;

    return was_offline && s->received > 1; /* первый пакет — это не «восстановилась» */
}

bool iv_link_stats_tick(iv_link_stats_t *s, int64_t now_us)
{
    if (!s->online) {
        return false;
    }
    if (now_us - s->last_rx_us <= (int64_t)IV_LINK_TIMEOUT_MS * 1000) {
        return false;
    }
    s->online = false;
    s->outages++;
    return true;
}

float iv_link_loss_ratio(const iv_link_stats_t *s)
{
    const uint32_t total = s->received + s->lost;
    return total ? (float)s->lost / (float)total : 0.0f;
}
