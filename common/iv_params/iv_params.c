#include "iv_params.h"

void iv_params_init(iv_params_t *p, const iv_control_params_t *initial)
{
    p->seq  = 0;
    p->data = *initial;
}

void iv_params_publish(iv_params_t *p, const iv_control_params_t *src)
{
    const uint32_t seq = __atomic_load_n(&p->seq, __ATOMIC_RELAXED);

    /* Нечётный счётчик — метка «данные сейчас несогласованы». Барьеры не дают
     * ни компилятору, ни процессору переставить запись полей относительно
     * счётчика: без них читатель на другом ядре увидел бы чётный счётчик
     * поверх наполовину обновлённых данных, и это была бы не редкая ошибка,
     * а тихо неверная нота. */
    __atomic_store_n(&p->seq, seq + 1, __ATOMIC_RELAXED);
    __atomic_thread_fence(__ATOMIC_RELEASE);

    p->data = *src;

    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_store_n(&p->seq, seq + 2, __ATOMIC_RELAXED);
}

bool iv_params_read(const iv_params_t *p, iv_control_params_t *dst)
{
    for (int attempt = 0; attempt < IV_PARAMS_RETRIES; ++attempt) {
        const uint32_t before = __atomic_load_n(&p->seq, __ATOMIC_ACQUIRE);
        if (before & 1u) {
            continue; /* писатель внутри записи */
        }

        const iv_control_params_t copy = p->data;

        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        if (__atomic_load_n(&p->seq, __ATOMIC_RELAXED) == before) {
            *dst = copy;
            return true;
        }
    }
    return false;
}
