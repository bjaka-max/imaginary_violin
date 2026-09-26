#include "bsp_neck.h"
#include "bsp_internal.h"

#include "driver/i2s_std.h"
#include "esp_attr.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8311_codec.h"
#include "esp_log.h"

static const char *TAG = "bsp.audio";

/* ES8311 — моно-кодек. Наружу отдаём стерео 16 бит: так проще и совместимо
 * с обычными аудиоцепочками, кодек сам берёт то, что ему нужно. */
#define AUDIO_CHANNELS 2
#define AUDIO_BITS     16

static i2s_chan_handle_t     s_tx  = NULL;
static esp_codec_dev_handle_t s_dev = NULL;

/* Учёт недокорма DMA.
 *
 * Прямого события «буфер кончился» у драйвера I2S нет, но есть событие «буфер
 * доигран», и его достаточно. Инвариант, замеренный на плате: в момент прихода
 * события заполненных, но ещё не отыгранных буферов ровно один — задача пишет
 * следующий блок уже после события, разблокированная им же. Значит здоровое
 * состояние это written == sent + 1 ДО инкремента sent; равенство written и
 * sent означает, что заполненных буферов не осталось и в кодек уйдёт тишина.
 *
 * Порядок проверки и инкремента здесь не косметика: сравнение после инкремента
 * даёт ошибку на единицу, при которой недокормом выглядит каждый блок. */
static volatile uint32_t s_written;   /* блоков отдано в I2S задачей */
static volatile uint32_t s_sent;      /* блоков доиграно, считает прерывание */
static volatile uint32_t s_underruns;
static volatile bool     s_counting;  /* до первой записи считать нечего */

static IRAM_ATTR bool on_sent(i2s_chan_handle_t handle, i2s_event_data_t *event, void *ctx)
{
    (void)handle; (void)event; (void)ctx;

    if (s_counting) {
        if (s_written == s_sent) {
            s_underruns++;
        }
        s_sent++;
    }
    return false; /* задачу будить не нужно */
}

esp_err_t bsp_audio_init(uint32_t sample_rate)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    /* Фиксируем размер и число DMA-буферов: от них напрямую зависит
     * аудиозадержка, а она в этом проекте — критерий приёмки. */
    chan_cfg.dma_desc_num  = 2;
    chan_cfg.dma_frame_num = 128;
    chan_cfg.auto_clear    = true; /* при недокорме выдаём тишину, а не мусор */
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_tx, NULL), TAG, "канал I2S");

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = BSP_I2S_MCLK,
            .bclk = BSP_I2S_BCLK,
            .ws   = BSP_I2S_WS,
            .dout = BSP_I2S_DOUT,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {0},
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_tx, &std_cfg), TAG, "режим I2S");

    /* Колбэк регистрируется до включения канала: в состоянии RUNNING драйвер
     * его уже не примет. */
    const i2s_event_callbacks_t cbs = { .on_sent = on_sent };
    ESP_RETURN_ON_ERROR(i2s_channel_register_event_callback(s_tx, &cbs, NULL),
                        TAG, "колбэк I2S");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_tx), TAG, "включение I2S");

    audio_codec_i2s_cfg_t data_cfg = { .port = I2S_NUM_0, .tx_handle = s_tx };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&data_cfg);
    ESP_RETURN_ON_FALSE(data_if, ESP_FAIL, TAG, "интерфейс данных кодека");

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port       = BSP_I2C_SYS_PORT,
        .addr       = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = bsp_i2c_sys,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(ctrl_if, ESP_FAIL, TAG, "интерфейс управления кодеком");

    /* hw_gain намеренно не заполняем, хотя в конфигурации производителя стоит
     * pa_gain: 6. Кодек не прибавляет это усиление, а ВЫЧИТАЕТ его из своего
     * выхода (`db_value -= hw_gain` в es8311_set_vol): поле описывает, сколько
     * уже добавляет внешний усилитель, чтобы суммарная громкость совпала с
     * запрошенной. С pa_gain = 6 звук стал бы на 6 дБ тише, а не громче.
     *
     * pa_pin = -1: усилитель включается не выводом кодека, а линией NS_MODE
     * расширителя TCA9554 — это делает bsp_board_init(). */
    es8311_codec_cfg_t es_cfg = {
        .ctrl_if     = ctrl_if,
        .gpio_if     = audio_codec_new_gpio(),
        .codec_mode  = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin      = -1,
        .master_mode = false, /* тактирование ведёт ESP32 */
        .use_mclk    = true,
    };
    const audio_codec_if_t *codec_if = es8311_codec_new(&es_cfg);
    ESP_RETURN_ON_FALSE(codec_if, ESP_FAIL, TAG, "кодек ES8311");

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = codec_if,
        .data_if  = data_if,
    };
    s_dev = esp_codec_dev_new(&dev_cfg);
    ESP_RETURN_ON_FALSE(s_dev, ESP_FAIL, TAG, "устройство кодека");

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = AUDIO_BITS,
        .channel         = AUDIO_CHANNELS,
        .sample_rate     = sample_rate,
    };
    ESP_RETURN_ON_FALSE(esp_codec_dev_open(s_dev, &fs) == 0, ESP_FAIL, TAG,
                        "открытие кодека");

    /* Кривая громкости по умолчанию линейна в децибелах: 0 -> -50 дБ, 100 -> 0 дБ.
     * То есть 70 это уже -15 дБ, и вместе с запасом по амплитуде сигнала звук
     * получается заметно тише, чем ожидаешь от «70 процентов». */
    esp_codec_dev_set_out_vol(s_dev, 90);

    ESP_LOGI(TAG, "звук поднят: %" PRIu32 " Гц, блок DMA %d кадров",
             sample_rate, chan_cfg.dma_frame_num);
    return ESP_OK;
}

esp_err_t bsp_audio_write(const int16_t *frames, size_t frame_count)
{
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "звук не поднят");
    const int len = (int)(frame_count * AUDIO_CHANNELS * sizeof(int16_t));

    if (esp_codec_dev_write(s_dev, (void *)frames, len) != 0) {
        return ESP_FAIL;
    }

    if (!s_counting) {
        /* До первой записи канал крутит пустые буферы, и считать их недокормом
         * нельзя: звука ещё никто не просил. Счёт начинается отсюда. */
        s_written  = 0;
        s_sent     = 0;
        s_counting = true;
    }
    s_written++;
    return ESP_OK;
}

void bsp_audio_stats(bsp_audio_stats_t *out)
{
    out->written   = s_written;
    out->sent      = s_sent;
    out->underruns = s_underruns;
}

void bsp_audio_stats_reset(void)
{
    s_counting  = false;
    s_underruns = 0;
}

void bsp_audio_set_volume(int percent)
{
    if (s_dev) {
        esp_codec_dev_set_out_vol(s_dev, percent);
    }
}
