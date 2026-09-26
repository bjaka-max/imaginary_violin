#include "bsp_neck.h"
#include "bsp_internal.h"

#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_io_expander_tca9554.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bsp.board";

i2c_master_bus_handle_t   bsp_i2c_sys   = NULL;
i2c_master_bus_handle_t   bsp_i2c_touch = NULL;
esp_io_expander_handle_t  bsp_expander  = NULL;

static esp_err_t bus_init(int port, int sda, int scl, i2c_master_bus_handle_t *out)
{
    i2c_master_bus_config_t cfg = {
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .i2c_port          = port,
        .sda_io_num        = sda,
        .scl_io_num        = scl,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&cfg, out);
}

esp_err_t bsp_board_init(void)
{
    ESP_RETURN_ON_ERROR(bus_init(BSP_I2C_SYS_PORT, BSP_I2C_SYS_SDA, BSP_I2C_SYS_SCL,
                                 &bsp_i2c_sys), TAG, "системная шина I2C");
    ESP_RETURN_ON_ERROR(bus_init(BSP_I2C_TOUCH_PORT, BSP_I2C_TOUCH_SDA, BSP_I2C_TOUCH_SCL,
                                 &bsp_i2c_touch), TAG, "шина I2C тача");

    ESP_RETURN_ON_ERROR(esp_io_expander_new_i2c_tca9554(
                            bsp_i2c_sys, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
                            &bsp_expander), TAG, "расширитель TCA9554");

    /* САМОЕ ПЕРВОЕ ДЕЙСТВИЕ после появления расширителя, и переставлять его
     * ниже нельзя.
     *
     * SYS_EN — не просто «системное питание», а защёлка. Кнопка питания подаёт
     * напряжение только пока её держат; удержать плату включённой после
     * отпускания обязана прошивка, подняв эту линию. Не поднимем вовремя —
     * устройство погаснет в момент отпускания кнопки. Инициализация дисплея
     * ниже занимает больше полсекунды, столько кнопку никто держать не будет.
     *
     * Обратное действие (сброс в ноль) выключает плату — см. bsp_power_off(). */
    ESP_RETURN_ON_ERROR(esp_io_expander_set_dir(bsp_expander, BSP_EXIO_SYS_EN,
                            IO_EXPANDER_OUTPUT), TAG, "направление защёлки питания");
    ESP_RETURN_ON_ERROR(esp_io_expander_set_level(bsp_expander, BSP_EXIO_SYS_EN, 1),
                        TAG, "защёлка питания");

    /* Прерывания — на вход, управляющие линии — на выход. */
    ESP_RETURN_ON_ERROR(esp_io_expander_set_dir(bsp_expander,
                            BSP_EXIO_TOUCH_INT | BSP_EXIO_IMU_INT1 |
                            BSP_EXIO_IMU_INT2 | BSP_EXIO_RTC_INT,
                            IO_EXPANDER_INPUT), TAG, "направления входов");
    /* NS_MODE — линия режима усилителя класса D: при нуле он молчит, сколько бы
     * кодек ни играл. По умолчанию линии расширителя настроены на вход,
     * то есть выключены. */
    ESP_RETURN_ON_ERROR(esp_io_expander_set_dir(bsp_expander,
                            BSP_EXIO_BL_EN | BSP_EXIO_LCD_RST | BSP_EXIO_NS_MODE,
                            IO_EXPANDER_OUTPUT), TAG, "направления выходов");
    ESP_RETURN_ON_ERROR(esp_io_expander_set_level(bsp_expander, BSP_EXIO_NS_MODE, 1),
                        TAG, "включение усилителя");

    /* Подсветка выключена до инициализации дисплея, иначе на экране будет
     * виден мусор из памяти контроллера. */
    ESP_RETURN_ON_ERROR(esp_io_expander_set_level(bsp_expander, BSP_EXIO_BL_EN, 0),
                        TAG, "выключение подсветки");
    ESP_RETURN_ON_ERROR(esp_io_expander_set_level(bsp_expander, BSP_EXIO_LCD_RST, 1),
                        TAG, "снятие сброса дисплея");

    /* ШИМ настраивается здесь, до того как кто-либо разрешит силовую цепь:
     * порядок взят из демо производителя — сперва осмысленная скважность,
     * потом питание, иначе на включении моргнёт непредсказуемой яркостью. */
    ESP_RETURN_ON_ERROR(bsp_backlight_pwm_init(), TAG, "ШИМ подсветки");
    bsp_display_set_brightness(80);

    /* Тач поднимается здесь, а не при первом чтении: читают его две задачи, и
     * создавать замок наперегонки нельзя. */
    ESP_RETURN_ON_ERROR(bsp_touch_init(), TAG, "тач");

    ESP_LOGI(TAG, "защёлка питания взведена, шины I2C, усилитель и ШИМ подсветки подняты");
    return ESP_OK;
}

void bsp_lcd_reset(void)
{
    /* Длительности взяты из демо производителя. */
    esp_io_expander_set_level(bsp_expander, BSP_EXIO_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(30));
    esp_io_expander_set_level(bsp_expander, BSP_EXIO_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(250));
    esp_io_expander_set_level(bsp_expander, BSP_EXIO_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(30));
}

/* Параметры ШИМ повторяют демо производителя: 8 бит, 50 кГц, тактирование от
 * RC_FAST. Частота выбрана не случайно — на слышимых частотах дроссель
 * подсветки свистит, а на низких заметно мерцание. */
#define BL_LEDC_TIMER    LEDC_TIMER_3
#define BL_LEDC_CHANNEL  LEDC_CHANNEL_1
#define BL_LEDC_MAX_DUTY 255

esp_err_t bsp_backlight_pwm_init(void)
{
    const ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num       = BL_LEDC_TIMER,
        .freq_hz         = 50 * 1000,
        .clk_cfg         = LEDC_SLOW_CLK_RC_FAST,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "таймер ШИМ подсветки");

    const ledc_channel_config_t channel = {
        .gpio_num   = BSP_LCD_BL_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = BL_LEDC_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = BL_LEDC_TIMER,
        .duty       = 0,   /* инверсия: 0 — максимальная яркость */
        .hpoint     = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG, "канал ШИМ подсветки");
    return ESP_OK;
}

void bsp_display_set_brightness(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    /* Инверсия. Проверено по двум местам в демо производителя: макросы
     * LCD_PWM_MODE_x заданы как (0xff - x), то есть LCD_PWM_MODE_255 это
     * скважность 0 и самая яркая подсветка; слайдер яркости в примере
     * тоже пишет (0xff - value). */
    const uint32_t duty = BL_LEDC_MAX_DUTY -
                          ((uint32_t)percent * BL_LEDC_MAX_DUTY / 100);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL);
}

void bsp_power_off(void)
{
    /* Снятие защёлки обесточивает плату. Оговорка: пока кнопка питания зажата,
     * она сама подаёт напряжение, поэтому реально плата погаснет только в
     * момент её отпускания. Считать погасший экран подтверждением выключения
     * нельзя — транзакция к расширителю могла и не пройти. */
    bsp_display_backlight(false);
    bsp_amp_enable(false);
    esp_io_expander_set_level(bsp_expander, BSP_EXIO_SYS_EN, 0);
}

void bsp_amp_enable(bool on)
{
    esp_io_expander_set_level(bsp_expander, BSP_EXIO_NS_MODE, on ? 1 : 0);
}

void bsp_display_backlight(bool on)
{
    /* Разрешение силовой цепи. Скважность ШИМ без этого бита не значит ничего. */
    esp_io_expander_set_level(bsp_expander, BSP_EXIO_BL_EN, on ? 1 : 0);
}
