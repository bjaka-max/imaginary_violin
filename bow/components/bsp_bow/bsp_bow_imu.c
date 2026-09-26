#include "bsp_bow.h"
#include "bsp_bow_internal.h"

#include "bmi270.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "bsp.bow.imu";

/* Диапазоны под ведение смычка: рука разгоняется заметно, но не до ударных
 * перегрузок, а угловые скорости при смене штриха высокие. */
#define ACC_RANGE  BMI270_ACC_RANGE_8_G
#define GYR_RANGE  BMI270_GYR_RANGE_1000_DPS

static bmi270_handle_t *s_imu = NULL;

/* Датчик работает вдвое быстрее опроса — в обоих режимах. Так каждый опрос
 * забирает свежую выборку: при равной частоте фазы неизбежно разъезжаются и
 * часть чтений возвращает уже прочитанное, что портит интегрирование
 * скорости. */
static const bmi270_config_t CFG_FULL = {
    .acce_odr   = BMI270_ACC_ODR_800_HZ,
    .acce_range = ACC_RANGE,
    .gyro_odr   = BMI270_GYR_ODR_800_HZ,
    .gyro_range = GYR_RANGE,
};

static const bmi270_config_t CFG_IDLE = {
    .acce_odr   = BMI270_ACC_ODR_50_HZ,
    .acce_range = ACC_RANGE,
    .gyro_odr   = BMI270_GYR_ODR_50_HZ,
    .gyro_range = GYR_RANGE,
};

esp_err_t bsp_bow_imu_init(void)
{
    ESP_RETURN_ON_ERROR(bsp_bow_board_init(), TAG, "шина I2C");

    /* BMI270 при инициализации требует загрузки конфигурационного блоба —
     * ради него взят готовый драйвер, а не написан свой. */
    const bmi270_driver_config_t drv = {
        .addr      = BMI270_I2C_ADDRESS_L,
        .interface = BMI270_USE_I2C,
        .i2c_bus   = bsp_bow_i2c,
    };
    ESP_RETURN_ON_ERROR(bmi270_create(&drv, &s_imu), TAG, "создание BMI270");

    uint8_t chip_id = 0;
    ESP_RETURN_ON_ERROR(bmi270_get_chip_id(s_imu, &chip_id), TAG, "chip id");
    ESP_LOGI(TAG, "BMI270 отвечает, chip id 0x%02x", chip_id);

    ESP_RETURN_ON_ERROR(bmi270_start(s_imu, &CFG_FULL), TAG, "запуск BMI270");

    ESP_LOGI(TAG, "IMU поднят: датчик 800 Гц, опрос %d Гц, ±8g, ±1000 dps",
             BSP_BOW_IMU_HZ);
    return ESP_OK;
}

esp_err_t bsp_bow_imu_set_rate(bsp_bow_imu_rate_t rate)
{
    ESP_RETURN_ON_FALSE(s_imu, ESP_ERR_INVALID_STATE, TAG, "IMU не поднят");
    /* bmi270_start здесь не перезагружает блоб — он только включает блоки и
     * пишет ODR с диапазонами, то есть стоит нескольких транзакций I2C. */
    return bmi270_start(s_imu, rate == BSP_BOW_IMU_IDLE ? &CFG_IDLE : &CFG_FULL);
}

/* Как прибор лежит на смычке.
 *
 * Оценке движения нужно, чтобы ось X смотрела вдоль смычка В СТОРОНУ КОНЦА:
 * по её знаку решается, идёт смычок к концу или к колодке, а от этого знака
 * берётся знак наклона линии движения — то есть выбор струны (iv_motion.h).
 * Какая ось куда смотрит, решает монтаж, и решается это здесь, один раз, а
 * не в оценке движения: та описывает физику и одинакова для любого монтажа.
 *
 * Прибор развёрнут на смычке концом к колодке, поэтому оси приводятся
 * поворотом на 180° вокруг вертикали прибора: X и Y меняют знак, Z остаётся.
 * Это настоящий поворот (определитель +1), а не отражение осей по одной:
 * отражение перевернуло бы правизну тройки, и гироскоп после него врал бы в
 * знаке вращения — а гироскопом поворачивается оценка гравитации.
 *
 * Видно этот поворот снаружи ровно одним: наклон линии движения меняет знак,
 * и раскладка струн зеркалится (G ↔ E). Ни скорости, ни признака покоя, ни
 * «низа» он не касается — вертикаль прибора осталась вертикалью.
 *
 * Проверено на инструменте, а не выведено из чертежа: ведение к концу смычка
 * и вверх даёт плюсовой угол и уводит к E. Проверять надо именно так — и
 * обязательно после того, как «центр смычка» в меню грифа снят заново:
 * сохранённый в NVS центр снят в прежних осях и сам по себе выглядит как
 * сбитый угол, так что по звуку эти две беды не различить.
 *
 * Калибровка из NVS снята в прежних осях: её смещение гироскопа по X и Y
 * теперь с обратным знаком. Величина там доли градуса в секунду, и ZUPT её
 * съедает, но после переворота калибровку лучше снять заново. */
#define MOUNT_X (-1.0f)
#define MOUNT_Y (-1.0f)
#define MOUNT_Z (+1.0f)

esp_err_t bsp_bow_imu_read(bsp_bow_imu_t *out)
{
    ESP_RETURN_ON_FALSE(out, ESP_ERR_INVALID_ARG, TAG, "out");
    ESP_RETURN_ON_FALSE(s_imu, ESP_ERR_INVALID_STATE, TAG, "IMU не поднят");

    ESP_RETURN_ON_ERROR(bmi270_get_acce_data(s_imu, &out->ax, &out->ay, &out->az),
                        TAG, "чтение акселерометра");
    ESP_RETURN_ON_ERROR(bmi270_get_gyro_data(s_imu, &out->gx, &out->gy, &out->gz),
                        TAG, "чтение гироскопа");

    out->ax *= MOUNT_X; out->ay *= MOUNT_Y; out->az *= MOUNT_Z;
    out->gx *= MOUNT_X; out->gy *= MOUNT_Y; out->gz *= MOUNT_Z;
    return ESP_OK;
}
