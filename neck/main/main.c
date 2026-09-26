/* Гриф — фаза 6: интерфейс, настройки, питание.
 *
 * Аудиотракт и синтез не менялись (фазы 3 и 4), раскладка жестов — тоже
 * (фаза 5). Изменилось то, что вокруг: у инструмента появился игровой экран,
 * меню настроек, память о настройках и внятное выключение.
 *
 * Три экрана и переходы между ними:
 *
 *   ИГРА          струны, нота, палец, смычок, связь, заряд. Рисуется
 *                 грязными прямоугольниками: полная перерисовка — 220 КБ по
 *                 QSPI, и она конкурирует с шинами, по которым идут тач и звук.
 *   МЕНЮ          строй, полутоны, громкость, яркость, центр смычка,
 *                 спаривание. Управляется тачем — кнопок всего две, и обе заняты.
 *   ДИАГНОСТИКА   загрузка ядра, задержка, потери, недокормы. Это то, чем
 *                 меряются критерии приёмки фаз 2-4, и убирать её нельзя.
 *
 * Кнопки: BOOT коротко — привязка к полутонам, BOOT долго — меню, PWR долго —
 * выключение (с сохранением настроек и картинкой на экране).
 *
 * Приоритет экрана ниже звука не только по номеру задачи: если аудиоядро
 * начинает захлёбываться, кадры прореживаются вдвое. Ронять звук ради
 * картинки нельзя ни при каких обстоятельствах. */
#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "bsp_neck.h"
#include "iv_protocol.h"
#include "iv_link.h"
#include "iv_params.h"
#include "iv_motion.h"
#include "iv_control.h"
#include "iv_menu.h"
#include "iv_power.h"
#include "iv_settings.h"
#include "iv_synth.h"
#include "iv_ui.h"

static const char *TAG = "neck";

#define BLOCK_FRAMES IV_BLOCK_SAMPLES
#define BLOCK_US     (1000000 / (IV_SAMPLE_RATE / IV_BLOCK_SAMPLES)) /* 2666 мкс */

/* Частота экрана. 25 кадров в секунду — не «чтобы плавно», а потолок, за
 * которым обмен по QSPI начинает мешать: даже дешёвый кадр это несколько
 * килобайт, а шина общая. */
#define UI_HZ        25

/* Раз во сколько кадров печатается статистика в лог: раз в две секунды. */
#define LOG_EVERY    (UI_HZ * 2)

/* Экран диагностики перерисовывается целиком — два десятка текстовых строк, —
 * и на игровой частоте он один занял бы три мегабайта в секунду по QSPI.
 * Цифры на нём меняются раз в секунду, так что пять кадров в секунду не
 * теряют ничего. */
#define DIAG_EVERY   (UI_HZ / 5)

/* Как часто перерисовывается игровой экран. Реже, чем идут кадры, и это
 * важнее, чем кажется: кадр стоит не своей высоты, а всего, что выше пальца
 * (см. iv_ui_present), то есть до двухсот двадцати килобайт по QSPI. Двадцать
 * пять раз в секунду такое соперничает со звуком, а глазу от указателя
 * положения хватает и двенадцати. */
#define GAME_EVERY   (UI_HZ / 12)

/* Загрузка аудиоядра, выше которой экран прореживается вдвое. Порог приёмки —
 * сорок процентов; здесь взято чуть ниже, чтобы деградировать заранее, а не
 * когда щелчки уже слышны. */
#define UI_YIELD_LOAD_PCT 35

/* Как часто меряется заряд. Раз в секунду: он меняется минутами, а чтение
 * АЦП стоит места в задаче, которая занята экраном. */
#define BATT_PERIOD_FRAMES UI_HZ

/* Автоповтор в меню: первый шаг сразу, дальше через задержку и с этим
 * периодом. Без автоповтора выставить громкость от 20 до 100 значило бы
 * шестнадцать отдельных касаний. */
#define MENU_REPEAT_DELAY_FRAMES (UI_HZ / 2)  /* полсекунды */
#define MENU_REPEAT_EVERY_FRAMES (UI_HZ / 8)  /* восемь шагов в секунду */

/* ---- наклон грифа --------------------------------------------------------- */

/* Сколько держать последнее положение пальца, когда тач перестал его видеть.
 *
 * Пропавшее касание — это не всегда снятый палец. Ёмкостный тач теряет его на
 * одну-две выборки от помехи или слабого нажатия, и без выдержки высота на
 * этот миг прыгает на открытую струну и обратно. Слышно это не как щелчок, а
 * как бульканье: замерено, что срывы восемь раз в секунду роняют периодичность
 * тона с 0.998 до 0.53, а определяемая высота уезжает на субгармонику.
 *
 * Шестьдесят миллисекунд — дольше любого срыва и короче любого настоящего
 * снятия пальца. Задержка тут безобидна: открытая струна не атака, спешить
 * к ней некуда. */
#define TOUCH_HOLD_MS   60

/* Свой IMU опрашивается реже слоя управления: ось выразительности медленная,
 * а чтение по I2C стоит трети миллисекунды и стоит в пути от смычка к звуку.
 * Каждый пятый цикл — это 40 Гц, для наклона инструмента с запасом. */
#define NECK_IMU_EVERY  5

/* Постоянная времени фьюжна. Длиннее, чем у смычка: грифу нужен не быстрый
 * отклик, а спокойный абсолютный угол — дрожь руки в тембр пропускать незачем.
 *
 * Калибровка покоя грифу не нужна, и это не экономия: наклон берётся из
 * отношения проекций гравитации, а отношение не зависит от масштаба
 * акселерометра; смещение нуля гироскопа влияет только на переходный процесс
 * и стирается той же гравитацией за постоянную времени. Смычку калибровка
 * нужна потому, что там ускорение интегрируется, и обе ошибки копятся. */
#define NECK_ORIENT_TAU 1.0f

/* ---- слагаемые бюджета задержки ----------------------------------------- */

/* Всё, кроме возраста параметров, измерено раньше или задано конструкцией.
 * Возраст параметров меряется здесь и сейчас. */
#define LAT_IMU_US   2000        /* опрос IMU смычка 500 Гц (фаза 1) */
#define LAT_AIR_US   1600        /* эфир в одну сторону (фаза 2, замер оборота) */
#define LAT_DMA_US   BLOCK_US    /* готовый блок ждёт, пока доиграет предыдущий */
#define LAT_CODEC_US 1000        /* ES8311 и аналоговый тракт, по документации */

/* ---- состояние ----------------------------------------------------------- */

typedef enum {
    UI_MODE_GAME = 0,
    UI_MODE_MENU,
    UI_MODE_DIAG,
} ui_mode_t;

/* Снимок параметров: пишет слой управления на ядре 0, читает аудиопоток на
 * ядре 1. Единственная точка обмена между ними. */
static iv_params_t s_params;

/* Слой управления: раскладка жестов. Живёт в задаче управления и правится
 * только ею — всё, что приходит извне (кнопка, меню), кладётся в отдельные
 * переменные ниже, а задача управления забирает их к себе каждый проход.
 * Так у структуры один писатель, и гонок за неё нет. */
static iv_control_t s_control;
static volatile bool  s_snap;
static volatile float s_a4_hz      = IV_A4_DEFAULT;
static volatile float s_angle_center;

/* Настройки и состояние меню. Правит задача экрана — она же единственная,
 * кто их сохраняет. */
static iv_settings_t s_settings;
static iv_menu_t     s_menu;

/* Какой экран показан. Пишет только задача экрана; читают задача управления
 * (в меню палец принадлежит меню, а не грифу) и задача кнопок.
 *
 * Кнопки сами ничего не рисуют и не сохраняют, а просят об этом через флаги
 * ниже. Иначе долгое нажатие затирало бы экран из другой задачи посреди
 * чужой передачи по QSPI — и попутно правило бы настройки, которыми владеет
 * задача экрана. */
static volatile ui_mode_t s_mode = UI_MODE_GAME;
static volatile ui_mode_t s_mode_req = UI_MODE_GAME;
static volatile bool      s_save_req;
static volatile bool      s_shutdown_req;

/* Метрики аудиопотока. Пишет аудиозадача раз в секунду, читает экран.
 * Каждое поле — одно слово, рвущийся снимок здесь безобиден. */
static volatile uint32_t s_load_pct;      /* загрузка аудиоядра, % */
static volatile uint32_t s_busy_max_us;   /* худший блок за окно */
static volatile uint32_t s_age_avg_us;    /* средний возраст параметров */
static volatile uint32_t s_age_max_us;
static volatile uint32_t s_param_misses;  /* сколько раз снимок не дался */
static volatile uint8_t  s_string;        /* струна под смычком, 0 = G */
static volatile float    s_neck_tilt;     /* наклон грифа, градусы — на экран */
static volatile float    s_touch_pos = -1.0f; /* ближнее к подставке касание: в лог */
static volatile int      s_touch_x;           /* и его поперечная координата, туда же */
/* Где стоит палец на каждой струне, 0..1; отрицательное — струна открыта.
 * Зажатых бывает две разом, поэтому это массив, а не номер струны: экран
 * рисует зажатую часть каждой, и рисовать он обязан то же, что звучит. */
static volatile float    s_finger_pos[IV_VOICES] = { -1.0f, -1.0f, -1.0f, -1.0f };

/* Размах положения пальца за окно лога. Дрожание касания идёт прямо в высоту
 * ноты, а высота — в длину линии задержки, и услышать его можно как «плавание»
 * тона. Отдельно от s_touch_pos потому, что показывает не значение, а его
 * разброс: одно число, по которому видно, надо ли сглаживать касание. */
static volatile float    s_touch_lo = 2.0f;
static volatile float    s_touch_hi = -1.0f;
/* Срывы касания за окно лога: сколько раз тач терял палец и сколько из них
 * пришлось прикрыть выдержкой. Размах положения такое не ловит — потерянный
 * палец в него просто не попадает, — а слышно именно это. */
static volatile uint32_t s_touch_drops;
static volatile uint32_t s_touch_errs;
/* Размах поперечной координаты за окно. Ею выбирается зажимаемая струна, и
 * если её настоящий диапазон не совпадает с шириной экрана, средние дорожки
 * окажутся недостижимы — а по звуку это выглядит как «зажимаются только
 * крайние струны». Одно число отвечает на это сразу. */
static volatile int      s_across_lo = 0x7fff;
static volatile int      s_across_hi = -1;
static volatile float    s_bow_angle;     /* угол линии движения — для экрана диагностики */
/* Кандидат в середину раскладки: среднее по последним секундам ведения,
 * взвешенное по скорости (см. IV_CENTER_TAU_S). Считает его раскладка, а
 * забирает меню — через эту пару, как и всё остальное между задачами.
 * s_center_ready снят, когда ведения было слишком мало: тогда меню не должно
 * запоминать ничего. */
static volatile float    s_center_cand;
static volatile bool     s_center_ready;
static volatile float    s_center_weight; /* набранное ведение — только на диагностику */
static volatile uint8_t  s_strings;        /* маска задействованных струн — на экран */

/* Заряд. Свой меряется АЦП, смычка приезжает в пакете. */
static volatile uint8_t  s_batt_neck;
static volatile uint16_t s_batt_neck_mv;
static volatile uint8_t  s_batt_bow;
static volatile bool     s_bow_charging;
/* Флаги последнего пакета — для экрана. Одно слово, рвущегося чтения нет. */
static volatile uint8_t  s_bow_flags;

/* ---- звук ---------------------------------------------------------------- */

static void audio_task(void *arg)
{
    (void)arg;
    static float   mono[BLOCK_FRAMES];
    static int16_t stereo[BLOCK_FRAMES * 2];

    /* Синтезатор статический, а не на стеке: линии задержки четырёх струн —
     * это десятки килобайт, стека задачи на них не хватит. */
    static iv_synth_t synth;
    iv_synth_init(&synth, (float)IV_SAMPLE_RATE);

    /* До первого снимка играть нечего: доли нулевые, смычка на струнах нет.
     * Высоты всё же осмысленные — открытые струны, — чтобы первый же снимок
     * не начинался с глиссандо ниоткуда. */
    iv_control_params_t cur = {0};
    for (int k = 0; k < IV_VOICES; ++k) {
        cur.freq_hz[k] = iv_open_hz[k];
    }
    int64_t  last_stamp   = 0;
    int64_t  window_start = esp_timer_get_time();
    uint32_t busy_sum = 0, busy_max = 0, blocks = 0;
    uint32_t age_sum = 0, age_max = 0, age_n = 0;

    while (1) {
        const int64_t t0 = esp_timer_get_time();

        /* Снимок без блокировок. Отказ — не беда: играем блок на прежних
         * параметрах, это 2.7 мс задержки вместо ожидания писателя. */
        iv_control_params_t fresh;
        if (iv_params_read(&s_params, &fresh)) {
            cur = fresh;
        } else {
            s_param_misses++;
        }

        /* Возраст меряем только по свежим данным смычка: иначе в статистику
         * попадёт либо время жизни одного пакета, либо собственные снимки
         * грифа при молчащем смычке. */
        if (cur.from_bow && cur.stamp_us != last_stamp) {
            last_stamp = cur.stamp_us;
            const uint32_t age = (uint32_t)(t0 - cur.stamp_us);
            age_sum += age;
            if (age > age_max) {
                age_max = age;
            }
            age_n++;
        }

        /* Параметры протухли — гасим звук. Это второй рубеж после watchdog'а
         * связи: тот ловит пропажу смычка, этот — остановку слоя управления. */
        float vel = cur.bow_velocity;
        if (t0 - cur.stamp_us > (int64_t)IV_LINK_TIMEOUT_MS * 1000) {
            vel = 0.0f;
        }

        iv_synth_params_t sp = {
            .bow_velocity = vel,
            .bow_force    = cur.bow_force,
            .brightness   = cur.brightness,
        };
        for (int k = 0; k < IV_SYNTH_VOICES; ++k) {
            /* Протухшие параметры снимают смычок со струн так же, как нулевая
             * скорость: доли и скорость обязаны врать одинаково, иначе синтез
             * увидит смычок, стоящий на струне и не двигающийся. */
            sp.bow_share[k] = (vel != 0.0f) ? cur.bow_share[k] : 0.0f;
            sp.freq_hz[k]   = cur.freq_hz[k];
        }
        iv_synth_set_params(&synth, &sp);
        iv_synth_render(&synth, mono, BLOCK_FRAMES);
        s_string = (uint8_t)synth.selected;
        uint8_t mask = 0;
        for (int k = 0; k < IV_SYNTH_VOICES; ++k) {
            if (sp.bow_share[k] > 0.0f) {
                mask |= (uint8_t)(1u << k);
            }
        }
        /* Смычок снят — на экране остаётся последняя струна: пустая полоса
         * читалась бы как «инструмент не работает». */
        s_strings = mask ? mask : (uint8_t)(1u << synth.selected);

        for (int i = 0; i < BLOCK_FRAMES; ++i) {
            const int16_t s = (int16_t)(mono[i] * 32767.0f);
            stereo[2 * i]     = s;
            stereo[2 * i + 1] = s;
        }

        /* Работа кончилась до записи: запись блокируется по устройству, и
         * считать её загрузкой процессора было бы враньём. */
        const uint32_t busy = (uint32_t)(esp_timer_get_time() - t0);
        busy_sum += busy;
        if (busy > busy_max) {
            busy_max = busy;
        }
        blocks++;

        if (bsp_audio_write(stereo, BLOCK_FRAMES) != ESP_OK) {
            ESP_LOGW(TAG, "запись в звук не прошла");
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        const int64_t now = esp_timer_get_time();
        if (now - window_start >= 1000000) {
            s_load_pct    = blocks ? busy_sum * 100 / (blocks * BLOCK_US) : 0;
            s_busy_max_us = busy_max;
            s_age_avg_us  = age_n ? age_sum / age_n : 0;
            s_age_max_us  = age_max;

            window_start = now;
            busy_sum = busy_max = blocks = 0;
            age_sum  = age_max  = age_n  = 0;
        }
    }
}

/* ---- слой управления ------------------------------------------------------ */

/* Экранное касание → касание раскладки: вдоль грифа 0..1 от порожка к
 * подставке, поперёк 0..1 от G к E.
 *
 * Отсчёт вдоль идёт не от края экрана, а от IV_UI_FB_TOP — от той же границы,
 * ниже которой рисуется гриф. Верхняя полоска занята панелью, и если бы
 * касание считалось от нуля, экран показывал бы одно, а звучало бы другое.
 * Потеря невелика и приходится на самый безобидный конец: у порожка нота и так
 * берётся снятым пальцем — это открытая струна. */
static iv_touch_t touch_of_point(const bsp_touch_point_t *p)
{
    iv_touch_t t;
    t.pos = (p->y <= IV_UI_FB_TOP)
                ? 0.0f
                : (float)(p->y - IV_UI_FB_TOP) / (float)(BSP_LCD_V_RES - 1 - IV_UI_FB_TOP);
    t.across = (float)p->x / (float)(BSP_LCD_H_RES - 1);
    return t;
}

/* Снимок всех касаний. Пустые слоты помечены pos < 0.
 *
 * Больше IV_TOUCH_MAX точек тач не отдаёт (BSP_TOUCH_MAX_POINTS), так что
 * выбирать тут не из чего и отбрасывать нечего: какие пришли, те и играют. */
static void touch_snapshot(const bsp_touch_state_t *t, iv_touch_t out[IV_TOUCH_MAX])
{
    for (int i = 0; i < IV_TOUCH_MAX; ++i) {
        out[i].pos    = -1.0f;
        out[i].across = -1.0f;
    }
    for (uint8_t i = 0; i < t->count && i < IV_TOUCH_MAX; ++i) {
        out[i] = touch_of_point(&t->points[i]);
    }
}

/* Сколько касаний в снимке. */
static int touch_count(const iv_touch_t t[IV_TOUCH_MAX])
{
    int n = 0;
    for (int i = 0; i < IV_TOUCH_MAX; ++i) {
        if (t[i].pos >= 0.0f) {
            n++;
        }
    }
    return n;
}

static void control_task(void *arg)
{
    (void)arg;

    iv_orient_t neck;
    iv_orient_init(&neck, (float)IV_BOW_RATE_HZ / NECK_IMU_EVERY, NECK_ORIENT_TAU);
    int imu_tick = 0;

    /* Последний снимок касаний и когда его видели: из них выдержка. */
    iv_touch_t held[IV_TOUCH_MAX];
    int64_t    held_us = 0;
    for (int i = 0; i < IV_TOUCH_MAX; ++i) {
        held[i].pos    = -1.0f;
        held[i].across = -1.0f;
    }

    while (1) {
        /* Просыпаемся по приходу пакета, а не по таймеру: иначе снимок в
         * среднем полпериода ждал бы уже готовые данные, и эти миллисекунды
         * ушли бы прямо в сквозную задержку. Таймаут держит опрос пальца
         * живым, когда смычок молчит. */
        iv_link_neck_wait(1000 / IV_BOW_RATE_HZ);

        bsp_touch_state_t touch = {0};
        if (bsp_touch_read(&touch) != ESP_OK) {
            touch.count = 0;
            s_touch_errs++;
        }
        iv_touch_t touches[IV_TOUCH_MAX];
        touch_snapshot(&touch, touches);

        /* Выдержка на пропаже: см. TOUCH_HOLD_MS. Держится снимок целиком, а
         * не отдельный палец: тач не обещает, что палец останется в том же
         * слоте, и сшивать слоты между кадрами было бы гаданием. Пропажей
         * считается уменьшение числа касаний — снялся один из двух или
         * единственный, разницы нет. */
        const int64_t now_us = esp_timer_get_time();
        if (touch_count(touches) < touch_count(held)
            && now_us - held_us < (int64_t)TOUCH_HOLD_MS * 1000) {
            memcpy(touches, held, sizeof(touches)); /* палец, скорее всего, на месте */
            s_touch_drops++;
        } else {
            memcpy(held, touches, sizeof(held));
            held_us = now_us;
        }

        /* В лог идёт ближнее к подставке касание: на зажатой струне слышно
         * именно его, и дрожание меряется по нему. */
        float lead_pos = -1.0f, lead_across = 0.0f;
        for (int i = 0; i < IV_TOUCH_MAX; ++i) {
            if (touches[i].pos > lead_pos) {
                lead_pos    = touches[i].pos;
                lead_across = touches[i].across;
            }
        }
        const int lead_x = (int)(lead_across * (float)(BSP_LCD_H_RES - 1));
        s_touch_pos = lead_pos;
        s_touch_x   = lead_x;
        if (lead_pos >= 0.0f) {
            if (lead_pos < s_touch_lo) { s_touch_lo = lead_pos; }
            if (lead_pos > s_touch_hi) { s_touch_hi = lead_pos; }
            if (lead_x < s_across_lo) { s_across_lo = lead_x; }
            if (lead_x > s_across_hi) { s_across_hi = lead_x; }
        }

        if (++imu_tick >= NECK_IMU_EVERY) {
            imu_tick = 0;
            bsp_imu_data_t imu;
            if (bsp_imu_read(&imu) == ESP_OK) {
                const float a[3] = { imu.ax, imu.ay, imu.az };
                const float g[3] = { imu.gx, imu.gy, imu.gz };
                iv_orient_update(&neck, a, g);
                s_neck_tilt = neck.pitch;
            }
        }

        iv_control_in_t in;
        memset(&in, 0, sizeof(in));
        for (int i = 0; i < IV_TOUCH_MAX; ++i) {
            /* В меню палец принадлежит меню: играть по нему было бы нельзя —
             * пользователь целится в пункт, а не в ноту. */
            if (s_mode == UI_MODE_GAME) {
                in.touch[i] = touches[i];
            } else {
                in.touch[i].pos    = -1.0f;
                in.touch[i].across = -1.0f;
            }
        }
        in.neck_tilt = neck.pitch;
        in.stamp_us  = esp_timer_get_time();

        iv_bow_packet_t p;
        int64_t rx_us = 0;
        if (iv_link_neck_online() && iv_link_neck_last(&p, NULL, &rx_us)) {
            /* Пока смычок калибруется, его данным верить нельзя — молчим.
             * Пока он дремлет — тоже: телеметрия в таких пакетах снята на
             * 25 Гц и означает только «лежит». */
            in.have_bow     = !(p.flags & (IV_FLAG_CALIBRATING | IV_FLAG_IDLE));
            in.bow_velocity = iv_unpack_vel(p.speed);
            /* Угол линии движения смычка к горизонту. Наклон корпуса (p.tilt)
             * гриф не читает намеренно: как смычок держат, звука не касается —
             * только то, что им делают. */
            in.bow_angle    = iv_unpack_angle(p.move_angle);
            in.bow_at_rest  = (p.flags & IV_FLAG_AT_REST) != 0;

            s_bow_angle    = in.bow_angle;
            s_bow_flags    = p.flags;
            s_batt_bow     = p.batt;
            s_bow_charging = (p.flags & IV_FLAG_CHARGING) != 0;

            /* Метка времени — момент приёма пакета, а не публикации: именно от
             * неё считается сквозная задержка, и приписывать себе чужие
             * миллисекунды нельзя. */
            in.stamp_us = rx_us;
            in.from_bow = true;
        }

        /* Настройки забираются сюда, а не правятся снаружи: у структуры
         * раскладки должен быть один писатель. */
        iv_control_set_snap(&s_control, s_snap);
        iv_control_set_tuning(&s_control, s_a4_hz);
        iv_control_center_angle(&s_control, s_angle_center);

        iv_control_params_t cp;
        iv_control_update(&s_control, &in, &cp);
        iv_params_publish(&s_params, &cp);

        float cand = 0.0f;
        s_center_ready  = iv_control_center_candidate(&s_control, &cand);
        s_center_cand   = cand;
        s_center_weight = iv_control_center_weight(&s_control);
        for (int k = 0; k < IV_VOICES; ++k) {
            s_finger_pos[k] = cp.finger_pos[k];
        }
    }
}

/* ---- настройки ------------------------------------------------------------- */

/* Применяет настройки к железу и к раскладке. Раскладке — через переменные,
 * которые задача управления заберёт сама (см. control_task). */
static void apply_settings(const iv_settings_t *s)
{
    bsp_audio_set_volume(s->volume);
    bsp_display_set_brightness(s->brightness);
    s_snap        = s->snap;
    s_a4_hz       = s->a4_hz;
    s_angle_center = s->angle_center;
}

/* ---- экран диагностики ------------------------------------------------------ */

static uint32_t end_to_end_us(void)
{
    return LAT_IMU_US + LAT_AIR_US + s_age_avg_us + LAT_DMA_US + LAT_CODEC_US;
}

static void draw_diag(void)
{
    iv_link_stats_t st;
    bsp_audio_stats_t au;
    /* Снимок может не даться — тогда рисуем нули, а не содержимое стека. */
    iv_control_params_t cp = {0};

    iv_link_neck_stats(&st);
    bsp_audio_stats(&au);
    iv_params_read(&s_params, &cp);

    iv_ui_row(0, IV_UI_YELLOW, IV_UI_BLACK, "DIAG");

    if (st.online) {
        iv_ui_row(2, IV_UI_BLACK, IV_UI_GREEN, " LINK OK");
    } else {
        iv_ui_row(2, IV_UI_WHITE, IV_UI_RED, " NO LINK");
    }

    iv_ui_row(4, s_load_pct > 40 ? IV_UI_RED : IV_UI_WHITE, IV_UI_BLACK,
              "LOAD %" PRIu32 "%%", s_load_pct);
    iv_ui_row(5, IV_UI_GRAY, IV_UI_BLACK, "BLK %" PRIu32 "US", s_busy_max_us);
    iv_ui_row(6, au.underruns ? IV_UI_RED : IV_UI_WHITE, IV_UI_BLACK,
              "UNDER %" PRIu32, au.underruns);
    iv_ui_row(7, IV_UI_WHITE, IV_UI_BLACK, "PARAM %.1fMS", s_age_avg_us / 1000.0);
    iv_ui_row(8, end_to_end_us() > 20000 ? IV_UI_RED : IV_UI_GREEN, IV_UI_BLACK,
              "E2E %.1fMS", end_to_end_us() / 1000.0);

    /* Задействованные струны, а не одна: на границе их две, и увидеть это
     * важнее, чем сэкономить строку. */
    char strings[IV_VOICES + 1];
    int  ns = 0;
    for (int k = 0; k < IV_VOICES; ++k) {
        if (cp.bow_share[k] > 0.0f) {
            strings[ns++] = "GDAE"[k];
        }
    }
    if (ns == 0) {
        strings[ns++] = '-'; /* смычок снят: не звучит ни одна */
    }
    strings[ns] = '\0';

    const int dom = cp.string & (IV_VOICES - 1);
    iv_ui_row(10, IV_UI_CYAN, IV_UI_BLACK, "NOTE %.0fHZ", (double)cp.freq_hz[dom]);
    iv_ui_row(11, IV_UI_CYAN, IV_UI_BLACK, "BOW %.2f", (double)cp.bow_velocity);
    iv_ui_row(12, IV_UI_CYAN, IV_UI_BLACK, "ANGLE %+.1f", (double)s_bow_angle);
    /* Центр раскладки — прямо под углом, потому что струну выбирает их
     * РАЗНОСТЬ, и порознь эти два числа ничего не значат. Красным, когда центр
     * уехал больше чем на струну: в этих краях «ведёшь горизонтально, звучит
     * крайняя» выглядит точно так же, как сбитый угол смычка, и различить их
     * иначе нельзя — а живёт центр в NVS и перепрошивку переживает. */
    /* Слева — запомненный центр, справа — тот, что запомнится по нажатию.
     * Прочерк значит «ведения мало, нажимать рано»: без этого отказ выглядит
     * как «нажал, и ничего не произошло», и непонятно, в чём дело. Запомненный
     * центр сам по себе НЕ следит за смычком — он меняется только нажатием, и
     * две цифры рядом показывают это нагляднее любой подписи. */
    char cand[8];
    if (s_center_ready) {
        snprintf(cand, sizeof(cand), "%+.0f", (double)s_center_cand);
    } else {
        snprintf(cand, sizeof(cand), "--");
    }
    iv_ui_row(13, fabsf(s_angle_center) >= IV_ANGLE_PER_STRING ? IV_UI_RED
                                                               : IV_UI_CYAN,
              IV_UI_BLACK, "CENTER %+.0f>%s", (double)s_angle_center, cand);
    iv_ui_row(14, IV_UI_CYAN, IV_UI_BLACK, "STRING %s", strings);
    /* Зажатые струны, по одной букве на каждую: их бывает две, по касанию на
     * струну. Прочерк — пальцы сняты, всё открыто. */
    char fingers[IV_VOICES + 1];
    int  nf = 0;
    for (int k = 0; k < IV_VOICES; ++k) {
        if (cp.finger_pos[k] >= 0.0f) {
            fingers[nf++] = "GDAE"[k];
        }
    }
    if (nf == 0) {
        fingers[nf++] = '-';
    }
    fingers[nf] = '\0';
    iv_ui_row(15, IV_UI_CYAN, IV_UI_BLACK, "FINGER %s", fingers);
    iv_ui_row(16, s_snap ? IV_UI_GREEN : IV_UI_GRAY, IV_UI_BLACK,
              s_snap ? "SNAP ON" : "SNAP OFF");

    iv_ui_row(17, IV_UI_GRAY, IV_UI_BLACK, "RX %" PRIu32, st.received);
    iv_ui_row(18, IV_UI_GRAY, IV_UI_BLACK, "LOSS %.3f%%",
              (double)(iv_link_loss_ratio(&st) * 100.0f));

    iv_ui_row(19, IV_UI_GRAY, IV_UI_BLACK, "NBAT %umV", s_batt_neck_mv);
    iv_ui_row(20, IV_UI_GRAY, IV_UI_BLACK, "BBAT %u%%", s_batt_bow);

    iv_ui_row(22, IV_UI_GRAY, IV_UI_BLACK, "TAP=MENU");
    iv_ui_present();
}

static void log_stats(void)
{
    iv_link_stats_t st;
    bsp_audio_stats_t au;
    iv_link_neck_stats(&st);
    bsp_audio_stats(&au);

    ESP_LOGI(TAG, "звук: загрузка %" PRIu32 " %%, худший блок %" PRIu32
                  " мкс из %d, недокормов %" PRIu32 ", блоков %" PRIu32,
             s_load_pct, s_busy_max_us, BLOCK_US, au.underruns, au.written);

    /* Рядом со звуком намеренно: если недокормы появляются вместе с рослыми
     * кадрами, виноват экран, и это видно одним взглядом. */
    uint32_t ui_us = 0, ui_rows = 0, ui_n = 0;
    iv_ui_present_stats(&ui_us, &ui_rows, &ui_n);
    ESP_LOGI(TAG, "экран: %" PRIu32 " кадров за окно, последний %" PRIu32
                  " строк за %" PRIu32 " мкс",
             ui_n, ui_rows, ui_us);

    /* Дрожание касания в полутонах: столько «гуляла» высота ноты за окно, если
     * палец лежал неподвижно. Десятые доли полутона слышны как плавание тона.
     * Границы сбрасываются здесь же — иначе показывался бы размах за всё время
     * с включения, а не за окно. */
    const float t_lo = s_touch_lo, t_hi = s_touch_hi;
    s_touch_lo = 2.0f;
    s_touch_hi = -1.0f;
    const uint32_t drops = s_touch_drops, errs = s_touch_errs;
    s_touch_drops = 0;
    s_touch_errs = 0;
    const int a_lo = s_across_lo, a_hi = s_across_hi;
    s_across_lo = 0x7fff;
    s_across_hi = -1;
    if (t_hi >= 0.0f) {
        ESP_LOGI(TAG, "касание: вдоль размах %.3f = %.2f полутона, "
                      "срывов %" PRIu32 ", ошибок чтения %" PRIu32,
                 (double)(t_hi - t_lo),
                 (double)((t_hi - t_lo) * IV_NECK_SEMITONES), drops, errs);
        /* Поперёк — в точках экрана и в дорожках: сразу видно, дотягивается ли
         * палец до средних струн вообще. */
        ESP_LOGI(TAG, "касание: поперёк %d..%d из 0..%d → дорожки %c..%c",
                 a_lo, a_hi, BSP_LCD_H_RES - 1,
                 "GDAE"[(a_lo * IV_VOICES / BSP_LCD_H_RES) & 3],
                 "GDAE"[(a_hi * IV_VOICES / BSP_LCD_H_RES) & 3]);
    } else {
        ESP_LOGI(TAG, "касание: не было, ошибок чтения %" PRIu32, errs);
    }

    ESP_LOGI(TAG, "задержка: IMU %.1f + эфир %.1f + параметры %.1f (макс %.1f)"
                  " + DMA %.1f + кодек %.1f = %.1f мс",
             LAT_IMU_US / 1000.0, LAT_AIR_US / 1000.0,
             s_age_avg_us / 1000.0, s_age_max_us / 1000.0,
             LAT_DMA_US / 1000.0, LAT_CODEC_US / 1000.0,
             end_to_end_us() / 1000.0);

    ESP_LOGI(TAG, "связь: принято %" PRIu32 ", потеряно %" PRIu32 " (%.3f %%), "
                  "снимок не дался %" PRIu32 " раз",
             st.received, st.lost, (double)(iv_link_loss_ratio(&st) * 100.0f),
             s_param_misses);

    iv_control_params_t cp = {0};
    iv_params_read(&s_params, &cp);
    /* Зажатые струны буквами: их бывает две, по касанию на струну. */
    char fingers[IV_VOICES + 1];
    int  nf = 0;
    for (int k = 0; k < IV_VOICES; ++k) {
        if (cp.finger_pos[k] >= 0.0f) {
            fingers[nf++] = "GDAE"[k];
        }
    }
    if (nf == 0) {
        fingers[nf++] = '-'; /* пальцы сняты: все струны открыты */
    }
    fingers[nf] = '\0';
    /* Центр раскладки печатается рядом с углом, и не для полноты: доли
     * считаются от РАЗНОСТИ этих двух, а хранится центр в NVS и переживает
     * перепрошивку. Сбитый центр выглядит в логе точно как сбитый угол —
     * «ведёшь горизонтально, звучит крайняя струна», — и без него в строке
     * эти два случая неразличимы. */
    ESP_LOGI(TAG, "жест: линия движения %+.1f° (центр %+.1f°) → "
                  "доли G%.2f D%.2f A%.2f E%.2f, "
                  "пальцы на %s, нота %.1f Гц, смычок %.2f, "
                  "наклон грифа %+.1f° (яркость %.2f), полутоны %s",
             (double)s_bow_angle, (double)s_angle_center,
             (double)cp.bow_share[0], (double)cp.bow_share[1],
             (double)cp.bow_share[2], (double)cp.bow_share[3],
             fingers,
             (double)cp.freq_hz[cp.string & (IV_VOICES - 1)],
             (double)cp.bow_velocity,
             (double)s_neck_tilt, (double)cp.brightness,
             s_snap ? "вкл" : "выкл");

    ESP_LOGI(TAG, "питание: гриф %u %% (%u мВ), смычок %u %%%s",
             s_batt_neck, s_batt_neck_mv, s_batt_bow,
             s_bow_charging ? " (заряжается)" : "");
}

/* ---- экран ----------------------------------------------------------------- */

static void switch_mode(ui_mode_t mode)
{
    if (s_mode == mode) {
        return;
    }
    s_mode     = mode;
    s_mode_req = mode;
    /* Экран, на который переходим, забывает нарисованное: под ним лежит чужая
     * картинка, и «изменилось только это» больше не работает. */
    iv_ui_game_reset();
    iv_ui_menu_reset();
    if (mode == UI_MODE_DIAG) {
        iv_ui_clear(IV_UI_BLACK);
    }
}

/* Сохранение настроек. Возвращается быстро, если ничего не менялось: запись
 * во flash стоит миллисекунд, а звать это приходится и на выключении. */
static void save_settings(void)
{
    iv_settings_sanitize(&s_settings);
    iv_settings_save(&s_settings);
}

/* Одно касание меню: разбор координат и исполнение действия. */
static void menu_apply(int x, int y)
{
    switch (iv_menu_touch(&s_menu, x, y, &s_settings)) {
    case IV_MENU_CHANGED:
        apply_settings(&s_settings);
        break;

    case IV_MENU_DO_CENTER:
        /* Центр берётся из последнего услышанного угла движения: постановка
         * руки у всех своя, и без этого крайние струны могут быть недостижимы.
         * Угол последний, а не текущий, и это важно: жать надо, поводив
         * смычком так, как собираешься играть, — у стоящего смычка угла нет. */
        /* Берётся НЕ последний услышанный угол, а среднее по последним
         * секундам ведения (см. IV_CENTER_TAU_S). Последним движением перед
         * тем, как дотянуться до экрана, бывает не ведение, а то, как смычок
         * отложили, — и раньше в центр попадало именно оно.
         *
         * Отказ виден без кабеля: значение пункта — сам центр, и если он не
         * изменился после нажатия, значит поводить смычком не успели. */
        if (!s_center_ready) {
            ESP_LOGW(TAG, "середина раскладки: не берусь, оставляю прежние "
                          "%+.1f°. Набрано ведения %.2f при пороге %.2f — "
                          "если веса хватает, значит движений в памяти два "
                          "разных и какое из них ведение, неясно. Поводи "
                          "смычком так, как играешь, и нажми ещё раз",
                     (double)s_settings.angle_center,
                     (double)s_center_weight, (double)IV_CENTER_MIN_WEIGHT);
            break;
        }
        s_settings.angle_center = s_center_cand;
        iv_settings_sanitize(&s_settings);
        apply_settings(&s_settings);
        ESP_LOGI(TAG, "середина раскладки: %+.1f° (среднее по ведению)",
                 (double)s_settings.angle_center);
        break;

    case IV_MENU_DO_FORGET:
        iv_link_neck_forget();
        break;

    case IV_MENU_DO_DIAG:
        switch_mode(UI_MODE_DIAG);
        break;

    case IV_MENU_DO_BACK:
        save_settings();
        switch_mode(UI_MODE_GAME);
        break;

    default:
        break;
    }
}

/* Опрос тача для меню. Работает по фронту нажатия, а не по факту касания:
 * иначе прижатый палец менял бы значение двадцать пять раз в секунду.
 * Автоповтор включается после паузы — как у клавиатуры, и по той же причине:
 * иначе громкость от 20 до 100 набиралась бы шестнадцатью касаниями. */
static void menu_touch_tick(void)
{
    static bool      held;
    static int       frames;
    static ui_mode_t seen_mode = UI_MODE_GAME;

    /* Смена экрана обрывает касание, даже если палец не отрывали. Иначе
     * долгое нажатие на диагностике сначала уводило бы в меню, а потом
     * автоповтором нажимало бы в нём то, что оказалось под пальцем. */
    if (s_mode != seen_mode) {
        seen_mode = s_mode;
        held   = false;
        frames = 0;
        return;
    }

    bsp_touch_state_t t = {0};
    bsp_touch_read(&t);

    if (t.count == 0) {
        held = false;
        frames = 0;
        return;
    }

    const int x = t.points[0].x;
    const int y = t.points[0].y;

    bool act = false;
    if (!held) {
        held = true;
        frames = 0;
        act = true;
    } else {
        frames++;
        if (frames >= MENU_REPEAT_DELAY_FRAMES
            && (frames - MENU_REPEAT_DELAY_FRAMES) % MENU_REPEAT_EVERY_FRAMES == 0) {
            act = true;
        }
    }

    if (act) {
        if (s_mode == UI_MODE_DIAG) {
            /* С диагностики любое касание возвращает в меню: своих органов
             * управления у неё нет и не нужно. */
            switch_mode(UI_MODE_MENU);
        } else {
            menu_apply(x, y);
        }
    }
}

static void battery_tick(void)
{
    uint16_t mv = 0;
    if (bsp_battery_read_mv(&mv)) {
        s_batt_neck_mv = mv;
        s_batt_neck    = iv_batt_percent(mv);
    }
}

static void draw_game(void)
{
    /* Снимок параметров здесь больше не нужен: с экрана ушли и нота, и полоса
     * ведения, а всё оставшееся приходит своими переменными. Лишнее чтение
     * seqlock — это лишняя попытка отобрать снимок у писателя. */

    /* Что показать про смычок. Дрёма — отдельное состояние: смычок исправен и
     * на связи, но играть по нему нельзя, и молчание в этот момент нормально.
     * Сваливать её в «готов» значило бы повторить ту же ошибку, из-за которой
     * инструмент молчал, а понять это было нельзя. */
    iv_bow_status_t bow = IV_BOW_OFFLINE;
    if (iv_link_neck_online()) {
        const uint8_t f = s_bow_flags;
        bow = (f & IV_FLAG_CALIBRATING) ? IV_BOW_CALIBRATING
            : (f & IV_FLAG_IDLE)        ? IV_BOW_IDLE
                                        : IV_BOW_READY;
    }

    iv_ui_game_t g = {
        .bow          = bow,
        .batt_neck    = s_batt_neck,
        .batt_bow     = s_batt_bow,
        .bow_charging = s_bow_charging,
        .strings      = s_strings,
        .snap         = s_snap,
    };
    for (int k = 0; k < IV_VOICES; ++k) {
        g.finger_pos[k] = s_finger_pos[k];
    }
    iv_ui_game_draw(&g);
}

static void shutdown_now(void)
{
    ESP_LOGI(TAG, "долгое нажатие питания — выключаюсь");
    /* Порядок важен: сперва картинка (иначе выключение выглядит зависанием),
     * потом настройки (иначе они не переживут выключение), и только потом
     * снятие защёлки. */
    iv_ui_shutdown_screen();
    save_settings();
    bsp_power_off();

    /* Дальше идти некуда. Защёлка снята, но плата гаснет только в момент
     * отпускания кнопки — пока её держат, питание подаёт она сама. Уходим в
     * бесконечное ожидание, чтобы не перерисовывать экран и не переписывать
     * настройки в эти секунды. */
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}

static void ui_task(void *arg)
{
    (void)arg;
    TickType_t next = xTaskGetTickCount();
    int  frame = 0;
    bool skip  = false;

    iv_ui_game_reset();

    while (1) {
        /* Просьбы от кнопок исполняются здесь: рисовать и сохранять имеет
         * право только эта задача. */
        if (s_shutdown_req) {
            shutdown_now();
        }
        if (s_save_req) {
            s_save_req = false;
            save_settings();
        }
        if (s_mode_req != s_mode) {
            switch_mode(s_mode_req);
        }

        /* Кнопка полутонов правит s_snap, а не настройки: у настроек один
         * владелец, и это здешняя задача. Забираем значение к себе — тогда
         * переключённое кнопкой переживёт выключение наравне с остальным. */
        s_settings.snap = s_snap;

        if (s_mode != UI_MODE_GAME) {
            menu_touch_tick();
        }

        if ((frame % BATT_PERIOD_FRAMES) == 0) {
            battery_tick();
        }

        /* Экран деградирует первым. Пропущенный кадр не видно, а щелчок в
         * звуке слышно всегда — приоритет между ними расставлен здесь. */
        skip = (s_load_pct > UI_YIELD_LOAD_PCT) && !skip;
        if (!skip) {
            switch (s_mode) {
            case UI_MODE_GAME:
                if ((frame % GAME_EVERY) == 0) {
                    draw_game();
                }
                break;
            case UI_MODE_MENU: iv_ui_menu_draw(&s_menu, &s_settings); break;
            case UI_MODE_DIAG:
                /* Диагностика — единственный экран, который рисуется целиком,
                 * поэтому она одна и прореживается по времени, а не по
                 * изменениям. */
                if ((frame % DIAG_EVERY) == 0) {
                    draw_diag();
                }
                break;
            }
        }

        if (++frame >= LOG_EVERY) {
            frame = 0;
            log_stats();
        }
        vTaskDelayUntil(&next, pdMS_TO_TICKS(1000 / UI_HZ));
    }
}

/* ---- кнопки --------------------------------------------------------------- */

static void button_task(void *arg)
{
    (void)arg;
    TickType_t next = xTaskGetTickCount();

    while (1) {
        if (bsp_button_poll(BSP_BTN_PWR) == BSP_BTN_EVENT_LONG) {
            s_shutdown_req = true;
        }

        switch (bsp_button_poll(BSP_BTN_BOOT)) {
        case BSP_BTN_EVENT_SHORT:
            /* Привязка к полутонам. Безладово играть в ноту трудно — полутон
             * это семь миллиметров экрана, — поэтому переключатель вынесен на
             * кнопку, а не спрятан в настройки. В меню он тоже есть: это одна
             * и та же величина, показанная с двух сторон. */
            s_snap = !s_snap;
            ESP_LOGI(TAG, "привязка к полутонам %s", s_snap ? "включена" : "выключена");
            break;

        case BSP_BTN_EVENT_LONG:
            /* Долгое нажатие водит между игрой и меню. Забыть спаренный
             * смычок теперь можно из меню — кнопке этого больше не поручают:
             * случайное долгое нажатие разрывало пару молча. */
            if (s_mode == UI_MODE_GAME) {
                s_mode_req = UI_MODE_MENU;
            } else {
                s_save_req = true;
                s_mode_req = UI_MODE_GAME;
            }
            break;

        default:
            break;
        }

        vTaskDelayUntil(&next, pdMS_TO_TICKS(BSP_BUTTON_POLL_MS));
    }
}

/* ---- запуск --------------------------------------------------------------- */

void app_main(void)
{
    /* Первым делом — плата: bsp_board_init() взводит защёлку питания, без
     * которой устройство погаснет, как только отпустят кнопку. */
    ESP_ERROR_CHECK(bsp_board_init());

    esp_chip_info_t chip;
    esp_chip_info(&chip);
    ESP_LOGI(TAG, "воображаемая скрипка — гриф, фаза 6");
    ESP_LOGI(TAG, "чип: %s, ядер: %d, ревизия %d, PSRAM %u КБ",
             CONFIG_IDF_TARGET, chip.cores, chip.revision,
             (unsigned)(esp_psram_get_size() / 1024));
    ESP_LOGI(TAG, "звук: %d Гц, блок %d кадров (%d мкс), двойная буферизация",
             IV_SAMPLE_RATE, IV_BLOCK_SAMPLES, BLOCK_US);
    ESP_LOGI(TAG, "синтез: волновод, %d струны, %u КБ состояния",
             IV_SYNTH_VOICES, (unsigned)(sizeof(iv_synth_t) / 1024));

    iv_control_init(&s_control);
    iv_menu_init(&s_menu);
    iv_settings_defaults(&s_settings);

    iv_control_params_t start = {
        .bow_velocity = 0.0f,
        .bow_force    = IV_BOW_FORCE,
        .brightness   = 0.5f,
        .string       = 2,
        .stamp_us     = 0, /* заведомо протухшие: до первого пакета звука нет */
    };
    for (int k = 0; k < IV_VOICES; ++k) {
        start.bow_share[k] = 0.0f; /* смычка на струнах нет, пока он не заговорил */
        start.freq_hz[k]   = iv_open_hz[k];
    }
    iv_params_init(&s_params, &start);

    ESP_ERROR_CHECK(bsp_display_init());
    bsp_display_backlight(true);
    ESP_ERROR_CHECK(iv_ui_init());

    ESP_ERROR_CHECK(bsp_imu_init());
    ESP_ERROR_CHECK(bsp_battery_init());
    ESP_ERROR_CHECK(bsp_audio_init(IV_SAMPLE_RATE));
    ESP_ERROR_CHECK(bsp_button_init());
    ESP_ERROR_CHECK(iv_link_init(IV_LINK_ROLE_NECK)); /* он же поднимает NVS */

    /* Настройки читаются после iv_link_init: NVS поднимает он, а заводить
     * второй источник правды на инициализацию flash незачем. Применять их
     * всё равно можно только сейчас — до кодека и подсветки громкость с
     * яркостью некуда деть. */
    iv_settings_load(&s_settings);
    apply_settings(&s_settings);

    /* Ядро 1 отдано звуку целиком, всё остальное живёт на ядре 0. */
    xTaskCreatePinnedToCore(audio_task,   "audio",   4096, NULL, 23, NULL, 1);
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL,  9, NULL, 0);
    xTaskCreatePinnedToCore(ui_task,      "ui",      4096, NULL,  4, NULL, 0);
    xTaskCreatePinnedToCore(button_task,  "button",  3072, NULL,  2, NULL, 0);

    ESP_LOGI(TAG, "аудиотракт запущен");
}
