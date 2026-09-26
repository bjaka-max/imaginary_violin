/* Внутренние связи между файлами BSP смычка. Наружу не выставляется. */
#ifndef BSP_BOW_INTERNAL_H
#define BSP_BOW_INTERNAL_H

#include "driver/i2c_master.h"

/* Общая шина: BMI270 и M5PM1 сидят на ней вдвоём. */
extern i2c_master_bus_handle_t bsp_bow_i2c;

#endif /* BSP_BOW_INTERNAL_H */
