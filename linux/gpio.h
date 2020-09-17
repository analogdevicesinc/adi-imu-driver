/*******************************************************************************
 *   @file   gpio.h
 *   @brief  Header file of gpio Driver.
 *   @author Sundar Palani <sundar.palani@analog.com>
********************************************************************************/

#ifndef __GPIO_DRIVER_H_
#define __GPIO_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

int gpio_export(const char* gpio);
int gpio_unexport(const char* gpio);
int gpio_set_direction(const char* gpio, const char* direction);
int gpio_read(const char* gpio, char* val);
int gpio_write(const char* gpio, const char* val);

#ifdef __cplusplus
}
#endif
#endif
