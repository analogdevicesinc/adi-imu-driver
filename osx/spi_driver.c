/*******************************************************************************
 *   @file   spi_driver.c
 *   @brief  Spidev Driver interface
 *   @author Sundar Palani <sundar.palani@analog.com>
********************************************************************************/

#include <unistd.h>        // Needed for SPI port
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>

#include "spi_driver.h"

#ifdef WIN32
#include <windows.h>
#elif _POSIX_C_SOURCE >= 199309L
#include <time.h>   // for nanosleep
#else
#include <unistd.h> // for usleep
#endif
void delay_MicroSeconds (uint32_t microseconds)
{
#ifdef WIN32
    Sleep(microseconds); // For windows, not tested. Might not work.
#elif _POSIX_C_SOURCE >= 199309L
    struct timespec ts;
    ts.tv_sec = microseconds / 1000000;
    ts.tv_nsec = (microseconds % 1000000) * 1000;
    nanosleep(&ts, NULL);
#else
    usleep(microseconds);
#endif
}


int spi_Init(adi_imu_Device_t *pDevice)
{
    pDevice->status = 0;

    return -1;
}

int spi_ReadWrite(adi_imu_Device_t *pDevice, uint8_t *txBuf, uint8_t *rxBuf, uint32_t xferLen, uint32_t numXfers, uint32_t numRepeats, uint32_t enRepeatTx)
{
    if (pDevice->status == 0) DEBUG_PRINT_RET(-1, "Error: Device not initialized properly.\n");

    return -1;
}
