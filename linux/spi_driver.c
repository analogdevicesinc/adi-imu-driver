/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		spi_driver.c
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Driver interface for linux SPI driver.
 **/


#include <unistd.h>        // Needed for SPI port
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>            // Needed for SPI port
#include <sys/ioctl.h>        // Needed for SPI port
#include <linux/types.h>
#include <linux/spi/spidev.h> // Needed for SPI port
#include <time.h>
#include <sys/stat.h>
#include <sys/time.h>

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

    int spi_fd = open(pDevice->spiDev, O_RDWR);
    if (spi_fd < 0) DEBUG_PRINT_RET(-1, "Error: failed to open spi device %s\n", pDevice->spiDev);
    
    pDevice->spiHandle = (adi_imu_DevHandler_t) spi_fd;

    int ret = 0;
    /*
    * spi mode
    */
    ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &pDevice->spiMode);
    if (ret == -1) DEBUG_PRINT_RET(-1, "Error: Failed to set spi mode (wr). Error: %s\n", strerror(errno));

    ret = ioctl(spi_fd, SPI_IOC_RD_MODE, &pDevice->spiMode);
    if (ret == -1) DEBUG_PRINT_RET(-1, "Error: Failed to set spi mode (rd). Error: %s\n", strerror(errno));

    /*
    * bits per word
    */
    ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &pDevice->spiBitsPerWord);
    if (ret == -1) DEBUG_PRINT_RET(-1, "Error: Failed to set bits per word (wr). Error: %s\n", strerror(errno));

    ret = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &pDevice->spiBitsPerWord);
    if (ret == -1) DEBUG_PRINT_RET(-1, "Error: Failed to set bits per word (rd). Error: %s\n", strerror(errno));

    /*
    * max speed hz
    */
    ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &pDevice->spiSpeed);
    if (ret == -1) DEBUG_PRINT_RET(-1, "Error: Failed to set max speed hz (wr). Error: %s\n", strerror(errno));

    ret = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &pDevice->spiSpeed);
    if (ret == -1) DEBUG_PRINT_RET(-1, "Error: Failed to set max speed hz (rd). Error: %s\n", strerror(errno));

    /* set status to SUCCESS after successful initialization */
    pDevice->status = 1;


    DEBUG_PRINT("SPI device: %s\n", pDevice->spiDev);
    DEBUG_PRINT("SPI mode: %d\n", pDevice->spiMode);
    DEBUG_PRINT("SPI bits per word: %d\n", pDevice->spiBitsPerWord);
    DEBUG_PRINT("SPI max speed: %d Hz (%d KHz)\n", pDevice->spiSpeed, pDevice->spiSpeed/1000);
    DEBUG_PRINT("SPI successfully initialized.\n");
    return 0;
}

int spi_ReadWrite(adi_imu_Device_t *pDevice, const uint8_t *txBuf, uint8_t *rxBuf, uint32_t xferLen, uint32_t numXfers, uint32_t numRepeats, uint32_t enRepeatTx)
{
    if (pDevice->status == 0) DEBUG_PRINT_RET(-1, "Error: Device not initialized properly.\n");

    int ret = 0;
    const uint32_t total_xfers = numXfers * numRepeats;

    struct spi_ioc_transfer tr[total_xfers];
    memset(tr, 0, sizeof(tr));

    for (int i=0; i<numRepeats; i++) {
        for (int j=0; j<numXfers; j++)
        {
            int xfer_idx = i * numXfers + j;
            if (enRepeatTx == 0)
                tr[xfer_idx].tx_buf = (uint64_t)(txBuf + xferLen * xfer_idx);
            else
                tr[xfer_idx].tx_buf = (uint64_t)(txBuf + xferLen * j);
            tr[xfer_idx].rx_buf = (uint64_t)(rxBuf + xferLen * xfer_idx);
            tr[xfer_idx].len = xferLen;
            tr[xfer_idx].speed_hz = pDevice->spiSpeed;
            tr[xfer_idx].delay_usecs = pDevice->spiDelay;
            tr[xfer_idx].bits_per_word = pDevice->spiBitsPerWord;
            tr[xfer_idx].cs_change = 1;
        //}
   // }
// #define DEBUG_SPI
#ifdef DEBUG_SPI
    printf("\n\nxferLength: %d numXfers: %d numRepeats: %d enRepeatTx: %d\n", xferLen, numXfers, numRepeats, enRepeatTx);
    printf("[SPI TX]: ");
    if(enRepeatTx == 0)
        for (int i=0; i< (xferLen * numXfers * numRepeats) ; i++) printf("%02X ", txBuf[i]);
    else
        for (int i=0; i< (xferLen * numXfers) ; i++) printf("%02X ", txBuf[i]);
    printf("\n");
#endif

    ret = ioctl((int) pDevice->spiHandle, SPI_IOC_MESSAGE(1), &tr[xfer_idx]);
    if (ret < 1) DEBUG_PRINT_RET(-1, "Error: Failed to send spi message. Error: %s\n", strerror(ret));
    
#ifdef DEBUG_SPI
    printf("[SPI RX]: ");
    for (int i=0; i<(xferLen * total_xfers); i++) printf("%02X ", rxBuf[i]);
    printf("\n");
#endif
	    }
    }
    return 0;
}
