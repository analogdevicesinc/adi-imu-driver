/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		spi_driver.c
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Driver interface for linux SPI devices.
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

inline int spi_Init(adi_imu_SpiDevice_t* device)
{
    device->status = IMUBUF_SPI_UNKNOWN;

    device->fd = open(device->dev, O_RDWR);
    if (device->fd < 0) DEBUG_PRINT_RET(Err_spi_InitFailed_e, "[SPI]: Error: failed to open spi device %s\n", device->dev);
    
    device->status = IMUBUF_SPI_OPENED;

    int ret = Err_imu_Success_e;
    /*
    * spi mode
    */
    ret = ioctl(device->fd, SPI_IOC_WR_MODE, &device->mode);
    if (ret == -1) DEBUG_PRINT_RET(Err_spi_InitFailed_e, "[SPI]: Error: Failed to set spi mode (wr). Error: %s\n", strerror(errno));

    ret = ioctl(device->fd, SPI_IOC_RD_MODE, &device->mode);
    if (ret == -1) DEBUG_PRINT_RET(Err_spi_InitFailed_e, "[SPI]: Error: Failed to set spi mode (rd). Error: %s\n", strerror(errno));

    /*
    * bits per word
    */
    ret = ioctl(device->fd, SPI_IOC_WR_BITS_PER_WORD, &device->bitsPerWord);
    if (ret == -1) DEBUG_PRINT_RET(Err_spi_InitFailed_e, "[SPI]: Error: Failed to set bits per word (wr). Error: %s\n", strerror(errno));

    ret = ioctl(device->fd, SPI_IOC_RD_BITS_PER_WORD, &device->bitsPerWord);
    if (ret == -1) DEBUG_PRINT_RET(Err_spi_InitFailed_e, "[SPI]: Error: Failed to set bits per word (rd). Error: %s\n", strerror(errno));

    /*
    * max speed hz
    */
    ret = ioctl(device->fd, SPI_IOC_WR_MAX_SPEED_HZ, &device->speed);
    if (ret == -1) DEBUG_PRINT_RET(Err_spi_InitFailed_e, "[SPI]: Error: Failed to set max speed hz (wr). Error: %s\n", strerror(errno));

    ret = ioctl(device->fd, SPI_IOC_RD_MAX_SPEED_HZ, &device->speed);
    if (ret == -1) DEBUG_PRINT_RET(Err_spi_InitFailed_e, "[SPI]: Error: Failed to set max speed hz (rd). Error: %s\n", strerror(errno));
    
    device->status = IMUBUF_SPI_CONFIGURED;
    device->status = IMUBUF_SPI_READY;

    DEBUG_PRINT("[SPI]: device: %s\n", device->dev);
    DEBUG_PRINT("[SPI]: mode: %d\n", device->mode);
    DEBUG_PRINT("[SPI]: bits per word: %d\n", device->bitsPerWord);
    DEBUG_PRINT("[SPI]: max speed: %d Hz (%d KHz)\n", device->speed, device->speed/1000);
    DEBUG_PRINT("[SPI]: SPI successfully initialized.\n");
    return device->fd;
}

inline int spi_ReadWrite(adi_imu_SpiDevice_t* device, const uint8_t *txBuf, uint8_t *rxBuf, uint32_t xferLen, uint32_t numXfers, uint32_t numRepeats, uint32_t enRepeatTx)
{
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
            tr[xfer_idx].speed_hz = device->speed;
            tr[xfer_idx].delay_usecs = device->delay;
            tr[xfer_idx].bits_per_word = device->bitsPerWord;
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

    ret = ioctl(device->fd, SPI_IOC_MESSAGE(1), &tr[xfer_idx]);
    if (ret < 1) DEBUG_PRINT_RET(Err_spi_RwFailed_e, "[SPI]: Error: Failed to send spi message. Error: %s\n", strerror(ret));
    
#ifdef DEBUG_SPI
    printf("[SPI RX]: ");
    for (int i=0; i<(xferLen * total_xfers); i++) printf("%02X ", rxBuf[i]);
    printf("\n");
#endif
	    }
    }
    return Err_imu_Success_e;
}
