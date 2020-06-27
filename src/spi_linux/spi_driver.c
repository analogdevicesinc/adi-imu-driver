/*******************************************************************************
 *   @file   spi_driver.c
 *   @brief  Spidev Driver interface
 *   @author Sundar Palani <sundar.palani@analog.com>
********************************************************************************/

#include <unistd.h>        // Needed for SPI port
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>            // Needed for SPI port
#include <sys/ioctl.h>        // Needed for SPI port
#include <linux/types.h>
#include <linux/spi/spidev.h> // Needed for SPI port

#include "spi_driver.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define DEBUG_PRINT(format, ...) printf(format , ##__VA_ARGS__)

#define PRINT_ERROR_EXIT(msg, ...) do {\
        DEBUG_PRINT(msg , ##__VA_ARGS__);\
        abort();\
    } while(0)

#define PRINT_ERROR_RET(ret, msg, ...) do {\
        DEBUG_PRINT(msg , ##__VA_ARGS__);\
        return ret;\
    } while(0)


int adi_imu_SpiInit(adi_imu_Device_t *pDevice)
{
    int spi_fd = open(pDevice->spiDev, O_RDWR);
    if (spi_fd < 0) PRINT_ERROR_RET(adi_imu_SpiInitFailed_e, "can't open device\n");
    
    pDevice->spiHandle = (adi_imu_DevHandler_t) spi_fd;

    int ret = adi_imu_Success_e;
    /*
    * spi mode
    */
    ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &pDevice->spiMode);
    if (ret == -1) PRINT_ERROR_RET(adi_imu_SpiInitFailed_e, "Failed to set spi mode (wr). Error: %s\n", strerror(errno));

    ret = ioctl(spi_fd, SPI_IOC_RD_MODE, &pDevice->spiMode);
    if (ret == -1) PRINT_ERROR_RET(adi_imu_SpiInitFailed_e, "Failed to set spi mode (rd). Error: %s\n", strerror(errno));

    /*
    * bits per word
    */
    ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &pDevice->spiBitsPerWord);
    if (ret == -1) PRINT_ERROR_RET(adi_imu_SpiInitFailed_e, "Failed to set bits per word (wr). Error: %s\n", strerror(errno));

    ret = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &pDevice->spiBitsPerWord);
    if (ret == -1) PRINT_ERROR_RET(adi_imu_SpiInitFailed_e, "Failed to set bits per word (rd). Error: %s\n", strerror(errno));

    /*
    * max speed hz
    */
    ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &pDevice->spiSpeed);
    if (ret == -1) PRINT_ERROR_RET(adi_imu_SpiInitFailed_e, "Failed to set max speed hz (wr). Error: %s\n", strerror(errno));

    ret = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &pDevice->spiSpeed);
    if (ret == -1) PRINT_ERROR_RET(adi_imu_SpiInitFailed_e, "Failed to set max speed hz (rd). Error: %s\n", strerror(errno));

    DEBUG_PRINT("spi mode: %d\n", pDevice->spiMode);
    DEBUG_PRINT("bits per word: %d\n", pDevice->spiBitsPerWord);
    DEBUG_PRINT("max speed: %d Hz (%d KHz)\n", pDevice->spiSpeed, pDevice->spiSpeed/1000);
    DEBUG_PRINT("SPI: device initialized.\n");
    return adi_imu_Success_e;
}

int adi_imu_SpiReadWrite(adi_imu_Device_t *pDevice, uint8_t *txBuf, uint8_t *rxBuf, uint32_t length)
{
    int ret = adi_imu_Success_e;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)txBuf,
        .rx_buf = (unsigned long)rxBuf,
        .len = length,
        .speed_hz = pDevice->spiSpeed,
        .delay_usecs = pDevice->spiDelay,
        .bits_per_word = pDevice->spiBitsPerWord,
        .cs_change = 1
    };

    ret = ioctl((int) pDevice->spiHandle, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) PRINT_ERROR_RET(adi_imu_SpiRwFailed_e, "can't send spi message. Error: %s\n", strerror(errno));
    return adi_imu_Success_e;
}
