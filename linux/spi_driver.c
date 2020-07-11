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

    DEBUG_PRINT("SPI mode: %d\n", pDevice->spiMode);
    DEBUG_PRINT("SPI bits per word: %d\n", pDevice->spiBitsPerWord);
    DEBUG_PRINT("SPI max speed: %d Hz (%d KHz)\n", pDevice->spiSpeed, pDevice->spiSpeed/1000);
    DEBUG_PRINT("SPI successfully initialized.\n");
    return 0;
}

int spi_ReadWrite(adi_imu_Device_t *pDevice, uint8_t *txBuf, uint8_t *rxBuf, uint32_t length)
{
    if (pDevice->status == 0) DEBUG_PRINT_RET(-1, "Error: Device not initialized properly.\n");

    int ret = 0;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)txBuf,
        .rx_buf = (unsigned long)rxBuf,
        .len = length,
        .speed_hz = pDevice->spiSpeed,
        .delay_usecs = pDevice->spiDelay,
        .bits_per_word = pDevice->spiBitsPerWord,
        .cs_change = 1
    };

#ifdef DEBUG_SPI
    printf("\n\n[SPI TX]: ");
    for (int i=0; i<length; i++) printf("0x%02X ", txBuf[i]);
    printf("\n");
#endif

    ret = ioctl((int) pDevice->spiHandle, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) DEBUG_PRINT_RET(-1, "Error: Failed to send spi message. Error: %s\n", strerror(errno));
    
#ifdef DEBUG_SPI
    printf("[SPI RX]: ");
    for (int i=0; i<length; i++) printf("0x%02X ", rxBuf[i]);
    printf("\n");
#endif

    return 0;
}
