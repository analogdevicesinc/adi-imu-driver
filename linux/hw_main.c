/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		comm_main.c
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Underlying communication abstraction for talking to IMU or IMU-Buffer.
 **/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>     
#include <time.h>
#include <termios.h>
#include <sys/ioctl.h> 
#include <sys/stat.h>
#include <sys/time.h>

#include "adi_imu_common.h"
#include "spi_driver.h"
#include "uart_driver.h"

#define HW_TX_BUF_LEN 4096
#define HW_RX_BUF_LEN 4096

unsigned char g_txbuf[HW_TX_BUF_LEN] = {0};
unsigned char g_rxbuf[HW_RX_BUF_LEN] = {0};

inline int hw_Init(adi_imu_Device_t *pDevice)
{
    int ret = Err_imu_Success_e;

    if(pDevice->devType == IMU_HW_SPI)
    {
        if((ret = spi_Init(&pDevice->spiDev)) < 0) return ret;
    }
    else if(pDevice->devType == IMU_HW_UART)
    {
        if((ret = uart_Init(&pDevice->uartDev)) < 0) return ret;
    }

    return ret;
}

inline int hw_FlushInput(adi_imu_Device_t *pDevice)
{
    if(pDevice->devType == IMU_HW_UART && pDevice->uartDev.status >= IMUBUF_UART_CONFIGURED)
    {
        return uart_FlushInput(&pDevice->uartDev);
    }
    return Err_imu_Success_e;
}

inline int hw_FlushOutput(adi_imu_Device_t *pDevice)
{
    if(pDevice->devType == IMU_HW_UART && pDevice->uartDev.status >= IMUBUF_UART_CONFIGURED)
    {
        return uart_FlushOutput(&pDevice->uartDev);
    }
    return Err_imu_Success_e;
}

inline int hw_GetPage(adi_imu_Device_t *pDevice)
{
    int ret = Err_hw_NoDeviceActive_e;
    uint8_t regAddr = REG_PAGE_ID & 0xFF;
    if(pDevice->devType == IMU_HW_SPI && pDevice->spiDev.status >= IMUBUF_SPI_CONFIGURED)
    {
        memset(g_txbuf, 0, 4);
        memset(g_rxbuf, 0, 4);
        g_txbuf[0] = regAddr;
        /* send read request */
        pDevice->spiDev.delay = (pDevice->spiDev.delay > IMU_MIN_STALL_US) ? pDevice->spiDev.delay : IMU_MIN_STALL_US;
        if ((ret = spi_ReadWrite(&pDevice->spiDev, g_txbuf, g_rxbuf, 2, 2, 1, 0)) < 0) return ret;
        pDevice->curPage = ((g_rxbuf[2] << 8) | g_rxbuf[3]);
        ret = Err_imu_Success_e;
    }
    else if(pDevice->devType == IMU_HW_UART && pDevice->uartDev.status >= IMUBUF_UART_CONFIGURED)
    {
        uint16_t readPgId = IMU_INVALID_PAGE_ID;
        memset(g_txbuf, 0, 20);
        memset(g_rxbuf, 0, 100);
        snprintf((char*)g_txbuf, 20, "read %d\r\n", regAddr);
        if ((ret = uart_WriteRead(&pDevice->uartDev, g_txbuf, strlen((char*)g_txbuf), g_rxbuf, 100)) < 0) 
        {
            pDevice->curPage = IMU_INVALID_PAGE_ID; // lets set to some unavailable page
            DEBUG_PRINT("[UART]: Failed to read page ID\n");
            hw_FlushInput(pDevice);
            hw_FlushOutput(pDevice);
            return ret;
        }
        if ((ret = uart_RxParse16bit(g_rxbuf, &readPgId, 1)) > 1) DEBUG_PRINT("Truncation while parsing UART-Rx buffer\n");
        pDevice->curPage = readPgId;
        ret = Err_imu_Success_e;
    }
    if(ret == Err_imu_Success_e)
    {
        if (pDevice->curPage < IMU_MIN_PAGE_ID(pDevice->imuProd) || (pDevice->curPage > IMU_MAX_PAGE_ID(pDevice->imuProd) && pDevice->curPage < IMU_BUF_MIN_PAGE_ID) || \
            pDevice->curPage > IMU_BUF_MAX_PAGE_ID)
            pDevice->curPage = IMU_INVALID_PAGE_ID;
    }
    return ret;
}

inline int hw_SetPage(adi_imu_Device_t *pDevice, uint16_t page, unsigned force)
{
    int ret = Err_imu_Success_e;
    if (force || pDevice->curPage != page)
    {
        ret = Err_hw_NoDeviceActive_e;
        if(pDevice->devType == IMU_HW_SPI && pDevice->spiDev.status >= IMUBUF_SPI_CONFIGURED)
        {
            memset(g_txbuf, 0, 2);
            memset(g_rxbuf, 0, 2);
            /* send write request */
            g_txbuf[0] = 0x80 | REG_PAGE_ID; g_txbuf[1] = page;
            pDevice->spiDev.delay = (pDevice->spiDev.delay > IMU_MIN_STALL_US) ? pDevice->spiDev.delay : IMU_MIN_STALL_US;
            if ((ret = spi_ReadWrite(&pDevice->spiDev, g_txbuf, g_rxbuf, 2, 1, 1, IMU_FALSE)) < 0) return ret;
            pDevice->curPage = page;
            ret = Err_imu_Success_e;
        }
        else if(pDevice->devType == IMU_HW_UART && pDevice->uartDev.status >= IMUBUF_UART_CONFIGURED)
        {
            memset(g_txbuf, 0, 20);
            snprintf((char*)g_txbuf, 20, "write 0 %x\r\n", page);
            if ((ret = uart_Write(&pDevice->uartDev, g_txbuf, strlen((char*)g_txbuf))) < 0) return ret;
            pDevice->curPage = page;
            ret = Err_imu_Success_e;
        }
    }
    return ret;
}

inline int hw_ReadRegs(adi_imu_Device_t *pDevice, const uint16_t* regs, const size_t len, uint16_t* val, unsigned en_contiguous)
{
    int ret = Err_hw_NoDeviceActive_e;

    // NOTE: This function assumes g_txbuf and g_rxbuf is large enough

    if(pDevice->devType == IMU_HW_SPI && pDevice->spiDev.status >= IMUBUF_SPI_CONFIGURED)
    {
        // Prepare transaction
        if ((ret = hw_SetPage(pDevice, (regs[0] >> 8) & 0xFF, 0)) < 0) return ret;
        uint8_t prev_pg_id = pDevice->curPage;
        int readIdx[len]; // actual index in tx buffer
        int j=0; // actual index in tx buffer
        for (int i=0; i<len; i++)
        {
            if(!en_contiguous)
            {
                uint8_t pageId = (regs[i] >> 8) & 0xFF;
                if(prev_pg_id != pageId) 
                {
                    g_txbuf[j++] = 0x80 | REG_PAGE_ID;
                    g_txbuf[j++] = pageId;
                    prev_pg_id = pageId;
                }
            }
            g_txbuf[j++] = regs[i] & 0xFF;
            g_txbuf[j++] = 0x00;
            readIdx[i] = j;
            if(i==len-1)
            {
                g_txbuf[j++] = 0x00;
                g_txbuf[j++] = 0x00;
            }
        }

        // send transaction
        pDevice->spiDev.delay = (pDevice->spiDev.delay > IMU_MIN_STALL_US) ? pDevice->spiDev.delay : IMU_MIN_STALL_US;
        if ((ret = spi_ReadWrite(&pDevice->spiDev, g_txbuf, g_rxbuf, 2, j/2, 1, 0)) < 0) return ret;
        
        // parse output
        for (int i=0; i<len; i++)
            val[i] = ((g_rxbuf[readIdx[i]] << 8) | g_rxbuf[readIdx[i]+1]);
        ret = Err_imu_Success_e;
    }
    else if(pDevice->devType == IMU_HW_UART && pDevice->uartDev.status >= IMUBUF_UART_CONFIGURED)
    {
        int retry_cnt = 2; // lets retry 2 times if we get bad command error
        for (int i=0; i<len; i++)
        {
            /* ensure we are in right page */
            if ((ret = hw_SetPage(pDevice, (regs[i] >> 8) & 0xFF, 0)) < 0) return ret;

            memset(g_txbuf, 0, 50);
            memset(g_rxbuf, 0, 500);

            if (en_contiguous)
                snprintf((char*)g_txbuf, 50, "read %x %x\r\n", regs[0]&0xFF, regs[len-1]&0xFF);
            else
                snprintf((char*)g_txbuf, 50, "read %x\r\n", regs[i]&0xFF);
            if ((ret = uart_WriteRead(&pDevice->uartDev, g_txbuf, strlen((char*)g_txbuf), g_rxbuf, 500)) < 0) 
            {
                while (i<len) val[i++] = 0;
                hw_FlushInput(pDevice);
                hw_FlushOutput(pDevice);
                return ret;
            }

            if (strstr((char*)g_rxbuf, "Error") != NULL)
            {
                if(retry_cnt--) {
                    i--;
                    DEBUG_PRINT("[UART] Cmd: %s, Error: %s\n", g_txbuf, g_rxbuf);
                    memset(g_rxbuf, 0, 200);
                    continue;
                }
                else
                    DEBUG_PRINT_RET(Err_uart_ReadFailed_e, "[UART] Cmd: %s, Error: %s\n", g_txbuf, g_rxbuf);
            }
            if (en_contiguous)
            {
                if ((ret = uart_RxParse16bit(g_rxbuf, val, len)) > len) DEBUG_PRINT("Truncation while parsing UART-Rx buffer\n");
                break;
            }
            else
            {
                uint16_t readVal;
                if ((ret = uart_RxParse16bit(g_rxbuf, &readVal, 1)) > 1) DEBUG_PRINT("Truncation while parsing UART-Rx buffer\n");
                // DEBUG_PRINT("Reg %x read: 0x%x\n", regAddr, readVal);
                val[i] = (uint16_t)readVal;
            }
        }
        ret = Err_imu_Success_e;
    }
    return ret;
}

// returns 0 on SUCCESS, negative on FAILURE
inline int hw_WriteRegs(adi_imu_Device_t *pDevice, const uint16_t* regs, const size_t len, uint16_t* val, unsigned en_contiguous)
{
    int ret = Err_hw_NoDeviceActive_e;

    // NOTE: This function assumes g_txbuf and g_rxbuf is large enough

    if(pDevice->devType == IMU_HW_SPI && pDevice->spiDev.status >= IMUBUF_SPI_CONFIGURED)
    {
        // Prepare transaction
        if ((ret = hw_SetPage(pDevice, (regs[0] >> 8) & 0xFF, 0)) < 0) return ret;
        uint8_t prev_pg_id = pDevice->curPage;
        int j=0; // actual index in tx buffer
        for (int i=0; i<len; i++)
        {
            if(!en_contiguous)
            {
                uint8_t pageId = (regs[i] >> 8) & 0xFF;
                if(prev_pg_id != pageId) 
                {
                    g_txbuf[j++] = 0x80 | REG_PAGE_ID;
                    g_txbuf[j++] = pageId;
                    prev_pg_id = pageId;
                }
            }
            uint8_t regAddr = regs[i] & 0xFF;
            g_txbuf[j++] = 0x80 | regAddr;
            g_txbuf[j++] = val[i] & 0xFF;
            g_txbuf[j++] = 0x80 | (regAddr+1);
            g_txbuf[j++] = (val[i] >> 8) & 0xFF;
        }

        // send transaction
        pDevice->spiDev.delay = (pDevice->spiDev.delay > IMU_MIN_STALL_US) ? pDevice->spiDev.delay : IMU_MIN_STALL_US;
        if ((ret = spi_ReadWrite(&pDevice->spiDev, g_txbuf, g_rxbuf, 2, j/2, 1, 0)) < 0) return ret;
        ret = Err_imu_Success_e;
    }
    else if(pDevice->devType == IMU_HW_UART && pDevice->uartDev.status >= IMUBUF_UART_CONFIGURED)
    {
        for (int i=0; i<len; i++)
        {
            /* ensure we are in right page */
            if ((ret = hw_SetPage(pDevice, (regs[i] >> 8) & 0xFF, 0)) < 0) return ret;

            uint8_t regAddr = regs[i] & 0xFF;
            memset(g_txbuf, 0, 50);
            snprintf((char*)g_txbuf, 20, "write %x %x\r\n", regAddr, val[i] & 0xFF);
            if ((ret = uart_Write(&pDevice->uartDev, g_txbuf, strlen((char*)g_txbuf))) < 0) return ret;
            delay_MicroSeconds(10000);
            snprintf((char*)g_txbuf, 20, "write %x %x\r\n", regAddr+1, (val[i]>>8) & 0xFF);
            if ((ret = uart_Write(&pDevice->uartDev, g_txbuf, strlen((char*)g_txbuf))) < 0) return ret;
        }
        ret = Err_imu_Success_e;
    }
    return ret;
}

inline int hw_ReadWriteRaw(adi_imu_Device_t *pDevice, const uint8_t* txbuf, size_t txlen, uint8_t* rxbuf, size_t rxlen)
{
    int ret = Err_hw_NoDeviceActive_e;

    if(pDevice->devType == IMU_HW_SPI && pDevice->spiDev.status >= IMUBUF_SPI_CONFIGURED)
    {
        pDevice->spiDev.delay = (pDevice->spiDev.delay > IMU_MIN_STALL_US) ? pDevice->spiDev.delay : IMU_MIN_STALL_US;
        if ((ret = spi_ReadWrite(&pDevice->spiDev, txbuf, rxbuf, (txlen > rxlen) ? rxlen : txlen, 1, 1, 0)) < 0) return ret;
    }
    else if(pDevice->devType == IMU_HW_UART && pDevice->uartDev.status >= IMUBUF_UART_CONFIGURED)
    {
		if ((ret = uart_WriteRead(&pDevice->uartDev, txbuf, txlen, rxbuf, rxlen)) < 0) return ret;
    }
    return ret;
}

int hw_ReadReg(adi_imu_Device_t *pDevice, uint16_t reg, uint16_t *val)
{
    return hw_ReadRegs(pDevice, &reg, 1, val, IMU_TRUE);
}

int hw_WriteReg(adi_imu_Device_t *pDevice, uint16_t reg, uint16_t val)
{
    return hw_WriteRegs(pDevice, &reg, 1, &val, IMU_TRUE);
}

int hw_ParseRaw(adi_imu_Device_t *pDevice, uint8_t* buf, uint16_t* out, size_t len)
{
    int ret = Err_imu_Success_e;
    if(pDevice->devType == IMU_HW_UART && pDevice->uartDev.status >= IMUBUF_UART_CONFIGURED)
    {
        if ((ret = uart_RxParse16bit(buf, out, len)) > len) DEBUG_PRINT("Truncation while parsing UART-Rx buffer\n");
    }
    else ret = Err_imu_UnsupportedProtocol_e;
    return ret;
}
