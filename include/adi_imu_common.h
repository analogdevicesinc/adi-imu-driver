/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		adi_imu_common.h
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Common header file to be shared between all libraries.
 **/

#ifndef __ADI_IMU_COMMON_H_
#define __ADI_IMU_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

/* pickup integer types */
#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

#ifndef BAREMETAL
#include <stdio.h>
#define DEBUG_PRINT(format, ...) printf(format , ##__VA_ARGS__)
#define DEBUG_PRINT_RET(ret, msg, ...) do {\
        DEBUG_PRINT(msg , ##__VA_ARGS__);\
        return ret;\
    } while(0)

#else
#define DEBUG_PRINT(format, ...) do{} while(0)
#define DEBUG_PRINT_RET(ret, msg, ...) do{} while(0)
#endif

#include <string.h>
#include "adi_imu_regmap.h"
#include "imu_spi_buffer_regmap.h"

// Useful macros
#define IMU_BSWAP_16(x)                     ( (((x) & 0xFF) << 8) | (((x) & 0xFF00) >> 8) )
#define IMU_BSWAP_32(x)                     ( (((x) & 0xFF) << 8) | (((x) & 0xFF00) >> 8) | (((x) & 0xFF0000) << 8) | (((x) & 0xFF000000) >> 8) )
#define TO_REG(val, pos, mask)              (((val) << pos) & mask)
#define FROM_REG(val, pos, mask)            (((val) & mask) >> pos)

#define IMU_INVALID_PAGE_ID                 (0x7F)

#define IMU_MIN_STALL_US 10 // minimum delay during register access after every 16bit transactions
#define IMU_MAX_SPI_CLK 6000000

#define IMU_BUF_MIN_FW_REV_REQUIRED (1.12)
#define IMU_BUF_MIN_STALL_US 100 // minimum delay during register access after every 16bit transactions
#define IMU_BUF_MAX_SPI_CLK 10000000
#define IMU_BUF_MAX_PPS_LOCK_TIMEOUT_MS 10000
#define IMU_BUF_MIN_PPS_LOCK_DURATION_MS 5000

typedef enum {
    // IMU related error enums
    Err_imu_Success_e = 0,
    Err_spi_InitFailed_e = -1,
    Err_spi_RwFailed_e = -2,
    Err_imu_ProdIdVerifyFailed_e = -3,
    Err_imu_BurstFrameInvalid_e = -4,
    Err_hw_NoDeviceActive_e = -5,
    Err_imu_SelfTestFailed_e = -6,
    Err_imu_SystemError_e = -7,
    Err_imu_UnsupportedHardware_e = -8,
    Err_uart_InitFailed_e = -9,
    Err_uart_ConfigFailed_e = -10,
    Err_uart_BaudRateNotSupported_e = -11,
    Err_uart_ReadFailed_e = -12,
    Err_uart_ReadEmpty_e = -13,
    Err_uart_WriteFailed_e = -14,
    Err_imu_InvalidPageId_e = -15,
    Err_imu_UnsupportedProtocol_e = -16,

    // IMU Buffer related error enums
    Err_Imubuf_BufLenOverflow_e = -500,
    Err_Imubuf_BufClearFailed_e = -501,
    Err_Imubuf_BufPPSLockTimedout_e = -502,
    Err_Imubuf_BufPPSLockUnstable_e = -503,
    Err_Imubuf_ScalingOutputFailed_e = -504,
    Err_Imubuf_FwRevNotSupported_e = -505,

} adi_imu_Error_e;

typedef enum {
    IMU_FALSE = 0,
    IMU_TRUE = 1
} adi_imu_Boolean_e;

typedef enum {
    IMU_HW_SPI = 0,
    IMU_HW_UART = 1,
} imu_buf_DevProtocol_e;

typedef enum {
    IMUBUF_SPI_UNKNOWN = 10,
    IMUBUF_SPI_OPENED = 11,
    IMUBUF_SPI_CONFIGURED = 12,
    IMUBUF_SPI_READY = 13,

    IMUBUF_UART_UNKNOWN = 110,
    IMUBUF_UART_OPENED = 111,
    IMUBUF_UART_CONFIGURED = 112,
    IMUBUF_UART_READY = 113,
} adi_imu_DevStatus_e;

typedef volatile void* adi_imu_DevHandler_t;

typedef struct {
    const char* dev;
    uint32_t speed;
    uint8_t mode;
    uint8_t bitsPerWord;
    uint32_t delay;

    /* used internally */
    int fd;
    adi_imu_DevStatus_e status;
} adi_imu_SpiDevice_t;

typedef struct {
    const char* dev;
    uint32_t baud;

    /* used internally */
    int fd;
    adi_imu_DevStatus_e status;
} adi_imu_UartDevice_t;

typedef struct {
    /* for verification */
    uint16_t prodId;

    /* gravity constant; if 1.0 accelerometer output is normalized to g */
    double g;

    adi_imu_Boolean_e enable_buffer;

    imu_buf_DevProtocol_e devType;
    adi_imu_SpiDevice_t spiDev;
    adi_imu_UartDevice_t uartDev;

    /* device handler (USED INTERNALLY)*/
    adi_imu_DevHandler_t devHandle;

    /* device status(USED INTERNALLY) */
    uint16_t curPage;
    uint16_t rangeModel;

} adi_imu_Device_t;

/* HW communication API (provided by user) */
int hw_Init(adi_imu_Device_t *pDevice);
int hw_FlushInput(adi_imu_Device_t *pDevice);
int hw_FlushOutput(adi_imu_Device_t *pDevice);
int hw_GetPage(adi_imu_Device_t *pDevice);
int hw_SetPage(adi_imu_Device_t *pDevice, uint16_t page, unsigned force);
int hw_ReadReg(adi_imu_Device_t *pDevice, uint16_t reg, uint16_t *val);
int hw_WriteReg(adi_imu_Device_t *pDevice, uint16_t reg, uint16_t val);
int hw_ReadRegs(adi_imu_Device_t *pDevice, const uint16_t* regs, const size_t len, uint16_t* val, unsigned en_contiguous);
int hw_WriteRegs(adi_imu_Device_t *pDevice, const uint16_t* regs, const size_t len, uint16_t* val, unsigned en_contiguous);
int hw_ReadWriteRaw(adi_imu_Device_t *pDevice, const uint8_t* txbuf, size_t txlen, uint8_t* rxbuf, size_t rxlen);
void delay_MicroSeconds (uint32_t microseconds);

// TO BE ABSTRACTED
int uart_rx_parse16(uint8_t* in, uint16_t* out, size_t len);
// extern int spi_Init(adi_imu_SpiDevice_t *pDevice);
// extern int spi_ReadWrite(adi_imu_SpiDevice_t *pDevice, const uint8_t *txBuf, uint8_t *rxBuf, uint32_t xferLen, uint32_t numXfers, uint32_t numRepeats, uint32_t enRepeatTx);

#ifdef __cplusplus
}
#endif
#endif