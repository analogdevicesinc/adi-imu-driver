/*
    File: adi_imu_driver.h

    Description:
    Driver interface for ADIS16xxx IMU

    Author: Sundar Palani <sundar.palani@analog.com>
*/

#ifndef __ADI_IMU_DRIVER_H_
#define __ADI_IMU_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* pickup integer types */
#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

#include "adi_imu_regmap.h"

#define IMU_TO_HALFWORD(buf, idx)   ( ((buf[idx] << 8) & 0xFF00) | (buf[1+idx] & 0xFF) )

#define IMU_TO_WORD(buf, idx)       ( (uint32_t)((buf[2+idx] << 24) & 0xFF000000) | (uint32_t)((buf[3+idx] << 16) & 0xFF0000) | (uint32_t)((buf[idx] << 8) & 0xFF00) | (uint32_t)(buf[1+idx] & 0xFF) )


#ifndef BAREMETAL
#include <stdio.h>
#define DEBUG_PRINT(format, ...) printf(format , ##__VA_ARGS__)

#define DEBUG_PRINT_RET(ret, msg, ...) do {\
        DEBUG_PRINT(msg , ##__VA_ARGS__);\
        return ret;\
    } while(0)

#else
#define IMU_DEBUG_PRINT(format, ...) {}
#define PRINT_ERROR_RET(ret, msg, ...) {}
#endif



enum adi_imu_Error_e {

    adi_imu_SystemError_e = -7,
    adi_imu_SelfTestFailed_e = -6,
    adi_imu_BadDevice_e = -5,
    adi_imu_BurstFrameInvalid_e = -4,
    adi_imu_ProdIdVerifyFailed_e = -3,
    adi_imu_SpiRwFailed_e = -2,
    adi_imu_SpiInitFailed_e = -1,
    adi_imu_Success_e = 0,
};

typedef volatile void* adi_imu_DevHandler_t;
typedef struct {
    /* for verification */
    uint16_t prodId;

    /* spi config */
    const char* spiDev;
    uint32_t spiSpeed;
    uint8_t spiMode;
    uint8_t spiBitsPerWord;
    uint32_t spiDelay;

    /* pin config */
    uint8_t pinRstn;
    uint8_t pinCsn;
    uint8_t pinSclk;
    uint8_t pinDout;
    uint8_t pinDin;
    uint8_t pinDio1;
    uint8_t pinDio2;
    uint8_t pinDio3;
    uint8_t pinDio4;

    /* device handler */
    adi_imu_DevHandler_t spiHandle;

    /* device status */
    uint8_t status; // 0: bad, greater than 1: good
    uint16_t curPage;

} adi_imu_Device_t;

typedef struct {
    uint16_t prodId;
    uint16_t fwRev;
    uint16_t fwDayMonth;
    uint16_t fwYear;
    uint16_t bootLoadVer;
    uint16_t serialNumber;
    uint16_t gyroModelId;
    uint16_t pageId;
    uint16_t decimationRate;
    uint16_t syncScale;
    uint16_t nullConfig;
    uint16_t clkConfig;
    uint16_t fnctioCtrl;
    uint16_t gpioCtrl;
    uint16_t ftrBank0;
    uint16_t ftrBank1;
} adi_imu_DevInfo_t;

typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t z;
} adi_imu_XYZOutput_t;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} adi_imu_XYZScale_t;

typedef struct {
    uint16_t data; 
} adi_imu_DiagStatus_t;

typedef struct {
    uint16_t data; 
} adi_imu_SysStatus_t;

typedef adi_imu_XYZOutput_t adi_imu_AcclOutput_t;
typedef adi_imu_XYZOutput_t adi_imu_GyroOutput_t;
typedef adi_imu_XYZOutput_t adi_imu_DelAngOutput_t;
typedef adi_imu_XYZOutput_t adi_imu_DelVelOutput_t;
typedef adi_imu_XYZOutput_t adi_imu_GyroBias_t;
typedef adi_imu_XYZOutput_t adi_imu_AcclBias_t;
typedef adi_imu_XYZScale_t adi_imu_GyroScale_t;
typedef adi_imu_XYZScale_t adi_imu_AcclScale_t;

typedef struct {
    uint16_t sysEFlag;
    uint16_t tempOut;
    adi_imu_GyroOutput_t gyro;
    adi_imu_AcclOutput_t accl;
    uint16_t dataCntOrTimeStamp;
    uint32_t crc;
} adi_imu_BurstOutput_t;

enum adi_imu_Polarity_e {
    NEGATIVE = 0,
    POSITIVE = 1
};

enum adi_imu_EdgeType_e {
    FALLING_EDGE = 0,
    RISING_EDGE = 1
};

enum adi_imu_GPIO_e {
    DIO1 = 0,
    DIO2 = 1,
    DIO3 = 2,
    DIO4 = 3
};

enum adi_imu_Direction_e {
    INPUT = 0,
    OUTPUT = 1
};

enum adi_imu_EnDis_e {
    DISABLE = 0,
    ENABLE = 1
};

enum adi_imu_ClockMode_e {
    SYNC = 0,
    PPS = 1
};


int adi_imu_Init                    (adi_imu_Device_t *pDevice);

int adi_imu_GetDevInfo              (adi_imu_Device_t *pDevice, adi_imu_DevInfo_t *pInfo);

int adi_imu_PrintDevInfo            (adi_imu_Device_t *pDevice, adi_imu_DevInfo_t *pInfo);

int adi_imu_CheckDiagStatus         (adi_imu_Device_t *pDevice, adi_imu_DiagStatus_t *pStatus);

int adi_imu_CheckSysStatus          (adi_imu_Device_t *pDevice, adi_imu_SysStatus_t *pStatus);

int adi_imu_SetDecimationRate       (adi_imu_Device_t *pDevice, uint16_t rate); /* Output data rate = 4250 / (DEC_RATE + 1) */

int adi_imu_ReadAccl                (adi_imu_Device_t *pDevice, adi_imu_AcclOutput_t *pData);

int adi_imu_ReadGyro                (adi_imu_Device_t *pDevice, adi_imu_GyroOutput_t *pData);

int adi_imu_ReadDelAng              (adi_imu_Device_t *pDevice, adi_imu_DelAngOutput_t *pData);

int adi_imu_ReadDelVel              (adi_imu_Device_t *pDevice, adi_imu_DelVelOutput_t *pData);

int adi_imu_ReadBurst               (adi_imu_Device_t *pDevice, adi_imu_BurstOutput_t *pData);

int adi_imu_GetAcclScale            (adi_imu_Device_t *pDevice, adi_imu_AcclScale_t *pData);

int adi_imu_GetGyroScale            (adi_imu_Device_t *pDevice, adi_imu_GyroScale_t *pData);

int adi_imu_GetAcclBias             (adi_imu_Device_t *pDevice, adi_imu_AcclBias_t *pData);

int adi_imu_GetGyroBias             (adi_imu_Device_t *pDevice, adi_imu_GyroBias_t *pData);

int adi_imu_SetPage                 (adi_imu_Device_t *pDevice, uint8_t pageId);

int adi_imu_Read                    (adi_imu_Device_t *pDevice, uint16_t pageIdRegAddr, uint16_t *val);

int adi_imu_ReadBurstRaw            (adi_imu_Device_t *pDevice, uint16_t pageIdRegAddr, uint8_t *buf, unsigned length);

int adi_imu_Write                   (adi_imu_Device_t *pDevice, uint16_t pageIdRegAddr, uint16_t val);

int adi_imu_ConfigGpio              (adi_imu_Device_t *pDevice, enum adi_imu_GPIO_e id, enum adi_imu_Direction_e direction);

int adi_imu_SetGpio                 (adi_imu_Device_t *pDevice, enum adi_imu_GPIO_e id);

int adi_imu_ClearGpio               (adi_imu_Device_t *pDevice, enum adi_imu_GPIO_e id);

int adi_imu_GetGpio                 (adi_imu_Device_t *pDevice, enum adi_imu_GPIO_e id, uint8_t* val);

int adi_imu_ConfigSyncClkMode       (adi_imu_Device_t *pDevice, enum adi_imu_ClockMode_e mode, enum adi_imu_EnDis_e clkEn, \
                                    enum adi_imu_EdgeType_e polarity, enum adi_imu_GPIO_e inputGpio);

int adi_imu_ConfigDataReady         (adi_imu_Device_t *pDevice, enum adi_imu_GPIO_e id, enum adi_imu_Polarity_e polarity);

int adi_imu_SetDataReady            (adi_imu_Device_t *pDevice, enum adi_imu_EnDis_e val);

int adi_imu_SetLineargComp          (adi_imu_Device_t *pDevice, enum adi_imu_EnDis_e val);

int adi_imu_SetPPercAlignment       (adi_imu_Device_t *pDevice, enum adi_imu_EnDis_e val);

int adi_imu_SoftwareReset           (adi_imu_Device_t *pDevice);

int adi_imu_ClearUserCalibration    (adi_imu_Device_t *pDevice);

// int adi_imu_UpdateFlashMemory       (adi_imu_Device_t *pDevice); // TODO: implement

int adi_imu_PerformSelfTest         (adi_imu_Device_t *pDevice);

// int adi_imu_UpdateBiasCorrection    (adi_imu_Device_t *pDevice); // TODO: implement

#ifdef __cplusplus
}
#endif
#endif