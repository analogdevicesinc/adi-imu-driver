#include <stdio.h>
#include <math.h>
#include "adi_imu_driver.h"
#include "spi_driver.h"
#include "imu_spi_buffer.h"

int main()
{
    adi_imu_Device_t imu;
    imu.prodId = 16545;
    imu.spiDev = "/dev/spidev0.0";
    imu.spiSpeed = 2000000;
    imu.spiMode = 3;
    imu.spiBitsPerWord = 8;
    imu.spiDelay = 100; // stall time (us); to be safe

    /* Initialize IMU BUF */
    int ret = imubuf_init(&imu);
    if (ret != adi_imu_Success_e) return ret;

    /* set DATA ready pin */
    if ((ret = adi_imu_ConfigDataReady(&imu, DIO1, RISING_EDGE)) < 0) return ret;
    if ((ret = adi_imu_SetDataReady(&imu, ENABLE)) < 0) return ret;

    /* Read and print IMU device info and config */
    adi_imu_DevInfo_t imuInfo;
    if ((ret = adi_imu_GetDevInfo(&imu, &imuInfo)) < 0) return ret;
    if ((ret = adi_imu_PrintDevInfo(&imu, &imuInfo)) < 0) return ret;

    /* Read and print iSensor SPI Buffer info and config*/
    imubuf_DevInfo_t imuBufInfo;
    if ((ret = imubuf_GetInfo(&imu, &imuBufInfo)) < 0) return ret;
    if ((ret = imubuf_PrintInfo(&imu, &imuBufInfo)) < 0) return ret;

    /* set DIO pin config (both input and output) for iSensor SPI buffer */
    imubuf_ImuDioConfig_t dioConfig;
    dioConfig.dataReadyPin = IMUBUF_DIO1;
    dioConfig.dataReadyPolarity = RISING_EDGE;
    dioConfig.ppsPin = 0x00;
    dioConfig.ppsPolarity = 0x0;
    dioConfig.passThruPin = 0x00;
    dioConfig.watermarkIrqPin = IMUBUF_DIO2;
    dioConfig.overflowIrqPin = 0x00;
    dioConfig.errorIrqPin = 0x00;
    if ((ret = imubuf_ConfigDio(&imu, dioConfig)) < 0) return ret;

    /* set register pattern to read/write IMU registers after every data ready interrupt */
    uint16_t bufPattern[] = {REG_DATA_CNT, REG_X_ACCL_LOW, REG_X_ACCL_OUT, REG_Y_ACCL_LOW, REG_Y_ACCL_OUT, REG_Z_ACCL_LOW, REG_Z_ACCL_OUT};
    uint16_t bufLen = (uint16_t) (sizeof(bufPattern)/sizeof(uint16_t));
    if ((ret = imubuf_SetPatternAuto(&imu, bufLen, bufPattern)) < 0) return ret;
    
    /* read back pattern for sanity check */
    uint16_t patternChk[7] = {0};
    uint16_t buflen = 0;
    if ((ret = imubuf_GetPattern(&imu, &buflen, patternChk)) < 0) return ret;
    printf("Pattern: ");
    for(int i=0; i< (buflen/2); i++) printf("0x%04X ", patternChk[i]);
    printf("\n");

    /* stop capture and clear any old buffered data */
    uint16_t curBufCnt = 0;
    if ((ret = imubuf_StopCapture(&imu, IMUBUF_TRUE, &curBufCnt)) < 0) return ret;

    typedef struct {
        int32_t acclX;
        int32_t acclY;
        int32_t acclZ;
    } acclOut;
    
    /* Burst read 10 samples */
    float acclLSB  = 0.25 * 9.81 / 65536000; /* 0.25mg/2^16 */
    float gyroLSB  = (4 * 10000 * 0.00625 / 655360000) * ( M_PI / 180); /* 0.00625 deg / 2^16 */
    float tempLSB = (1.0/80);

    /* start capture */
    if ((ret = imubuf_StartCapture(&imu, IMUBUF_FALSE, &curBufCnt)) < 0) return ret;

    uint16_t buf_len = 0;
    uint16_t buf[100] = {0};
    const uint16_t MAX_BUF_CNT_MEM = 100/bufLen;

    int32_t readBufCnt = 0;
    DEBUG_PRINT("\nReading max 5 buffers at a time\n");
    for(int j=0; j<5; j++){
        adi_imu_DelayMicroSeconds(1000000);
        if ((ret = imubuf_ReadBufferAutoMax(&imu, 5, &readBufCnt, buf, &buf_len)) < 0) return ret;
        for (int n=0; n<readBufCnt; n++) {
                acclOut* aOut = (acclOut*) (buf + n * buf_len + 1);
                DEBUG_PRINT("Reading buffer: [%d, %f, %f, %f] \n", buf[0 + n * buf_len], aOut->acclX * acclLSB, aOut->acclY * acclLSB, aOut->acclZ * acclLSB);
        }
    }
    DEBUG_PRINT("\nReading all available data at a time\n");
    for(int j=0; j<10; j++){
        adi_imu_DelayMicroSeconds(1000000);
        if ((ret = imubuf_ReadBufferAutoMax(&imu, MAX_BUF_CNT_MEM, &readBufCnt, buf, &buf_len)) <0) return ret;
        for (int n=0; n<readBufCnt; n++) {
                acclOut* aOut = (acclOut*) (buf + n * buf_len + 1);
                DEBUG_PRINT("Reading buffer: [%d, %f, %f, %f] \n", buf[0 + n * buf_len], aOut->acclX * acclLSB, aOut->acclY * acclLSB, aOut->acclZ * acclLSB);
        }
    }

    /* stop capture */
    if (( ret = imubuf_StopCapture(&imu, IMUBUF_FALSE, &curBufCnt)) < 0) return ret;

    return 0;
}