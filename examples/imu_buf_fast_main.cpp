#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "adi_imu_driver.h"
#include "spi_driver.h"
#include "imu_spi_buffer.h"
#include <pthread.h>
#include <semaphore.h>
#include <vector>
#include <queue>
#include <cstring>
#include <errno.h>
#include <chrono>

#define BURSTS_PER_READ 10
pthread_t thread_imu;
pthread_mutex_t lock_imu_data;
sem_t sem_stop_data_fetch;
uint16_t curDataCnt = 0;
uint16_t curreadidx = 0;

typedef struct  {
    adi_imu_Device_t* imu;
} threadParams_t;

threadParams_t g_thread_params;
std::queue<std::vector<uint8_t> > imqqueue;
typedef struct {
    int16_t dummy1; // dummy response from previous last transaction
    int16_t dummy2; // dummy response for buf_retrieve command
    uint16_t sysEFlag;
    int16_t tempOut;
    int32_t gyroX;
    int32_t gyroY;
    int32_t gyroZ;
    int32_t acclX;
    int32_t acclY;
    int32_t acclZ;
    uint16_t dataCntOrTimeStamp;
    uint32_t crc;
    // uint32_t ts;
} __attribute__ ((packed)) BufOutputRaw_t;

void printbuf(const char* header, uint16_t* buf, int buflen)
{
    printf("%s", header);
    for (int i=0; i<buflen; i++)
        printf("0x%04X ", buf[i]);
    printf("\n");
}

void cleanup(adi_imu_Device_t *imu)
{

    sem_post(&sem_stop_data_fetch);
    /* wait 1 second for thread to quit */
    delay_MicroSeconds(1000000);

    uint16_t curBufCnt = 0;
    int ret = 0;
    imubuf_StopCapture(imu, &curBufCnt);

    // read any error flags
    imubuf_DevInfo_t imuBufInfo;
    imubuf_GetInfo(imu, &imuBufInfo);
    imubuf_PrintInfo(imu, &imuBufInfo);
    exit(0);
}

int imu_create_thread(const char* thread_name, pthread_t* thread, void *(start_routine) (void *), void *arg){
	int err = pthread_create(thread, NULL, start_routine, arg);
	if (err != 0) {
		printf("can't create thread [%s]:[%s]\n", thread_name, strerror(err));
		return -1;
	}

	err = pthread_setname_np(*thread, thread_name);
	if (err != 0) {
		printf("can't set thread name for thread [%s]:[%s]\n", thread_name, strerror(err));
		return -1;
	}
	return 0;
}

int imu_cancel_thread(pthread_t thread)
{
	char thread_name[16];
	pthread_getname_np(thread, thread_name, 16);
	printf("Cancelling thread [%s]..\n", thread_name);
	int err = pthread_cancel( thread );
	if (err == 0){
		printf("Cancel success, detaching thread [%s]..done\n", thread_name);
		pthread_detach( thread );
	}
	if (err == ESRCH) {
		printf("can't cancel thread [%s] :[%s]\n", thread_name, strerror(err));
		return -1;
	}
	printf("Cancelling thread [%s]..done\n", thread_name);
	return 0;
}

void imu_store_data(std::vector<uint8_t> data)
{
	pthread_mutex_lock(&lock_imu_data);
	imqqueue.push(data);
	pthread_mutex_unlock(&lock_imu_data);
}

void imu_process_next_data(adi_imu_Device_t* imu)
{
	pthread_mutex_lock(&lock_imu_data);
    if (!imqqueue.empty()) {
        // printf("Fetching queue..\n");
        int size = imqqueue.front().size() / sizeof(BufOutputRaw_t);
        uint8_t* buf = imqqueue.front().data();
        pthread_mutex_unlock(&lock_imu_data);
        for (int i=0; i<size; i++){
            adi_imu_BurstOutput_t out;
            // auto start = std::chrono::high_resolution_clock::now();
            // printbuf("::", (uint16_t *)buf + (sizeof(BufOutputRaw_t)/2) * i , sizeof(BufOutputRaw_t)/2);
            if (adi_imu_ScaleBurstOut_1(imu, buf + i * sizeof(BufOutputRaw_t) + 4, FALSE, &out) >= 0)
            {
                // auto end = std::chrono::high_resolution_clock::now();
                // std::cout << "avg processing time: " << std::chrono::duration_cast<std::chrono::microseconds>(end-start).count() <<" us" << std::endl;

                // printf("\ndatacnt_Or_ts=%d, sys_status=%d, temp=%lf\u2103, accX=%lf, accY=%lf, accZ=%lf, gyroX=%lf, gyroY=%lf, gyroZ=%lf\n", out.dataCntOrTimeStamp, out.sysEFlag, out.tempOut, out.accl.x, out.accl.y, out.accl.z, out.gyro.x, out.gyro.y, out.gyro.z);
                uint16_t dc = out.dataCntOrTimeStamp;
                // if (j == 24444 && n == 0) dc = 0x9; // insert fail condition
                if (curreadidx > 2 && dc != 0 && dc != curDataCnt) {
                    if (dc % 1000 == 0) printf("%d\n", dc);
                    if (curDataCnt != 0 && dc != (curDataCnt+1)){
                        printf("%d\n%d ##", curDataCnt, dc);
                        printf("\nTEST FAILED\n");
                        cleanup(imu);
                    }
                    curDataCnt = dc;
                }
                curreadidx++;
            // printf("Pitch = %f deg \n", 180 * atan2(out.accl.x, sqrt(out.accl.y*out.accl.y + out.accl.z*out.accl.z))/M_PI);
            // printf("Roll = %f deg\n", 180 * atan2(out.accl.y, sqrt(out.accl.x*out.accl.x + out.accl.z*out.accl.z))/M_PI);
            }
            else{
                printf("\nTEST FAILED\n");
                cleanup(imu);
            }

        }

	    pthread_mutex_lock(&lock_imu_data);
        imqqueue.pop();
        pthread_mutex_unlock(&lock_imu_data);

    }
    else {
        pthread_mutex_unlock(&lock_imu_data);
        // printf("Empty queue..\n");
    }
}

void *imu_fetch_data_loop(void *thread_params)
{
    threadParams_t* params = (threadParams_t*) thread_params;
    /* allocate memory */
    std::vector<uint8_t> myarray (sizeof(BufOutputRaw_t) * BURSTS_PER_READ, 0);
    int ret = 0;
    uint16_t buf_len = 0;
    int32_t readBufCnt = 0;
    // if ((ret = imubuf_ReadBufferAutoMax(params->imu, BURSTS_PER_READ, &readBufCnt, (uint16_t *)myarray.data(), &buf_len)) < 0)
    while (true) {
        if (sem_trywait(&sem_stop_data_fetch) == 0){
            printf("Stopped fecthing data\n");
            break;
        }
        // printf("Fetching data..\n");
        readBufCnt = 0;
        buf_len = 0;
        // auto start = std::chrono::high_resolution_clock::now();
        if ((ret = imubuf_ReadBufferAutoMax(params->imu, BURSTS_PER_READ, &readBufCnt, (uint16_t *)myarray.data(), &buf_len)) < 0)
            printf("Error: During Imu data fetch. (returned %d)\n", ret);
        // auto end = std::chrono::high_resolution_clock::now();
        if (readBufCnt > 0)
            imu_store_data(myarray);
        // printf("Fetching data..done\n");
        // std::cout << "avg fetching time: " << std::chrono::duration_cast<std::chrono::microseconds>(end-start).count() <<" us" << std::endl;
    }
    return (void*)1;
}

int main()
{
    adi_imu_Device_t imu;
    imu.prodId = 16545;
    imu.g = 1.0;
    imu.spiDev = "/dev/spidev1.0";
    imu.spiSpeed = 4000000;
    imu.spiMode = 3;
    imu.spiBitsPerWord = 8;
    imu.spiDelay = 100; // stall time (us); to be safe

    /* initialize spi device */
    int ret = spi_Init(&imu);
    if (ret < 0) return ret;

    /* detect is buffer board is present */
    ret = imubuf_Detect(&imu);
    if (ret < 0) return ret;

    /* Initialize IMU BUF first to stop any activity*/
    ret = imubuf_init(&imu);
    if (ret != adi_imu_Success_e) return ret;

    /* Read and print iSensor SPI Buffer info and config*/
    imubuf_DevInfo_t imuBufInfo;
    if ((ret = imubuf_GetInfo(&imu, &imuBufInfo)) < 0) return ret;
    if ((ret = imubuf_PrintInfo(&imu, &imuBufInfo)) < 0) return ret;

    /* software reset */
    // if ((ret = imubuf_SoftwareReset(&imu)) < 0) return ret;
    
    /* Initialize IMU */
    ret = adi_imu_Init(&imu);
    if (ret != adi_imu_Success_e) return ret;

    /* Set DATA ready pin */
    if ((ret = adi_imu_ConfigDataReady(&imu, DIO1, POSITIVE)) < 0) return ret;
    if ((ret = adi_imu_SetDataReady(&imu, ENABLE)) < 0) return ret;

    /* Set output data rate */
    if ((ret = adi_imu_SetOutputDataRate(&imu, 1000)) < 0) return ret;
    
    /* Read and print IMU device info and config */
    adi_imu_DevInfo_t imuInfo;
    if ((ret = adi_imu_GetDevInfo(&imu, &imuInfo)) < 0) return ret;
    if ((ret = adi_imu_PrintDevInfo(&imu, &imuInfo)) < 0) return ret;

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

    /* enable burst mode */
    imubuf_BufConfig_t config;
    config.overflowAction = 0;
    config.imuBurstEn = 0;
    config.bufBurstEn = 0;
    if ((ret = imubuf_ConfigBuf(&imu, config)) < 0) return ret;

    #define MAX_BUF_LENGTH 1000 // should greater than (imu_output_rate / fetch_rate). Ex: (4000Hz / 200Hz) = 20
    typedef struct {
        int16_t dummy1; // dummy response from previous last transaction
        int16_t dummy2; // dummy response for buf_retrieve command
        uint16_t sysEFlag;
        int16_t tempOut;
        int32_t gyroX;
        int32_t gyroY;
        int32_t gyroZ;
        int32_t acclX;
        int32_t acclY;
        int32_t acclZ;
        uint16_t dataCntOrTimeStamp;
        uint32_t crc;
        // uint32_t ts;
    } __attribute__ ((packed)) BufOutputRaw_t;

    BufOutputRaw_t bufRawOut[MAX_BUF_LENGTH] = {0};
    adi_imu_BurstOutput_t burstOut = {0};

    /* set register pattern to read/write IMU registers after every data ready interrupt */
     uint16_t bufPattern[] = {REG_SYS_E_FLAG, REG_TEMP_OUT,\
                            REG_X_GYRO_LOW, REG_X_GYRO_OUT, REG_Y_GYRO_LOW, REG_Y_GYRO_OUT, REG_Z_GYRO_LOW, REG_Z_GYRO_OUT,\
                            REG_X_ACCL_LOW, REG_X_ACCL_OUT, REG_Y_ACCL_LOW, REG_Y_ACCL_OUT, REG_Z_ACCL_LOW, REG_Z_ACCL_OUT, \
                            REG_DATA_CNT, REG_CRC_LWR, REG_CRC_UPR}; //, REG_ISENSOR_BUF_TIMESTAMP_LWR, REG_ISENSOR_BUF_TIMESTAMP_UPR};
    // uint16_t bufPattern[] = {REG_SYS_E_FLAG, REG_DATA_CNT, REG_TEMP_OUT };
    uint16_t bufPatternLen = (uint16_t) (sizeof(bufPattern)/sizeof(uint16_t));
    if ((ret = imubuf_SetPatternAuto(&imu, bufPatternLen, bufPattern)) < 0) return ret;

    if ((ret = imubuf_GetInfo(&imu, &imuBufInfo)) < 0) return ret;
    if ((ret = imubuf_PrintInfo(&imu, &imuBufInfo)) < 0) return ret;

    /* start capture */
    uint16_t curBufCnt = 0;
    if ((ret = imubuf_StartCapture(&imu, IMUBUF_TRUE, &curBufCnt)) < 0) return ret;
    imu.spiDelay = 20;  // kernel latency is large enough for stall time

    if (pthread_mutex_init(&lock_imu_data, NULL) !=0)
		printf("\n mutex 'lock_imu_data' init failed!");

	if ( sem_init ( &sem_stop_data_fetch, 0, 0) != 0)
	printf("semaphore sem_stop_data_fetch init failed!\n");

    g_thread_params.imu = &imu;
    /* Burst read 10 samples */
    printf("\nPerforming burst read..\n");
    imu_create_thread("adimu", &thread_imu, imu_fetch_data_loop, (void*) &g_thread_params);
    int i=0;
    // Using adi_imu_ReadBurstRaw
    for (i=0; i<500000; i++){
        delay_MicroSeconds(5);
        imu_process_next_data(&imu);
    }
    printf("%d iteration completed\n", i);

    sem_post(&sem_stop_data_fetch);
    /* wait 1 second for thread to quit */
    delay_MicroSeconds(1000000);

    imu.spiDelay = 100;
    /* stop capture */
    if (( ret = imubuf_StopCapture(&imu, &curBufCnt)) < 0) return ret;
    printf("\n\nTEST PASSED\n");

    return 0;
}