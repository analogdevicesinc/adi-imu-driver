/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		imu_fast_main.c
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Example usage of IMU driver.
 **/


#include <iostream>
#include <cstdio>
#include <cmath>
#include "adi_imu_driver.h"
#include "spi_driver.h"
#include <pthread.h>
#include <semaphore.h>
#include <array>
#include <queue>
#include <cstring>
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
std::queue<std::array<uint8_t, MAX_BRF_LEN_BYTES*BURSTS_PER_READ> > imqqueue;

void printbuf(const char* header, uint16_t* buf, int buflen)
{
    printf("%s", header);
    for (int i=0; i<buflen; i++)
        printf("0x%04X ", buf[i]);
    printf("\n");
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

void cleanup(adi_imu_Device_t *imu)
{
    // adi_imu_PerformSelfTest(imu);
    exit(0);
}

void imu_store_data(std::array<uint8_t,MAX_BRF_LEN_BYTES*BURSTS_PER_READ> data)
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
        uint8_t* buf = imqqueue.front().data();
        pthread_mutex_unlock(&lock_imu_data);
        for (int i=0; i<BURSTS_PER_READ; i++){
            adi_imu_BurstOutput_t out;
                // printbuf("buffer: ", (uint16_t*) (buf + i * MAX_BRF_LEN_BYTES), MAX_BRF_LEN_BYTES/2);
            if (adi_imu_ScaleBurstOut_1(imu, buf + i * MAX_BRF_LEN_BYTES, TRUE, &out) >= 0)
            {
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
            // printf("datacnt_Or_ts=%d, sys_status=%d, temp=%lf\u2103, accX=%lf, accY=%lf, accZ=%lf, gyroX=%lf, gyroY=%lf, gyroZ=%lf\n", out.dataCntOrTimeStamp, out.sysEFlag, out.tempOut, out.accl.x, out.accl.y, out.accl.z, out.gyro.x, out.gyro.y, out.gyro.z);
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
    std::array<uint8_t,MAX_BRF_LEN_BYTES*BURSTS_PER_READ> myarray;
    int ret = 0;
    while (true) {
        if (sem_trywait(&sem_stop_data_fetch) == 0){
            printf("Stopped fecthing data\n");
            break;
        }
        // printf("Fetching data..\n");
        // auto start = std::chrono::high_resolution_clock::now();
        if ((ret = adi_imu_ReadBurstRaw(params->imu, myarray.data(), BURSTS_PER_READ)) < 0){
            printf("Error: During Imu data fetch. (returned %d)\n", ret);
        }
        // auto end = std::chrono::high_resolution_clock::now();
        // std::cout << "avg fetching time: " << std::chrono::duration_cast<std::chrono::microseconds>(end-start).count() <<" us" << std::endl;
        
        if (ret >= 0)
            imu_store_data(myarray);
        // printf("Fetching data..done\n");
    }
    return (void*)1;
}

int main()
{
    adi_imu_Device_t imu;
    imu.prodId = 16545;
    imu.g = 1.0;
    imu.spiDev = "/dev/spidev1.0";
    imu.spiSpeed = 2000000;
    imu.spiMode = 3;
    imu.spiBitsPerWord = 8;
    imu.spiDelay = 0;

    /* Initialize spi */
    int ret = spi_Init(&imu);
    if (ret < 0) return ret;

    /* Initialize IMU */
    ret = adi_imu_Init(&imu);
    if (ret != adi_imu_Success_e) return ret;

    /* Set output data rate */
    if ((ret = adi_imu_SetOutputDataRate(&imu, 2000)) < 0) return ret;
    
    // /* Set DATA ready pin */
    if ((ret = adi_imu_ConfigDataReady(&imu, DIO2, POSITIVE)) < 0) return ret;
    if ((ret = adi_imu_SetDataReady(&imu, ENABLE)) < 0) return ret;
    if ((ret = adi_imu_ConfigSyncClkMode(&imu, SYNC, DISABLE, FALLING_EDGE, DIO1)) < 0) return ret;

    // /* Read and print IMU info and config */
    // adi_imu_DevInfo_t imuInfo;
    // if ((ret = adi_imu_GetDevInfo(&imu, &imuInfo)) < 0) return ret;
    // if ((ret = adi_imu_PrintDevInfo(&imu, &imuInfo)) < 0) return ret;

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
    for (i=0; i<400000; i++){
        imu_process_next_data(&imu);
        delay_MicroSeconds(10);
    }
    printf("%d iteration completed\n", i);

    sem_post(&sem_stop_data_fetch);

    /* Perform self test and display results*/
    // if ((ret = adi_imu_PerformSelfTest(&imu)) < 0) return ret;

    return 0;
}