/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		asyncio.cpp
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Asynchronous File IO helpers for linux.
 **/

#include <iostream>
#include <cstdio>
#include <cmath>
#include <semaphore.h>
#include <array>
#include <queue>
#include <cstring>
#include <chrono>
#include "asyncio.h"

pthread_t g_thread_h;
pthread_mutex_t g_mutex_lock;
sem_t g_sem_stop_signal;

std::queue<AsyncIOBufElement_e> imqqueue;

int asyncio_create_thread(const char* thread_name, pthread_t* thread, void *(start_routine) (void *), void *arg){
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

int asyncio_cancel_thread(pthread_t thread)
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

void asyncio_put_element(AsyncIOBufElement_e element)
{
    // BufferElement_e buffer;
    // buffer.buf = (uint8_t *) malloc(element.size);
    // buffer.size = element.size;
    // std::memcpy(buffer.buf, element.buf, element.size);
	pthread_mutex_lock(&g_mutex_lock);
	imqqueue.push(element);
	pthread_mutex_unlock(&g_mutex_lock);
}

int asyncio_get_element(AsyncIOBufElement_e* data)
{
	pthread_mutex_lock(&g_mutex_lock);
    if (!imqqueue.empty()) {
        // printf("Fetching queue..\n");
        *data = imqqueue.front();
        pthread_mutex_unlock(&g_mutex_lock);
        return 0;
    }
    else {
        pthread_mutex_unlock(&g_mutex_lock);
        return -1;
    }
}

void asyncio_remove_element()
{
    pthread_mutex_lock(&g_mutex_lock);
    imqqueue.pop();
    pthread_mutex_unlock(&g_mutex_lock);
}

void asyncio_stop()
{
    sem_post(&g_sem_stop_signal);
}

int asyncio_init()
{
    if (pthread_mutex_init(&g_mutex_lock, NULL) !=0){
		printf("\n mutex 'g_mutex_lock' init failed!");
        return -1;
    }

	if ( sem_init ( &g_sem_stop_signal, 0, 0) != 0){
	    printf("semaphore g_sem_stop_signal init failed!\n");
        return -1;
    }
    return 0;
}

int asyncio_is_stop_requested()
{
    if (sem_trywait(&g_sem_stop_signal) == 0){
        printf("asyncio requested to STOP\n");
        return 0;
    }
    return -1;
}

// void *imu_asyncio_loop(void *thread_params)
// {
//     printf("Async IO loop started..\n");
//     while (true) {
//         if (asyncio_is_stop_requested() == 0) break;
//         asyncio_to_disk(fpostProc);
//     }
//     return (void *)1;
// }

int asyncio_start(const char* thread_name, void *(f) (void *), void *arg)
{
    return asyncio_create_thread(thread_name, &g_thread_h, f, arg);
}
