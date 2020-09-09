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
void (*fpostProc)(AsyncIOBufElement_e);

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

void asyncio_to_queue(AsyncIOBufElement_e element)
{
    // BufferElement_e buffer;
    // buffer.buf = (uint8_t *) malloc(element.size);
    // buffer.size = element.size;
    // std::memcpy(buffer.buf, element.buf, element.size);
	pthread_mutex_lock(&g_mutex_lock);
	imqqueue.push(element);
	pthread_mutex_unlock(&g_mutex_lock);
}

void asyncio_to_disk(void (*f_postproc)(AsyncIOBufElement_e))
{
	pthread_mutex_lock(&g_mutex_lock);
    if (!imqqueue.empty()) {
        // printf("Fetching queue..\n");
        AsyncIOBufElement_e burst_data = imqqueue.front();
        pthread_mutex_unlock(&g_mutex_lock);
        if (f_postproc != NULL) f_postproc(burst_data);
        // free(burst_data.buf);
	    pthread_mutex_lock(&g_mutex_lock);
        imqqueue.pop();
        pthread_mutex_unlock(&g_mutex_lock);
    }
    else {
        pthread_mutex_unlock(&g_mutex_lock);
        // printf("Empty queue..\n");
    }
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

void *imu_asyncio_loop(void *thread_params)
{
    printf("File IO loop started..\n");
    while (true) {
        if (sem_trywait(&g_sem_stop_signal) == 0){
            printf("asyncio requested to STOP\n");
            break;
        }
        asyncio_to_disk(fpostProc);
    }
    return (void *)1;
}

int asyncio_start(const char* thread_name, void (*f)(AsyncIOBufElement_e))
{
    fpostProc = f;
    return asyncio_create_thread(thread_name, &g_thread_h, imu_asyncio_loop, NULL);
}
