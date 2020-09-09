#include <iostream>
#include <cstdio>
#include <cmath>
#include <semaphore.h>
#include <array>
#include <queue>
#include <cstring>
#include <chrono>
#include "fileio.h"

pthread_t g_thread_h;
pthread_mutex_t g_mutex_lock;
sem_t g_sem_stop_signal;
void (*fpostProc)(uint8_t*, size_t);

typedef struct {
    uint8_t* buf;
    size_t size;
} BufferElement_e;

std::queue<BufferElement_e> imqqueue;

int fileio_create_thread(const char* thread_name, pthread_t* thread, void *(start_routine) (void *), void *arg){
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

int fileio_cancel_thread(pthread_t thread)
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

void fileio_to_queue(uint8_t* buf, size_t size)
{
    uint8_t* data = (uint8_t *) malloc(size);
    std::memcpy(data, buf, size);
    BufferElement_e buffer = {data, size};
	pthread_mutex_lock(&g_mutex_lock);
	imqqueue.push(buffer);
	pthread_mutex_unlock(&g_mutex_lock);
}

void fileio_to_disk(void (*f_postproc)(uint8_t*, size_t))
{
	pthread_mutex_lock(&g_mutex_lock);
    if (!imqqueue.empty()) {
        // printf("Fetching queue..\n");
        BufferElement_e burst_data = imqqueue.front();
        pthread_mutex_unlock(&g_mutex_lock);
        if (f_postproc != NULL) f_postproc(burst_data.buf, burst_data.size);
        free(burst_data.buf);
	    pthread_mutex_lock(&g_mutex_lock);
        imqqueue.pop();
        pthread_mutex_unlock(&g_mutex_lock);
    }
    else {
        pthread_mutex_unlock(&g_mutex_lock);
        // printf("Empty queue..\n");
    }
}

void fileio_stop()
{
    sem_post(&g_sem_stop_signal);
}

int fileio_init()
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

void *imu_fileio_loop(void *thread_params)
{
    printf("File IO loop started..\n");
    while (true) {
        if (sem_trywait(&g_sem_stop_signal) == 0){
            printf("fileio requested to STOP\n");
            break;
        }
        fileio_to_disk(fpostProc);
    }
    return (void *)1;
}

int fileio_start(const char* thread_name, void (*f)(uint8_t*, size_t))
{
    fpostProc = f;
    return fileio_create_thread(thread_name, &g_thread_h, imu_fileio_loop, NULL);
}
