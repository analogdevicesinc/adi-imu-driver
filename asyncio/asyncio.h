/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		asyncio.h
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Asynchronous File IO helpers for linux.
 **/

#ifndef __ASYNCIO_H_
#define __ASYNCIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <pthread.h>

typedef struct {
    uint8_t* buf;
    size_t size;
} AsyncIOBufElement_e;

int asyncio_init();
int asyncio_start(const char* thread_name, void *(f) (void *), void *arg);
void asyncio_stop();
int asyncio_is_stop_requested();
void asyncio_put_element(AsyncIOBufElement_e element);
int asyncio_get_element(AsyncIOBufElement_e* data);
void asyncio_remove_element();

#ifdef __cplusplus
}
#endif
#endif