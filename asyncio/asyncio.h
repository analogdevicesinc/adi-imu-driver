/*******************************************************************************
 *   @file   asyncio.h
 *   @brief  Helpers to write data to file based devices (optionally in a separate thread)
 *   @author Sundar Palani <sundar.palani@analog.com>
********************************************************************************/

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
int asyncio_start(const char* thread_name, void (*f)(AsyncIOBufElement_e));
void asyncio_stop();
void asyncio_to_queue(AsyncIOBufElement_e element);

#ifdef __cplusplus
}
#endif
#endif