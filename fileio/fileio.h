/*******************************************************************************
 *   @file   fileio.h
 *   @brief  Helpers to write data to file based devices (optionally in a separate thread)
 *   @author Sundar Palani <sundar.palani@analog.com>
********************************************************************************/

#ifndef __FILEIO_H_
#define __FILEIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <pthread.h>

int fileio_init();
int fileio_start(const char* thread_name, void (*f)(uint8_t*, size_t));
void fileio_stop();
void fileio_to_queue(uint8_t* buf, size_t size);

#ifdef __cplusplus
}
#endif
#endif