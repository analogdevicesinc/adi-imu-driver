/*
    File: util.c

    Description:
    utility helpers

    Author: Sundar Palani <sundar.palani@analog.com>
*/

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "util.h"

#ifdef WIN32
#include <windows.h>
#elif _POSIX_C_SOURCE >= 199309L
#include <time.h>   // for nanosleep
#else
#include <unistd.h> // for usleep
#endif
void sleep_us(int microseconds) // cross-platform sleep function
{
#ifdef WIN32
    Sleep(microseconds); // For windows, not tested. Might not work.
#elif _POSIX_C_SOURCE >= 199309L
    struct timespec ts;
    ts.tv_sec = microseconds / 1000000;
    ts.tv_nsec = (microseconds % 1000000) * 1000;
    nanosleep(&ts, NULL);
#else
    usleep(microseconds);
#endif
}
