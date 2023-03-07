/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		uart_driver.c
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Driver interface for linux serial devices.
 **/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>     
#include <time.h>
#include <termios.h>
#include <sys/ioctl.h> 
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/select.h>

#include "uart_driver.h"
// #define DEBUG_UART

inline int uart_RxParse16bit(uint8_t* in, uint16_t* out, size_t len)
{
   char* pEndPrev, *pEnd=(char*)in;
   for(int i=0; i<len; i++)
   {
      out[i] = strtol(pEnd, &pEnd, 16);
      if(i > 0 && pEndPrev == pEnd) return (i+1);
      pEndPrev = pEnd;
   }
   return (len<=1) ? 1 : len+1; // can be used as error condition to check truncation
}

inline int _uart_setup(int fd, int speed)
{
    /*
        Sets up serial I/O on the provided file descriptor
    */

    struct termios tty;
    
    //  Get existing tty flags
    if (tcgetattr(fd, &tty) < 0) DEBUG_PRINT_RET(Err_uart_ConfigFailed_e, "[UART]: Error from tcgetattr: %s\n", strerror(errno));

    //  Modify flags for desired setup
    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY | IUCLC | IMAXBEL);
    tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_lflag |= (ICANON | ISIG); /* enable canonical mode */
    tty.c_oflag &= ~(OPOST | ONLCR);

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0; // * 100 ms

#ifdef __APPLE__
    //  Mac OS hack: use standard baud rate then use special ioctl for baud rate only
    cfsetospeed(&tty, (speed_t)115200);
    cfsetispeed(&tty, (speed_t)115200);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) DEBUG_PRINT_RET(Err_uart_ConfigFailed_e, "[UART]: Error from tcsetattr: %s\n", strerror(errno));
    if (ioctl(fd, IOSSIOSPEED, &speed) < 0) DEBUG_PRINT_RET(Err_uart_ConfigFailed_e, "[UART]: Error from ioctl (Mac OS custom baud rate): %s\n", strerror(errno));
#else
    //  Linux hack: use existing termios structure with recently defined
    //  flags for higher baud rates
    //  (note: TCSETS2 ioctl approach using termios2 doesn't work on Nano)
    const int baud_rate_pairs[][2]={
        {9600, B9600},
        {19200, B19200},
        {38400, B38400},
        {57600, B57600},
        {115200, B115200},
        {230400, B230400},
        {460800, B460800},
        {500000, B500000},
        {576000, B576000},
        {921600, B921600},
        {1000000, B1000000},
        {1152000, B1152000},
        {1500000, B1500000},
        {2000000, B2000000},
        {2500000, B2500000},
        {3000000, B3000000},
        {3500000, B3500000},
        {4000000, B4000000}
    };
    speed_t baud_code = 0;
    for (unsigned i = 0; i < sizeof(baud_rate_pairs); i++) {
        if (baud_rate_pairs[i][0] == speed) {
            baud_code = (speed_t)baud_rate_pairs[i][1];
            break;
        }
    }
    if (baud_code == 0)
        DEBUG_PRINT_RET(Err_uart_BaudRateNotSupported_e, "[UART]: Warning: no matching baud rate found, serial port may not work\n");
    else
    {
        DEBUG_PRINT("[UART]: Selected baud code %d for baud rate %d\n", baud_code, speed);
        cfsetospeed(&tty, baud_code);
        cfsetispeed(&tty, baud_code);
    }
    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) DEBUG_PRINT_RET(Err_uart_ConfigFailed_e, "[UART]: Error from tcsetattr: %s\n", strerror(errno));
#endif

    return 0;
}

inline int uart_Init(adi_imu_UartDevice_t* device)
{
    device->status = IMUBUF_UART_UNKNOWN;

    device->fd = open(device->dev, O_RDWR | O_NOCTTY);
    if (device->fd < 0) DEBUG_PRINT_RET(Err_uart_InitFailed_e, "[UART]: Error: failed to open uart device %s (Error: %s) \n", device->dev, strerror(errno));

    device->status = IMUBUF_UART_OPENED;

    int ret = _uart_setup(device->fd, device->baud);
    if(ret < 0) DEBUG_PRINT_RET(Err_uart_InitFailed_e, "[UART]: Error: failed to setup uart device %s \n", device->dev);

    device->status = IMUBUF_UART_CONFIGURED;

    DEBUG_PRINT("[UART]: device: %s\n", device->dev);
    DEBUG_PRINT("[UART]: Baudrate: %d\n", device->baud);
    DEBUG_PRINT("[UART]: UART successfully initialized.\n");
    return 0;
}

inline int uart_Read(adi_imu_UartDevice_t* device, uint8_t *buf, size_t bufLen)
{
    fd_set set;
    FD_ZERO(&set); /* clear the set */
    FD_SET(device->fd, &set); /* add our file descriptor to the set */

    struct timeval timeout;
    timeout.tv_sec = 7;
    timeout.tv_usec = 0;

    int rdlen = select(device->fd + 1, &set, NULL, NULL, &timeout);
    if(rdlen == -1)
        DEBUG_PRINT_RET(Err_uart_ReadFailed_e, "[UART]: Read errored. Error: %s\n", strerror(errno)); /* an error accured */
    else if(rdlen == 0)
        DEBUG_PRINT_RET(Err_uart_ReadTimedout_e, "[UART]: Read timedout.\n"); /* a timeout occured */
    else
        rdlen = read(device->fd, buf, bufLen-1);
    if (rdlen < 0) 
    {
        if(errno != EAGAIN) // for non-block mode
            DEBUG_PRINT_RET(Err_uart_ReadFailed_e, "[UART]: Read errored. Ret: %d Error: %s\n", rdlen, strerror(errno));
        else
        {
            buf[0] = '\0';
            return 0;
        }
    }
    else if(rdlen == 0) 
    {
        buf[rdlen] = '\0';
        return Err_uart_ReadEmpty_e;
    }
    else{
        // appending null at the end to operate as string
        buf[rdlen] = '\0';
    }
#ifdef DEBUG_UART
    printf("[UART RX]: %s\n", buf);
#endif
    return rdlen;
}

inline int uart_ReadLine(adi_imu_UartDevice_t* device, uint8_t *buf, size_t bufLen)
{
    int rdlen = 0;
    for(int i=0; i<(bufLen-1); i++)
    {
        buf[i] = '\0';
        rdlen = read(device->fd, buf+i, 1);
        if (rdlen < 0) 
        {
            if(errno != EAGAIN) // for non-block mode
                DEBUG_PRINT_RET(Err_uart_ReadFailed_e, "[UART]: Read errored. Ret: %d Error: %s\n", rdlen, strerror(errno));
            else {
                i--;
                continue;
            }
        }
        else if(rdlen == 0) 
        {
            buf[rdlen] = '\0';
            return Err_uart_ReadEmpty_e;
        }
        else if(buf[i] == '\n') {
            rdlen = i+1;
            // appending null at the end to operate as string
            buf[rdlen] = '\0';
            break;
        }
    }
#ifdef DEBUG_UART
    printf("[UART RX]: %s\n", buf);
#endif
    return rdlen;
}

inline int uart_Write(adi_imu_UartDevice_t* device, const uint8_t *buf, size_t bufLen)
{
#ifdef DEBUG_UART
    printf("[UART TX]: %s\n", buf);
#endif
    delay_MicroSeconds(10); // TODO: Check if need to adjust once FW supports all modules
    int ret = write(device->fd, buf, bufLen);
    if (ret != bufLen) DEBUG_PRINT_RET(Err_uart_WriteFailed_e, "[UART]: Write errored. Ret: %d Error: %s\n", ret, strerror(errno));
    tcdrain(device->fd);    /* delay for output */
    return Err_imu_Success_e;
}

inline int uart_FlushInput(adi_imu_UartDevice_t* device)
{
    tcflush(device->fd, TCIFLUSH);
    return Err_imu_Success_e;
}

inline int uart_FlushOutput(adi_imu_UartDevice_t* device)
{
    tcflush(device->fd, TCOFLUSH);
    return Err_imu_Success_e;
}

inline int uart_WriteRead(adi_imu_UartDevice_t* device, const uint8_t *txBuf, size_t txBufLen, uint8_t *rxBuf, size_t rxBufLen)
{
    int ret = Err_imu_Success_e;
    if (txBuf != NULL && txBufLen > 0)
    {
        if ((ret = uart_Write(device, txBuf, txBufLen)) < 0) return ret;
    }

    if ((txBuf != NULL && txBufLen > 0) && (rxBuf != NULL && rxBufLen > 0))
        delay_MicroSeconds(100000);

    if (rxBuf != NULL && rxBufLen > 0)
    {
        if ((ret = uart_Read(device, rxBuf, rxBufLen)) < 0) return ret;
    }
    return ret;
}

// int uart_Write_and_Read(adi_imu_Device_t *pDevice, uint8_t *txBuf, uint8_t *rxBuf, size_t xferLen, uint32_t numXfers, uint32_t numRepeats, uint32_t enRepeatTx)
// {
//     int ret = 0;
//     const uint32_t total_xfers = numXfers * numRepeats;
//     for (int i=0; i<numRepeats; i++)
//     {
//         for (int j=0; i<numXfers; i++)
//         {
//             int xfer_idx = i * numXfers + j;
//             uint8_t* _txBuf, _rxBuf;
//             if (enRepeatTx == 0)
//                 _txBuf = txBuf + xferLen * xfer_idx;
//             else
//                 _txBuf = txBuf + xferLen * j;
//             _rxBuf = rxBuf + xferLen * xfer_idx;

//             ret = uart_Write(pDevice, _txBuf, xferLen);
//             if (ret < 0) return ret;

//             delay_MicroSeconds(1000);
            
//             ret = uart_Read(pDevice, _rxBuf, xferLen);
//             if (ret < 0) return ret;
//         }
//     }
//     return 0;
// }
