#ifndef _RETARGET_H__
#define _RETARGET_H__

#include "stm32f4xx_hal.h"
#include <sys/stat.h>
#include <stdio.h>

void RetargetInit(UART_HandleTypeDef *huart);

__attribute__((weak)) int _isatty(int fd);

__attribute__((weak)) int _write(int fd, char *ptr, int len);

__attribute__((weak)) int _close(int fd);

__attribute__((weak)) int _lseek(int fd, int ptr, int dir);

__attribute__((weak)) int _read(int fd, char *ptr, int len);

__attribute__((weak)) int _fstat(int fd, struct stat *st);

#endif //#ifndef _RETARGET_H__