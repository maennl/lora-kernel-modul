//
// Created by marcel on 05.11.18.
//

#ifndef KERNELMODUL_LORA_H
#define KERNELMODUL_LORA_H

#include <linux/ioctl.h>

#define MAX_PKT_LENGTH           255


typedef struct{
    char *buffer;
    int8_t rssi;
    long snr;
} package;

#define LORA_CMD_BASE 'm'

#define LORA_CMD_MODULE_RESET     _IO  (LORA_CMD_BASE, 0)
#define LORA_CMD_MODULE_VERSION   _IOR (LORA_CMD_BASE, 1,__u8)
#define LORA_CMD_MODULE_INIT      _IO  (LORA_CMD_BASE, 2)
#define LORA_CMD_SLEEP            _IO  (LORA_CMD_BASE, 3)
#define LORA_CMD_RECEIVE          _IOW (LORA_CMD_BASE, 4,u16)
#define LORA_CMD_IDLE             _IO  (LORA_CMD_BASE, 5)
#define LORA_CMD_GET_PACKAGE      _IOWR(LORA_CMD_BASE, 6,package)
#define LORA_CMD_PARSE_PACKAGE    _IOWR(LORA_CMD_BASE, 7,u16)
#define LORA_CMD_DI0_FIRED        _IOR (LORA_CMD_BASE, 8,bool)
#define LORA_CMD_DATA_AVAILABLE   _IOR (LORA_CMD_BASE, 9,__u16)
#define LORA_CMD_TRANSMIT         _IO  (LORA_CMD_BASE,10)



#endif //KERNELMODUL_LORA_H
