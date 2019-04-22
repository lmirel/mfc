/*
 Copyright (c) 2015 Mathieu Laurendeau
 License: GPLv3
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <stdint.h>
#include <limits.h>

#ifdef WIN32
#define PACKED __attribute__((gcc_struct, packed))
#else
#define PACKED __attribute__((packed))
#endif

#define USART_BAUDRATE 500000

// the atmega32u4 has 2.5Kbytes SRAM
#define MAX_DESCRIPTORS_SIZE 1024

#define MAX_DESCRIPTORS 32 // should not exceed 255

// the atmega32u4 supports up to 6 non-control endpoints
#define MAX_ENDPOINTS 6

#define MAX_PACKET_SIZE_EP0 64

#define MAX_PAYLOAD_SIZE_EP 64 // for non-control endpoints

typedef struct PACKED {
  uint16_t offset;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} s_descriptorIndex;

typedef struct PACKED {
  uint8_t number; // 0 means end of table
  uint8_t type;
  uint8_t size;
} s_endpointConfig;

typedef struct PACKED {
  uint8_t endpoint; // 0 means nothing to send
  uint8_t data[MAX_PAYLOAD_SIZE_EP];
} s_endpointPacket; // should not exceed 255 bytes

typedef enum {
  E_TYPE_DESCRIPTORS,   //0
  E_TYPE_INDEX,         //1
  E_TYPE_ENDPOINTS,     //2
  E_TYPE_RESET,         //3
  E_TYPE_CONTROL,       //4
  E_TYPE_CONTROL_STALL, //5
  E_TYPE_IN,            //6
  E_TYPE_OUT,           //7
  E_TYPE_DEBUG,         //8
} e_packetType;

#define BYTE_LEN_0_BYTE   0x00
#define BYTE_LEN_1_BYTE   0x01

// this is the max serial packet size
#define MAX_PACKET_SIZE 256

typedef struct PACKED {
  uint8_t type;
  uint8_t length;
} s_header;

// this should not exceed 255 bytes
#define MAX_PACKET_VALUE_SIZE (MAX_PACKET_SIZE - sizeof(s_header))

typedef struct PACKED
{
  s_header header;
  uint8_t value[MAX_PACKET_VALUE_SIZE];
} s_packet;

#endif
