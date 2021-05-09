
/*
 Copyright (c) 2016 Mathieu Laurendeau <mat.lau@laposte.net>
 License: GPLv3

  adapted from https://github.com/matlo/GIMX
  check http://gimx.fr
*/
#include <ff_decode.h>

#define uint8_t unsigned char

#define BYTE_OUT_REPORT  7
#define BYTE_SEND_REPORT 6

#define NAME_LENGTH 128

#define FF_DEBUG 1
#define dprintf(...) \
    do { \
        if(FF_DEBUG) { \
            printf(__VA_ARGS__); \
            fflush(stdout); \
        } \
    } while (0)

static void hdump(const unsigned char* packet, unsigned char length) {
    int i;
    for (i = 0; i < length; ++i) {
        printf("0x%02x ", packet[i]);
    }
}

static const char * cmd_names[] = {
    [FF_LG_CMD_DOWNLOAD]           = "DOWNLOAD",
    [FF_LG_CMD_DOWNLOAD_AND_PLAY]  = "DOWNLOAD_AND_PLAY",
    [FF_LG_CMD_PLAY]               = "PLAY",
    [FF_LG_CMD_STOP]               = "STOP",
    [FF_LG_CMD_DEFAULT_SPRING_ON]  = "DEFAULT_SPRING_ON",
    [FF_LG_CMD_DEFAULT_SPRING_OFF] = "DEFAULT_SPRING_OFF",
    [FF_LG_CMD_RESERVED_1]         = "RESERVED_1",
    [FF_LG_CMD_RESERVED_2]         = "RESERVED_2",
    [FF_LG_CMD_NORMAL_MODE]        = "NORMAL_MODE",
    [FF_LG_CMD_SET_LED]            = "SET_LED",
    [FF_LG_CMD_SET_WATCHDOG]       = "SET_WATCHDOG",
    [FF_LG_CMD_RAW_MODE]           = "RAW_MODE",
    [FF_LG_CMD_REFRESH_FORCE]      = "REFRESH_FORCE",
    [FF_LG_CMD_FIXED_TIME_LOOP]    = "FIXED_TIME_LOOP",
    [FF_LG_CMD_SET_DEFAULT_SPRING] = "SET_DEFAULT_SPRING",
    [FF_LG_CMD_SET_DEAD_BAND]      = "SET_DEAD_BAND",
};

const char * ff_lg_get_cmd_name(unsigned char header) {
    if (header == FF_LG_CMD_EXTENDED_COMMAND) {
        return "EXTENDED_COMMAND";
    } else {
        unsigned char cmd = header & FF_LG_CMD_MASK;
        if (cmd < sizeof(cmd_names) / sizeof(*cmd_names)) {
            return cmd_names[cmd];
        } else {
            return "UNKNOWN";
        }
    }
}

char * slot_names[] = {
        [0b0000] = "",
        [0b0001] = "1",
        [0b0010] = "2",
        [0b0011] = "1,2",
        [0b0100] = "3",
        [0b0101] = "1,3",
        [0b0110] = "2,3",
        [0b0111] = "1,2,3",
        [0b1000] = "4",
        [0b1001] = "1,4",
        [0b1010] = "2,4",
        [0b1011] = "1,2,4",
        [0b1100] = "3,4",
        [0b1101] = "1,3,4",
        [0b1110] = "2,3,4",
        [0b1111] = "1,2,3,4",
};

const char * ff_lg_get_slot_names(unsigned char header) {
    if (header == FF_LG_CMD_EXTENDED_COMMAND) {
        return "";
    } else {
        return slot_names[header >> 4];
    }
}

static struct {
    unsigned char value;
    const char * name;
} ext_cmd_names[] = {
    { FF_LG_EXT_CMD_CHANGE_MODE_DFP,           "CHANGE_MODE_DFP" },
    { FF_LG_EXT_CMD_WHEEL_RANGE_200_DEGREES,   "WHEEL_RANGE_200_DEGREES" },
    { FF_LG_EXT_CMD_WHEEL_RANGE_900_DEGREES,   "WHEEL_RANGE_900_DEGREES" },
    { FF_FT_EXT_CMD_WHL_DIGITS,                 "WHL_DIGITS" },
    { FF_LG_EXT_CMD_REVERT_IDENTITY,           "REVERT_IDENTITY" },
    { FF_LG_EXT_CMD_CHANGE_MODE_G25,           "CHANGE_MODE_G25" },
    { FF_LG_EXT_CMD_CHANGE_MODE_G25_NO_DETACH, "CHANGE_MODE_G25_NO_DETACH" },
    { FF_LG_EXT_CMD_SET_RPM_LEDS,               "SET_RPM_LEDS" },
    { FF_FT_EXT_CMD_RPM_LEDS,                  "RPM_LEDS" },
    { FF_LG_EXT_CMD_CHANGE_WHEEL_RANGE,        "CHANGE_WHEEL_RANGE" },
};

const char * ff_lg_get_ext_cmd_name(unsigned char ext) {
    unsigned int i;
    for (i = 0; i < sizeof(ext_cmd_names) / sizeof(*ext_cmd_names); ++i) {
        if(ext_cmd_names[i].value == ext) {
            return ext_cmd_names[i].name;
        }
    }
    static char unknown[] = "UNKNOWN (255) ";
    snprintf(unknown, sizeof(unknown), "UNKNOWN (0x%x)", ext);
    return unknown;
}

static const char * ftype_names [] = {
    [FF_LG_FTYPE_CONSTANT]                           = "CONSTANT",
    [FF_LG_FTYPE_SPRING]                             = "SPRING",
    [FF_LG_FTYPE_DAMPER]                             = "DAMPER",
    [FF_LG_FTYPE_AUTO_CENTER_SPRING]                 = "AUTO_CENTER_SPRING",
    [FF_LG_FTYPE_SAWTOOTH_UP]                        = "SAWTOOTH_UP",
    [FF_LG_FTYPE_SAWTOOTH_DOWN]                      = "SAWTOOTH_DOWN",
    [FF_LG_FTYPE_TRAPEZOID]                          = "TRAPEZOID",
    [FF_LG_FTYPE_RECTANGLE]                          = "RECTANGLE",
    [FF_LG_FTYPE_VARIABLE]                           = "VARIABLE",
    [FF_LG_FTYPE_RAMP]                               = "RAMP",
    [FF_LG_FTYPE_SQUARE_WAVE]                        = "SQUARE_WAVE",
    [FF_LG_FTYPE_HIGH_RESOLUTION_SPRING]             = "HIGH_RESOLUTION_SPRING",
    [FF_LG_FTYPE_HIGH_RESOLUTION_DAMPER]             = "HIGH_RESOLUTION_DAMPER",
    [FF_LG_FTYPE_HIGH_RESOLUTION_AUTO_CENTER_SPRING] = "HIGH_RESOLUTION_AUTO_CENTER_SPRING",
    [FF_LG_FTYPE_FRICTION]                           = "FRICTION",
};

const char * ff_lg_get_ftype_name(unsigned char ftype) {
    if (ftype < sizeof(ftype_names) / sizeof(*ftype_names)) {
        return ftype_names[ftype];
    } else {
        return "UNKNOWN";
    }
}

void ff_lg_decode_extended(const unsigned char data[FF_LG_OUTPUT_REPORT_SIZE]) {

    dprintf("%s %s", ff_lg_get_cmd_name(data[0]), ff_lg_get_ext_cmd_name(data[1]));

    switch(data[1]) {
    case FF_LG_EXT_CMD_CHANGE_MODE_DFP:
    case FF_LG_EXT_CMD_WHEEL_RANGE_200_DEGREES:
    case FF_LG_EXT_CMD_WHEEL_RANGE_900_DEGREES:
    case FF_LG_EXT_CMD_CHANGE_MODE_G25:
    case FF_LG_EXT_CMD_CHANGE_MODE_G25_NO_DETACH:
      break;
#if 0
    case FF_FT_EXT_CMD_CHANGE_MODE:
    {
      const char * mode = NULL;
      switch(data[2]) {
      case 0x00:
        mode = "Logitech Driving Force EX";
        break;
      case 0x01:
        mode = "Logitech Driving Force Pro";
        break;
      case 0x02:
        mode = "Logitech G25 Racing Wheel";
        break;
      case 0x03:
        mode = "Logitech Driving Force GT";
        break;
      case 0x04:
        mode = "Logitech G27 Racing Wheel";
        break;
      case 0x08:
        mode = "WHL LED";
        break;
      case 0x13:
        mode = "RPM LED";
        break;
      }
      if(mode == NULL) {
          dprintf(" - unknown mode (0x%02x)", data[2]);
      }
      else {
          dprintf(" - %s", mode);
      }
      dprintf(" - %s", data[3] ? "DETACH" : "NO DETACH");
    }
      break;
#endif
    case FF_LG_EXT_CMD_REVERT_IDENTITY:
      dprintf(" - %s", data[2] ? "REVERT" : "DO NOT REVERT");
      break;
    case FF_LG_EXT_CMD_SET_RPM_LEDS:
      dprintf(" - 0x%02x", data[2]);
      break;
    case FF_LG_EXT_CMD_CHANGE_WHEEL_RANGE:
      dprintf(" - %hu", (data[3] << 8) | data[2]);
      break;
    default:
      dprintf(" - ");
      hdump(data + 2, FF_LG_OUTPUT_REPORT_SIZE - 2);
      break;
    }
    //dprintf("\n");
}

void ff_lg_decode_command(const unsigned char data[FF_LG_OUTPUT_REPORT_SIZE])
{
    if(data[0] == FF_LG_CMD_EXTENDED_COMMAND) {
        ff_lg_decode_extended(data);
        return;
    }

    dprintf("%s ", ff_lg_get_cmd_name(data[0]));
    const char * slots = ff_lg_get_slot_names(data[0]);
    if (*slots != '\0') {
        dprintf("- %s", slots);
    }

    switch(data[0] & FF_LG_CMD_MASK) {
    case FF_LG_CMD_PLAY:
    case FF_LG_CMD_STOP:
        break;
    case FF_LG_CMD_DOWNLOAD:
    case FF_LG_CMD_DOWNLOAD_AND_PLAY:
    case FF_LG_CMD_REFRESH_FORCE:
        dprintf(" - %s", ff_lg_get_ftype_name(data[1]));
        dprintf(" - ");
        hdump(data + 2, FF_LG_OUTPUT_REPORT_SIZE - 2);
        break;
    case FF_LG_CMD_DEFAULT_SPRING_ON:
    case FF_LG_CMD_DEFAULT_SPRING_OFF:
    case FF_LG_CMD_NORMAL_MODE:
    case FF_LG_CMD_RAW_MODE:
        break;
    case FF_LG_CMD_SET_LED:
        dprintf(" - 0x%02x", data[1]);
        break;
    case FF_LG_CMD_SET_WATCHDOG:
        dprintf(" - 0x%02x", data[1]);
        break;
    case FF_LG_CMD_FIXED_TIME_LOOP:
        dprintf(" - %s", data[1] ? "ON" : "OFF");
        break;
    case FF_LG_CMD_SET_DEFAULT_SPRING:
        dprintf(" - ");
        hdump(data + 1, FF_LG_OUTPUT_REPORT_SIZE - 1);
        break;
    case FF_LG_CMD_SET_DEAD_BAND:
        dprintf(" - %s", data[1] ? "ON" : "OFF");
        break;
    default:
        dprintf("!UNK");
        break;
    }
    //dprintf("\n");
}
