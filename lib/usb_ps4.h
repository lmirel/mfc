/*
 Copyright (c) 2014 Mathieu Laurendeau
 License: GPLv3
 */

/*
origin: GIMX https://github.com/matlo/GIMX
modification history
--------------------
01a,08jan2015, mirel lazar <mirel.t.lazar@gmail.com>
               adapted for motion feedback controller
*/

#ifndef USB_CON_H_
#define USB_CON_H_

#include <poll.h>

int usb_init(int usb_number, unsigned short vendor, unsigned short product);
int usb_close(int usb_number);
int usb_send_ctrl(int usb_number, unsigned char* buffer, unsigned char length);
int usb_send_intr(int usb_number, unsigned char* buffer, unsigned char length);
int usb_get_ack (int usb_number);

int usb_poll_interrupts();

int usb_fill_fds(struct pollfd fds[]);

int usb_handle_events(int unused);
#if 0
void usb_set_event_callback(int (*fp)(GE_Event*));
#endif

#endif /* USB_CON_H_ */
