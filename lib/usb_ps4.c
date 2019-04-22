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

#include "usb_ps4.h"
#include "../shared/emu_adapter.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <libusb-1.0/libusb.h>

#if !defined(LIBUSB_API_VERSION) && !defined(LIBUSBX_API_VERSION)
const char * LIBUSB_CALL libusb_strerror(enum libusb_error errcode)
{
  return libusb_error_name(errcode);
}
#endif

#define USB_INTERRUPT_ENDPOINT_IN  0x04
#define USB_INTERRUPT_ENDPOINT_OUT 0x03
#define USB_INTERRUPT_PACKET_SIZE  64

#define USB_QUEUE_MAX   150

static libusb_context* ctx = NULL;
static libusb_device** devs = NULL;
static ssize_t cnt = 0;
static int nb_opened = 0;

static int usb_state_indexes[5] =
{ };

static struct usb_state
{
  libusb_device_handle* devh;
  int ack;
  int joystick_id;
//s_report report;
} usb_states[5];

const struct libusb_pollfd** pfd_usb = NULL;

void usb_callback_intr(struct libusb_transfer* transfer)
{
  int usb_number = *(int*) transfer->user_data;
  struct usb_state* state = usb_states + usb_number;

  struct libusb_control_setup* setup = libusb_control_transfer_get_setup(
      transfer);

  //fprintf (stderr, "\n#!USB:usb_callback_intr");

  if (transfer->type == LIBUSB_TRANSFER_TYPE_INTERRUPT)
  {
    //
    if (0)
      fprintf (stderr, "\n#!USB: INT!");
    if (transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {
      // process joystick events
      if (transfer->actual_length == USB_INTERRUPT_PACKET_SIZE)
      {
        if (transfer->endpoint == (USB_INTERRUPT_ENDPOINT_IN | LIBUSB_ENDPOINT_IN))
        {
          state->ack = 1;
          //
          if (0)
          {
            printf("\n#WHL@out %dB@0x%x: ", transfer->actual_length, transfer->endpoint);
            int i;
            for (i = 0; i < transfer->actual_length; i++)
              printf("%02x ", transfer->buffer[i]);
          }
          extern void wheel_process_report(unsigned char *rep, int rlen);
          wheel_process_report((unsigned char *) transfer->buffer,
              transfer->actual_length);
          //adapter_send_report (0, transfer->actual_length, (unsigned char *) transfer->buffer);
        } // input report
      }
      else
      {
#if 0
        fprintf(stderr, "\n#!incorrect packet size (%d vs %d) on interrupt endpoint", transfer->actual_length, USB_INTERRUPT_PACKET_SIZE);
#endif
      }
    }
    else if (transfer->status != LIBUSB_TRANSFER_TIMED_OUT
        && transfer->status != LIBUSB_TRANSFER_CANCELLED)
    {
      fprintf(stderr, "\n#!usb_callback:libusb_transfer failed with error 0x%x",
          transfer->status);
    }
  }
}

void usb_callback_ctrl(struct libusb_transfer* transfer)
{
  int usb_number = *(int*) transfer->user_data;
  struct usb_state* state = usb_states + usb_number;

  struct libusb_control_setup* setup = libusb_control_transfer_get_setup(
      transfer);

  //fprintf (stderr, "\n#!USB:usb_callback_ctrl");

  if (transfer->type == LIBUSB_TRANSFER_TYPE_CONTROL)
  {
    //fprintf (stderr, "\n#!USB: TC!");
    if (transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {
      //fprintf (stderr, "\n#!USB: TC done!");
      if (setup->bmRequestType & LIBUSB_ENDPOINT_IN)
      {
        //fprintf (stderr, "\n#!USB: EP IN!");
        if (transfer->actual_length > 0xff)
        {
          fprintf(stderr, "\n#!wLength (%hu) is higher than %lu\n",
              transfer->actual_length, BUFFER_SIZE - LIBUSB_CONTROL_SETUP_SIZE);
        }
        else
        {
          unsigned char *data = libusb_control_transfer_get_data(transfer);
          extern int emu_send_ctrl (int usb_number, unsigned char *data, int length);
          if (emu_send_ctrl (usb_number, data, transfer->actual_length) < 0)
          {
            fprintf(stderr, "\n#!can't forward data to the adapter\n");
          }
          //
          if (data[2] == 0xf1 && data[0] == 0xf2)
          {
            fprintf (stderr, "\n#W:!usb_callback_ctrl:auth issue, wheel reset required: 0x%0x in 0x20", data[1]);
          }
          //
          if (0)
          {
            printf("\n#usb_callback_ctrl:DATA out %dB: ", transfer->actual_length);
            int i;
            for (i = 0; i < transfer->actual_length; i++)
              printf("%02x ", data[i]);
            fflush(stdout);
          }
        }
      }
    }
    else
    {
      char *usberr[] = {"okay", "error", "tmo", "cancel", "stall", "no device", "overflow", "other"};
      char *luerr = usberr[0];
      #if 1
      switch (transfer->status)
      {
        case LIBUSB_TRANSFER_ERROR:
          luerr = usberr[1];
          break;
        case LIBUSB_TRANSFER_TIMED_OUT:
          luerr = usberr[2];
          break;
        case LIBUSB_TRANSFER_CANCELLED:
          luerr = usberr[3];
          break;
        case LIBUSB_TRANSFER_STALL:
          luerr = usberr[4];
          break;
        case LIBUSB_TRANSFER_NO_DEVICE:
          luerr = usberr[5];
          break;
        case LIBUSB_TRANSFER_OVERFLOW:
          luerr = usberr[6];
          break;
        default:
          luerr = usberr[7];
      }
      #endif
      fprintf(stderr,
          "\n#!usb_callback_ctrl:libusb_transfer failed with status 0x%x (%s) (bmRequestType=0x%02x, bRequest=0x%02x, wValue=0x%04x)",
          transfer->status, luerr,
          setup->bmRequestType, setup->bRequest, setup->wValue);
#if 0
      if (transfer->status == LIBUSB_TRANSFER_STALL)
      {
        fprintf(stderr, "\n#W:clearing STALL on EP0");
        libusb_clear_halt (state->devh, 0);
      }
#endif
    }
  }
}

static int usb_poll_interrupt (int usb_number)
{
  struct usb_state* state = usb_states + usb_number;
  unsigned int size = USB_INTERRUPT_PACKET_SIZE;
  unsigned char* buf = calloc(size, sizeof(char));
  //
  struct libusb_transfer* transfer = libusb_alloc_transfer(0);
  transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER | LIBUSB_TRANSFER_FREE_TRANSFER;
  libusb_fill_interrupt_transfer(transfer, state->devh,
      USB_INTERRUPT_ENDPOINT_IN | LIBUSB_ENDPOINT_IN, buf, size,
      (libusb_transfer_cb_fn) usb_callback_intr, usb_state_indexes + usb_number,
      1000);
  //
  if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS)
  {
    fprintf (stderr, "\n#!usb_poll_interrupt:libusb_submit_transfer");
    free (transfer->buffer);
    libusb_free_transfer (transfer);
    return 0;
  }
  else
    state->ack = 0;
  //fprintf (stderr, "\n#usb_poll_interrupt:libusb_submit_transfer ok");
  //
  return 1;
}

int usb_poll_interrupts()
{
  if (0)
	fprintf (stderr, "\n#polling usb ints..");
  if (usb_states[0].devh /*&& usb_states[i].ack > 0*/)
  {
    return usb_poll_interrupt(0);
    //printf ("polling usb ints..\n");
  }
  return 0;
}

int usb_fill_fds(struct pollfd fds[])
{
  int poll_i;
  for (poll_i = 0; pfd_usb[poll_i] != NULL; ++poll_i)
  {
    //GE_AddSource(pfd_usb[poll_i]->fd, usb_number, usb_handle_events, usb_handle_events, usb_close);
    fds[poll_i].fd = pfd_usb[poll_i]->fd;
    fds[poll_i].events = POLLOUT | POLLIN | POLLERR | POLLHUP | POLLNVAL;
    //fds[poll_i].events = POLLOUT | POLLHUP;
  }
  return poll_i;
}

int usb_get_ack (int usb_number)
{
  struct usb_state* state = usb_states + usb_number;
  return state->ack;
}

int usb_handle_events(int usb_number)
{
  return libusb_handle_events (ctx);
}

int usb_init(int usb_number, unsigned short vendor, unsigned short product)
{
  int ret = -1;
  int dev_i;

  struct usb_state* state = usb_states + usb_number;
  //
  usb_state_indexes[usb_number] = usb_number;
  memset(state, 0x00, sizeof(*state));
  state->joystick_id = -1;
  //
  if (!ctx)
  {
    ret = libusb_init(&ctx);
    if (ret < 0)
    {
      fprintf(stderr, "\n#!libusb_init: %s.\n", libusb_strerror(ret));
      return -1;
    }
  }
  //always get the new list of devices
  if (devs)
  {
    libusb_free_device_list(devs, 1);
    devs = NULL;
  }
  if (!devs)
  {
    cnt = libusb_get_device_list(ctx, &devs);
    if (cnt < 0)
    {
      fprintf(stderr, "\n#!libusb_get_device_list: %s.\n",
          libusb_strerror(cnt));
      return -1;
    }
    //printf ("\nUSB:got %ddevs", cnt);
  }
  //
  for (dev_i = 0; dev_i < cnt; ++dev_i)
  {
    struct libusb_device_descriptor desc;
    ret = libusb_get_device_descriptor(devs[dev_i], &desc);
    if (!ret)
    {
      //printf ("\nUSB:found %04x:%04x", desc.idVendor, desc.idProduct);
      if (desc.idVendor == vendor && desc.idProduct == product)
      {
        libusb_device_handle* devh;
        ret = libusb_open(devs[dev_i], &devh);
        if (ret != LIBUSB_SUCCESS)
        {
          fprintf(stderr, "\n#!libusb_open: %s.\n", libusb_strerror(cnt));
          return -1;
        }
        else
        {
          ret = libusb_reset_device(devh);
          if(ret != LIBUSB_SUCCESS)
          {
            fprintf(stderr, "libusb_reset_device: %s.\n", libusb_strerror(ret));
            libusb_close(devh);
            return -1;
          }
          //
          if (libusb_kernel_driver_active(devh, 0))
          {
            ret = libusb_detach_kernel_driver(devh, 0);
            if (ret != LIBUSB_SUCCESS)
            {
              fprintf(stderr, "\n#!libusb_detach_kernel_driver: %s.\n",
                  libusb_strerror(ret));
              libusb_close(devh);
              return -1;
            }
          }
          if (1)
          {
            ret = libusb_set_configuration (devh, 1);
            if (ret != LIBUSB_SUCCESS)
            {
              fprintf(stderr, "\n#!libusb_set_configuration: %s.\n",
                  libusb_strerror(ret));
              libusb_close(devh);
              return -1;
            }
          }
          ret = libusb_claim_interface (devh, 0);
          if (ret != LIBUSB_SUCCESS)
          {
            fprintf(stderr, "\n#!libusb_claim_interface: %s.\n",
                libusb_strerror(ret));
            libusb_close(devh);
          }
          else
          {
            pfd_usb = libusb_get_pollfds (ctx);
            if (0)
            {
              for (int pi = 0; pfd_usb[pi] != NULL; ++pi)
                printf ("\n#usb poll fd %d", pfd_usb[pi]->fd);
            }
            state->devh = devh;
            state->ack = 0;
            if (usb_poll_interrupt (usb_number) == 0)
            {
              return -1;
            }
            return 0;
          }
        }
      }
    }
  }
  //
  return -1;
}

int usb_close(int usb_number)
{
  struct usb_state* state = usb_states + usb_number;
  //
  if (state->devh)
  {
    libusb_reset_device(state->devh);
    libusb_release_interface(state->devh, 0);
#if 1
#if !defined(LIBUSB_API_VERSION) && !defined(LIBUSBX_API_VERSION)
#ifndef WIN32
    libusb_attach_kernel_driver(state->devh, 0);
#endif
#endif
#endif
    libusb_close(state->devh);
    state->devh = NULL;
    if (devs)
    {
      libusb_free_device_list(devs, 1);
      devs = NULL;
    }
  }
  //
  if (pfd_usb)
    free (pfd_usb);

  return 1;
}

int usb_exit(void)
{
  if (ctx)
  {
    libusb_exit (ctx);
    ctx = NULL;
  }
  return 1;
}

int usb_send_ctrl(int usb_number, unsigned char* buffer, unsigned char length)
{
  struct usb_state* state = usb_states + usb_number;
  //
  if (!state->devh)
  {
    fprintf(stderr, "\n#W:no usb device opened for index %d", usb_number);
    return -1;
  }
  //
  struct libusb_control_setup* control_setup =
      (struct libusb_control_setup*) buffer;
  if (control_setup->wLength > BUFFER_SIZE - LIBUSB_CONTROL_SETUP_SIZE)
  {
    fprintf(stderr, "\n#!wLength (%hu) is higher than %lu\n",
        control_setup->wLength, BUFFER_SIZE - LIBUSB_CONTROL_SETUP_SIZE);
    return -1;
  }
  //
  unsigned int size = length;
  if (control_setup->bmRequestType & LIBUSB_ENDPOINT_IN)
  {
    size += control_setup->wLength;
  }
  //
  unsigned char* buf = calloc(size, sizeof(unsigned char));
  if (!buf)
  {
    fprintf(stderr, "\n#!calloc failed\n");
    return -1;
  }
  //
  memcpy (buf, buffer, length);
  struct libusb_transfer* transfer = libusb_alloc_transfer(0);
  transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER | LIBUSB_TRANSFER_FREE_TRANSFER;
  libusb_fill_control_transfer(transfer, state->devh, buf,
      (libusb_transfer_cb_fn) usb_callback_ctrl, usb_state_indexes + usb_number,
      1000);
  //
  if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS)
  {
    fprintf(stderr, "\n#!usb_send:libusb_submit_transfer");
    free(transfer->buffer);
    libusb_free_transfer(transfer);
    return (-1);
  }
  //
  if (0)
  {
    printf("\n#DATA in %dB: ", length);
    int i;
    for (i = 0; i < length; i++)
      printf("%02x ", buffer[i]);
    fflush(stdout);
  }
  //
  return 0;
}

int usb_send_intr(int usb_number, unsigned char* buffer, unsigned char length)
{
  struct usb_state* state = usb_states + usb_number;
  //
  if (!state->devh)
  {
    fprintf(stderr, "\n#!usb_send_int:no usb device opened for index %d\n", usb_number);
    return -1;
  }
  //
  unsigned char* ibuf = calloc(length, sizeof(unsigned char));
  if (!ibuf)
  {
    fprintf(stderr, "\n#!usb_send_int:calloc failed\n");
    return -1;
  }
  memcpy (ibuf, buffer, length);
  //
  int blen;
#if 0
  //libusb_fill_bulk_transfer (transfer, state->devh, LIBUSB_ENDPOINT_IN, buf, length, NULL, NULL, 1000);
  int ret = libusb_interrupt_transfer (state->devh, USB_INTERRUPT_ENDPOINT_OUT | LIBUSB_ENDPOINT_OUT, buf, length, &blen, 1000);
  free (ibuf);
#else
  struct libusb_transfer* transfer = libusb_alloc_transfer (0);
  if (!transfer)
  {
    free(ibuf);
    fprintf(stderr, "\n#!usb_send_int:libusb_alloc_transfer failed\n");
    return -1;
  }
  transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER | LIBUSB_TRANSFER_FREE_TRANSFER;
  libusb_fill_interrupt_transfer (transfer, state->devh,
      USB_INTERRUPT_ENDPOINT_OUT | LIBUSB_ENDPOINT_OUT, ibuf, length, (libusb_transfer_cb_fn) usb_callback_intr,
      usb_state_indexes + usb_number, 1000);
  if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS)
  {
    fprintf(stderr, "\n#!usb_send_int:libusb_submit_transfer");
    free (transfer->buffer);
    libusb_free_transfer(transfer);
    return -1;
  }
#endif
  if (0)
  {
    printf("\n#FFB@in %dB: ", length);
    int i;
    for (i = 0; i < length; i++)
      printf("%02x ", buffer[i]);
    fflush(stdout);
  }
  //
  return 0;
}
