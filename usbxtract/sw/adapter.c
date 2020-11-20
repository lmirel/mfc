/*
 Copyright (c) 2015 Mathieu Laurendeau <mat.lau@laposte.net>
 License: GPLv3

 2019 mirel.t.lazar@gmail.com
 adapted for USB message extraction
 */

#include <adapter.h>
#include <gserial.h>
#include <gpoll.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_ADAPTERS 7

#define PRINT_ERROR_OTHER(msg) fprintf(stderr, "%s:%d %s: %s\n", __FILE__, __LINE__, __func__, msg);
//use flag 0x20 for capture
static char adapterDbg = 0;

static struct {
  s_packet packet;
  unsigned int bread;
  int serial;
  ADAPTER_READ_CALLBACK fp_packet_cb;
} adapters[MAX_ADAPTERS];

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
static int csock = -1;
static int cfile = -1;
static struct sockaddr_in mfc_si_other;
static s_packet cpkt; //client data packet
static s_packet dpkt; //debug packet
// {<E_TYPE_DEBUG>, 5, vid[0], vid[1], pid[0], pid[1], <delta_ts>}
extern int vid, pid;
//
unsigned long get_millis ()
{
  struct timespec lts;
  //get current time
  clock_gettime (CLOCK_REALTIME, &lts);
  return (lts.tv_sec * 1000L + lts.tv_nsec / 1000000L);
}
//
static int client_init ()
{
  int s = -1;
  //open capture file
  if (adapterDbg == 0x20)
    cfile = open ("/tmp/usbx.cap", O_WRONLY | O_CREAT | O_TRUNC, 666);
  if ((s = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf ("\n#E:socket");
    return 0;
  }
  //destination
  memset((char *) &mfc_si_other, 0, sizeof (mfc_si_other));
  mfc_si_other.sin_family = AF_INET;
  mfc_si_other.sin_port = htons (64402);
  mfc_si_other.sin_addr.s_addr = htonl(0x7f000001L); //htonl (INADDR_BROADCAST);
  csock = s;
  //prep debug data
  dpkt.header.type = E_TYPE_DESCRIPTORS;//converts to PKTT_CTRL in MFC
  dpkt.header.length = 6;
  dpkt.value[0] = (vid >> 8) & 0xff;
  dpkt.value[1] = vid & 0xff;
  dpkt.value[2] = (pid >> 8) & 0xff;
  dpkt.value[3] = pid & 0xff;
  dpkt.value[4] = 0;  //ts[0]
  dpkt.value[5] = 0;  //ts[1]
  //write debug pkt first
  (void)sendto (csock, (const void *)&dpkt, dpkt.header.length + 2, 0, (struct sockaddr*)&mfc_si_other, sizeof (mfc_si_other));
  //switch back to debug pkt type
  dpkt.header.type = E_TYPE_INDEX;
  //
  return s;
}
//
static int client_close ()
{
  if (csock > 0)
    close (csock);
  csock = -1;
  if (cfile > 0)
    close (cfile);
  cfile = -1;
  //
  return 1;
}
//
static int client_send (s_packet *packet)
{
  int bs = 0;
  //dump adapter data
  if (adapterDbg)
  {
    //update debug pkt timestamp
    static unsigned long llts = 0;
    unsigned long clts = get_millis ();
    unsigned long mdt = (llts == 0)?0:(clts - llts);
    llts = clts;
    dpkt.value[4] = (mdt >> 8) & 0xff;
    dpkt.value[5] = mdt & 0xff;
  }
  //
  if (0 && packet->header.length > 60)
  {
    int i;
    printf ("\n#PKT:%03d bytes, type %x\n##", packet->header.length, packet->header.type);
    for (i = 0; i < packet->header.length; i++)
      printf ("%02x ", packet->value[i]);
    fflush (stdout);
  }
  if (csock < 0)
    client_init ();
  if (packet->header.length > 0)
  {
    //save msg to capture file
    if (cfile > 0)
    {
      //write debug
      bs = write (cfile, (const void *)&dpkt, dpkt.header.length + 2);
      //write USB data
      bs = write (cfile, (const void *)packet, packet->header.length + 2);
    }
    //
    switch (packet->header.type)
    {
      case E_TYPE_IN:
      case E_TYPE_OUT:
        bs = -1;
        if (csock > 0)
        {
          if (adapterDbg)
            //write debug pkt first
            bs = sendto (csock, (const void *)&dpkt, dpkt.header.length + 2, 0, (struct sockaddr*)&mfc_si_other, sizeof (mfc_si_other));
          //write USB data
          bs = sendto (csock, (const void *)packet, packet->header.length + 2, 0, (struct sockaddr*)&mfc_si_other, sizeof (mfc_si_other));
        }
        break;
      case E_TYPE_CONTROL:
        //check auth
        if (packet->value[0] == 0xf1)
        {
          if (packet->value[2] == 0x00)
          {
            printf ("\n#i:auth stage %02X", packet->value[1]);
            fflush (stdout);
          }
        }
        if (0 && packet->header.length)
        {
          int i;
          printf ("\n#i:CTL %03d bytes, type %x\n##", packet->header.length, packet->header.type);
          for (i = 0; i < packet->header.length; i++)
            printf ("%02x ", packet->value[i]);
        }
        break;
      default:
        ;//not used
    } //switch
  } //if pkt len
  //
  if (bs < 0)
  {
    printf ("\n#E:client_send");
    fflush (stdout);
  }
  return bs;
}

void adapter_init(void) __attribute__((constructor (101)));
void adapter_init(void) {
  unsigned int i;
  for (i = 0; i < sizeof(adapters) / sizeof(*adapters); ++i) 
  {
    adapters[i].serial = -1;
  }
}

char adapter_debug (char dbg)
{
  char ret = adapterDbg;
  if (dbg != 0xff)
    adapterDbg = dbg;
  return ret;
}

static inline int adapter_check(int adapter, const char * file, unsigned int line, const char * func) 
{
  if (adapter < 0 || adapter >= MAX_ADAPTERS) 
  {
    fprintf(stderr, "%s:%d %s: invalid device\n", file, line, func);
    return -1;
  }
  if (adapters[adapter].serial < 0) 
  {
    fprintf(stderr, "%s:%d %s: no such adapter\n", file, line, func);
    return -1;
  }
  return 0;
}
#define ADAPTER_CHECK(device,retValue) \
  if(adapter_check(device, __FILE__, __LINE__, __func__) < 0) { \
    return retValue; \
  }

static int adapter_recv(int adapter, const void * buf, int status) 
{
  if (adapterDbg & 0x0f)
  {
    fprintf (stdout, "\n#d:adapter recv %d", status);
    fflush (stdout);
  }
  ADAPTER_CHECK(adapter, -1)

  if (status < 0) 
  {
    return -1;
  }

  int ret = 0;

  if (adapterDbg & 0x0f)
  {
    fprintf (stdout, "\n#d:adapter read %d", status);
    fflush (stdout);
  }
  if(adapters[adapter].bread + status <= sizeof(s_packet)) 
  {
    memcpy((unsigned char *)&adapters[adapter].packet + adapters[adapter].bread, buf, status);
    adapters[adapter].bread += status;
    unsigned int remaining;
    if(adapters[adapter].bread < sizeof(s_header))
    {
      remaining = sizeof(s_header) - adapters[adapter].bread;
    }
    else
    {
      remaining = adapters[adapter].packet.header.length - (adapters[adapter].bread - sizeof(s_header));
    }
    if(remaining == 0)
    {
      //store for network processing
      memcpy (&cpkt, &adapters[adapter].packet, adapters[adapter].packet.header.length + 2);
      //
      ret = adapters[adapter].fp_packet_cb(adapter, &adapters[adapter].packet);
      adapters[adapter].bread = 0;
      gserial_set_read_size(adapters[adapter].serial, sizeof(s_header));
      //send to network for processing
      client_send (&cpkt);
    }
    else
    {
      gserial_set_read_size(adapters[adapter].serial, remaining);
    }
  }
  else
  {
    // this is a critical error (no possible recovering)
    fprintf (stderr, "%s:%d %s: invalid data size (count=%u, available=%zu)\n", __FILE__, __LINE__, __func__, status, sizeof(s_packet) - adapters[adapter].bread);
    return -1;
  }

  return ret;
}

int adapter_send (int adapter, unsigned char type, const unsigned char * data, unsigned int count) 
{

  ADAPTER_CHECK(adapter, -1)

  if (count != 0 && data == NULL) 
  {
    PRINT_ERROR_OTHER("data is NULL")
    return -1;
  }

  do 
  {
    unsigned char length = MAX_PACKET_VALUE_SIZE;
    if (count < length) 
    {
      length = count;
    }
    s_packet packet = { .header = { .type = type, .length = length } };
    if (data) 
    {
      memcpy (packet.value, data, length);
    }
    data += length;
    count -= length;
    //store for network processing
    memcpy (&cpkt, &packet, length + 2);
    //
    int ret = gserial_write (adapters[adapter].serial, &packet, 2 + length);
    if(ret < 0) 
    {
      return -1;
    }
    if (adapterDbg & 0x0f)
    {
      fprintf (stdout, "\n#d:adapter sent %dB", ret);
      fflush (stdout);
    }
    //send to network for processing
    client_send (&cpkt);
  } while (count > 0);

  return 0;
}

int adapter_open(const char * port, ADAPTER_READ_CALLBACK fp_read, ADAPTER_WRITE_CALLBACK fp_write, ADAPTER_CLOSE_CALLBACK fp_close) 
{
  extern int sbaud;
  char tbuf[PATH_MAX];
  int serial = -1;
  if (realpath (port, tbuf))
    serial = gserial_open (tbuf, sbaud);
  if (serial < 0) 
  {
    if (adapterDbg & 0x0f)
    {
      fprintf (stdout, "\n#e:failed to open adapter '%s'->'%s':%d", port, tbuf, serial);
      fflush (stdout);
    }
    return -1;
  }

  unsigned int i;
  for (i = 0; i < sizeof(adapters) / sizeof(*adapters); ++i) 
  {
    if (adapters[i].serial < 0) 
    {
      adapters[i].serial = serial;
      adapters[i].fp_packet_cb = fp_read;
      int ret = gserial_register (serial, i, adapter_recv, fp_write, fp_close, gpoll_register_fd);
      if (ret < 0) 
      {
        return -1;
      }
      if (adapterDbg & 0x0f)
      {
        fprintf (stdout, "\n#d:adapter opened '%s'->'%s':%d", port, tbuf, serial);
        fflush (stdout);
      }
      return i;
    }
  }

  gserial_close(serial);

  return -1;
}

int adapter_close ()
{
  client_close ();
  return 0;
}

