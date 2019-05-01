/*
 Copyright (c) 2015 mirel lazar <mirel.t.lazar@gmail.com>
 License: GPLv3

 */

/*
modification history
--------------------
01a,08jan2015, mirel lazar <mirel.t.lazar@gmail.com>
               adapted for motion feedback controller
*/
#define _POSIX_C_SOURCE 199309L

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <termios.h>
#include <time.h>

#include <sys/ioctl.h>
#include <sys/uio.h>

#include <libintl.h>
#define _(STRING)    gettext(STRING)

#include <errno.h>

#include "extras.h"

#define DEBUG 0
#define debug_print(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); if (DEBUG) fflush (stderr);} while (0)

/*
 * broadcast socket work
 */
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int inet_aton(const char *cp, struct in_addr *inp);

/*
  int pkt_type;
  int pkt_dof_type;
  int data[6];    //{pitch, roll, yaw, surge, sway, heave}
  int speed;
  //
Pitch is the tilt of the car forwards or backwards in [°]
Roll is how much the car is dipped to the left or right in [°]
Yaw is the heading of the car (north, east, south, west) in [°]

Surge means the acceleration of the car in longitudinal direction [g]
Sway means the acceleration of the car in lateral direction [g]
Heave means the acceleration up and down [g]

//motion data packet
pkt[0] = PKTT_DATA;
pkt[1] = PKTT_2DOF;
//motion
pkt[2] = pl_pitch;  //pitch
pkt[3] = pl_roll;   //roll
pkt[4] = 0;         //yaw
pkt[5] = 0;         //surge
pkt[6] = 0;         //sway
pkt[7] = 0;         //heave
//speed
pkt[8] = 0;         //speed
*/
int mfc_pkt[MFC_PKT_SIZE] = {0};

int mfc_bcast_sock = -1;
struct sockaddr_in mfc_si_other, si_me;
//
int mfc_bcast_prep (char *dst, int svr)
{
  //sockets
  if (!dst || !*dst)
    return 0;
  int s = -1;
  if ((s = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf ("\n#ERR:socket");
    return 0;
  }
#if 0
  int broadcast = 1;
  setsockopt (s, SOL_SOCKET, SO_BROADCAST,
          &broadcast, sizeof (broadcast));
#endif
  if (svr)
  {
    memset ((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons (MFCSVR_PORT);
    si_me.sin_addr.s_addr = htonl (INADDR_ANY);
    if (bind (s, (struct sockaddr*)&si_me, sizeof (si_me)) == -1)
    {
      fprintf(stderr, "\n#ERR:bind");
      close (s);
      return 0;
    }
  }
  else
  {
    //destination
    memset((char *) &mfc_si_other, 0, sizeof (mfc_si_other));
    mfc_si_other.sin_family = AF_INET;
    mfc_si_other.sin_port = htons (MFCSVR_PORT);
  #if 1
    mfc_si_other.sin_addr.s_addr = inet_addr (dst); //htonl (INADDR_BROADCAST);
    if (mfc_si_other.sin_addr.s_addr == -1)
    {
      close (s);
      return 0;
    }
  #else
    if (inet_aton(dst, &mfc_si_other.sin_addr) == 0)
    {
      fprintf(stderr, "\n#!inet_aton() failed\n");
      close (s);
      return 0;
    }
  #endif
  }
  mfc_bcast_sock = s;
  mfc_pkt[0] = PKTT_CTRL;
  //
  return s;
}

int mfc_bcast_send ()
{
  int bs = 0;
  if (mfc_bcast_sock > 0)
  {
    bs = sendto (mfc_bcast_sock, &mfc_pkt, sizeof(mfc_pkt), 0, (struct sockaddr*)&mfc_si_other, sizeof(mfc_si_other));
    if (bs < 0)
      printf("\n#ERR:sendto");
    else
      debug_print ("\n#i:%d<%dB sent", mfc_bcast_sock, bs);
  }
  fflush (stdout);
  return bs;
}

int mfc_bcast_receive ()
{
  //get network data
  if (mfc_bcast_sock < 0)
    return 0;
  int rb = 0, tb = 0, pks = sizeof (mfc_pkt), rtmo = 500;
  while (tb < pks && rtmo)
  {
    rb = recvfrom (mfc_bcast_sock, mfc_pkt + tb, pks - tb, 0, NULL, 0);
    if (rb > 0)
      tb += rb;
    else if (rb == 0)
      rtmo--;
    else
    {
      printf ("\n#E:recvfrom() failed");
      break;
    }
  }
  debug_print ("\n#i:%d>%dB vs %d", mfc_bcast_sock, tb, pks);
  if (1)
  {
    for (int i = 0; i < tb; i++)
      debug_print (", %d", mfc_pkt[i]);
  }
  //
  return tb;
}

int *mfc_bcast_pktget ()
{
  return mfc_pkt;
}

int mfc_bcast_pktlen ()
{
  return sizeof(mfc_pkt);
}

void mfc_bcast_close ()
{
  if (mfc_bcast_sock > 0)
    close (mfc_bcast_sock);
  mfc_bcast_sock = -1;
}


int bcast_sock = -1;
struct sockaddr_in si_other;
void bcast_prep (char *dst)
{
  //sockets
  if (!dst || !*dst)
    return;
  int s;
  if ((s = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf ("\n#ERR:socket");
    return;
  }
#if 0
  int broadcast = 1;
  setsockopt (s, SOL_SOCKET, SO_BROADCAST,
          &broadcast, sizeof (broadcast));
#endif
  memset ((char *) &si_me, 0, sizeof(si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons (65002);
  si_me.sin_addr.s_addr = htonl (INADDR_ANY);
  if (bind (s, (struct sockaddr*)&si_me, sizeof (si_me)) == -1)
  {
    fprintf(stderr, "\n#ERR:bind");
    close (s);
    return;
  }
  memset((char *) &si_other, 0, sizeof (si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons (65001);
#if 0
  si_other.sin_addr.s_addr = inet_addr (dst); //htonl (INADDR_BROADCAST);
  if (si_other.sin_addr.s_addr == -1)
  {
    close (s);
    return;
  }
#else
  if (inet_aton(dst, &si_other.sin_addr) == 0)
  {
    fprintf(stderr, "\n#!inet_aton() failed\n");
    close (s);
    return;
  }
#endif
  bcast_sock = s;
}

#if 0
void bcast_send ()
{
#if 1
  if (bcast_sock > 0 && sendto (bcast_sock, &pkt, sizeof(SEvtPacket), 0, (struct sockaddr*)&si_other, sizeof(si_other)) == -1)
  {
          printf("\n#ERR:sendto");
  }
  //else
  //    printf ("%d .pkt %d sent\n", pkt.gaugedata[gTimeStamp], pktk);
#endif
}
#endif

void bcast_close ()
{
  if (bcast_sock > 0)
    close (bcast_sock);
}

long get_map (long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long get_cmap (long x, long in_min, long in_max, long out_min, long out_max)
{
  long rv = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (rv > out_max)
    rv = out_max;
  if (rv < out_min)
    rv = out_min;
  return rv;
}

float get_map_f (float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//[b3 b2 b1 b0]
float get_float (char *buf, int off)
{
  long lv = (long)((buf[off+3]&0xff)<<24 | (buf[off+2]&0xff)<<16 | (buf[off+1]&0xff)<<8 | (buf[off]&0xff));
  //printf ("\n#lv n %ld", lv);
  float rv;
  //lv = ntohl (lv);
  //printf ("\n#lv h %ld", lv);
  memcpy (&rv, &lv, 4);
  return rv;
}

unsigned int get_uint (char *buf, int off)
{
  long lv = (long)((buf[off+3]&0xff)<<24 | (buf[off+2]&0xff)<<16 | (buf[off+1]&0xff)<<8 | (buf[off]&0xff));
  //printf ("\n#lv n %ld", lv);
  unsigned int rv;
  //lv = ntohl (lv);
  //printf ("\n#lv h %ld", lv);
  memcpy (&rv, &lv, 4);
  return rv;
}

int get_int (char *buf, int off)
{
  long lv = (long)((buf[off+3]&0xff)<<24 | (buf[off+2]&0xff)<<16 | (buf[off+1]&0xff)<<8 | (buf[off]&0xff));
  //printf ("\n#lv n %ld", lv);
  int rv;
  //lv = ntohl (lv);
  //printf ("\n#lv h %ld", lv);
  memcpy (&rv, &lv, 4);
  return rv;
}

unsigned short get_ushort (char *buf, int off)
{
  return (unsigned short)(buf[off+1]<<8|buf[off]);
}

short get_short (char *buf, int off)
{
  return (short)(buf[off+1]<<8|buf[off]);
}

//axis is: -1..-32768<L R>32768..1 (or the reverse)
int normal_axis (int val, int max)
{
  int rv;
  if (val < 0)
    rv = get_map (val, -1, -32768, -32768, -1);//max + val;
  else
    rv = get_map (val, 32768, 0, 0, 32768);//val;
  //
  return rv;
}

int normal_pedal (int val, int max)
{
  int rv;
  if (val < 0)
    rv = get_map (val, -1, -32768, -32768, -1);//max + val;
  else
    rv = get_map (val, 32768, 0, 0, 32768);//val;
  //
  return rv;
}

/* brake reports 0 to -32768*/
int normal_brake (int val, int max)
{
  int rv;
  if (val < 0)
    rv = get_map (val, -1, -32768, 0, -16384);//max + val;
  else
    rv = get_map (val, 32768, 0, -16384, -32768);//val;
  //
  return rv;
}

/* accel reports 0 to 32768 */
int normal_accel (int val, int max)
{
  int rv;
  if (val < 0)
    rv = get_map (val, -1, -32768, 0, 16384);//max + val;
  else
    rv = get_map (val, 32768, 0, 16384, 32768);//val;
  //
  return rv;
}

//ffb is: 128..255<L R>1..127
int normal_ffb (int val, int max)
{
  int rv;
  if (val > 127)
    rv = max - val;
  else
    rv = - val;
  //
  return rv;
}

//ffb2 is: 128..255<L R>1..127
int normal_ffb2 (int val, int mid)
{
  return (val > mid)?(val - mid) : -(mid - val);
}

unsigned long get_millis ()
{
  struct timespec lts;
  //get current time
  clock_gettime (CLOCK_REALTIME, &lts);
  return (lts.tv_sec * 1000L + lts.tv_nsec / 1000000L);
}
