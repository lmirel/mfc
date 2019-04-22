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
/*
 * The baud rate in bps.
 */
//#define TTY_BAUDRATE B500000 //0.5Mbps
#define TTYE_BAUDRATE B9600 //0.5Mbps

#define DEBUG 0
#define debug_print(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); if (DEBUG) fflush (stderr);} while (0)

/*
 * Connect to a serial port.
 */
int s7ledS_sendCmd (char* pd, int cl);
void cfmakeraw(struct termios *termios_p);

static int s7lfd = -1;
int s7led_connect(char* portname)
{
  struct termios options;
  int fd;

  if ((fd = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
  {
    fprintf(stderr, "#can't connect to %s\n", portname);
  }
  else
  {
    tcgetattr(fd, &options);
    cfsetispeed(&options, TTYE_BAUDRATE);
    cfsetospeed(&options, TTYE_BAUDRATE);
    cfmakeraw(&options);
    if (tcsetattr(fd, TCSANOW, &options) < 0)
    {
      fprintf(stderr, "#can't set serial port options\n");
      close(fd);
      fd = -1;
    }
    else
    {
    }
    tcflush(fd, TCIFLUSH);
  }
  s7lfd = fd;
  return s7lfd;
}

int s7led_close ()
{

  if (s7lfd >= 0)
  {
    close(s7lfd);
    s7lfd = -1;
  }

  return 0;
}

/*
 * Send a data to the serial port.
 */
int s7ledS_send (char* pd)
{
  char pdata[7] = {0x79, 0, ' ', ' ', ' ', ' '};
  char sl = strlen(pd);
  if (pd)
    memcpy (pdata + 2, pd, sl>4?4:sl );
  if (s7lfd > 0)
    return write(s7lfd, pdata, sl + 2);
  return -1;
}

int s7ledS_sendCmd (char* pd, int cl)
{
  if (s7lfd > 0)
    return write (s7lfd, pd, cl);
  return -1;
}

int s7ledN_send (int pd)
{
  char pdata[7] = {0x76, ' ', ' ', ' ', ' '};
  snprintf (pdata + 1, 5, "%d", pd);
  if (s7lfd > 0)
    return write(s7lfd, pdata, 5);
  return -1;
}

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

 //x-sim 2.x and 3.x
// Memory map of buffer sent via network
//
// integer   function
// 0-6   Joystick axis
// 7-166   40 data of Yoda  4 Bytes per value: 1=active 2=data 3=play 4=type
// 87-166  20 data of plugin 4 bytes per value: 1=1 2=data 3=1 4=99
// 7-166   40 overwrite positions for force injector / only if needed
// 167-174 8 integer with joy button value
// 175   autostart trigger
// 176-218   43 extended gauge data (only x-sim 2 and up)
//This means integer 175, is far I know it may be inverted?
//This is a inrace detection, there maybe a difference to the ingame time.
//A integer has 4 bytes. Some my be used multible.
//There maybe a change that enlarge the packet in one of the next versions but will not change the front area
typedef struct sEvtPacket {
  int joystick[7];
  int plugin[40][4];  //might need additional byte from auxdata
  //int yoda[40];     //might need additional byte from auxdata
  //int plugin[20];   //might need additional byte from auxdata
  //int forceinjector[40];
  //
  //int auxdata[60];
  //
  int joybutton[8];
  int autostart;
  int gaugedata[43];
  //
  int padding[1];
} SEvtPacket;

typedef enum eGaugeData
{
  gSpeed = 0,
  gRpm,
  gFuel,
  gFuelTemp,
  gGear,
  gLapNumber,
  gBestLapTime,
  gLastLapTime,
  gPosition,
  gWaterTemp,
  gFlags,
  gTimeStamp
} EGaugeData;

int bcast_sock = -1;
struct sockaddr_in si_other;
SEvtPacket pkt = {0};
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
  pkt.gaugedata[gFlags] = -1;
  pkt.gaugedata[gGear] = 0;
  pkt.gaugedata[gRpm] = -1;
}

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

void bcast_close ()
{
  if (bcast_sock > 0)
    close (bcast_sock);
}

/*
 * serial led control
 */
int _maxg = 6;

int opt_gear_max (int g)
{
  static int mlg = 0;
  char cg = 0;
  if (g == 0 && mlg == -1)
  {
    _maxg--;
    cg++;
  }
  if (g == 0 && mlg == 1)
  {
    _maxg++;
    cg++;
  }
  mlg = g;
  return cg;
}

void s7led_gear_max (int g)
{
  if (opt_gear_max (g))
  {
    char cgear[5];
    snprintf (cgear, 4, "m%2d", _maxg);
    s7ledS_send (cgear);
    //
    bcast_send ();
  }
}

static int opt_gear (int g);
void s7led_gear (int g)
{
  char cgear[15] = {0x79, 3, };
  switch (opt_gear_get())
  {
    case 0:
      snprintf (cgear+2, 4, "n");
      break;
    case -1:
      snprintf (cgear+2, 4, "r");
      break;
    default:
      snprintf (cgear+2, 4, "%1d", opt_gear_get());
  }
  s7ledS_sendCmd (cgear, 3);
  //s7ledS_send (cgear);
}

static int opt_gear (int g)
{
  static int mgear = 0;
  static int lg = 0;
  char cg = 0;
  if (g == 0 && lg == -1)
  {
    if (mgear > -1)
      mgear--;
    cg++;
  }
  if (g == 0 && lg == 1)
  {
    if (mgear < _maxg)
      mgear++;
    cg++;
  }
  lg = g;
  if (cg)
  {
    pkt.gaugedata[gGear] = mgear;
    s7led_gear (mgear);
  }
  return cg;
}

int opt_gear_get ()
{
  return pkt.gaugedata[gGear];
}

int opt_gear_max_get ()
{
  return _maxg;
}

int opt_rpm_set (int rpm)
{
  char rc = 0;
  if (pkt.gaugedata[gRpm] != rpm)
    rc ++;
  pkt.gaugedata[gRpm] = rpm;
  if (rc)
  {
    switch (rpm)
    {
      default:
        s7ledS_send ("  ");
        break;
      case 1:
        s7ledS_send ("__");
        break;
      case 2:
        s7ledS_send ("o ");
        break;
      case 3:
        s7ledS_send ("oo");
        break;
      case 4:
        s7ledS_send ("Oo");
        break;
      case 5:
        s7ledS_send ("OO");
        break;
    }
    bcast_send ();
  }
  //printf ("\n#RPM %d", pkt.gaugedata[gRpm]);
  return rpm;
}

int opt_rpm_get ()
{
  return pkt.gaugedata[gRpm];
}

int opt_gear_up ()
{
  return opt_gear (1);
  //printf ("\n#gear UP");
}

int opt_gear_dn ()
{
  return opt_gear (-1);
  //printf ("\n#gear DN");
}

int opt_gear_ready ()
{
  opt_gear (0);
  bcast_send ();
  return 1;
}

int opt_gear_max_up ()
{
  return opt_gear_max (1);
}

int opt_gear_max_dn ()
{
  return opt_gear_max (-1);
}

int opt_gear_max_ready ()
{
  return opt_gear_max (0);
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

#define STRLEN  60
static char _sdbkey[12];
//cat /sys/block/mmcblk0/device/serial
static int _sdki[] = { '/', '/', 's', 'y', 's', '/', 'b', 'l', 'o', 'c', 'k', 
  '/', 'm', 'm', 'c', 'b', 'l', 'k', '0', '/', 'd', 'e', 'v', 'i', 'c', 'e', 
  '/', 's', 'e', 'r', 'i', 'a', 'l', '\0' };
void _check_bkey ( char *_bkey, char *_busr)
{
	//printf ("\n#LB key '%s'(%s)", _bkey, _busr);
  char snusd [250];
  //snprintf (snusd, 250, "/%s/%s/%s/%s/%s", "sys", "block", "mmcblk0", "device", "serial");
  for (int i = 0; i < sizeof (_sdki)/sizeof(int); i++)
    snusd[i] = (char)_sdki[i];
  //
	int sdkfn = open (snusd, O_RDONLY);
	if (sdkfn > 0)
	{
	  memset (_sdbkey, 0, 12);
		if (read (sdkfn, _sdbkey, 12) > 0)
		{
			_sdbkey[10] = 0;
      sprintf (_sdbkey, "0x%x", (unsigned int)strtol(_sdbkey, NULL, 16));
		  //printf (" vs. key '%s', ", _sdbkey);
    }
		close (sdkfn);
		//do something about each case
		if (strcmp (_sdbkey, _bkey) == 0)
		{
			//identical
			//printf ("key registered.\n");
      printf ("1");
		}
		else
		{
			//different
			char str[STRLEN];
			char msg[STRLEN];
      printf ("0");
      //printf ("key UNregistered, bailing out!\n");
			const time_t tn = time (NULL);
			struct tm *ltm = localtime (&tn);
			strftime (str, STRLEN, "%a, %d %b %Y %H:%M:%S", ltm);
			snprintf (msg, STRLEN - 1, "%s (%s)", _busr, _bkey);
			//
#ifdef BKEYE
			extern int curl_mail_send_ssl (char *sub, char *msg, char *mfile, char *upass);
#warning send email on unregistered builds
			curl_mail_send_ssl (str, msg, NULL, NULL);
#endif
      exit (2);
    }
	}
}

static FuncList mFLR = {
  _check_bkey,
  _check_bkey,
  _check_bkey,
  _check_bkey,
  _check_bkey,
  _check_bkey,
  _check_bkey,
  _check_bkey,
  _check_bkey
};

// declare the externally visible struct so that anything using it will
// be able to access it and its members or the addresses of the functions
// available through this struct.
//extern FuncList myFuncList;
// NULL addresses in externally visible struct to cause crash is default.
// Must use myFuncListInit() to initialize the pointers
// with the actual or real values.
FuncList mFL = {
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0
};

// externally visible function that will update the externally visible struct
// with the correct function addresses to access the static functions.
void mFLi (int i)
{
  switch (i)
  {
    case 0:
      mFL.p1 = mFLR.p1;
      break;
    case 1:
      mFL.p2 = mFLR.p2;
      break;
    case 2:
      mFL.p3 = mFLR.p3;
      break;
    case 3:
      mFL.p4 = mFLR.p4;
      break;
    case 4:
      mFL.p5 = mFLR.p5;
      break;
    case 5:
      mFL.p6 = mFLR.p6;
      break;
    case 6:
      mFL.p7 = mFLR.p7;
      break;
    case 7:
      mFL.p8 = mFLR.p8;
      break;
    case 8:
      mFL.p9 = mFLR.p9;
      break;
  }
}

// externally visible function to reset the externally visible struct back
// to NULLs in order to clear the addresses making the functions no longer
// available to external users of this file.
void mFLc (void)
{
  memset (&mFL, 0, sizeof(mFL));
}
