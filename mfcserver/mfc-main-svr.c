/*
 Copyright (c) 2015 mirel lazar <mirel.t.lazar@gmail.com>
 License: GPLv3
 */

#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>

/* Unix */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <poll.h>

/* C */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>

#include <libusb-1.0/libusb.h>
#ifdef _USE_PIGPIO_
//for demo platform control
#include <pigpio.h>
#endif
//our own
#include "scn_adapter.h"
#include "extras.h"


#define DEBUG 0
#define debug_print(fmt, ...) \
  do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); if (DEBUG) fflush (stderr); } while (0)
#define debug_printl(lvl, fmt, ...) \
  do { if (DEBUG >= lvl) fprintf(stderr, fmt, __VA_ARGS__); if (DEBUG >= lvl) fflush (stderr); } while (0)


#define uint8_t unsigned char

#define NAME_LENGTH 128

int usleep(long usec);
int p2dof_home_init ();

#define MRANGE_MAX  10000

#include <sched.h>
#include <stdio.h>

int set_prio()
{
  /*
   * Set highest priority & scheduler policy.
   */
  struct sched_param p =
  { .sched_priority = sched_get_priority_max(SCHED_FIFO) };

  if( sched_setscheduler(0, SCHED_FIFO, &p) < 0 )
  {
    perror("sched_setscheduler");
    return -1;
  }
  return 0;
}

/*
 *
 */
void mfc_mot_start ();
/*
 *
 */
int fd;
/*
 *
 */

//int mfc_emufd = -1;
int mfc_motfd = -1;
int mfc_svrfd = -1;
#ifdef _USE_PIGPIO_
//gpio demo platform
#define PGPIO_L   17  //rpi pin 11 - left servo
#define PGPIO_R   27  //rpi pin 13 - right servo
int mfc_gpiop = 1;
int gpio_demo_on = 1;
#endif
//int mfc_whlfd = -1;
//int mfc_joyfd = -1;
/*
 *
 */
char *pp_hidraw = "/dev/hidraw0";
char _p_hidraw[250];
char *pp_ttyusb = "/dev/ttyUSB0";
char _p_ttyusb[250];
char _odbg = 0;
char _omot = 0;   //don't process motion, just debug it
char _ostdin = 1;
char _ofifo = 0;
char _owheel = 0, _ogame = 0, _omotion = 1, _offb = 1, _owweight = 1, _ojswap = 0;
char _oemu = 0;
char _oml = 2;  //motion limit
int  _odz = 10;  //motion dead zone
int  _oin = 3;   //motion wheel input multiplier
int  _ovib = 20; //vibrations multiplier
int  _odrv1  = 0;  //driver 1 offset
int  _odrv2  = 0;  //driver 2 offset
int js_pos_dz = 0;
int js_pos_max = 0;
int _omspeed = 4;
char *pp_jrk1 = "/dev/ttyUSB1";
//char *pp_jrk2 = "/dev/ttyACM2";
char bcast_addr[35] = {0};
char _done = 0;

#define MOTION_FREQ_MS      30
#define MOTION_VIB_GT6_MUL  8
#define FFB_TMO             2000 //msecs

void terminate (int sig)
{
  _done = 1;
}

static unsigned int dtime_ms ()
{
  static unsigned long lms = 0;
  unsigned long cms = get_millis ();
  unsigned long ms = cms - lms;
  lms = cms;
  return (unsigned int)ms;
}

unsigned long mtime_get (int reset)
{
  static unsigned long _tlast = 0;
  unsigned long _mtime;
  //init start
  unsigned long _tnow = get_millis ();
  if (_tlast == 0)
    _tlast = _tnow;
  //get current time
  _mtime = _tnow - _tlast;
  //
  if (reset)
    _tlast = _tnow;
  //
  if (0) 
    printf ("\n#mtime %lu reset %d", _mtime, reset);
  //
  return _mtime;
}


int stdin_process_message (char *buf)
{
  switch (buf[0])
  {
    case 'd': //debug enabled
      _odbg = atoi((const char *)buf + 1);
      printf ("\n#debug level: %d", _oin);
      break;
    case 'm': //send motion data
      _omotion = atoi(buf + 1);
      printf ("\n#m:motion: %s", _omotion?"on":"off");
      if (_omotion)
      {
        p2dof_home_init ();
        //set speed
        mfc_mot_start ();
      }
      break;
    case 's': //motion speed: more means faster, 0 is fastest
      _omspeed = atoi(buf + 1);
      //kangSetSpeed (_omspeed);
      printf ("\n#motion speed: %d", _omspeed);
      //set speed
      mfc_mot_start ();
      //
      break;
    default:
      printf ("\n#!w:unknown option: %c", buf[0]);
  }
  fflush (stdout);
  return 1;
}

int env_init (int argc, char **argv)
{
  /*
   * init params and stuff
   */
  //
  int i;
  for (i = 1; i < argc; i++)
  {
    if (argv[i][0] == '-')
      switch (argv[i][1])
      {
        case 'l': //motion response
          _oml = atoi(argv[i]+2);
          if (_oml < 0 || _oml > 100)
            _oml = 2;
          break;
        case 'd': //debug enabled
          _odbg++;
          break;
        case 'm': //debug enabled
          _omot++;
          break;
        case 's': //motion speed: more means faster, 0 is fastest
          _omspeed = atoi(argv[i]+2);
          if (_omspeed < 1 || _omspeed > 7)
            _omspeed = 4;
          break;
        default:
          printf ("\n#w:unknown option -%c", argv[i][1]);
      }
  }
  /*
   * summary configuration
   *
   */
  printf ("\n# ##");
  printf ("\n#MFC server");
  printf ("\n#running configuration:");
  printf ("\n#   motion response %dms (-l%d) range [0..100]", _oml, _oml);
  printf ("\n#      motion speed %d (-s%d) range [1..7]", _omspeed, _omspeed);
  printf ("\n#   verbosity level %d (-d%d)", _odbg, _odbg);
  printf ("\n#    motion process %s (-m)", _omot?"no":"yes");
  printf ("\n# ##");
  //
  return 1;
}

//
//--
//
#ifdef _USE_FIFOS_

#define MFC_INPFIFO  "/tmp/mfcsinp"
#define MFC_OUTFIFO  "/tmp/mfcsout"
#define MFC_ERRFIFO  "/tmp/mfcserr"

int mfc_pipe_new (char *name, int perm)
{
  unlink(name);
  mkfifo(name, perm);

  if (chmod(name, perm) < 0)
  {
    printf ("\nE:unable to set permissions to input fifo '%s' as %x", 
      MFC_INPFIFO, perm);
    //DBG(DBG_ALWAYS, "Can't set permissions (%d) for %s, %m", perm, name);
    return 0;
  }
  return 1;
}

int mfc_fifoinp = -1;
int mfc_fifoinp_new ()
{
  if (mfc_fifoinp < 0) //don't recreate it
  {
    mfc_pipe_new (MFC_INPFIFO, 0662);
    if ((mfc_fifoinp = open (MFC_INPFIFO, O_RDONLY|O_NONBLOCK, S_IRUSR | S_IRGRP | S_IROTH)) == -1)
    {
      printf ("\n#E: unable to open input fifo '%s'", MFC_INPFIFO);
      return 0;
    }
  }
  return 1;
}

int mfc_fifoinp_del ()
{
  if (mfc_fifoinp > 0)
  {
    close (mfc_fifoinp);
    unlink(MFC_INPFIFO);
    mfc_fifoinp = -1;
  }
}
#endif

//
int rkntr = 0, rkffb = 0, rkdat = 0, rkdrop = 0, pl, rl;
/*
* timestamping
*/
long mtime = 0, rtime = 0, ntime = 0;
long ffbtime = 0;
//

//
// MOTion management
/*
--
pi@retropie:~ $ ls -la /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A504KG*
lrwxrwxrwx 1 root root 13 May 15 21:25 /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A504KGF1-if00-port0 -> ../../ttyUSB1
lrwxrwxrwx 1 root root 13 May 15 21:25 /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A504KGHA-if00-port0 -> ../../ttyUSB0
--
  idVendor           0x0403 Future Technology Devices International, Ltd
  idProduct          0x6001 FT232 USB-Serial (UART) IC
  iManufacturer           1 FTDI
  iProduct                2 FT232R USB UART
  iSerial                 3 A504KGF1
      iInterface              2 FT232R USB UART
--
  idVendor           0x0403 Future Technology Devices International, Ltd
  idProduct          0x6001 FT232 USB-Serial (UART) IC
  bcdDevice            6.00
  iManufacturer           1 FTDI
  iProduct                2 FT232R USB UART
  iSerial                 3 A504KGHA
      iInterface              2 FT232R USB UART
--
*/

/*
*
* --
interface for 2dof with roll and pitch using SCN6 actuators
SCN6 actuators use homing to 0 (full retract) and have a range of [0, -10000]
--
roll(left- to right+), pitch(down- to up+) based on [R0, P0] coords when leveled

the 2dof platform will move to new [R, P] positions based on FFB and wheel input
roll- is L+ and R-
roll+ is L- and R+
--
pitch- is L- and R-
pitch+ is L+ and R+
--
ranges:
roll [-5000, 5000]
pitch [-5000, 5000]
* note: all calls to position change need to map to these ranges. home is [0, 0]
--
*/

typedef struct {
  int lfd, rfd;       //left SCN6 fd / right SCN6 fd
  int croll, cpitch;  //current roll / pitch positions
  char invr, invp;    //invert roll / pitch
} p2dof;
int actl_max = 0;
int l_min = 0;
int l_max = 0;
int actr_max = 0;
int r_min = 0;
int r_max = 0;

int p2dof_init (char *lport, char *rport);
int p2dof_set_home ();
//speed values
#define P2DOF_SPD_GEAR1   0x02EE
#define P2DOF_SPD_GEAR2   0x04E2
#define P2DOF_SPD_GEAR3   0x06D6
#define P2DOF_SPD_GEAR4   0x08CA
#define P2DOF_SPD_GEAR5   0x0ABE
#define P2DOF_SPD_GEAR6   0x0CB2
#define P2DOF_SPD_GEAR7   0x0EA6
int p2dof_set_speed (int spd);
int p2dof_set_pos (int *pdat);
int p2dof_set_poss (int *pdat);
int p2dof_release ();
int p2dof_fill_fds (struct pollfd fds[], int idx);

p2dof mfc2dof = {-1,-1, 0,0, 0,0};

#include <termios.h>
void cfmakeraw(struct termios *termios_p);

//
static libusb_context* ctx = NULL;
static libusb_device** devs = NULL;
static ssize_t cnt = 0;
#define FTDI_VID  0x0403
#define FTDI_PID  0x6001
static char scnports[2][256] = {"/dev/ttyUSB0", "/dev/ttyUSB1"};
static char *lscnp = scnports[0], *rscnp = scnports[1];

static int p2scn_find ()
{
  int dev_i, ret, bp, pk = 0;
  libusb_device_handle *handle = NULL;
	char description[256];
	unsigned char string[256];
  //
  if (!ctx)
  {
    ret = libusb_init (&ctx);
    if (ret < 0)
    {
      fprintf (stderr, "\n#!libusb_init: %s.\n", libusb_strerror(ret));
      return -1;
    }
  }
  //always get the new list of devices
  if (devs)
  {
    libusb_free_device_list(devs, 1);
    devs = NULL;
  }
  cnt = libusb_get_device_list (ctx, &devs);
  if (cnt < 0)
  {
    fprintf(stderr, "\n#!libusb_get_device_list: %s.\n",
        libusb_strerror(cnt));
    return -1;
  }

  for (dev_i = 0; dev_i < cnt; ++dev_i)
  {
    struct libusb_device_descriptor desc;
    ret = libusb_get_device_descriptor(devs[dev_i], &desc);
    if (!ret)
    {
      //printf ("\n#i:USB found %04x:%04x", desc.idVendor, desc.idProduct);
      if (desc.idVendor == FTDI_VID && desc.idProduct == FTDI_PID)
      {
        ret = libusb_open (devs[dev_i], &handle);
        if (!ret)
        {
          ret = libusb_get_string_descriptor_ascii (handle, desc.iManufacturer, string, sizeof(string));
          if (!ret)
            continue;
          bp = snprintf (description, 256 - bp, "%s", string);
          ret = libusb_get_string_descriptor_ascii (handle, desc.iProduct, string, sizeof(string));
          if (!ret)
            continue;
          bp += snprintf (description + bp, 256 - bp, " %s", string);
          ret = libusb_get_string_descriptor_ascii (handle, desc.iSerialNumber, string, sizeof(string));
          if (!ret)
            continue;
          bp += snprintf (description + bp, 256 - bp, " %s", string);
          //
          char *ldp = description;
          while (*ldp++ > 0)
            if (*ldp == ' ')
              *ldp = '_';
          //
          //printf ("\n#i:%s", description);
          if (pk < 2)
            snprintf (scnports[pk++], 256, "/dev/serial/by-id/usb-%s-if00-port0", description);
          //
          libusb_close (handle);
        }
      }
    }
  }//for
  //did we find at least 2 FTDI ports?
  if (pk > 1)
  {
    //sort them alphabetically
    if (strcmp (scnports[0], scnports[1]) > 0)
    {
      lscnp = scnports[1]; rscnp = scnports[0];
    }
    else
    {
      lscnp = scnports[0]; rscnp = scnports[1];
    }
  }
  //always get the new list of devices
  if (devs)
  {
    libusb_free_device_list(devs, 1);
    devs = NULL;
  }
  //
  return pk;
}

static int p2scn_home_init ()
{
  //send on
  printf ("\n#i:homing platform..");
  //
  scn_set_son (mfc2dof.lfd);
  scn_set_son (mfc2dof.rfd);
  //send home
  scn_set_home (mfc2dof.lfd);
  scn_set_home (mfc2dof.rfd);
  //wait for position to be reported as 0
  int mk = 10;
  while (scn_get_pos (mfc2dof.lfd) && mk)
  {
    usleep (200000);
    mk--;
  }
  //
  mk = 10;
  while (scn_get_pos (mfc2dof.rfd) && mk)
  {
    usleep (200000);
    mk--;
  }
  //look for range max
  //flush the speed response
  p2dof_set_speed (P2DOF_SPD_GEAR1);  //slow
  //set both to negative max allowed by the driver
  printf ("\n#i:looking for max positions..");
  fflush (stdout);
  //set to -edge: 0xC0000000
  scn_set_pos (mfc2dof.lfd, 0xC0000000);
  //set to -edge: 0xC0000000
  scn_set_pos (mfc2dof.rfd, 0xC0000000);
  int crpos = 0x3FFFFFFF, rpos;
  for (rpos = scn_get_pos (mfc2dof.rfd); crpos != rpos; crpos = scn_get_pos (mfc2dof.rfd))
  {
    usleep (200000);
    rpos = crpos;
  }
  //
  int clpos = 0x3FFFFFFF, lpos;
  for (lpos = scn_get_pos (mfc2dof.lfd); clpos != lpos; clpos = scn_get_pos (mfc2dof.lfd))
  {
    usleep (200000);
    lpos = clpos;
  }
  actl_max = lpos;
  actr_max = rpos;
  //compute 5% deadzone range
  printf (" left range [0..%d], right range [0..%d]", actl_max, actr_max);
  int _dz = actl_max * 3 / 100;
  l_min = _dz;
  l_max = actl_max - _dz;
  _dz = actr_max * 3 / 100;
  r_min = _dz;
  r_max = actr_max - _dz;
  printf ("\n#i:actual left range [%d..%d], right range [%d..%d]", l_min, l_max, r_min, r_max);
  //
  printf ("\n#i:wait for homing to finish..");
  fflush (stdout);
  p2dof_set_home ();
  printf (" homing done");
  fflush (stdout);
  //
  return 1;
}

int p2dof_home_init ()
{
  return p2scn_home_init ();
}

static int p2scn_init (char *lport, char *rport)
{
  //char lscnp[] = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A504KGF1-if00-port0";
  //char rscnp[] = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A504KGHA-if00-port0";
  //
  int scnpk = p2scn_find ();
  if (scnpk < 2)
  {
    printf ("\n#E:found %d of 2 FTDI adapters for SCN", scnpk);
    return -1;
  }
  //
  mfc2dof.lfd = open (lscnp, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (mfc2dof.lfd == -1)
  {
    printf ("\n#W:SCN L not found on %s", lscnp);
    return -1;
  }
  mfc2dof.rfd = open (rscnp, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (mfc2dof.rfd == -1)
  {
    printf ("\n#W:SCN R not found on %s", rscnp);
    close (mfc2dof.lfd);
    return -1;
  }
  struct termios options;
  tcgetattr (mfc2dof.lfd, &options);
  //
  cfsetispeed (&options, B9600);
  cfsetospeed (&options, B9600);
  cfmakeraw (&options);
  //
  if (tcsetattr (mfc2dof.lfd, TCSANOW, &options) < 0)
  {
    printf ("\n#E:can't set L serial port options '%s'", lscnp);
    close (mfc2dof.lfd);
    close (mfc2dof.rfd);
    return -1;
  }
  //right actuator
  tcgetattr (mfc2dof.rfd, &options);
  //
  cfsetispeed (&options, B9600);
  cfsetospeed (&options, B9600);
  cfmakeraw (&options);
  //
  if (tcsetattr (mfc2dof.rfd, TCSANOW, &options) < 0)
  {
    printf ("\n#E:can't set R serial port options '%s'", rscnp);
    close (mfc2dof.lfd);
    close (mfc2dof.rfd);
    return -1;
  }
  //home actuators
  printf("\n#i:connected to L '%s', on %d", lscnp, mfc2dof.lfd);
  printf("\n#i:connected to R '%s', on %d", rscnp, mfc2dof.rfd);
  //
  if (scn_get_status (mfc2dof.lfd) == 0 || scn_get_status (mfc2dof.rfd) == 0)
  {
    printf ("\n#E:motion platform not responding, bailing out");
    _done = 1;
    return -1;
  }
  //
  p2dof_home_init ();
  //
  return mfc2dof.lfd;
}

int p2dof_init (char *lport, char *rport)
{
  return p2scn_init (lport, rport);
}

static int p2scn_release ()
{
  //send off
  scn_set_soff (mfc2dof.lfd);
  scn_set_soff (mfc2dof.rfd);
  //
  close (mfc2dof.lfd);
  close (mfc2dof.rfd);
  //
  return 0;
}

int p2dof_release ()
{
  return p2scn_release ();
}
#if 0
static int p2scn_fill_fds (struct pollfd fds[], int idx)
{
  fds[idx].fd = mfc2dof.lfd;
  fds[idx].events = POLLIN | POLLHUP;
  fds[idx + 1].fd = mfc2dof.rfd;
  fds[idx + 1].events = POLLIN | POLLHUP;
  //
  return 2;
}
#endif
static int _lpkt[MFC_PKT_SIZE] = {0};
static int p2scn_set_home ()
{
  char rsp[20];
  if (scn_get_status (mfc2dof.lfd) == 0 || scn_get_status (mfc2dof.rfd) == 0)
  {
    printf ("\n#e:motion platform not responding, bailing out");
    _done = 1;
    return -1;
  }
  //flush the speed response
  p2dof_set_speed (P2DOF_SPD_GEAR1);  //slow
  //flush the positioning response
  p2dof_set_pos (_lpkt);
  scn_get_response (mfc2dof.lfd, rsp);
  scn_get_response (mfc2dof.rfd, rsp);
  //
  int mk = 5;
  while (3 != get_map (scn_get_pos (mfc2dof.lfd), l_min, l_max, 1, 6) && mk)
  {
    //printf ("\n#i:L%d map %d", mfc2dof.lfd, get_map (scn_get_pos (mfc2dof.lfd), l_min, l_max, 1, 6));
    usleep (200000);
    mk--;
  }
  //
  mk = 5;
  while (3 != get_map (scn_get_pos (mfc2dof.rfd), r_min, r_max, 1, 6) && mk)
  {
    //printf ("\n#i:R%d map %d", mfc2dof.rfd, get_map (scn_get_pos (mfc2dof.rfd), r_min, r_max, 1, 6));
    usleep (200000);
    mk--;
  }
  //
  return 0;
}

int p2dof_set_home ()
{
  return p2scn_set_home ();
}

#ifdef _USE_PIGPIO_
void gpio_demo_stop ()
{
  //stop the demo platform as well
  if (mfc_gpiop && gpio_demo_on)
  {
    gpioServo (PGPIO_L, 0);
    gpioServo (PGPIO_R, 0);
  }
}

void gpio_demo_spos (int lp, int rp)
{
  if (mfc_gpiop && gpio_demo_on)
  {
    //left
    int pp = get_map (lp, -10000, 0, 2000, 1000);
    gpioServo (PGPIO_L, pp);
    //right
    pp = get_map (rp, -10000, 0, 1000, 2000);
    gpioServo (PGPIO_R, pp);
  }
}
#endif// _USE_PIGPIO_

/*
spd: speeds 
doc: speed order default value set range is 0000H..57E4H with unit of 0.2r/min
doc: accel order default value set range is 0001H..07FFH with unit of 0.1r/min
*simtools 20.000x 0.2rpm = 4000rpm 0x4E20
*simtools accel 1000 (0x03E8)

0 fastest: 3750 x 0.2rpm = 750rpm 0x0EA6
1 fastest: 3250 x 0.2rpm = 650rpm 0x0CB2
2 fastest: 2750 x 0.2rpm = 550rpm 0x0ABE
3 fastest: 2250 x 0.2rpm = 450rpm 0x08CA
4 fastest: 1750 x 0.2rpm = 350rpm 0x06D6
5 fastest: 1250 x 0.2rpm = 250rpm 0x04E2 
6 slowest: 750  x 0.2rpm = 150rpm 0x02EE
*/
int p2dof_set_speed (int spd)
{
  char rsp[20];
  if (_odbg)
    printf ("\n#i:motion %d speed 0x%04X", spd, spd);
  //
  if (spd < P2DOF_SPD_GEAR1 || spd > P2DOF_SPD_GEAR7)
    spd = P2DOF_SPD_GEAR1;
  
  //
  scn_set_vel (mfc2dof.lfd, spd, DEFA_ACMD);
  scn_get_response (mfc2dof.lfd, rsp);
  scn_set_vel (mfc2dof.rfd, spd, DEFA_ACMD);
  scn_get_response (mfc2dof.rfd, rsp);
  //
  return 0;
}

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

int p2dof_set_poss (int *pdata)
{
  //todo: set speed
  if (0)  //setting the speed is slow for now, don't do it
    p2dof_set_speed (pdata[8]);
  //
  p2dof_set_pos (pdata);
  //
  return 0;
}

//2DOF motion positions
int dof2l = 0;
int dof2r = 0;
#define SMOOTHMOVE  50
static int p2scn_set_pos (int *pdata)
{
  //simplest profiling EVER!
  #if 0
  dof2l = pdata[MFC_PIPITCH] + pdata[MFC_PIROLL]; //pitch + roll
  dof2r = pdata[MFC_PIPITCH] - pdata[MFC_PIROLL]; //pitch - roll
  #else
  dof2l = pdata[MFC_PIPITCH] + pdata[MFC_PISURGE] + pdata[MFC_PIHEAVE] + pdata[MFC_PIROLL] + pdata[MFC_PISWAY];
  dof2r = pdata[MFC_PIPITCH] + pdata[MFC_PISURGE] + pdata[MFC_PIHEAVE] - pdata[MFC_PIROLL] - pdata[MFC_PISWAY];
  #endif
  //
  if (_odbg)
    printf ("\n#i:in pos L%d, R%d", dof2l, dof2r);
  //left
  dof2l = get_cmap (dof2l, -10000, 10000, l_max, l_min);
  //right
  dof2r = get_cmap (dof2r, -10000, 10000, r_max, r_min);
  //smooth curve: shave some positions to avoid the motor to move constantly
  dof2l -= (int)dof2l % SMOOTHMOVE;
  dof2r -= (int)dof2r % SMOOTHMOVE;
  //avoid an issue with lack of move for this position
  if (dof2l == -5000)
    dof2l++;
  if (dof2r == -5000)
    dof2r++;
  //char rsp[20];
  if (_omot == 0)
  {
    if (_odbg)
    {
      printf ("\n#i@%04d:cmap pos L%d, R%d", dtime_ms(), dof2l, dof2r);
      int *_cpkt = pdata;
      printf ("\n#i.roll:% 6d (r: % 5d / s: % 5d) | pitch: % 6d \t(p: % 5d / s: % 5d / h: % 5d)",
        _cpkt[MFC_PIROLL] + _cpkt[MFC_PISWAY], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY],
        _cpkt[MFC_PIPITCH] + _cpkt[MFC_PISURGE] + _cpkt[MFC_PIHEAVE],
        _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE]);
    }
    scn_set_pos (mfc2dof.lfd, dof2l);
    //scn_get_response (mfc2dof.lfd, rsp);
    scn_set_pos (mfc2dof.rfd, dof2r);
    //scn_get_response (mfc2dof.rfd, rsp);
#ifdef _USE_PIGPIO_
    if (0)
      gpio_demo_spos (dof2l, dof2r);
#endif
  }
  else
  {
    int *_cpkt = pdata;
    if (_odbg)
      printf ("\n#i.roll:% 6d (r: % 5d / s: % 5d) | pitch: % 6d \t(p: % 5d / s: % 5d / h: % 5d)",
        _cpkt[MFC_PIROLL] + _cpkt[MFC_PISWAY], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY],
        _cpkt[MFC_PIPITCH] + _cpkt[MFC_PISURGE] + _cpkt[MFC_PIHEAVE],
        _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE]);
    printf ("\n#i:pos L%d, R%d", dof2l, dof2r);
  }
  //
  return 1;
}

int p2dof_set_pos (int *pdata)
{
  return p2scn_set_pos (pdata);
}

//
// --
//
void mfc_svr_add ()
{
  if (mfc_svrfd < 0)
  {
    mfc_svrfd = mfc_bcast_prep ("127.0.0.1", 1);
    if (_odbg)
      printf ("\n#i:SVR connected to %d", mfc_svrfd);
  }
}

void mfc_svr_del ()
{
  if (mfc_svrfd > 0)
  {
    printf ("\n#i:closing server socket..");
    mfc_bcast_close ();
  }
  mfc_svrfd = -1;
  printf ("\n#w:SVR disconnected");
  fflush (stdout);
}

//
// --
//
void mfc_mot_start ()
{
  printf ("\n#i:FFB ON, gear %d of 7", _omspeed);
  if (mfc_motfd > 0)
  {
    switch (_omspeed)
    {
      case 2:
        p2dof_set_speed (P2DOF_SPD_GEAR2); //slower
        break;
      case 3:
        p2dof_set_speed (P2DOF_SPD_GEAR3); //slow
        break;
      case 4:
        p2dof_set_speed (P2DOF_SPD_GEAR4); //normal
        break;
      case 5:
        p2dof_set_speed (P2DOF_SPD_GEAR5); //faster
        break;
      case 6:
        p2dof_set_speed (P2DOF_SPD_GEAR6); //even faster
        break;
      case 7:
        p2dof_set_speed (P2DOF_SPD_GEAR7); //fastest
        break;
      default:
        p2dof_set_speed (P2DOF_SPD_GEAR1); //slowest
        break;
    }
    //kangAllStart ();
    printf ("\n#i:motion platform started");
  }
}
//
void mfc_mot_add ()
{
  if (mfc_motfd < 0)
  {
    mfc_motfd = p2dof_init (NULL, NULL);
    //
    if (mfc_motfd > 0)
      mfc_mot_start ();
    //
    fflush (stdout);
  }
}
//
// --
//
void mfc_mot_del ()
{
  if (mfc_motfd > 0)
  {
    //printf ("\n#i:homing motion platform..");
    //fflush (stdout);
    //p2dof_set_home ();
    //kangLSetPosition (2040 + _odrv1);
    //kangRSetPosition (2040 + _odrv2);
    //usleep (1000000); //sleep 1sec to leave enough time for the last packet to be sent
    //stop for now
    p2dof_release ();
  }
  mfc_motfd = -1;
  printf ("\n#w:MOT disconnected");
  fflush (stdout);
}

void mfc_mot_stop ()
{
  printf ("\n#i:FFB OFF!");
  if (mfc_motfd > 0)
  {
    p2dof_set_home ();
    printf ("\n#i:motion platform stopped");
  }
}

void mfc_setup_fds ()
{
  if (mfc_motfd < 0)
  {
    mfc_mot_add ();
    fflush (stdout);
  }
  if (mfc_motfd < 0)
    _done = 1;
  else if (mfc_svrfd < 0)
  {
    mfc_svr_add ();
    fflush (stdout);
  }
}

int axp = 0, ayp = 0;
int main (int argc, char **argv)
{
  env_init (argc, argv);
  set_prio();
  (void) signal(SIGINT, terminate);
  (void) signal(SIGTERM, terminate);
  (void) signal(SIGHUP, terminate);
  //
  /**
   *  allow dead zone of X% relative to wheel center
   */
  //js_pos_dz = (32767 * _odz) / 200;
  //printf ("\n#DZ wpos %d", js_pos_dz);
  /**
   * reduce motion range to X% max
   */
  //js_pos_max = (MRANGE_MAX * _oml) / 200;
  //printf ("\n#ML wpos %d\n", js_pos_max);
#ifdef _USE_PIGPIO_
  if (gpioInitialise() < 0)
    mfc_gpiop = 0;
  else
    gpioSetSignalFunc (SIGINT, terminate);
#endif
  //
  //add command control pipe
  //mfc_fifoinp_new ();
  int i, _mstart, mms = 0, cms, rkdrop = 0;
  int _motion = 1;
  unsigned char buf[254];
  //
  struct pollfd fds[15];
  nfds_t poll_i;
  //poll return is 0 on timeout
  int pr = 0;
  int fdof_mode = PKTT_0DOF;
  mfc_setup_fds ();
  if (mfc_motfd > 0)
  {
    printf ("\n#i:ready");
    fflush (stdout);
  }
  while (!_done)
  {
    //if we have poll timeout, we can look for configuration changes
    if (pr == 0)
    {
      mfc_setup_fds ();
    }
    //prep FDs
    memset((void*)fds, 0, sizeof(fds));
    poll_i = 0;
    //add server fd
    if (mfc_svrfd > 0)
    {
      fds[poll_i].fd = mfc_svrfd;
      fds[poll_i].events = POLLIN | POLLHUP;
      poll_i++;
    }
    //add motion fd
    if (mfc_motfd > 0)
    {
      fds[poll_i].fd = mfc2dof.lfd;
      fds[poll_i].events = POLLIN | POLLHUP;
      poll_i++;
      fds[poll_i].fd = mfc2dof.rfd;
      fds[poll_i].events = POLLIN | POLLHUP;
      poll_i++;
    }
    //
    if (_ostdin)
    {
      //add stdin fd
      fds[poll_i].fd = STDIN_FILENO;
      fds[poll_i].events = POLLIN;
      poll_i++;
    }
#ifdef _USE_FIFOS_
    //in fifo
    if (_ofifo)
    {
      if (mfc_fifoinp > 0)
      {
        fds[poll_i].fd = mfc_fifoinp;
        fds[poll_i].events = POLLIN | POLLHUP;
        poll_i++;
      }
    }
#endif
    //poll all events
    if (poll_i > 0)
    {
      pr = poll (fds, poll_i, 5000);
    }
    else
    {
      pr = 0;
      usleep (5000000); //5sec
      printf (".");
      fflush (stdout);
      //check actuators status
      if (scn_get_status (mfc2dof.lfd) == 0 || scn_get_status (mfc2dof.rfd) == 0)
      {
        printf ("\n#e:motion platform not responding, bailing out");
        _done = 1;
      }
    }
    //ready to process events
    if (pr == 0)
    {
      //default dof mode
      fdof_mode = PKTT_0DOF;
      //check actuators status
      if (scn_get_status (mfc2dof.lfd) == 0 || scn_get_status (mfc2dof.rfd) == 0)
      {
        printf ("\n#e:motion platform not responding, bailing out");
        _done = 1;
      }
      //turn off motors
      if (_motion && mfc_motfd > 0)
      {
        _motion = 0;
        scn_set_soff (mfc2dof.lfd);
        scn_set_soff (mfc2dof.rfd);
        printf ("\n#i@04%lu:STOP motion", mtime);
        //
#ifdef _USE_PIGPIO_
        gpio_demo_stop ();
#endif
      }
    }
    else if (pr < 0)
    {
      printf ("\n#e:error <0x%0x/%s> while polling on %dfds", errno, strerror (errno), (int)poll_i);
      fflush (stdout);
      _done = 1;
    }
    else if (pr > 0)
    {
      /*
       * read/update timestamp
       */
      rtime += mtime;   //update run time
      ffbtime += mtime;
      if (0)
        printf ("\n#i:mtime %lu vs ffbtime %lu", mtime, ffbtime);
      if (ffbtime > 5000) //did we not have ffb data for more than 5s?
      {
        if (_mstart)
        {
          _mstart = 0;
          //mfc_mot_stop ();
        }
      }
      //turn on motors
      if (_motion == 0)
      {
        _motion = 1;
        scn_set_son (mfc2dof.lfd);
        scn_set_son (mfc2dof.rfd);
        printf ("\n#i@04%lu:START motion", mtime);
      }
      //update ffb time
      for (i = 0; i < poll_i; ++i)
      {
        if(fds[i].revents & POLLIN || fds[i].revents & POLLPRI || fds[i].revents & POLLOUT || fds[i].revents & POLLHUP)
        {
          //if (1||/*mtime < 2 || */(rtime % 100) == 0)
          if ((rkntr % 500) == 0)
            printf ("\n#i:poll %dfds:%s:%d@%lums/%lusec, %d/%d pkts(evt/drop)", (int)poll_i, fds[i].revents & POLLIN?"in":"out", fds[i].fd,
                mtime, rtime / 1000, rkntr, rkdrop);
          //
        }
        else
          continue;
        //
        if (0) printf ("\n#i:%d polled", fds[i].fd);
        //
        if(fds[i].revents & POLLIN || fds[i].revents & POLLPRI || fds[i].revents & POLLHUP )
        {
          if (fds[i].revents & POLLPRI) printf ("\n#i:%d polled PRI", fds[i].fd);
#ifdef _USE_FIFOS_
          //command fifo
          if (fds[i].fd == mfc_fifoinp)
          {
            //did we get a disconnect?
            if (fds[i].revents & POLLHUP)
            {
              mfc_fifoinp_del ();
              mfc_fifoinp_new ();
              break;
            }
            //
            if (read (mfc_fifoinp, buf, 50) > 0)
            {
              char *pb = buf;
              buf[49] = '\0';
              while (*pb != '\r' && *pb != '\n' && *pb != '\0')
                pb++;
              *pb = '\0';
              printf("\n#i:IN FIFO read 0x%2.2X '%s'", (((unsigned int) buf[0]) & 0xFF), buf);
            }
            break;
          }//cmd fifo
#endif
          //MOTion
          /*
          */
          if (fds[i].fd == mfc2dof.lfd || fds[i].fd == mfc2dof.rfd)//MOTion
          {
            //did we get a disconnect?
            if (fds[i].revents & POLLHUP)
            {
              mfc_mot_del();
              break;
            }
            char rsp[20];
            scn_get_response (fds[i].fd, rsp);
            //
            break;
          }//MOTion
          //server
          /*
          */
          if (fds[i].fd == mfc_svrfd)//network data
          {
            //did we get a disconnect?
            if (fds[i].revents & POLLHUP)
            {
              mfc_svr_del();
              break;
            }
            if (mfc_bcast_receive () > 0)
            {
              //we received a message from the network
              rkntr++;
              int *pkt = mfc_bcast_pktget ();
              int pktl = mfc_bcast_pktlen ();
              cms = mtime_get (1);
              mms += cms;
              //
              if (_odbg)
                printf ("\n#i:%04lums bcast received pkt 0x%02x len %dB", (long unsigned int)cms, pkt[MFC_PIDOF], pktl);
              //
              if (_odbg > 3)
              {
                int i;
                for (i = 0; i < pktl; i++)
                  printf (", %d", pkt[i]);
              }
              if (pkt[0] == PKTT_DATA)
              {
                //check for native motion mode
                if (fdof_mode == PKTT_2DOFN)
                {
                  //we have native data, skip other modes
                  if (pkt[1] != PKTT_2DOFN)
                  {
                    if (_odbg > 1)
                      printf ("\n#w:in native mode, drop non native pkt 0x%02x", pkt[MFC_PIDOF]);
                    break;
                  }
                }
                else
                {
                  if (fdof_mode != pkt[MFC_PIDOF])
                    printf ("\n#i:switching DOF mode to 0x%02x", pkt[MFC_PIDOF]);
                  //
                  fdof_mode = pkt[MFC_PIDOF];
                }
                //wait a few ms before next motion event to avoid throttle
                if (mms >= _oml)
                {
                  //motion data packet
                  if (_omotion)
                  {
                    if (_odbg)
                      printf ("\n#i:%06lums motion type 0x%02x", (long unsigned int)mms, pkt[MFC_PIDOF]);
                    //using 2DOF, native or ffb
                    //p2dof_set_poss (pkt[3], pkt[4], pkt[2]);
                    p2dof_set_poss (pkt);
                  }
                  //else
                    //rkdrop++;
                  //reset throttle
                  mms = 0;
                }
                else
                {
                  rkdrop++;
                  if (1||_odbg > 1)
                  {
                    printf ("\n#w@%04d:drop early pkt %d type 0x%02x", mms, rkdrop, pkt[MFC_PIDOF]);
                    int *_cpkt = pkt;
                    if (1||_odbg > 2)
                      printf ("\n#w.roll:% 6d (r: % 5d / s: % 5d) | pitch: % 6d \t(p: % 5d / s: % 5d / h: % 5d)",
                        _cpkt[MFC_PIROLL] + _cpkt[MFC_PISWAY], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY],
                        _cpkt[MFC_PIPITCH] + _cpkt[MFC_PISURGE] + _cpkt[MFC_PIHEAVE],
                        _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE]);
                  }
                }
              }
              fflush (stdout);
            }
            //
            break;
          }//MOTion
          //
          if (fds[i].fd == STDIN_FILENO)
          {
            //read input from shell
            if (read(STDIN_FILENO, buf, 50) > 2)
            {
              printf("\n#i:poll() stdin read 0x%2.2X", (((unsigned int) buf[0]) & 0xFF));
              stdin_process_message ((char *)buf);

            }
            else
            {
              if ((((unsigned int) buf[0]) & 0xFF) == 0x51) //'Q' = 0x51 is pressed
              {
                _done = 1;
              }
              /* !! disabled to avoid mem flood when running in background
              else
                if ((((unsigned int) buf[0]) & 0xFF) != 0x0A) //'CR' is pressed
                  printf ("\n#!w:missing parameter for option: %c", buf[0]);
                  */
            }
            break; //one event at a time
          }
        }
      } //for
    } //if poll
  } //while 1
  //
  printf ("\n#i:for %lusec, processes %d/%d pkts(evt/drop)", rtime / 1000, rkntr, rkdrop);
  printf ("\n#i:handled %dpkts, cleaning up..", rkntr + rkffb + rkdat + rkdrop);
  //
#ifdef _USE_FIFOS_
  mfc_fifoinp_del ();
#endif
  mfc_svr_del ();
  mfc_mot_del ();
  //pigpio demo platform
#ifdef _USE_PIGPIO_
  if (mfc_gpiop)
  {
    gpioServo (PGPIO_L, 0);
    gpioServo (PGPIO_R, 0);
    gpioTerminate ();
  }
#endif
  //
  printf ("\n#i:done!\n");
  return 0;
}
