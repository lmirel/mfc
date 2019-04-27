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
#include <getopt.h>
#include <termios.h>

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

//int usleep(long usec);

int xdof_home_init ();

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
char _odbg = 0;
char _omot = 0;   //don't process motion, just debug it
char _ostdin = 1;
char _ofifo = 0;
char _omotion = 1;
char _oml = 2;
int _omspeed = 5;
char bcast_addr[35] = {0};
char _done = 0;

#define MOTION_FREQ_MS      30

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
      printf ("\n#debug level: %d", _odbg);
      break;
    case 'm': //send motion data
      _omotion = atoi(buf + 1);
      printf ("\n#m:motion: %s", _omotion?"on":"off");
      if (_omotion)
      {
        xdof_home_init ();
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

static void usage()
{
  printf("#usage: sudo mfc-server\n");
}

typedef enum {
  oi_dummy = 0,
  oi_scn,
  oi_arduino,
} out_if;
char *out_ifn[] = {"dummy/debug", "SCN5/6", "arduino", ""};
static int out_ifx = oi_dummy;
static int odpid = 0, odvid = 0;

int env_init (int argc, char *argv[])
{
  /*
   * init params and stuff
   */
  //
  int c;

  struct option long_options[] = {
    /* These options don't set a flag. We distinguish them by their indices. */
    { "help",    no_argument,       0, 'h' },
    { "version", no_argument,       0, 'V' },
    { "latency", required_argument, 0, 'l' },
    { "speed",   required_argument, 0, 's' },
    { "debug",   required_argument, 0, 'd' },
    { "arduino", no_argument,       0, 'a' },
    { "scn",     no_argument,       0, 'n' },
    //{ "dummy",   no_argument,       0, 'y' },
    { "output-device",  required_argument, 0, 'o' },
    { 0, 0, 0, 0 }
  };

  while (1)
  {
    /* getopt_long stores the option index here. */
    int option_index = 0;

    c = getopt_long (argc, argv, "o:s:d:l:hmVan", long_options, &option_index);

    /* Detect the end of the options. */
    if (c == -1)
      break;

    switch (c) {

    case 'h':
      usage ();
      exit (0);
      break;

    case 'l': //motion response
      _oml = atoi (optarg);
      if (_oml < 0 || _oml > 100)
        _oml = 2;
      break;

    case 'd': //debug enabled
      _odbg++;
      break;

    case 'm': //motion enabled
      _omot++;
      break;

    case 'o': //output device vid:pid
      if (sscanf (optarg, "%04x:%04x", &odvid, &odpid) < 2)
      {
        printf ("invalid option: --output-device %s\n", optarg);
        usage ();
        return -1;
      }
      if (odvid == 0)
      {
        printf ("invalid option: --output-device %s\n", optarg);
        usage ();
        return -1;
      }
      if (odpid == 0)
      {
        printf ("invalid option: --output-device %s\n", optarg);
        usage ();
        return -1;
      }
      break;

    case 'a': //use Arduino output interface
      out_ifx = oi_arduino;
      break;

    case 'n': //use Arduino output interface
      out_ifx = oi_scn;
//#define FTDI_VID  0x0403
//#define FTDI_PID  0x6001
      if (odvid == 0 && odpid == 0)
      {
        odvid = 0x0403;
        odpid = 0x6001;
      }
      break;

    case 's': //motion speed: more means faster, 0 is fastest
      _omspeed = atoi (optarg);
      if (_omspeed < 1 || _omspeed > 7)
        _omspeed = 4;
      break;

    case 'V':
      printf("mfc-server %s\n", MFC_VERSION);
      exit(0);
      break;

    case '?':
      usage();
      exit(-1);
      break;

    default:
      printf("unrecognized option: %c\n", c);
      break;
    }
  }

  /*
   * summary configuration
   *
   */
  printf ("\n# ##");
  printf ("\n#MFC server %s", MFC_VERSION);
  printf ("\n#running configuration:");
  printf ("\n#   motion response %dms (-l%d) range [0..100]", _oml, _oml);
  printf ("\n#      motion speed %d (-s %d) range [1..7]", _omspeed, _omspeed);
  printf ("\n#   verbosity level %d (-d %d)", _odbg, _odbg);
  printf ("\n#  output interface %s", out_ifn[out_ifx]);
  printf ("\n#     output device %04x:%04x", odvid, odpid);
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
/*
* timestamping
*/
long rtime = 0;
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

#define DOF_MAX   6
/*
 * DOF assignments
 *
 * 1 DOF: dof_back
 * 2 DOF: dof_left, dof_right
 * 3 DOF: dof_left, dof_right, dof_back
 * 4 DOF: dof_left, dof_right, dof_aleft, dof_aright
 * 5 DOF: dof_left, dof_right, dof_aleft, dof_aright, dof_back
 * 6 DOF: dof_left, dof_right, dof_aleft, dof_aright, dof_back, dof_aback
 *
 * when enumerating the number of devices found,
 * we assign them based on the above.
 * in the same way, we submit motion positioning control based on this
 */
typedef enum {
  dof_left = 0,
  dof_right,
  dof_aleft,
  dof_aright,
  dof_back,
  dof_aback
} DOF_POS;
//one dof struct
typedef struct {
  int ctlfd;        //actuator control fd
  char *ctlport;    //actuator control port: /dev/serial/by-id/usb..
  int amin, amax;   //actual range
  int cmin, cmax;   //current range
  int pos;          //current position - computed
} p1dof;
static p1dof mfc_dof[DOF_MAX];
static int dof_count = 0; //number of DOFs detected

int xdof_init (char *lport, char *rport);
int xdof_set_home ();
//speed values
#define P2DOF_SPD_GEAR1   0x02EE
#define P2DOF_SPD_GEAR2   0x04E2
#define P2DOF_SPD_GEAR3   0x06D6
#define P2DOF_SPD_GEAR4   0x08CA
#define P2DOF_SPD_GEAR5   0x0ABE
#define P2DOF_SPD_GEAR6   0x0CB2
#define P2DOF_SPD_GEAR7   0x0EA6
int xdof_set_speed (int spd);
int xdof_set_pos (int *pdat);
int xdof_set_poss (int *pdat);
int xdof_release ();
int xdof_fill_fds (struct pollfd fds[]);

void cfmakeraw(struct termios *termios_p);

//
static libusb_context* ctx = NULL;
static libusb_device** devs = NULL;
static ssize_t cnt = 0;
static char scnports[DOF_MAX][255] = {"/dev/ttyUSB0", "/dev/ttyUSB1"};

static int scnAdof_set_pos (int *pdata);

static int scnAdof_find ()
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
      if (desc.idVendor == odvid && desc.idProduct == odpid)
      {
        ret = libusb_open (devs[dev_i], &handle);
        if (!ret)
        {
          ret = libusb_get_string_descriptor_ascii (handle, desc.iManufacturer, string, sizeof(string));
          if (!ret)
            continue;
          bp = snprintf (description, 255 - bp, "%s", string);
          ret = libusb_get_string_descriptor_ascii (handle, desc.iProduct, string, sizeof(string));
          if (!ret)
            continue;
          bp += snprintf (description + bp, 255 - bp, " %s", string);
          ret = libusb_get_string_descriptor_ascii (handle, desc.iSerialNumber, string, sizeof(string));
          if (!ret)
            continue;
          bp += snprintf (description + bp, 255 - bp, " %s", string);
          //
          char *ldp = description;
          while (*ldp++ > 0)
            if (*ldp == ' ')
              *ldp = '_';
          //
          //printf ("\n#i:%s", description);
          if (pk < DOF_MAX)
            snprintf (scnports[pk++], 254, "/dev/serial/by-id/usb-%s-if00-port0", description);
          //
          libusb_close (handle);
        }
      }
    }
  }//for
  //sort the ports such that we can assign them accordingly
  int i, j;
  if (pk > 0)
  {
    for (i = 0; i < pk; i++)
    {
      for (j = i + 1; j < pk; j++)
      {
        if(strcmp (scnports[i], scnports[j]) > 0)
        {
          strcpy ((char *)string, scnports[i]);
          strcpy (scnports[i], scnports[j]);
          strcpy (scnports[j], (char *)string);
        }
      }
    }
    for (i = 0; i < pk; i++)
      printf ("\n#i:DOF adapter #%d on port '%s'", i + 1, scnports[i]);
  }
  //
  //always get the new list of devices
  if (devs)
  {
    libusb_free_device_list(devs, 1);
    devs = NULL;
  }
  //
  return pk;
}

static int scnAdof_home_init ();

static int scnAdof_init (char *lport, char *rport)
{
  //char lscnp[] = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A504KGF1-if00-port0";
  //char rscnp[] = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A504KGHA-if00-port0";
  //
  int scnpk = scnAdof_find ();
  if (scnpk == 0)
  {
    printf ("\n#E:found %d DOF adapters", scnpk);
    return -1;
  }
  //
  if (scnpk <= DOF_MAX)
    dof_count = scnpk;
  else
  {
    printf ("\n#E:found %d DOF adapters - we'll handle only the first 6", scnpk);
    dof_count = DOF_MAX;
  }
  //set-up DOF control port
  int i;
  for (i = 0; i < DOF_MAX; i++)
  {
    mfc_dof[i].ctlport = NULL;
    mfc_dof[i].ctlfd = -1;
    mfc_dof[i].pos = 0;
  }
  //
  switch (dof_count)
  {
    case 1: //1 DOF: back
      mfc_dof[dof_back].ctlport = scnports[0];
      break;
    case 2: //2 DOF: left, right
      mfc_dof[dof_left].ctlport = scnports[0];
      mfc_dof[dof_right].ctlport = scnports[1];
      break;
    case 3: //3 DOF: left, right, back
      mfc_dof[dof_left].ctlport = scnports[0];
      mfc_dof[dof_right].ctlport = scnports[1];
      mfc_dof[dof_back].ctlport = scnports[2];
      break;
    case 4: //4 DOF: left, right, aleft, aright
      mfc_dof[dof_left].ctlport = scnports[0];
      mfc_dof[dof_right].ctlport = scnports[1];
      mfc_dof[dof_aleft].ctlport = scnports[2];
      mfc_dof[dof_aright].ctlport = scnports[3];
      break;
    case 5: //5 DOF: left, right, aleft, aright, back
      mfc_dof[dof_left].ctlport = scnports[0];
      mfc_dof[dof_right].ctlport = scnports[1];
      mfc_dof[dof_aleft].ctlport = scnports[2];
      mfc_dof[dof_aright].ctlport = scnports[3];
      mfc_dof[dof_back].ctlport = scnports[4];
      break;
    case 6: //6 DOF: left, right, aleft, aright, back, aback
      mfc_dof[dof_left].ctlport = scnports[0];
      mfc_dof[dof_right].ctlport = scnports[1];
      mfc_dof[dof_aleft].ctlport = scnports[2];
      mfc_dof[dof_aright].ctlport = scnports[3];
      mfc_dof[dof_back].ctlport = scnports[4];
      mfc_dof[dof_aback].ctlport = scnports[5];
      break;
    default:
      printf ("\n#E:cannot manage %d DOF adapters", dof_count);
      return -1;
  }
  //set-up FDs
  struct termios options;
  for (i = 0; i < DOF_MAX; i++)
  {
    if (mfc_dof[i].ctlport)
    {
      //open
      mfc_dof[i].ctlfd = open (mfc_dof[i].ctlport, O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (mfc_dof[i].ctlfd == -1)
      {
        printf ("\n#E:DOF adapter not found on %s", mfc_dof[i].ctlport);
        return -1;
      }
      //set options
      tcgetattr (mfc_dof[i].ctlfd, &options);
      //
      cfsetispeed (&options, B9600);
      cfsetospeed (&options, B9600);
      cfmakeraw (&options);
      //
      if (tcsetattr (mfc_dof[i].ctlfd, TCSANOW, &options) < 0)
      {
        printf ("\n#E:cannot set options for DOF adapter %s", mfc_dof[i].ctlport);
        return -1;
      }
      printf("\n#i:connected to DOF#%d '%s', on %d", i + 1, mfc_dof[i].ctlport, mfc_dof[i].ctlfd);
      //check adapter
      if (scn_get_status (mfc_dof[i].ctlfd) == 0)
      {
        printf ("\n#E:motion platform not responding, bailing out");
        return -1;
      }
    }
  }
  //
  scnAdof_home_init ();
  //
  return 1;
}

int xdof_init (char *lport, char *rport)
{
  switch (out_ifx)
  {
    case oi_arduino:
      break;
    case oi_scn:
      return scnAdof_init (NULL, NULL);
    default: //oi_dummy
      ;
  }
  return 1;
}

static int scnAdof_set_home ();
static int scnAdof_home_init ()
{
  //send on
  printf ("\n#i:homing platform..");
  fflush (stdout);
  //
  int i, j;
  char rsp[20];
  for (i = 0; i < DOF_MAX; i++)
  {
    if (mfc_dof[i].ctlfd > 0)
    {
      //turn DOF on
      scn_set_son (mfc_dof[i].ctlfd);
      //set speed to min
      scn_set_vel (mfc_dof[i].ctlfd, P2DOF_SPD_GEAR1, DEFA_ACMD);
      scn_get_response (mfc_dof[i].ctlfd, rsp);
      //send home
      scn_set_home (mfc_dof[i].ctlfd);
      //
      mfc_dof[i].amin = 0;
      mfc_dof[i].amax = 0x3FFFFFFF;
      printf("\n#i:homing DOF#%d '%s', on %d", i + 1, mfc_dof[i].ctlport, mfc_dof[i].ctlfd);
      fflush (stdout);
    }
  }
  //wait for position to be reported as 0
  int mk;
  for (j = 0; j < 10; j++)
  {
    mk = 0;
    for (i = 0; i < DOF_MAX; i++)
    {
      if (mfc_dof[i].ctlfd > 0)
      {
        mfc_dof[i].amin = scn_get_pos (mfc_dof[i].ctlfd);
        if (mfc_dof[i].amin != 0)
        {
          mk ++;
          sleep (1);
        }
      }
    }
    //we're done
    if (mk == 0)
      break;
  }
  if (mk)
  {
    printf ("\n#w:one of the DOFs didn't home properly!");
  }
  //look for range max
  //set both to negative max allowed by the driver
  printf ("\n#i:looking for max positions..");
  fflush (stdout);
  for (i = 0; i < DOF_MAX; i++)
  {
    if (mfc_dof[i].ctlfd > 0)
    {
      //set to -edge: 0xC0000000
      scn_set_pos (mfc_dof[i].ctlfd, 0xC0000000);
    }
  }
  //wait for position to not change
  int cmax;
  for (j = 0; j < 10; j++)
  {
    mk = 0;
    for (i = 0; i < DOF_MAX; i++)
    {
      if (mfc_dof[i].ctlfd > 0)
      {
        cmax = scn_get_pos (mfc_dof[i].ctlfd);
        if (mfc_dof[i].amax != cmax)
        {
          mk ++;
          mfc_dof[i].amax = cmax;
          sleep (1);
        }
      }
    }
    //we're done
    if (mk == 0)
      break;
  }
  if (mk)
  {
    printf ("\n#w:one of the DOFs didn't extend properly!");
  }
  //adjust work ranges by 3%
  int wdz;
  for (i = 0; i < DOF_MAX; i++)
  {
    if (mfc_dof[i].ctlfd > 0)
    {
      wdz = (mfc_dof[i].amax - mfc_dof[i].amin) * 3 / 100;
      mfc_dof[i].cmin = wdz;
      mfc_dof[i].cmax = mfc_dof[i].amax - wdz;
      printf("\n#i:DOF#%d range %d..%d, usable %d..%d", i + 1,
          mfc_dof[i].amin, mfc_dof[i].amax, mfc_dof[i].cmin, mfc_dof[i].cmax);
    }
  }
  //
  printf ("\n#i:wait for homing to finish..");
  fflush (stdout);
  //
  scnAdof_set_home ();
  sleep (3);
  printf (" homing done");
  fflush (stdout);
  //
  return 1;
}

int xdof_home_init ()
{
  switch (out_ifx)
  {
    case oi_arduino:
      break;
    case oi_scn:
      return scnAdof_home_init ();
    default: //oi_dummy
      ;
  }
  return 1;
}

int scnAdof_fill_fds(struct pollfd fds[])
{
  int dk = 0, i;
  for (i = 0; i < DOF_MAX; i++)
  {
    if (mfc_dof[i].ctlfd > 0)
    {
      fds[dk].fd = mfc_dof[i].ctlfd;
      fds[dk].events = POLLIN | POLLHUP;
      dk++;
    }
  }
  return dk;
}

int xdof_fill_fds(struct pollfd fds[])
{
  switch (out_ifx)
  {
    case oi_arduino:
      break;
    case oi_scn:
      return scnAdof_fill_fds (fds);
    default: //oi_dummy
      ;
  }
  return 0;
}

static int scnAdof_release ()
{
  int i;
  for (i = 0; i < DOF_MAX; i++)
  {
    if (mfc_dof[i].ctlfd > 0)
    {
      //turn DOF off
      scn_set_soff (mfc_dof[i].ctlfd);
      //close
      close (mfc_dof[i].ctlfd);
    }
  }
  //
  return 0;
}

int xdof_release ()
{
  switch (out_ifx)
  {
    case oi_arduino:
      break;
    case oi_scn:
      return scnAdof_release ();
    default: //oi_dummy
      ;
  }
  return 1;
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
static int scnAdof_set_home ()
{
  char rsp[20];
  int i; for (i = 0; i < DOF_MAX; i++)
  {
    if (mfc_dof[i].ctlfd > 0)
    {
      if (scn_get_status (mfc_dof[i].ctlfd) == 0)
      {
        printf ("\n#E:motion platform not responding, bailing out");
        _done = 1;
        return -1;
      }
      //set speed to min
      scn_set_vel (mfc_dof[i].ctlfd, P2DOF_SPD_GEAR1, DEFA_ACMD);
      scn_get_response (mfc_dof[i].ctlfd, rsp);
    }
  }
  //flush the speed response
  scnAdof_set_pos (_lpkt);
  //
  for (i = 0; i < DOF_MAX; i++)
  {
    if (mfc_dof[i].ctlfd > 0)
    {
      scn_get_response (mfc_dof[i].ctlfd, rsp);
    }
  }
  //
#if 0
  //
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
#endif
  //
  return 0;
}

int xdof_set_home ()
{
  switch (out_ifx)
  {
    case oi_arduino:
      break;
    case oi_scn:
      return scnAdof_set_home ();
    default: //oi_dummy
      ;
  }
  //
  return 1;
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
static int scnAdof_set_speed (int spd)
{
  char rsp[20];
  int i;
  for (i = 0; i < DOF_MAX; i++)
  {
    if (mfc_dof[i].ctlport)
    {
      scn_set_vel (mfc_dof[i].ctlfd, spd, DEFA_ACMD);
      scn_get_response (mfc_dof[i].ctlfd, rsp);
    }
  }
  return 0;
}

int xdof_set_speed (int spd)
{
  if (_odbg)
    printf ("\n#i:motion %d speed 0x%04X", spd, spd);
  //
  if (spd < P2DOF_SPD_GEAR1 || spd > P2DOF_SPD_GEAR7)
    spd = P2DOF_SPD_GEAR1;
  
  //
  switch (out_ifx)
  {
    case oi_arduino:
      break;
    case oi_scn:
      return scnAdof_set_speed (spd);
    default: //oi_dummy
      ;
  }
  //
  return 1;
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

int xdof_set_poss (int *pdata)
{
  xdof_set_pos (pdata);
  //
  return 0;
}

//2DOF motion positions
int dof2l = 0;
int dof2r = 0;
int dof1b = 0;
#define SMOOTHMOVE  50
static int scndof_move (int idx)
{
  if (mfc_dof[idx].ctlfd > 0)
    scn_set_pos (mfc_dof[idx].ctlfd, mfc_dof[idx].pos);
  return 1;
}

static int dmyAdof_set_pos (int *pdata)
{
  int *_cpkt = pdata;
  printf ("\n#i.roll:% 6d (r: % 5d / s: % 5d) | pitch: % 6d \t(p: % 5d / s: % 5d / h: % 5d)",
    _cpkt[MFC_PIROLL] + _cpkt[MFC_PISWAY], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY],
    _cpkt[MFC_PIPITCH] + _cpkt[MFC_PISURGE] + _cpkt[MFC_PIHEAVE],
    _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE]);
  return 1;
}

static int scnAdof_set_pos (int *pdata)
{
  //do some common computing here, to be reused
  if (dof_count > 1 && dof_count < 6)
  {
    dof2l = pdata[MFC_PIPITCH] + pdata[MFC_PISURGE] + pdata[MFC_PIHEAVE] + pdata[MFC_PIROLL] + pdata[MFC_PISWAY];
    dof2r = pdata[MFC_PIPITCH] + pdata[MFC_PISURGE] + pdata[MFC_PIHEAVE] - pdata[MFC_PIROLL] - pdata[MFC_PISWAY];
    if (_odbg)
      printf ("\n#i:in pos L%d, R%d", dof2l, dof2r);
    //left
    mfc_dof[dof_left].pos  = get_cmap (dof2l, -10000, 10000, mfc_dof[dof_left].cmax, mfc_dof[dof_left].cmin);
    //smooth curve: shave some positions to avoid the motor to move constantly
    mfc_dof[dof_left].pos -= mfc_dof[dof_left].pos % SMOOTHMOVE;
    //avoid an issue with lack of move for this position
    if (mfc_dof[dof_left].pos == -5000)
      mfc_dof[dof_left].pos++;
    //right
    mfc_dof[dof_right].pos  = get_cmap (dof2r, -10000, 10000, mfc_dof[dof_right].cmax, mfc_dof[dof_right].cmin);
    //smooth curve: shave some positions to avoid the motor to move constantly
    mfc_dof[dof_right].pos -= mfc_dof[dof_right].pos % SMOOTHMOVE;
    //avoid an issue with lack of move for this position
    if (mfc_dof[dof_right].pos == -5000)
      mfc_dof[dof_right].pos++;
    if (dof_count > 3)
    {
      //left
      mfc_dof[dof_aleft].pos  = get_cmap (dof2l, -10000, 10000, mfc_dof[dof_aleft].cmax, mfc_dof[dof_aleft].cmin);
      //smooth curve: shave some positions to avoid the motor to move constantly
      mfc_dof[dof_aleft].pos -= mfc_dof[dof_aleft].pos % SMOOTHMOVE;
      //avoid an issue with lack of move for this position
      if (mfc_dof[dof_aleft].pos == -5000)
        mfc_dof[dof_aleft].pos++;
      //right
      mfc_dof[dof_aright].pos  = get_cmap (dof2r, -10000, 10000, mfc_dof[dof_aright].cmax, mfc_dof[dof_aright].cmin);
      //smooth curve: shave some positions to avoid the motor to move constantly
      mfc_dof[dof_aright].pos -= mfc_dof[dof_aright].pos % SMOOTHMOVE;
      //avoid an issue with lack of move for this position
      if (mfc_dof[dof_aright].pos == -5000)
        mfc_dof[dof_aright].pos++;
      //
    }
  }
  //do we use the back / yaw dof?
  if (mfc_dof[dof_back].ctlfd > 0)
  {
    dof1b = pdata[MFC_PIYAW];
    if (_odbg)
      printf ("\n#i:back pos R%d", dof1b);
    dof1b = get_cmap (dof1b, -10000, 10000, mfc_dof[dof_back].cmax, mfc_dof[dof_back].cmin);
    //smooth curve: shave some positions to avoid the motor to move constantly
    mfc_dof[dof_back].pos -= mfc_dof[dof_back].pos % SMOOTHMOVE;
    //avoid an issue with lack of move for this position
    if (mfc_dof[dof_back].pos == -5000)
      mfc_dof[dof_back].pos++;
    //move it to position
    scn_set_pos (mfc_dof[dof_back].ctlfd, mfc_dof[dof_back].pos);
    //scn_get_response (mfc_dof[dof_back].ctlfd, rsp);
  }
  if (1)
  {
    //left
    scndof_move (dof_left);
    //right
    scndof_move (dof_right);
    //alt left
    scndof_move (dof_aleft);
    //alt right
    scndof_move (dof_aright);
    //back
    scndof_move (dof_back);
    //alt back
    scndof_move (dof_aback);
    if (_odbg)
      printf ("\n#i@%04d:DOF pos L%d, R%d, B %d", dtime_ms(),
          mfc_dof[dof_left].pos, mfc_dof[dof_right].pos, mfc_dof[dof_back].pos);
  }
  //
  if (_odbg)
  {
    printf ("\n#i@%04d:cmap pos L%d, R%d, B %d", dtime_ms(), dof2l, dof2r, dof1b);
    dmyAdof_set_pos (pdata);
  }
#ifdef _USE_PIGPIO_
    if (0)
      gpio_demo_spos (dof2l, dof2r);
#endif
  //
  return 1;
}

int xdof_set_pos (int *pdata)
{
  switch (out_ifx)
  {
    case oi_arduino:
      break;
    case oi_scn:
      return scnAdof_set_pos (pdata);
    default: //oi_dummy
      dmyAdof_set_pos (pdata);
  }
  return 1;
}

int scnAdof_onoff (int on)
{
  int i;
  if (on)
  {
    //on
    for (i = 0; i < DOF_MAX; i++)
      if (mfc_dof[i].ctlfd > 0)
        //turn DOF on
        scn_set_son (mfc_dof[i].ctlfd);
  }
  else
  {
    //off
    for (i = 0; i < DOF_MAX; i++)
      if (mfc_dof[i].ctlfd > 0)
        //turn DOF off
        scn_set_soff (mfc_dof[i].ctlfd);
  }
  return 1;
}

int xdof_start ()
{
  switch (out_ifx)
  {
    case oi_arduino:
      break;
    case oi_scn:
      return scnAdof_onoff (1);
    default: //oi_dummy
      ;
  }
  return 1;
}

int xdof_stop ()
{
  switch (out_ifx)
  {
    case oi_arduino:
      break;
    case oi_scn:
      return scnAdof_onoff (0);
    default: //oi_dummy
      ;
  }
  return 1;
}

//this takes long to execute
int scnAdof_get_status ()
{
  int i; for (i = 0; i < DOF_MAX; i++)
  {
    if (mfc_dof[i].ctlfd > 0)
    {
      if (scn_get_status (mfc_dof[i].ctlfd) == 0)
      {
        return 0;
      }
    }
  }
  return 1;
}

int xdof_get_status ()
{
  switch (out_ifx)
  {
    case oi_arduino:
      break;
    case oi_scn:
      return scnAdof_get_status ();
    default: //oi_dummy
      ;
  }
  return 1;
}
//
int scnAdof_consume (int fd)
{
  int i; for (i = 0; i < DOF_MAX; i++)
  {
    if (mfc_dof[i].ctlfd == fd)
    {
      char rsp[20];
      scn_get_response (fd, rsp);
      return 1;
    }
  }
  return 0;
}
//
int xdof_consume_fd (int fd)
{
  switch (out_ifx)
  {
    case oi_arduino:
      break;
    case oi_scn:
      return scnAdof_consume (fd);
    default: //oi_dummy
      ;
  }
  return 1;
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
        xdof_set_speed (P2DOF_SPD_GEAR2); //slower
        break;
      case 3:
        xdof_set_speed (P2DOF_SPD_GEAR3); //slow
        break;
      case 4:
        xdof_set_speed (P2DOF_SPD_GEAR4); //normal
        break;
      case 5:
        xdof_set_speed (P2DOF_SPD_GEAR5); //faster
        break;
      case 6:
        xdof_set_speed (P2DOF_SPD_GEAR6); //even faster
        break;
      case 7:
        xdof_set_speed (P2DOF_SPD_GEAR7); //fastest
        break;
      default:
        xdof_set_speed (P2DOF_SPD_GEAR1); //slowest
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
    mfc_motfd = xdof_init (NULL, NULL);
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
    //stop for now
    xdof_release ();
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
    xdof_set_home ();
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
  int rkntr = 1, rkffb = 0, rkdat = 0, rkdrop = 0;
  int i, mms = 0, cms;
  int _motion = 1;
  unsigned long mtime = 0;
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
    //add motion fd
    if (mfc_motfd > 0)
    {
      poll_i = xdof_fill_fds (fds);
    }
    //add server fd
    if (mfc_svrfd > 0)
    {
      fds[poll_i].fd = mfc_svrfd;
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
      sleep (1); //1sec
      printf (".");
      fflush (stdout);
      //check actuators status
      if (xdof_get_status () == 0)
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
      if (xdof_get_status () == 0)
      {
        printf ("\n#e:motion platform not responding, bailing out");
        _done = 1;
      }
      //turn off motors
      if (_motion)
      {
        _motion = 0;
        xdof_stop ();
        printf ("\n#i@04%lu:STOP motion", mtime);
        fflush (stdout);
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
      //turn on motors
      if (_motion == 0)
      {
        _motion = 1;
        xdof_start ();
        printf ("\n#i@04%lu:START motion", mtime);
      }
      //update ffb time
      for (i = 0; i < poll_i; ++i)
      {
        if(fds[i].revents & (POLLIN | POLLPRI | POLLOUT | POLLHUP))
        {
          if ((rkntr % 500) == 0)
            printf ("\n#i:poll %dfds:%s:%d@%lums/%lusec, %d/%d pkts(evt/drop)", (int)poll_i, fds[i].revents & POLLIN?"IN":"OUT", fds[i].fd,
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
          if (xdof_consume_fd (fds[i].fd))//MOTion
          {
            //did we get a disconnect?
            if (fds[i].revents & POLLHUP)
            {
              mfc_mot_del();
              break;
            }
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
                    xdof_set_poss (pkt);
                  }
                  //else
                    //rkdrop++;
                  //reset throttle
                  mms = 0;
                }
                else
                {
                  rkdrop++;
                  if (_odbg > 1)
                  {
                    printf ("\n#w@%04d:drop early pkt %d type 0x%02x", mms, rkdrop, pkt[MFC_PIDOF]);
                    int *_cpkt = pkt;
                    if (_odbg > 2)
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
