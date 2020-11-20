/*
*
* MFC extractor: translates USB HID data into movement
*
* good with Thrustmaster T300RS, Fanatec Elite, Logitech
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/time.h>
#include <getopt.h>

#include "extras.h"

#define UDP_MAX_PACKETSIZE  1024
#define UDP_PORT            20777

#define DEBUG 0
#define debug_print(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

int inet_aton (const char *cp, struct in_addr *inp);
//int usleep(long usec);
#if 0
//used by pkt processing logic
unsigned int dtime_ms ()
{
  static unsigned long lms = 0;
  unsigned long cms = get_millis ();
  unsigned long ms = cms - lms;
  lms = cms;
  return (unsigned int)ms;
}
#endif
#if 0
//used by motion computation logic
static unsigned int dtime2_ms ()
{
  static unsigned long lms2 = 0;
  unsigned long cms = get_millis ();
  unsigned long ms = cms - lms2;
  lms2 = cms;
  return (unsigned int)ms;
}
#endif

static int motion_reset (unsigned int mdt);
static int motion_compute (unsigned int mdt);
static int get_accel_pitch (int lpacc);

int motion_process_dummy (char *report, int retval, unsigned long mtime);
int motion_process_thrustmaster (char *report, int retval, unsigned long mtime);
int motion_process_fanatec (char *report, int retval, unsigned long mtime);
int motion_process_logitech (char *report, int retval, unsigned long mtime);

#include <sched.h>
#include <stdio.h>

int set_prio ()
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


char _done = 0;
void terminate (int sig)
{
  _done = 1;
}

/**
A. right axis composition
a. g-force - longitudinal/pitch - overwrite
b. g-force - lateral - add

B. right axis composition
a. g-force - longitudinal/pitch - overwrite
b. g-force - lateral (invert) - add
*/
int pf_shiftspd = 100; //[50 .. 100]
int _pitchprc = 65;
int _rollprc = 100;
int _yawprc = 100;
int _surgeprc = 100;
int _swayprc = 100;
int _heaveprc = 100;
char _odbg = 0;
char _mot = 0;        //don't process motion
//capture/debug
char _cap = 0;
#define CAP_FFB 0x02
#define CAP_WHL 0x04
#define CAP_ALL 0x06
char _fil = 0;        //capture in/out data to file
#define SWAY_CUTOFF   64
//act as a toy: use wheel input to control the platform
int _toyfd = -1;
unsigned char _nlat = 25; //25ms default network latency
#define LINE_MAX  255

//supported devices
// 0eb7:0e04 Fanatec
// 046d:c260 Logitech
// 044f:b66d Thrustmaster
typedef struct {
  char *pdev;
  char *pdv;
  int (*pf)(char *buf, int bl, unsigned long dt);
} proc_list;

proc_list _procs[] = {
    {"0000:0000", "!Dummy",       motion_process_dummy},
    {"0eb7:0e04", "Fanatec",      motion_process_fanatec},
    {"046d:c260", "Logitech",     motion_process_logitech},
    {"044f:b66d", "Thrustmaster", motion_process_thrustmaster},
};
int _p_idx = 0; //dummy
//
char *_pdev = NULL;    //device used to extract USB data
static int *_cpkt = NULL;
static int _cpktl = 0;

static void usage()
{
  printf ("usage: sudo mfcxtract\n\n");
  printf ("--fanatec output processing protocol for Fanatec wheels\n");
  printf ("--logitech output processing protocol for Logitech wheels\n");
  printf ("--thrustmaster output processing protocol for Thrustmaster wheels\n");
  printf ("--roll <prc>  ;max roll percentage\n");
  printf ("--pitch <prc> ;max pitch percentage\n");
  printf ("--yaw <prc>   ;max yaw percentage\n");
  printf ("--surge <prc> ;max surge percentage\n");
  printf ("--sway <prc>  ;max sway percentage\n");
  printf ("--heave <prc> ;max heave percentage\n");
  printf ("--latency <ms> ;motion update frequency: 5..99ms\n");
  printf ("\n");
}

int env_init (int argc, char *argv[])
{
  /*
   * init params and stuff
   */
  //
  int c;

  struct option long_options[] = {
    /* These options don't set a flag. We distinguish them by their indices. */
    { "help",     no_argument,       0, 'H' },
    { "version",  no_argument,       0, 'V' },
    { "debug",    required_argument, 0, 'D' },
    { "capture",  required_argument, 0, 'c' },
    //
    { "latency",  required_argument, 0, 'l' },
    //
    { "roll",     required_argument, 0, 'r' },
    { "pitch",    required_argument, 0, 'p' },
    { "yaw",      required_argument, 0, 'y' },
    { "surge",    required_argument, 0, 's' },
    { "sway",     required_argument, 0, 'w' },
    { "heave",    required_argument, 0, 'h' },
    //
    { "fanatec",      no_argument,       0, 'f' },
    { "logitech",     no_argument,       0, 'g' },
    { "thrustmaster", no_argument,       0, 't' },
    //{ "dummy",   no_argument,       0, 'y' },
    { 0, 0, 0, 0 }
  };

  while (1)
  {
    /* getopt_long stores the option index here. */
    int option_index = 0;

    c = getopt_long (argc, argv, "c:r:p:y:s:w:h:l:HVDfgt", long_options, &option_index);

    /* Detect the end of the options. */
    if (c == -1)
      break;

    switch (c) {

    case 'H':
      usage ();
      exit (0);
      break;

    //profiling params
    case 'r': //roll %
      _rollprc = atoi (optarg);
      //if (_rollprc < 0 || _rollprc > 100)
      //  _rollprc = 100;
      break;
    case 'p': //pitch %
      _pitchprc = atoi (optarg);
      //if (_pitchprc < 0 || _pitchprc > 100)
      //  _pitchprc = 100;
      break;
    case 'y': //yaw %
      _yawprc = atoi (optarg);
      //if (_yawprc < 0 || _yawprc > 100)
      //  _yawprc = 100;
      break;
    case 's': //surge %
      _surgeprc = atoi (optarg);
      //if (_surgeprc < 0 || _surgeprc > 100)
      //  _surgeprc = 100;
      break;
    case 'w': //sway %
      _swayprc = atoi (optarg);
      //if (_swayprc < 0 || _swayprc > 100)
      //  _swayprc = 100;
      break;
    case 'h': //heave %
      _heaveprc = atoi (optarg);
      //if (_heaveprc < 0 || _heaveprc > 100)
      //  _heaveprc = 100;
      break;
      //
    case 'f': //fanatec device
      _p_idx = 1;
      break;
    case 'g': //logitech device
      _p_idx = 2;
      break;
    case 't': //fanatec device
      _p_idx = 3;
      break;
      //
    case 'l': //network packet latency/frequency
    {
      unsigned char nlat = (unsigned char)atoi (optarg);
      if (nlat > 4 && nlat < 100)
        _nlat = nlat;
    }
      break;
    case 'D': //debug
      _odbg = atoi (optarg);
      break;
    case 'c': //data capture
      _cap = atoi (optarg);
      //if (_cap == 0)
      //  _cap = CAP_FFB | CAP_WHL;
      printf ("\n#i.cap %d", _cap);
      break;
    case 'V':
      printf("mfcxtract %s\n", MFC_VERSION);
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
#if 0
  //roll and pitch percentage
  for (int i = 1; i < argc; i++)
  {
    if (argv[i][0] == '-')
      switch (argv[i][1])
      {
        case 'd': //device
          _pdev = argv[i]+2;
          //set-up the motion processor function
          for (int j = 0; _procs[j].pdev != NULL; j++)
          {
            if (strncmp (_pdev, _procs[j].pdev, 9) == 0)
            {
              _p_idx = j;
              break;
            }
          }
          break;
        case 'm': //debug
          _mot++;
          break;
        case 't':
          //open /dev/hidraw0
          _toyfd = open ("/dev/hidraw0", O_RDONLY | O_NONBLOCK);
          if (_toyfd > 0)
            nfds = 2;
      }
  }
  #endif
  //configuration summary
  printf ("\n# ##");
  printf ("\n#MFC extractor client %s", MFC_VERSION);
  printf ("\n#running configuration:");
  //printf ("\n#      roll range %d [1..10]", _rollspd);
  //printf ("\n#    accel. speed %d (-a%d) range [1..10]", _accspd, _accspd);
  //printf ("\n#shifter feedback %d (-s%d) range [1..10]", _shiftspd, _shiftspd);
  //printf ("\n#vibrat. feedback %d (-v%d) range [1..10]", _vibfbk, _vibfbk);
  printf ("\n#  pitch feedback %3d%% (-p %d) (pedals)", _pitchprc, _pitchprc);
  printf ("\n#  surge feedback %3d%% (-s %d) (gears)", _surgeprc, _surgeprc);
  printf ("\n#  heave feedback %3d%% (-h %d) (ffb vibs)", _heaveprc, _heaveprc);
  printf ("\n#   roll feedback %3d%% (-r %d) (ffb roll)", _rollprc, _rollprc);
  printf ("\n#   sway feedback %3d%% (-w %d) (wheel)", _swayprc, _swayprc);
  printf ("\n#    yaw feedback %3d%% (-y %d) (wheel)", _yawprc, _yawprc);
  printf ("\n# verbosity level %4d (-D %d)", _odbg, _odbg);
  printf ("\n#    capture mode %s   (-c)", _cap?((_cap&CAP_ALL)==CAP_ALL?"all":(_cap&CAP_FFB?"ffb":"whl")):"off");
  printf ("\n#motion processor %s", _procs[_p_idx].pdv);
  printf ("\n#motion frequency %ums (-l%u) (ms)", (unsigned int)_nlat, (unsigned int)_nlat);
  printf ("\n# ##");
  //
  return 1;
}

int main (int argc, char **argv, char **envp)
{
  struct pollfd fdset[3];
  int nfds = 1;
  int timeout, rc;
  int lport = UDP_PORT;
  lport = 64402;
  //
  env_init (argc, argv);
  //
  int cs = mfc_bcast_prep ("127.0.0.1", 0);
  if (cs < 3)
  {
    printf ("\n#e:can't connect to MFC server on port %d", MFCSVR_PORT);
    exit(1);
  }
  printf ("\n#i:<%d:MFC server on port %d", cs, MFCSVR_PORT);
  if (_cpkt == NULL)
  {
    _cpkt = mfc_bcast_pktget ();
    _cpktl = mfc_bcast_pktlen ();
    //motion data packet
    _cpkt[MFC_PITYPE] = PKTT_DATA;
    _cpkt[MFC_PIDOF]  = PKTT_2DOF;
  }
  if (_cpkt == NULL)
  {
    printf ("\n#e:can't get MFC packet reference, aborting");
    exit(2);
  }
  //
#define POLL_TIMEOUT 5000
  timeout = POLL_TIMEOUT;

#if 0
  //uptime
  FILE* fp;
  double uptime, idle_time;
  /* Read the system uptime and accumulated idle time from /proc/uptime.  */
  fp = fopen ("/proc/uptime", "r");
#endif
  //sockets
  struct sockaddr_in si_me;
  int s, i;
  if((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf ("\n#e:can't initialize network, aborting");
    exit(1);
  }
#if 0
  int broadcast=1;
  setsockopt(s, SOL_SOCKET, SO_BROADCAST,
          &broadcast, sizeof (broadcast));
#endif
/* receiver */
  memset((char *) &si_me, 0, sizeof(si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(lport);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if(bind(s, (struct sockaddr*)&si_me, sizeof(si_me))==-1)
  {
    printf ("\n#e:can't set-up network, aborting");
    exit(1);
  }
#if 0
/* sender */
  memset((char *) &si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons(65001);
  pktk = 990;
  if (inet_aton("127.0.0.1", &si_other.sin_addr)==0) {
    fprintf(stderr, "inet_aton() failed\n");
    exit(1);
  }
#endif
  printf ("\n#i:>%d:listening on port %d", s, lport);
  //only send 3 PAUSEd packets
  (void) signal(SIGINT, terminate);
  (void) signal(SIGTERM, terminate);
  (void) signal(SIGHUP, terminate);
  (void) set_prio();
  //
  int ppkt = 1;
  char packetBuffer[UDP_MAX_PACKETSIZE];
  //
  int rlen = 0;
  int ndts = -1, ldts = dtime_ms ();
  int ppid = 0, vvid = 0;
  //motion measurements
  unsigned int ms_mot = (unsigned int)get_millis();
  unsigned int ms_now = 0;
  printf("\n#i:ready.");
  fflush (stdout);
  //
  while (!_done)
  {
    memset ((void*)fdset, 0, sizeof(fdset));

    fdset[0].fd = s;
    fdset[0].events = POLLIN;
    if (_toyfd > 0)
    {
      fdset[1].fd = _toyfd;
      fdset[1].events = POLLIN;
    }
    rc = poll (fdset, nfds, timeout);

    if (rc < 0) 
    {
      printf("\n#e:poll() failed!");
      _done = 1;
      break;
    }
    //timeout
    if (rc == 0)
    {
      printf(".");
      fflush (stdout);
      sleep (1);
    }
    //take the time
    ms_now = (unsigned int)get_millis();
    //printf ("\n#i:%04x.have data %d", ms_now, (rc == 0)?0:1);
    //toy
    if (fdset[1].revents & POLLIN)
    {
      if ((rlen = read (_toyfd, packetBuffer + 3, UDP_MAX_PACKETSIZE)) > 0)
      {
        //prep motion packet
        packetBuffer[0] = PKTT_IN;
        packetBuffer[1] = rlen + 1;
        packetBuffer[2] = 0x84;
        rlen += 3;
        //motion process
        motion_process_fanatec (packetBuffer, rlen, ldts);
        //
        printf ("\n#i.WHL@%04d: ", ldts);
        for (i = 0; i < rlen; i++)
          printf ("%02x ", (unsigned char)packetBuffer[i]);
      }
    }
    //network
    if (fdset[0].revents & POLLIN)
    {
      //recvfrom(RecvSocket,  packetBuffer , SMS_UDP_MAX_PACKETSIZE, 0, (SOCKADDR *) & Sender, &SenderAddrSize);
      //if ((rlen = recvfrom (s, (void *)&packetBuffer, SMS_UDP_MAX_PACKETSIZE, 0, (struct sockaddr*)&si_other, &slen))==-1)
      if ((rlen = recvfrom (s, (void *)&packetBuffer, UDP_MAX_PACKETSIZE, 0, NULL, NULL)) == -1)
      {
        printf("\n#w:recvfrom() failed");
      }
      else
      {
        ldts = dtime_ms ();
        //debug?!
        if (0)
          printf ("\n#i.PKT len %d", rlen);
        switch ((char) packetBuffer[0])
        {
          case (PKTT_DATA):  //USB pkt arrival timestamp data
          {
            ndts = packetBuffer[6] << 8 | packetBuffer[7];
            //printf ("\n#i.DBG@%04d: ", _dts);
            //select proper processing function based on VID+PID?!
            break;
          }
          //
          case (PKTT_CTRL): //wheel VID&PID
          {
            vvid = packetBuffer[2] << 8 | packetBuffer[3];
            ppid = packetBuffer[4] << 8 | packetBuffer[5];
            printf ("\n//i.WHL %04x:%04x ", vvid, ppid);
            //select proper processing function based on VID+PID?!
            break;
          }
          //
          case (PKTT_OUT):  //FFB data
          {
            if (_cap&CAP_FFB || _odbg)
            {
              if (ndts != -1)
              {
                ldts = ndts;
                ndts = -1;  //reset network ts as we need another
              }
              fprintf (stdout, "\n#i.FFB@%04d: ", ldts);
              for (i = 0; i < rlen; i++)
                fprintf (stdout, "%02x ", packetBuffer[i]);
            }
            break;
          }
          case (PKTT_IN): //WHL data
          {
            if (_cap&CAP_WHL || _odbg)
            {
              if (ndts != -1)
              {
                ldts = ndts;
                ndts = -1;  //reset network ts as we need another
              }
              fprintf (stdout, "\n#i.WHL@%04d: ", ldts);
              for (i = 0; i < rlen; i++)
                fprintf (stdout, "%02x ", packetBuffer[i]);
            }
            //
            break;
          }
          default:
          {
            if (1)
            {
              printf ("\n#w.UNK@%04d: ", ldts);
              for (i = 0; i < rlen; i++)
                printf ("0x%02x, ", packetBuffer[i]);
            }
          }
        }//switch
        //
        if (_procs[_p_idx].pf (packetBuffer, rlen, ldts))
          ppkt++;
        //
        if ((ppkt % 500) == 0)
          printf ("\n#i:received %dpkts", ppkt);
      }
    }
    //fflush (stdout);
    //send the motion packet
    if (rc && (ms_now - ms_mot >= _nlat))
    {
      if (0||_odbg)
        printf ("\n#i:%4x.motion@%ums vs %ums p%06dr%06d", ms_now, ms_now - ms_mot,
          _nlat, _cpkt[MFC_PIPITCH], _cpkt[MFC_PIROLL]);
      //inc motion packets
      //mpktt++;
      //send the packet
      _cpkt[MFC_PISPEED] = ms_now;  //hide the timestamp here, for future ref
      mfc_bcast_send ();
      //
      motion_reset (ms_now - ms_mot);
      //reset motion time to now
      ms_mot = ms_now;
    }
  }
  //
  printf("\n#i:cleaning up.. done.\n");
  //
  close (s);
  mfc_bcast_close ();
  if (_toyfd > 0)
    close (_toyfd);
  //
  return 0;
}

/**
compute platform position data

A. right axis composition
a. g-force - longitudinal(pitch) - overwrite
b. g-force - lateral(roll) - add

B. right axis composition
a. g-force - longitudinal(pitch) - overwrite
b. g-force - lateral(roll) * invert - add

        float llv = (-1)*fv[1] + fv[0]; //(-)pitch + roll
        float lrv = (-1)*fv[1] - fv[0]; //(-)pitch - roll
*/
int max_accel = 0;
int pl_roll, pl_pitch = 0;  //entire platform roll/pitch
//platform acceleration leveling
int pw_roll, pw_pitch = 0;  //wheel forces
int pf_roll, pf_pitch = 0, pf_sway = 0, pf_nudge = 0;  //ffb forces
int pv_roll = 0;  //vibration forces
int pv_pitch = 0;
int pw_heave = 0, pw_heave_dir = 1; //used for heave 'boating' effect
int lpacc, lpbrk = 0;       //local values for accel+brake for axis combo
char _wd = 'W'; //wheel data source
int vib_k = 0;
int _olat  = 10;    //network latency/frequency

static int motion_reset (unsigned int mdt)
{
  //reset forces and vibrations
  //pf_roll = pf_pitch = pv_roll = 0;
  pf_pitch = 0;
  //pf_sway = 0;
  pf_nudge = 0;
  pv_pitch = 0;
  pv_roll = 0;
  //
  return 0;
}

static int motion_compute (unsigned int mdt)
{
  static int sway_flag = 0;
#if 0
  //handle network throttle
  static unsigned long llts = 0;
  unsigned long clts = get_millis ();
  unsigned long mdt = clts - llts;
  static unsigned long mpktt = 0;
  static unsigned long mpktd = 0;
  //toy mode?
  if (_toyfd > 0)
    _wd = 'F';
  //process this now or drop?
  if (1)
  {
    static char lprio = 0;
    //don't process non FFB messages
    if (_wd == 'U')
    {
      if (_odbg > 1)
        printf ("\n#E:>>>%c%04lums drop", _wd, mdt);
      return 0;
    }
    if (_wd != 'F')
    {
      if (_odbg > 2)
        printf ("\n#w:%c%04lums drop", _wd, mdt);
      //if we don't get anything for a while, use this for motion
      if (mdt > 3 * _olat && mdt < 100)
      {
        //we force the next FFB message to go anyway
        lprio = 2;
        if (_odbg > 1)
          printf ("\n#w:%c%04lums force motion 1", _wd, mdt);
#if 0
#define BOAT_SPD  20
#define BOAT_AMP  200
      if (pw_pitch > 32000)
      {
        //generate heave 'boating' efect
        //full accel or brake
        if (pw_heave_dir)
          pw_heave += BOAT_SPD;
        else
          pw_heave -= BOAT_SPD;
        //boating boundaries
        if (pw_heave > BOAT_AMP)
          pw_heave_dir = 0; //go the other way
        if (pw_heave < -BOAT_AMP)
          pw_heave_dir = 1; //go the other way
        printf ("\n#w:boating at %d", pw_heave);
        _cpkt[MFC_PIHEAVE] += pw_heave;
      }
#endif
      }
      else
        return 0;
    }
    //inc total packets
    mpktt++;
    //
    if (mdt >= _olat || lprio || llts == 0)
    {
      if (lprio)
        lprio --;
      llts = clts;
      //
      //printf ("\n#w:%c%04lums force motion 2", _wd, mdt);
    }
    else
    {
      mpktd++;
      //drop pkt and return
      if (_odbg > 1)
        printf ("\n#w:%c%04lums %ludrop(%lu) (%05d, %05d)", _wd, mdt, mpktd, mpktt, pl_pitch, pl_roll);
      return 0;
    }
  }
#endif
  /*
  * wheel roll should cap at -16k..+16k
  *
  * when we have vibrations, that needs to feel like 50%, 
  * then wheel 25%(20%) and ffb 25%(30%) otherwise, use 50% wheel and 50% ffb
  *
  *
PITCH	Pitch is the tilt of the car forwards or backwards in [°]             //UP AND DOWN HILLS	
SURGE	Surge means the acceleration of the car in longitudinal direction [g] //MAX= ACCERATION AND MIN = BRAKE	
HEAVE	Heave means the acceleration up and down [g]					                //BUMPS ON ROAD	
//
ROLL	Roll is how much the car is dipped to the left or right in [°]        //LEFT AND RIGHT, BANKS ,BUMPS WORKS OPPOSITE TO SWAY	
SWAY	Sway means the acceleration of the car in lateral direction [g]				//LEFT AND RIGHT BODY ROLL	
//
YAW	Yaw is the heading of the car (north, east, south, west) in [°]         //car heading
  *
    //motion
    _cpkt[MFC_PIPITCH] = -pl_pitch;  //pitch
    _cpkt[MFC_PIROLL]  = -pl_roll;   //roll
    _cpkt[MFC_PIYAW]   = 0;          //yaw
    _cpkt[MFC_PISURGE] = 0;          //surge
    _cpkt[MFC_PISWAY]  = 0;          //sway
    _cpkt[MFC_PIHEAVE] = 0;          //heave
  */
  //motion
  #if 1
  //pedals-based pitch
  _cpkt[MFC_PIPITCH] = -get_cmap (pw_pitch, -MFC_WHL_MAX, MFC_WHL_MAX, -10000, 10000);
  //gear changes
  _cpkt[MFC_PISURGE] = -get_cmap (pf_pitch, -100, 100, -10000, 10000);
  //steering vibrations / 'boating': pv_pitch
  _cpkt[MFC_PIHEAVE] = -get_cmap (pv_pitch, -128, 128, -10000, 10000);
  //ffb roll
  if (_toyfd > 0)
  {
    _cpkt[MFC_PIROLL]  = get_cmap (pw_roll, -MFC_HWHL_MAX, MFC_HWHL_MAX, -MFC_HPOS_MAX, MFC_HPOS_MAX);
    //printf ("\n#d.pw roll %d", pw_roll);
  }
  else
    _cpkt[MFC_PIROLL]  = get_cmap (pf_roll, -128, 128, -MFC_HPOS_MAX, MFC_HPOS_MAX);
  //add steering roll to platform roll?
  //printf ("\n#p.roll1 %d", _cpkt[MFC_PIROLL]);
  _cpkt[MFC_PIROLL] += get_cmap (pw_roll, -MFC_HWHL_MAX/2, MFC_HWHL_MAX/2, -MFC_HPOS_MAX, MFC_HPOS_MAX);
  //printf ("\n#p.roll2 %d > %d", pw_roll, _cpkt[MFC_PIROLL]);
#if 1
  //if we have strong wheel move, also move the platform
  if (abs (pf_roll > SWAY_CUTOFF))
  {
    //traction loss also occurs here?!
    //TODO: maybe check next pf_roll to be close to -1/1
    pf_nudge = ((pf_roll > 0) ? (pf_roll - SWAY_CUTOFF) : (pf_roll + SWAY_CUTOFF));
    sway_flag = pf_nudge;
  }
#endif
  //sway logic / traction loss
  if (pf_sway)
  {
    //reduce sway unless we need to grow it
    //maybe drop more than just 1 unit?
    //keep in mind that this goes from -127..0..127
    if (pf_sway > 0)
    {
      pf_sway -= 1;
      if (pf_sway < 0)
        pf_sway = 0;
    }
    else
    {
      pf_sway += 1;
      if (pf_sway > 0)
        pf_sway = 0;
    }
    printf ("\n#d.sway1 move %d", pf_sway);
  }
  //else if (pf_nudge)
  //{
  //  pf_sway = get_cmap (pw_roll, -16400, 16400, -64, 64) + pf_nudge;
  //}
  //
  if (sway_flag && abs (pf_roll) <= 1) //traction loss when wheel pos command follows wheel pos
  {
    sway_flag = 0;
    pf_sway = get_cmap (pw_roll, -MFC_HWHL_MAX, MFC_HWHL_MAX, -64, 64);
    //traction loss: take 1
    //add traction loss relevant to wheel turn (pw_roll) up to half effect (64)
    //TODO: make it progressively grow
    //or
    if (0)
    {
      if (pf_sway > 0)
      {
        //increased drift
        pf_sway += 20;
        if (pf_sway > 96)
          pf_sway = 96;
      }
      else if (pf_sway < 0)
      {
        //increased drift
        pf_sway -= 20;
        if (pf_sway < -96)
          pf_sway = -96;
      }
      else
      {
        //start based on wheel position
        pf_sway = get_cmap (pw_roll, -MFC_HWHL_MAX, MFC_HWHL_MAX, -64, 64);
      }
    }
  }
  if (pf_sway) printf ("\n#d.sway2 move %d", pf_sway);
  //sway move: traction loss and strong ffb
  _cpkt[MFC_PISWAY]  = get_cmap (pf_sway + pf_nudge, -128, 128, -MFC_HPOS_MAX, MFC_HPOS_MAX);
  //steering direction
  _cpkt[MFC_PIYAW]   = get_cmap (pw_roll, -MFC_HWHL_MAX, MFC_HWHL_MAX, -MFC_POS_MAX, MFC_POS_MAX);
  //
  if (0||_odbg > 1)
    printf ("\n#i.raw roll:% 6d (r: % 5d / s: % 5d) | pitch: % 6d \t(p: % 5d / s: % 5d / h: % 5d)",
      _cpkt[MFC_PIROLL] + _cpkt[MFC_PISWAY], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY],
      _cpkt[MFC_PIPITCH] + _cpkt[MFC_PISURGE] + _cpkt[MFC_PIHEAVE],
      _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE]);
  //
  _cpkt[MFC_PIPITCH] = _cpkt[MFC_PIPITCH] * _pitchprc / 100;
  _cpkt[MFC_PIROLL]  = _cpkt[MFC_PIROLL]  * _rollprc  / 100;
  _cpkt[MFC_PIYAW]   = _cpkt[MFC_PIYAW]   * _yawprc   / 100;
  _cpkt[MFC_PISURGE] = _cpkt[MFC_PISURGE] * _surgeprc / 100;
  _cpkt[MFC_PISWAY]  = _cpkt[MFC_PISWAY]  * _swayprc  / 100;
  _cpkt[MFC_PIHEAVE] = _cpkt[MFC_PIHEAVE] * _heaveprc / 100;
  if (_odbg > 1)
    printf ("\n#i.adj roll:% 6d (r: % 5d / s: % 5d) | pitch: % 6d \t(p: % 5d / s: % 5d / h: % 5d)",
      _cpkt[MFC_PIROLL] + _cpkt[MFC_PISWAY], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY],
      _cpkt[MFC_PIPITCH] + _cpkt[MFC_PISURGE] + _cpkt[MFC_PIHEAVE],
      _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE]);
  //normalize movement direction according to FFB data
  //in this manner, if the FFB force moves to + or -
  //the other forces follow it, to comply with FFB movement direction
  if (1)
  {
#if 0
    //roll: MFC_PIROLL + MFC_PISWAY
    if (_cpkt[MFC_PIROLL] > 0)
    {
      if (_cpkt[MFC_PISWAY] < 0)
        _cpkt[MFC_PISWAY] *= -1;
    }
    else
    {
      if (_cpkt[MFC_PISWAY] > 0)
        _cpkt[MFC_PISWAY] *= -1;
    }
#endif
    //pitch: MFC_PISURGE + MFC_PIPITCH + MFC_PIHEAVE
    if (_cpkt[MFC_PIPITCH] > 0)
    {
      if (_cpkt[MFC_PISURGE] < 0)
        _cpkt[MFC_PISURGE] *= -1;
      if (_cpkt[MFC_PIHEAVE] < 0)
        _cpkt[MFC_PIHEAVE] *= -1;
    }
    else
    {
      if (_cpkt[MFC_PISURGE] > 0)
        _cpkt[MFC_PISURGE] *= -1;
      if (_cpkt[MFC_PIHEAVE] > 0)
        _cpkt[MFC_PIHEAVE] *= -1;
    }
    //yaw: MFC_PIYAW + MFC_PISWAY
    if (_cpkt[MFC_PIYAW] > 0)
    {
      if (_cpkt[MFC_PISWAY] < 0)
        _cpkt[MFC_PISWAY] *= -1;
    }
    else
    {
      if (_cpkt[MFC_PISWAY] > 0)
        _cpkt[MFC_PISWAY] *= -1;
    }
    //
    if (_odbg > 1)
      printf ("\n#i.nor roll:% 6d (r: % 5d / s: % 5d) | pitch: % 6d \t(p: % 5d / s: % 5d / h: % 5d)",
        _cpkt[MFC_PIROLL] + _cpkt[MFC_PISWAY], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY],
        _cpkt[MFC_PIPITCH] + _cpkt[MFC_PISURGE] + _cpkt[MFC_PIHEAVE],
        _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE]);
  }
  #else
  _cpkt[MFC_PIPITCH] = -get_cmap (pf_pitch, 0, 100, -500, 500);
  _cpkt[MFC_PISURGE] = -get_cmap (pw_pitch, -32800, 32800, -3500, 3500);
  _cpkt[MFC_PIHEAVE] = -get_cmap (pv_pitch, -125, 125, -1000, 1000);
  //
  _cpkt[MFC_PIROLL]  = get_cmap (pf_roll, -pf_roll_max, pf_roll_max, -3500, 3500);
  _cpkt[MFC_PISWAY]  = get_cmap (pw_roll, -16400, 16400, -1500, 1500);
  //
  _cpkt[MFC_PIYAW]   = 0;//get_cmap (pw_roll, -16400, 16400, -10000, 10000);
  #endif
  //speed
  _cpkt[MFC_PISPEED] = 0;//speed
  //
  if (_odbg)
    printf ("\n#i@%04d.roll:% 6d (r: % 5d / s: % 5d) | pitch: % 6d \t(p: % 5d / s: % 5d / h: % 5d)",
      mdt, _cpkt[MFC_PIROLL] + _cpkt[MFC_PISWAY], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY],
      _cpkt[MFC_PIPITCH] + _cpkt[MFC_PISURGE] + _cpkt[MFC_PIHEAVE],
      _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE]);
  if (0 || _odbg > 1)
    printf ("\n#i:pitch \t%d \t%d \t%d \troll \t%d \t%d \t%d",
      _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY], _cpkt[MFC_PIYAW]);
  //
  return 1;
}

int motion_process_dummy (char *report, int rlen, unsigned long dtime)
{
  //update delta time
  if (dtime == -1)
    dtime = 4;
  _wd = 'U';
  //ffb wheel pos: ffb roll
  //
  switch ((char) report[0])
  {
    case (PKTT_DEBUG):  //debug data
      return 0;
    //
    case (PKTT_OUT):   //FFB data
      /*
       * FFB data processing
       */
    {
      //unhandled
      _wd = 'U';
      if (_odbg & 0x01)
      {
        printf ("\n#w!FFB@%04lu: ", dtime);
        for (int i = 0; i < rlen; i++)
          printf ("%02x ", report[i]);
      }
      return 0;
    }
      break;
    case (PKTT_IN):   //WHL data
      /*
       * WHL data processing
       */
    {
      //unhandled
      _wd = 'W';
      if (_odbg & 0x02)
      {
        printf ("\n#w!WHL@%04lu: ", dtime);
        for (int i = 0; i < rlen; i++)
          printf ("%02x ", report[i]);
      }
      return 0;
    }
      break;
  }
  return 1;
}

/*
 * Fanatec CSL - compat mode > Logitech G29 Driving Force Racing Wheel (PS4)
 * --vid 046d --pid c260
 * Bus 001 Device 031: ID 046d:c260 Logitech, Inc.
 *-------------
 *
 *
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 c7 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a8 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 8a 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 7b 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 6c 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 4d 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 3e 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 20 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 11 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 f2 80 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 f8 12 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 11 08 83 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 21 0c 05 00 05 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 11 08 83 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 f8 12 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 21 0c 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 f8 12 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
 */
//#w!FFB@0004: 07 21 03 30 f8 12 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
unsigned char ffb_logi_lights1[] =
//<type>,<len>,  03    30    01    08 <FF..80..01>
 { 0x07, 0x21, 0x03, 0x30, 0xf8, 0x12, 0xFF };

//#w!FFB@0004: 07 21 03 30 11 08 b3 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
unsigned char ffb_logi_move1[] =
//<type>,<len>,  03    30    01    08 <FF..80..01>
 { 0x07, 0x21, 0x03, 0x30, 0x11, 0x08, 0xFF, 0x80 };

//#w!FFB@0004: 07 21 03 30 21 0c 04 00 04 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
unsigned char ffb_logi_move2[] =
//<type>,<len>,  03    30    01    08 <FF..80..01>
 { 0x07, 0x21, 0x03, 0x30, 0x21, 0x0c, 0xFF, 0x00, 0xFF, 0x00, 0x01 };

//#w!FFB@0004: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
unsigned char ffb_logi_kpl1[] =
//<type>,<len>,  03    30    01    08 <FF..80..01>
 { 0x07, 0x21, 0x03, 0x30, 0x13, 0x00, 0x00 };

int motion_process_logitech (char *report, int rlen, unsigned long dtime)
{
  //update delta time
  if (dtime == -1)
    dtime = 4;
  //default pkt: unhandled
  _wd = 'U';
  //ffb wheel pos: ffb roll
  static int lwpos = 0, cwpos = 0;
  //
  switch ((char) report[0])
  {
    case (PKTT_DEBUG):  //debug data
      return 0;
    //
    case (PKTT_OUT):   //FFB data
      /*
       * FFB data processing
       */
      {
        _wd = 'F';
        if (memcmp ((const void *) (report + 2), (const void *) (ffb_logi_lights1 + 2), 4) == 0)
        {
          //TODO: process movement for lights
          _wd = 'F';
          if (0)
          {
            printf ("\n#i.FFB@%04lu: ", dtime);
            for (int i = 0; i < rlen; i++)
              printf ("%02x ", report[i]);
          }
        }
        else
        if (memcmp ((const void *) (report + 2), (const void *) (ffb_logi_move1 + 2), 4) == 0)
        {
          //TODO: process movement for wheel position
          _wd = 'F';
          if (0)
          {
            printf ("\n#i.FFB@%04lu: ", dtime);
            for (int i = 0; i < rlen; i++)
              printf ("%02x ", report[i]);
          }
          /* wheel ffb position
          * 128..255 | 0..127
          */
          cwpos = report[6]; //normal_ffb2 (((int) report[6]), 0x080);
          pf_roll = (cwpos > lwpos)? (cwpos - lwpos):-(lwpos - cwpos);
          lwpos = cwpos;
          if (_odbg > 2)
            printf ("\n#d.WHLPOS1 ffb roll %d / %d", pf_roll, ((int) report[6]));
        }
        else
        if (memcmp ((const void *) (report + 2), (const void *) (ffb_logi_move2 + 2), 4) == 0)
        {
          //TODO: process movement - unknown
          _wd = 'F';
          if (0)
          {
            printf ("\n#i.FFB@%04lu: ", dtime);
            for (int i = 0; i < rlen; i++)
              printf ("%02x ", report[i]);
          }
        }
        else
        if (memcmp ((const void *) (report + 2), (const void *) (ffb_logi_kpl1 + 2), 4) == 0)
        {
          //nothing to do here: keepalive
          _wd = 'U';
        }
        else
        if (_odbg)
        {
          _wd = 'U';
          printf ("\n#w!FFB@%04lu: ", dtime);
          for (int i = 0; i < rlen; i++)
            printf ("%02x ", report[i]);
        }
        return 0;
      }
      break;
    case (PKTT_IN):   //WHL data
      /*
       * WHL data processing
       */
      {
        //unhandled
        _wd = 'U';
        if (_odbg)
        {
          printf ("\n#i.WHL@%04lu: ", dtime);
          for (int i = 0; i < rlen; i++)
            printf ("%02x ", report[i]);
        }
        //get accel/brake
        /*
        * wheel: 1..32768<L R>-32768..-1
        *   acc: up: -1..-32768 > 32768..1 :dn
        * brake: up: -1..-32768 > 32768..1 :dn
        */
        pw_roll = normal_axis (get_short (report, 46), 0x0ffff); //from wheel turn
        lpacc   = normal_accel (get_short (report, 48), 0x0ffff);
        lpbrk   = normal_brake (get_short (report, 50), 0x0ffff);
        //lhbrk   = normal_brake (get_short (report, 54), 0x0ffff); //handbrake
        //
        if (_odbg > 2)
          printf ("\n#RAW whl %d acc %d brk %d", pw_roll, lpacc, lpbrk);
        //
        if (lpbrk < -5)
          pw_pitch = lpbrk;  //cut off accel if brake pressed
        else
        {
          //
          pw_pitch = get_accel_pitch (lpacc); //it should stay nose-up while accelerating
        }
        //pw_pitch = lpbrk; //accel-brake
        //
        //gear shifting: only when accelerating or braking
        if (lpbrk || lpacc)
        {
        //gear down
          if (report[9] & 0x01)
          {
            //printf ("\ngear down\n");
            max_accel = 0;
            if (lpbrk)
              //ffb_vib_gear[5] = pf_shiftspd;
              pf_pitch = pf_shiftspd;
            else
              pf_pitch = -pf_shiftspd;
              //ffb_vib_gear[5] = pf_shiftspd;
            //return motion_process (ffb_vib_gear, 0, mtime);
            //not reacheable
            /*
            mdelta += MOTION_FREQ_MS; //force event motion
            pv_pitch = 0x01;
            c_accel = lpacc;
            max_accel = lpacc;
            //pw_pitch = c_accel;
            */
          }
          //gear up
          if (report[9] & 0x02)
          {
            //printf ("\ngear up\n");
            max_accel = 0;
            if (lpbrk)
              //ffb_vib_gear[5] = pf_shiftspd;
              pf_pitch = pf_shiftspd;
            else
              //ffb_vib_gear[5] = -pf_shiftspd;
              pf_pitch = -pf_shiftspd;
            //return motion_process (ffb_vib_gear, 0, mtime);
          }
        }//gear shifting
      }//IN PKT: WHL
      break;
  }//switch pkt type
  //
  if (motion_compute (dtime))
  {
    //reset forces and vibrations - done after network send
    //pf_roll = pf_pitch = pv_roll = 0;
    //pf_pitch = 0;
    //pv_pitch = 0;
    //pv_roll = 0;
  }
  //
  return (_wd != 'U' ? 1 : 0);
}

/*
 * Fanatec CSL - native mode
 * --vid 0eb7 --pid 0e04
 * Bus 001 Device 029: ID 0eb7:0e04 Endor AG
 */

/*
 * GTSport
 *
# center
dat: 07 (41) 03 30 01 08 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
# right
dat: 07 (41) 03 30 01 08 50 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
# left
dat: 07 (41) 03 30 01 08 a0 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
 *
 *
 *
#FFB
#i:FFB pkt: 07 41 03 30 f8 09 01 a0 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f8 09 01 06 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 03 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 03 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 03 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 23 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 23 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 23 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 33 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00                                                          #w!ffb pkt: 07 41 03 30 33 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 33 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 33 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 43 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 43 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 43 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 53 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 53 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 53 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 63 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 63 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 63 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 73 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 73 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 73 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 83 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 83 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 83 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 93 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 93 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 93 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 a3 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 a3 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 a3 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 b3 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 b3 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 b3 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 c3 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 c3 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 c3 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 d3 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 d3 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 d3 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 e3 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 e3 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00                                                          #w!ffb pkt: 07 41 03 30 e3 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 e3 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f3 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f3 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f3 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f5 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f8 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f8 09 07 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f8 09 08 01 ff 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f8 09 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 11 0b 80 80 ff 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 11 0b 80 80 ff 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#--
#i:FFB pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 11 0b 80 80 ff 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 11 0b 80 80 ff 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 11 0b 80 80 ff 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 11 0b 80 80 ff 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#-- ---
#
# Dirt Rally
#w!ffb pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 24 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 1c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 0a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 1d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 17 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 17 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 38 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 28 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 41 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 41 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 0d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 2d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 2d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 32 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
//-- ---
//Dirt 4
//-- start
#w!ffb pkt: 07 09 03 30 f8 81 ff ff 00 00 00
#w!ffb pkt: 07 09 03 30 f5 00 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 03 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 03 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 03 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 13 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 13 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 13 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 23 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 23 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 23 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 33 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 33 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 33 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 43 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 43 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 43 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 53 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 53 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 53 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 63 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 63 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 63 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 73 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 73 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 73 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 83 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 83 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 83 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 93 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 93 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 93 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 a3 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 a3 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 a3 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 b3 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 b3 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 b3 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 c3 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 c3 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 c3 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 d3 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 d3 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 d3 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 e3 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 e3 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 e3 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 f3 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 f3 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 f3 0c 00 00 00 00 00 ..
//-- wheel strength?!
#w!ffb pkt: 07 09 03 30 f8 13 01 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 03 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 07 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 0f 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 1f 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 3f 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 14 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 7f 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 7f 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 ff 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 ff 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 ff 01 00 00 00
#w!ffb pkt: 07 09 03 30 f8 14 00 00 00 00 00
 *
 */
//wheel position message
//also present as 9 bytes message
//#w!ffb pkt: 07 09 03 30 01 08 9c 00 00 00 00
unsigned char ffb_whlpos1[] =
//<type>,<len>,  03    30    01    08 <FF..80..01>
 { 0x07, 0x41, 0x03, 0x30, 0x01, 0x08, 0xFF };

 //also present as 9 bytes message
 //#w!ffb pkt: 07 09 03 30 f8 09 08 01 f8 00 00
unsigned char ffb_lights[] =
 //<type>,<len>,  03    30    f8    09    08, Y:1  Y2:Y3:R1:R2 R3:B1:B2:B3
  { 0x07, 0x41, 0x03, 0x30, 0xf8, 0x09, 0x08, 0x01, 0xFF };

//no impact on wheel led, just the rev leds
unsigned char ffb_lights2[] =
 //<type>,<len>,  03    30    f8    13  B2:B1:R3:R2 R1:Y3:Y2:Y1  B3    D1    D2    D3
  { 0x07, 0x09, 0x03, 0x30, 0xf8, 0x13, 0xff, 0x01, 0x00, 0x00, 0x00 };

/*
 * 9byte msg: the 3 digits
 *
//                                 "    5   "
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 6d 00
//                                 "    4   "
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 66 00
//                                 "    3   "
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 4f 00
//                                 "    2   "
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 5b 00
//                                 "    1   "
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 06 00
//                                 "    N   "
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 37 00
//                                 "    R   "
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 31 00
//                                 "0   0  0"
#w!ffb pkt: 07 09 03 30 f8 09 01 02 3f 3f 3f
//--
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 37 00
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 06 00
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 5b 00
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 4f 00
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 66 00
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 6d 00
//-- ---
#w!FFB@-001: 07 09 03 30 f8 13 ff 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 ff 01 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 ff 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 7f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 3f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 1f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 0f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 07 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 03 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 01 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 00 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 01 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 03 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 07 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 0f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 1f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 3f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 7f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 ff 00 00 00 00
#i:received 5000pkts
#w!FFB@-001: 07 09 03 30 f8 13 ff 01 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 ff 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 7f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 3f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 1f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 3f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 7f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 ff 00 00 00 00
#i:received 5500pkts
#w!FFB@-001: 07 09 03 30 f8 13 ff 01 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 ff 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 7f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 3f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 7f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 3f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 1f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 0f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 00 00 00 00 00
 *
 *
 */
unsigned char ffb_digits[] =
 //<type>,<len>,  03    30    f8    09    01    02    D1    D2    D3
  { 0x07, 0x09, 0x03, 0x30, 0xf8, 0x09, 0x01, 0x02, 0xFF, 0xFF, 0xFF };

//wheel position message: weak force
unsigned char ffb_whlpos2[] =
 //<type>,<len>,  03    05    01    00    00    00  <FF..80..01>
  { 0x07, 0x21, 0x03, 0x05, 0x01, 0x00, 0x00, 0x00, 0xFF };

/*
 * Dirt4
 * 9byte msg: vibrations?
 *
#w!ffb pkt: 07 09 03 30 11 0c 06 00 06 00 ff
#w!ffb pkt: 07 09 03 30 11 0c 07 00 07 00 ff
#w!ffb pkt: 07 09 03 30 11 0c 06 00 06 00 ff
#w!ffb pkt: 07 09 03 30 11 0c 05 00 05 00 ff
 *
 * */
unsigned char ffb_whlvib1[] =
 //<type>,<len>,  03    30    11    0c    FF    00    FF    00    ff>
  { 0x07, 0x09, 0x03, 0x30, 0x11, 0x0c, 0xFF, 0x00, 0xFF, 0x00, 0xff };
static int get_accel_pitch (int lpacc)
{
  static int c_accel = 0;
  static const int pf_accspd = 350; //[50 .. 500]
  //deal with acceleration platform leveling
  if (max_accel == lpacc) //constant acceleration
  {
    //fprintf (stderr, "\n#i:1 lpacc %d \t max_acc %d \t c_acc %d", lpacc, max_accel, c_accel);
    c_accel -= 20;//this needs to drop / ramp down regardless of the steering movement
    //
    if (c_accel < 0)
      c_accel = 0;
    //fprintf (stderr, "\n#i:2 lpacc %d \t max_acc %d \t c_acc %d", lpacc, max_accel, c_accel);
  }
  else if (max_accel < lpacc)//accelerate || max_accel > lpacc/*deccelerate*/)
  {
    c_accel += pf_accspd;//this needs to grow / ramp up regardless of the steering movement
    if (c_accel > lpacc)
      c_accel = lpacc;
    max_accel = c_accel;
  }
  else
  {
    max_accel = lpacc;
    c_accel = lpacc;
    //fprintf (stderr, "\n#i:3 lpacc %d \t max_acc %d \t c_acc %d", lpacc, max_accel, c_accel);
  }
  //
  return c_accel + pf_accspd; //it should stay nose-up while accelerating
}

int motion_process_fanatec (char *report, int rlen, unsigned long dtime)
{
  //update delta time
  if (dtime == -1)
    dtime = 4;
  _wd = 'U';
  //ffb wheel pos: ffb roll
  static int lwpos = 127, cwpos = 127, revs = 0; //wheel center pos default
  static char max_revs = 0;
  //max revs
  if (revs == 9)
  {
    max_revs = ~max_revs;
    if (max_revs)
      pf_pitch = pf_shiftspd/2;
    else
      pf_pitch = 0;
  }
  //
  switch ((char) report[0])
  {
    case (PKTT_DEBUG):  //debug data
      return 0;
    //
    case (PKTT_OUT):   //FFB data
      /*
       * FFB data processing
       */
    {
      //
      _wd = 'F';
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_digits + 2), 5) == 0)
      {
        //nothing to do here: digits
        _wd = 'U';
      }
      else if (memcmp ((const void *) (report + 2), (const void *) (ffb_lights2 + 2), 4) == 0)
      {
        _wd = 'U';
        //nothing to do here: lights
        revs = count_ones(report[6]) + (report[7] & 0x01);
        //printf ("\n#d.rev2 level %02x %02x/%d", report[6], report[7], revs);
        if (0||_odbg)
        {
          printf ("\n#w!FFB@%04lu: ", dtime);
          for (int i = 0; i < rlen; i++)
            printf ("%02x ", report[i]);
        }
        //pw_pitch = get_cmap ((long)revs, 0, 255, 0, MFC_HWHL_MAX);
      }
      else if (memcmp ((const void *) (report + 2), (const void *) (ffb_lights + 2), 5) == 0)
      {
        _wd = 'W'; //overwrite to wheel to avoid movement during standstill
        //lights: process max revs led
        //#d.rev level 00/0
        //#d.rev level 80/1
        //#d.rev level c0/3
        //#d.rev level e0/7
        //#d.rev level f0/15
        //#d.rev level f8/31
        //#d.rev level fc/63
        //#d.rev level fe/127
        //#d.rev level ff/255
#if 0
        revs = (int)count_ones((unsigned char)report[8]) + (report[7] & 0x01);
        if (revs)
        {
          printf ("\n#d.rev1 level %02x %02x/%d", report[8], report[7], revs);
          if (1||_odbg)
          {
            printf ("\n#w!FFB@%04lu: ", dtime);
            for (int i = 0; i < rlen; i++)
              printf ("%02x ", report[i]);
          }
        }
        //use revs level to pitch the platform: raise or dive
        pw_pitch = get_cmap ((long)revs, 0, 255, 0, MFC_WHL_MAX);
        //
#endif
      }
      else if (memcmp ((const void *) (report + 2), (const void *) (ffb_whlvib1 + 2), 4) == 0)
      {
        //TODO:vibrations?
        _wd = 'U';
        //printf ("\n#d.whlvib %0x", report[6]);
      }
      else
        /*
        * GT Sport
        */
       if (memcmp ((const void *) (report + 2), (const void *) (ffb_whlpos1 + 2), 4) == 0)
       {
         /* small values when wheel is turned right
          * large values when wheel is turned left
          * - both varies little for small vibs, very much for big vibs
          */
         /*
         * process vibrations
         */
         /* wheel ffb position
         * 128..255 | 0..127
         */
         cwpos = report[6];
         pf_roll = normal_ffb2 (cwpos, lwpos);  //ffb wheel delta
         //pf_roll = normal_ffb3 (cwpos); //ffb wheel pos
         //pf_roll = (cwpos > lwpos)? (cwpos - lwpos):-(lwpos - cwpos);
         if (0||_odbg > 2)
           printf ("\n#d.WHLPOS1 ffb roll %d / %d / %d", pf_roll, ((int) report[6]),
               normal_ffb2 (cwpos, lwpos));
         //
         lwpos = cwpos;
         //
       }
      /*
      * Dirt Rally
      */
     else
       if (memcmp ((const void *) (report), (const void *) (ffb_whlpos2), 8) == 0)
     {
       /* small values when wheel is turned right
        * large values when wheel is turned left
        * - both varies little for small vibs, very much for big vibs
        */
       /*
       * process vibrations
       */
       /* wheel ffb position
       * 128..255 | 0..127
       */
       cwpos = report[8];
       pf_roll = normal_ffb2 (cwpos, lwpos);
       //pf_roll = (cwpos > lwpos)? (cwpos - lwpos):-(lwpos - cwpos);
       lwpos = cwpos;
       if (0||_odbg > 2)
         printf ("\n#d.WHLPOS2 ffb roll %d / %d", pf_roll, ((int) report[8]));
       //
     }
#if 0
      else
        /*
         * Dirt4
#W:***pkt type: 0xee: ee 40 48 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 3a 05 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 11 ff ff 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 35 60 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 31 02 08 40 10 27 00 ff ff 50 00 60 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
         * WRC5/6, Dirt Rally
         * inspect wheel vibration data
FFB@002ms: 0002 01 ee 40 3d 4a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FFB@002ms: 0002 01 ee 40 3d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FFB@002ms: 0002 01 ee 40 3d 12 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FFB@002ms: 0002 01 ee 40 3d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FFB@002ms: 0002 01 ee 40 3d 0a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FFB@002ms: 0002 01 ee 40 3d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
         */
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_wrc5 + 2), 1) == 0)
      {
        /* small values when wheel is turned right
         * large values when wheel is turned left
         * - both varies little for small vibs, very much for big vibs
         */
        pf_roll = - normal_ffb (((int) report[3]), 0x0ff);
        if (_odbg > 2)
          printf ("\n#i:wrc/drally ffb roll %d / %d", pf_roll, ((int) report[3]));
      }
#endif
      else
      {
        //unhandled
        _wd = 'U';
        if (_odbg)
        {
          printf ("\n#w!FFB@%04lu: ", dtime);
          for (int i = 0; i < rlen; i++)
            printf ("%02x ", report[i]);
        }
        return 0;
      }
    } // FFB data
      break;
      /*
       *
       */
    case (PKTT_IN):  //wheel input data
      /*
       * wheel data processing
       *
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 15 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 f8 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 da 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 bd 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a0 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 82 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 65 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 48 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 2b 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0d 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 f0 7d ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0d 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 2b 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 48 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 65 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 82 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a0 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 bd 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 da 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 f8 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 15 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 32 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 50 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00       *
//DIY handbrake test
#i.WHL@0000: 06 40 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 40 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 ff ff ff ff ff ff 00 00 00 00 00 00 00 00 00 00 00 00 00
       */
    {
      //
      _wd = 'W';      //
      //get accel/brake
      /*
      * wheel: 1..32768<L R>-32768..-1
      *   acc: up: -1..-32768 > 32768..1 :dn
      * brake: up: -1..-32768 > 32768..1 :dn
      */
      pw_roll = normal_axis (get_short (report, 46), 0x0ffff); //from wheel turn
      lpacc   = normal_accel (get_short (report, 48), 0x0ffff);
      lpbrk   = normal_brake (get_short (report, 50), 0x0ffff);
      //lhbrk   = normal_brake (get_short (report, 54), 0x0ffff); //handbrake
      //
      if (_odbg > 2)
        printf ("\n#RAW whl %d acc %d brk %d", pw_roll, lpacc, lpbrk);
      //
      if (lpbrk < -5)
        pw_pitch = lpbrk;  //cut off accel if brake pressed
      else
      {
        //add revs to nose pitch
        pw_pitch = get_cmap ((long)get_accel_pitch(lpacc), 0, MFC_WHL_MAX, 0, MFC_HWHL_MAX);
        pw_pitch += get_cmap ((long)revs, 0, 9, 0, MFC_HWHL_MAX);
      }
      //pw_pitch = lpbrk; //accel-brake
      //
      #if 1 //disable shifting motion, leave it for the game to deal with it
      //gear shifting: only when accelerating or braking
      if (lpbrk || lpacc)
      {
      //gear down
        if (report[9] & 0x01)
        {
          //printf ("\ngear down\n");
          max_accel = 0;
          if (lpbrk)
            //ffb_vib_gear[5] = pf_shiftspd;
            pf_pitch = pf_shiftspd;
          else
            pf_pitch = -pf_shiftspd;
            //ffb_vib_gear[5] = pf_shiftspd;
          //return motion_process (ffb_vib_gear, 0, mtime);
          //not reacheable
          /*
          mdelta += MOTION_FREQ_MS; //force event motion
          pv_pitch = 0x01;
          c_accel = lpacc;
          max_accel = lpacc;
          //pw_pitch = c_accel;
          */
        }
        //gear up
        if (report[9] & 0x02)
        {
          //printf ("\ngear up\n");
          max_accel = 0;
          if (lpbrk)
            //ffb_vib_gear[5] = pf_shiftspd;
            pf_pitch = pf_shiftspd;
          else
            //ffb_vib_gear[5] = -pf_shiftspd;
            pf_pitch = -pf_shiftspd;
          //return motion_process (ffb_vib_gear, 0, mtime);
        }
      }
      #endif
    } // wheel data
      break;
    default:
      return 0;
  }
  //emphasis on vibration roll
#if 0
  if (vib_k == 1)
    pv_roll = 150;
  else if (vib_k == 2)
    pv_roll = -150;
#endif
  //
  if (motion_compute (dtime))
  {
#if 0
#define BOAT_SPD  20
#define BOAT_AMP  200
    //generate heave 'boating' efect
    if (lpbrk < -32000 || lpacc > 32000)
    {
      //full accel or brake
      if (pw_heave_dir)
        pw_heave += BOAT_SPD * (lpbrk < -32000?3:1);
      else
        pw_heave -= BOAT_SPD * (lpbrk < -32000?3:1);
    }
    //boating boundaries
    if (pw_heave > BOAT_AMP)
      pw_heave_dir = 0; //go the other way
    if (pw_heave < -BOAT_AMP)
      pw_heave_dir = 1; //go the other way
    printf ("\n#i:boating at %d", pw_heave);
#endif
    //vibration switch
    if (vib_k == 1)
    {
      //printf ("\n#v:vib left");
      vib_k = 2;
    }
    else if (vib_k == 2)
    {
      //printf ("\n#v:vib right");
      vib_k = 1;
    }
    //add heave 'boating' efect
    //pv_pitch = pw_heave;
  }
  else if (1 && _wd == 'F')
  {
    printf ("\n#d:dropped by motion_compute");
  }
  //
  return 1;
}

/*
 * Thrustmaster T300RS
 *
* PS menu
#W:***pkt type: 0xee: ee 40 48 05 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 48 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#*
*/
unsigned char ffb_var1[] =
{ 0xee, 0x40, 0x39, 0x02, 0x41, 0x01, 0x00, 0x00 };

/*
* 35 spring/damper update effect
*
* 35 - update effect
* 30 - id
* 00 - unk
* ff - max intensity level: right
* ff - max intensity level: left
* ff - intensity level: signed, min=-103, max=103, center=0
* ff - direction: right = 0x00, left = 0xff
* 00 - unk
* 00 - unk
* 4c - unk
* 4c - unk
*
* ee 40 35 30 00 4c 4c 00 00 00 00 4c 4c 00 00
*/
unsigned char ffb_whlsprg1[] =
{ 0xee, 0x40, 0x35, 0x30, 0x00, 0x4c, 0x4c, 0x00, 0x00, 0x00, 0x00, 0x4c, 0x4c, 0x00, 0x00 };

/*
* 35 spring update effect
*
* 35 - update effect
* 50 - id
* 00 - unk
* ff - max intensity level: right
* ff - max intensity level: left
* ff - intensity level: signed, min=-103, max=103, center=0
* ff - direction: right = 0x00, left = 0xff
* 00 - unk
* 00 - unk
*
* ee 40 35 50 00 2b 2b 00 00 64 00 64 64 00 00
*/
unsigned char ffb_whlsprg2[] =
{ 0xee, 0x40, 0x35, 0x50, 0x00, 0x2b, 0x2b, 0x00, 0x00, 0x64, 0x00, 0x64, 0x64, 0x00, 0x00 };

unsigned char ffb_vib1[] =
//{ 0xee, 0x40, 0x34, 0x90, 0x00, 0xXX(intensity) };
//FFB@00072ms: ee 40 34 90 00 02 00 00 19 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
{ 0xee, 0x40, 0x34, 0x90, 0x00, 0xFF, 0x00 };

unsigned char ffb_vib1_off[] =
//{ 0xee, 0x40, 0x34, 0x90, 0x00, 0xXX(intensity) };
{ 0xee, 0x40, 0x34, 0x90, 0x00, 0x00, 0x00 };

//weak vibrations?!
//FFB@00027ms: ee 40 34 70 00 12 00 00 64 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
unsigned char ffb_vib2[] =
//{ 0xee, 0x40, 0x34, 0x70, 0x00, 0xXX(intensity) };
{ 0xee, 0x40, 0x34, 0x70, 0x00, 0xFF, 0x00 };

unsigned char ffb_vib2_off[] =
//{ 0xee, 0x40, 0x34, 0x70, 0x00, 0xXX(intensity) };
{ 0xee, 0x40, 0x34, 0x70, 0x00, 0x00, 0x00 };

unsigned char ffb_vib_on[] =
{ 0xee, 0x40, 0x39, 0x04, 0x01, 0x01, 0x00 };

unsigned char ffb_vib_off[] =
{ 0xee, 0x40, 0x39, 0x04, 0x00, 0x01, 0x00 };

unsigned char ffb_wrc5[] =
//{ 0xee, 0x40, 0x33, 0x40, 0x00, 0x15 };
{ 0xee, 0x40, 0x3d, 0xFF, 0x00, 0x00, 0x00 };

unsigned char ffb_wrc5_on[] =
{ 0xee, 0x40, 0x39, 0x04, 0x41, 0x00, 0x00 };

unsigned char ffb_wrc5_off[] =
{ 0xee, 0x40, 0x39, 0x04, 0x00, 0x00, 0x00 };

unsigned char ffb_pcars[] =
//{ 0xee, 0x40, 0x33, 0x40, 0x00, 0x15 };
//{ 0xee, 0x40, 0x33, 0x50, 0x00, 0x15 };
//{ 0xee, 0x40, 0x33, 0x30, 0x00, 0xFF };
{ 0xee, 0x40, 0x33, 0x10, 0x00, 0xFF };

unsigned char ffb_pcars_on[] =
{ 0xee, 0x40, 0x35, 0x30, 0x00, 0x00, 0x00 };

unsigned char ffb_pcars_off[] =
{ 0xee, 0x40, 0x35, 0x30, 0x00, 0x64, 0x64 };

/*
GT Sport update
#pkt@000ms/067 bytes
##07 41 03 3e 00 01 02 b9 fd 00 49 53 14 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#pkt@000ms/067 bytes
##07 41 03 3e 00 01 02 08 fe 00 49 46 14 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#pkt@000ms/067 bytes
##07 41 03 3e 00 01 02 1c fe 00 49 58 24 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#pkt@000ms/067 bytes
##07 41 03 3e 00 01 02 f8 fd 00 49 47 7c 00 c8 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#pkt@000ms/067 bytes
##07 41 03 3e 00 01 02 c0 fd 00 49 43 12 00 0b 00 07 00 0b 00 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
*/
unsigned char ffb_gtsp2[] =
//{ 0xee, 0x40, 0x33, 0x40, 0x00, 0x15 };
//{ 0xee, 0x40, 0x33, 0x50, 0x00, 0x15 };
{ 0xee, 0x40, 0x3e, 0x00, 0x01, 0x02, 0xFF, 0xFF, 0x00, 0x49 };

unsigned char ffb_dcvr[] =
//{ 0xee, 0x40, 0x33, 0x40, 0x00, 0x15 };
//{ 0xee, 0x40, 0x33, 0x50, 0x00, 0x15 };
{ 0xee, 0x40, 0x35, 0x20, 0x00, 0x07, 0x07, 0x00 };

unsigned char ffb_dcvr_on[] =
{ 0xee, 0x40, 0x35, 0x30, 0x00, 0x00, 0x00 };

unsigned char ffb_dcvr_off[] =
{ 0xee, 0x40, 0x35, 0x30, 0x00, 0x64, 0x64 };
unsigned char ffb_byte_knt = 3;
unsigned char ffb_byte_pos = 7;

int motion_process_thrustmaster (char *report, int rlen, unsigned long mtime)
{
  //update delta time
  _wd = 'U';
  //
  switch ((char) report[0])
  {
    case (PKTT_DEBUG):  //debug data
    {
      unsigned int dts = report[6] << 8 | report[7];
      printf ("\n#i:DBG pkt @%04d: ", dts);
      for (int i = 0; i < rlen; i++)
        printf ("%02x ", report[i]);
    }
    break;
    //
    case (PKTT_OUT):   //FFB data
      /*
       * FFB data processing
       */
    {
      if (1)
      {
        printf ("\n#i:FFB pkt: ");
        for (int i = 0; i < rlen; i++)
          printf ("%02x ", report[i]);
      }
      //advance the report data pointer
      report++;
      //
      _wd = 'F';
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_var1 + 2), 5) == 0)
      {
        //nothing to do here, standard FFB keepalive 1
        _wd = 'U';
      }
      else
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_whlsprg1 + 2), 3) == 0)
      {
        //nothing to do here, spring/damper data on 
        _wd = 'U';
      }
      else
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_whlsprg2 + 2), 3) == 0)
      {
        //nothing to do here, standard FFB keepalive 1
        _wd = 'U';
      }
      else
      //check vibrations START
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_vib_on + 2), 5) == 0)
      {
        //pv_pitch = ((int) report[5])>127?((int) report[5])-255:((int) report[5]);
        //printf ("\n#i:vibration START");
        vib_k = 1;
      }
      else
      //check vibrations
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_vib_off + 2), 5) == 0)
      {
        //pv_roll = -((int) report[5]);
        //pv_pitch = ((int) report[5])>127?((int) report[5])-255:((int) report[5]);
        //printf ("\n#i:vibration STOP");
        vib_k = 0;
        _wd = 'U';
      }
      else
      //check vibrations
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_vib1 + 2), 3) == 0)
      {
        //pv_roll = -((int) report[5]);
        pv_pitch = ((int) report[5])>127?((int) report[5])-255:((int) report[5]);
        //printf ("\n#i:vibration1 intensity %d", pv_pitch);
      }
      else
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_vib2 + 2), 3) == 0)
      {
        //pv_roll = -((int) report[5]);
        //pv_pitch = ((int) report[5]);
        pv_pitch = ((int) report[5])>127?((int) report[5])-255:((int) report[5]);
        //printf ("\n#i:vibration2 intensity %d", pv_pitch);
      }
      else
      /*
       * GT Sport
#W:***pkt type: 0xee: ee 40 48 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 03 1e 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 11 54 d5 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 3b 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 03 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 11 ff ff 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 32 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 31 00 01 00 00 00 00 00 00 10 00 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 39 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 35 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 31 01 07 40 ff ff 00 ff ff 30 00 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 39 01 01 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 35 60 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 31 02 07 00 00 00 00 00 00 50 00 60 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 39 02 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 35 70 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 35 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 31 03 08 00 00 00 00 00 00 70 00 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 39 03 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
       * PCARS/PCARS2/GT Sport
       * inspect wheel vibration data
       * sample: [len: 0x40] 33 40 00 XX
       * -or-
       * sample: [len: 0x40] 33 50 00 XX
       * -or- GT Sport
       * sample: [len: 0x40] 33 10 00 XX
       * -or- PCARS2
       * sample: [len: 0x40] 33 30 00 XX
       */
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_pcars + 2), 3) == 0)
      {
        /* small values when wheel is turned right
         * large values when wheel is turned left
         * - both varies little for small vibs, very much for big vibs
         */
        /*
        * process vibrations
        */
        /* wheel ffb position
        * 128..255 | 0..127
        */
        pf_roll = normal_ffb (((int) report[5]), 0x0ff);
        if (_odbg > 2)
          printf ("\n# PCARS ffb roll %d / %d", pf_roll, ((int) report[5]));
        //
      }
      else
/*
* GT Sport update
#W:*pkt type: 0xee: ee 40 3e 00 01 02 4d 03 00 49 46 14 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 a2 04 00 49 58 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 c3 03 00 49 47 7c 00 c8 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 ee fd 00 49 43 12 00 0a 00 08 00 08 00 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 dd fd 00 49 53 14 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 c4 01 00 49 46 14 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 f0 ff 00 49 58 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 e9 f8 00 49 47 7c 00 c8 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 93 fa 00 49 43 12 00 0a 00 08 00 08 00 05 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 6f 00 00 49 53 14 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 89 00 00 49 46 14 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
* inspect wheel vibration data
* sample: [len: 0x40]     3e 00 01 02 YY XX 00
*/
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_gtsp2 + 2), 4) == 0)
      {
        /* small values when wheel is turned right
         * large values when wheel is turned left
         * - both varies little for small vibs, very much for big vibs
         */
        /*
        * process vibrations
        */
        /* wheel ffb position
        * 128..255 | 0..127
        */
        pf_roll = normal_ffb (((int) report[7]), 0x0ff);
        if (_odbg > 2)
          printf ("\n# PCARS ffb roll %d / %d", pf_roll, ((int) report[5]));
        //
      }
      else
        /*
         * Dirt4
#W:***pkt type: 0xee: ee 40 48 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 3a 05 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 11 ff ff 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 35 60 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 31 02 08 40 10 27 00 ff ff 50 00 60 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
         * WRC5/6, Dirt Rally
         * inspect wheel vibration data
FFB@002ms: 0002 01 ee 40 3d 4a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FFB@002ms: 0002 01 ee 40 3d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FFB@002ms: 0002 01 ee 40 3d 12 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FFB@002ms: 0002 01 ee 40 3d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FFB@002ms: 0002 01 ee 40 3d 0a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FFB@002ms: 0002 01 ee 40 3d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
         */
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_wrc5 + 2), 1) == 0)
      {
        /* small values when wheel is turned right
         * large values when wheel is turned left
         * - both varies little for small vibs, very much for big vibs
         */
        pf_roll = - normal_ffb (((int) report[3]), 0x0ff);
        if (_odbg > 2)
          printf ("\n#i:wrc/drally ffb roll %d / %d", pf_roll, ((int) report[3]));
      }
      else
        /*
         * DriveClub VR
         * inspect wheel vibration data
FFB@00004ms: ee 40 35 20 00 06 06 f2 ff 00 00 14 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0
FFB@00012ms: ee 40 35 20 00 06 06 f1 ff 00 00 14 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0
FFB@00003ms: ee 40 35 20 00 06 06 f0 ff 00 00 14 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0
FFB@00003ms: ee 40 35 20 00 06 06 ef ff 00 00 14 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0
FFB@00003ms: ee 40 35 20 00 06 06 f0 ff 00 00 14 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0
FFB@00003ms: ee 40 35 20 00 06 06 f1 ff 00 00 14 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0
         */
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_dcvr + 2), 2) == 0)
      {
        pf_roll = - normal_ffb (((int) report[7]), 0x0ff);
        if (_odbg > 2)
          printf ("\n#i:wrc/drally ffb roll %d / %d", pf_roll, ((int) report[7]));
      }
      else
      {
        //unhandled
        _wd = 'U';
        printf ("\n#w!ffb pkt: ");
        for (int i = 0; i < rlen; i++)
          printf ("%02x ", report[i - 1]);
      }
    } // FFB data
      break;
      /*
       *
       */
    case (PKTT_IN):  //wheel input data
      /*
       * wheel data processing
       *
#pkt@000ms/067 bytes
##06 41 84 01 80 80 80 80 08 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0f 80 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#pkt@000ms/067 bytes
##06 41 84 01 80 80 80 80 08 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0e 80 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#pkt@000ms/067 bytes
##06 41 84 01 80 80 80 80 08 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0c 80 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#pkt@000ms/067 bytes
##06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0c 80 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
       *
#!WHL@out 64B:
 01 80 80 80 80 08 00 00
 00 00 00 00 00 00 00 00
 00 00 00 00 00 00 00 00
 00 00 00 00 00 00 00 00
 00 00 00 00 00 00 00 00
 00 00 00 a6 7f ff ff ff
 ff ff ff 00 ff ff 00 00
 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a3 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a4 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a6 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a8 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ab 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 af 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 b2 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 b6 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 bb 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 c0 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 c3 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 c8 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
       *
       */
    {
      //
      if (1)
      {
        printf ("\n#i:WHL pkt: ");
        for (int i = 0; i < rlen; i++)
          printf ("%02x ", report[i]);
      }
      //advance the report data pointer
      report++;
      _wd = 'W';      //
      //get accel/brake
      /*
      * wheel: 1..32768<L R>-32768..-1
      *   acc: up: -1..-32768 > 32768..1 :dn
      * brake: up: -1..-32768 > 32768..1 :dn
      */
      pw_roll   = normal_axis (get_short (report, 45), 0x0ffff); //from wheel turn
      //pw_roll = get_map (pw_roll);
      //pw_pitch  = normal_axis (get_short (report, 47), 0x0ffff); //from accel
      //pw_pitch = normal_axis (get_short (report, 49), 0x0ffff); //from brake
      lpbrk = normal_brake (get_short (report, 49), 0x0ffff);
      lpacc = normal_accel (get_short (report, 47), 0x0ffff);
      if (lpbrk < -5)
        pw_pitch = lpbrk;  //cut off accel if brake pressed
      else
      {
        //
        pw_pitch = get_accel_pitch(lpacc); //it should stay nose-up while accelerating
      }
      //pw_pitch = lpbrk; //accel-brake
      //
      if (_odbg > 2)
        printf ("\n#RAW whl %d acc %d brk %d", get_short (report, 45), get_short (report, 47), get_short (report, 49));
      //
      //gear shifting: only when accelerating or braking
      if (lpbrk || lpacc)
      {
      //gear down
        if (report[8] & 0x01)
        {
          //printf ("\ngear down\n");
          max_accel = 0;
          if (lpbrk)
            //ffb_vib_gear[5] = pf_shiftspd;
            pf_pitch = pf_shiftspd;
          else
            pf_pitch = -pf_shiftspd;
            //ffb_vib_gear[5] = pf_shiftspd;
          //return motion_process (ffb_vib_gear, 0, mtime);
          //not reacheable
          /*
          mdelta += MOTION_FREQ_MS; //force event motion
          pv_pitch = 0x01;
          c_accel = lpacc;
          max_accel = lpacc;
          //pw_pitch = c_accel;
          */
        }
        //gear up
        if (report[8] & 0x02)
        {
          //printf ("\ngear up\n");
          max_accel = 0;
          if (lpbrk)
            //ffb_vib_gear[5] = pf_shiftspd;
            pf_pitch = pf_shiftspd;
          else
            //ffb_vib_gear[5] = -pf_shiftspd;
            pf_pitch = -pf_shiftspd;
          //return motion_process (ffb_vib_gear, 0, mtime);
        }
      }
    } // wheel data
      break;
    default:
      /*
       * wheel data processing
       */
      if (_odbg > 2)
      {
        printf ("\n#w!UNK pkt: ");
        for (int i = 0; i < rlen; i++)
          printf ("%02x ", report[i]);
      }
  }
  //emphasis on vibration roll
#if 0
  if (vib_k == 1)
    pv_roll = 150;
  else if (vib_k == 2)
    pv_roll = -150;
#endif
  //
  if (motion_compute (mtime))
  {
#if 0
#define BOAT_SPD  20
#define BOAT_AMP  200
    //generate heave 'boating' efect
    if (lpbrk < -32000 || lpacc > 32000)
    {
      //full accel or brake
      if (pw_heave_dir)
        pw_heave += BOAT_SPD * (lpbrk < -32000?3:1);
      else
        pw_heave -= BOAT_SPD * (lpbrk < -32000?3:1);
    }
    //boating boundaries
    if (pw_heave > BOAT_AMP)
      pw_heave_dir = 0; //go the other way
    if (pw_heave < -BOAT_AMP)
      pw_heave_dir = 1; //go the other way
    printf ("\n#i:boating at %d", pw_heave);
#endif
    //vibration switch
    if (vib_k == 1)
    {
      //printf ("\n#v:vib left");
      vib_k = 2;
    }
    else if (vib_k == 2)
    {
      //printf ("\n#v:vib right");
      vib_k = 1;
    }
    //reset forces and vibrations
    //pf_roll = pf_pitch = pv_roll = 0;
    pf_pitch = 0;
    pv_pitch = 0;
    pv_roll = 0;
    //add heave 'boating' efect
    //pv_pitch = pw_heave;
  }
  //
  return 0;
}
