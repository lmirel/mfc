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

#include "extras.h"

#define UDP_MAX_PACKETSIZE  256
#define UDP_PORT            20777

#define DEBUG 0
#define debug_print(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

int inet_aton (const char *cp, struct in_addr *inp);
//int usleep(long usec);

//used by pkt processing logic
static unsigned int dtime_ms ()
{
  static unsigned long lms = 0;
  unsigned long cms = get_millis ();
  unsigned long ms = cms - lms;
  lms = cms;
  return (unsigned int)ms;
}
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
int pf_accspd = 350; //[50 .. 500]
int _pitchprc = 65;
int _rollprc = 100;
int _yawprc = 100;
int _surgeprc = 100;
int _swayprc = 65;
int _heaveprc = 100;
char _odbg = 0;
char _mot = 0;        //don't process motion
char _cap = 0;
char _fil = 0;        //capture in/out data to file

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
    {"044f:b66d", "Thrustmaster",  motion_process_thrustmaster},
};
int _p_idx = 1; //dummy
//
char *_pdev = NULL;    //device used to extract USB data
static int *_cpkt = NULL;
static int _cpktl = 0;
int main (int argc, char **argv, char **envp)
{
  struct pollfd fdset[3];
  int nfds = 1;
  int timeout, rc;
  int lport = UDP_PORT;
  
  //roll and pitch percentage
  for (int i = 1; i < argc; i++)
  {
    if (argv[i][0] == '-')
      switch (argv[i][1])
      {
        //profiling params
        case 'r': //roll %
          _rollprc = atoi (argv[i]+2);
          //if (_rollprc < 0 || _rollprc > 100)
          //  _rollprc = 100;
          break;
        case 'p': //pitch %
          _pitchprc = atoi (argv[i]+2);
          //if (_pitchprc < 0 || _pitchprc > 100)
          //  _pitchprc = 100;
          break;
        case 'y': //yaw %
          _yawprc = atoi (argv[i]+2);
          //if (_yawprc < 0 || _yawprc > 100)
          //  _yawprc = 100;
          break;
        case 's': //surge %
          _surgeprc = atoi (argv[i]+2);
          //if (_surgeprc < 0 || _surgeprc > 100)
          //  _surgeprc = 100;
          break;
        case 'w': //sway %
          _swayprc = atoi (argv[i]+2);
          //if (_swayprc < 0 || _swayprc > 100)
          //  _swayprc = 100;
          break;
        case 'h': //heave %
          _heaveprc = atoi (argv[i]+2);
          //if (_heaveprc < 0 || _heaveprc > 100)
          //  _heaveprc = 100;
          break;
        case 'v': //debug
          _odbg = atoi (argv[i]+2);
          break;
        case 'd': //device
          _pdev = argv[i]+2;
          //set-up the motion processor function
          for (int i = 1; _procs[i].pdev != NULL; i++)
            if (strncmp (_pdev, _procs[i].pdev, 9) == 0)
            {
              _p_idx = i;
              break;
            }
          break;
        case 'c': //data capture
          _cap++;
          break;
        case 'm': //debug
          _mot++;
          break;
      }
  }
  //configuration summary
  lport = 64402;
  printf ("\n# ##");
  printf ("\n#MFC extractor client");
  printf ("\n#running configuration:");
  //printf ("\n#      roll range %d [1..10]", _rollspd);
  //printf ("\n#    accel. speed %d (-a%d) range [1..10]", _accspd, _accspd);
  //printf ("\n#shifter feedback %d (-s%d) range [1..10]", _shiftspd, _shiftspd);
  //printf ("\n#vibrat. feedback %d (-v%d) range [1..10]", _vibfbk, _vibfbk);
  printf ("\n#  pitch feedback %d%% (-p%d) (pedals)", _pitchprc, _pitchprc);
  printf ("\n#  surge feedback %d%% (-s%d) (gears)", _surgeprc, _surgeprc);
  printf ("\n#  heave feedback %d%% (-h%d) (ffb vibs)", _heaveprc, _heaveprc);
  printf ("\n#   roll feedback %d%% (-r%d) (ffb roll)", _rollprc, _rollprc);
  printf ("\n#   sway feedback %d%% (-w%d) (wheel)", _swayprc, _swayprc);
  printf ("\n#    yaw feedback %d%% (-y%d) (wheel)", _yawprc, _yawprc);
  printf ("\n# verbosity level %d (-d%d)", _odbg, _odbg);
  printf ("\n#    capture mode %s (-c)", _cap?"on":"off");
  printf ("\n#motion processor %s", _procs[_p_idx].pdv);
  printf ("\n# ##");
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
  if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
  {
    printf("socket\n");
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
  if (bind(s, (struct sockaddr*)&si_me, sizeof(si_me))==-1)
  {
    fprintf(stderr, "bind() failed\n");
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
  (void) set_prio ();
  //
  int ppkt = 1;
  char packetBuffer[UDP_MAX_PACKETSIZE];
  //
  int rlen = 0;
  int _dts = -1, ldts = 0;
  int ppid = 0, vvid = 0;
  printf("\n#i:ready.");
  fflush (stdout);
  //
  while (!_done)
  {
    memset ((void*)fdset, 0, sizeof(fdset));

    fdset[0].fd = s;
    fdset[0].events = POLLIN;

    rc = poll (fdset, nfds, timeout);      

    if (rc < 0) 
    {
      printf("\n#e:poll() failed!");
      _done = 1;
      break;
    }
        
    if (rc == 0) 
    {
      printf(".");
      fflush (stdout);
      usleep (1000000);
    }
              
    if (fdset[0].revents & POLLIN)
    {
      //recvfrom(RecvSocket,  packetBuffer , SMS_UDP_MAX_PACKETSIZE, 0, (SOCKADDR *) & Sender, &SenderAddrSize);
      //if ((rlen = recvfrom (s, (void *)&packetBuffer, SMS_UDP_MAX_PACKETSIZE, 0, (struct sockaddr*)&si_other, &slen))==-1)
      if ((rlen = recvfrom (s, (void *)&packetBuffer, UDP_MAX_PACKETSIZE, 0, NULL, NULL)) == -1)
      {
        printf("\n#w:recvfrom() failed.");
      }
      else
      {
        ldts = dtime_ms ();
        //debug?!
        if (0)
          printf ("\n#i.PKT len %d", rlen);
        switch ((char) packetBuffer[0])
        {
          case (PKTT_DEBUG):  //debug data
          {
            if (_dts == -1)
            {
              vvid = packetBuffer[0] << 8 | packetBuffer[1];
              ppid = packetBuffer[2] << 8 | packetBuffer[3];
              printf ("\n//i.WHL %04x:%04x ", vvid, ppid);
            }
            _dts = packetBuffer[6] << 8 | packetBuffer[7];
            //printf ("\n#i.DBG@%04d: ", _dts);
            //select proper processing function based on VID+PID?!
            break;
          }
          //
          case (PKTT_OUT):   //FFB data
          {
            if (_cap || _odbg)
            {
              fprintf (stdout, "\n#i.FFB@%04d: ", _dts==-1?ldts:_dts);
              for (i = 0; i < rlen; i++)
                fprintf (stdout, "%02x ", packetBuffer[i]);
            }
            break;
          }
          case (PKTT_IN):   //WHL data
          {
            if (_cap || _odbg)
            {
              if (_dts == -1)
                _dts = dtime_ms ();
              fprintf (stdout, "\n#i.WHL@%04d: ", _dts==-1?ldts:_dts);
              for (i = 0; i < rlen; i++)
                fprintf (stdout, "%02x ", packetBuffer[i]);
            }
            //
            break;
          }
          default:
          {
            if (0)
            {
              if (_dts == -1)
                _dts = dtime_ms ();
              printf ("\n#w.UNK@%04d: ", _dts==-1?ldts:_dts);
              for (i = 0; i < rlen; i++)
                printf ("%02x ", packetBuffer[i]);
            }
          }
        }//switch
        //
        if (_procs[_p_idx].pf (packetBuffer, rlen, _dts==-1?ldts:_dts))
          ppkt++;
        //
        if ((ppkt % 500) == 0)
          printf ("\n#i:received %dpkts", ppkt);
      }
    }
    fflush (stdout);
  }
  //
  printf("\n#i:cleaning up.. done.\n");
  //
  close (s);
  mfc_bcast_close ();
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
int pl_roll, pl_pitch = 0;  //entire platform roll/pitch
//platform acceleration leveling
int max_accel = 0;
int c_accel = 0;

int pw_roll, pw_pitch = 0;  //wheel forces
int pf_roll, pf_pitch = 0;  //ffb forces
int pv_roll = 0;  //vibration forces
int pv_pitch = 0;
int pw_heave = 0, pw_heave_dir = 1; //used for heave 'boating' effect
int lpacc, lpbrk = 0;       //local values for accel+brake for axis combo
char _wd = 'W'; //wheel data source
int vib_k = 0;
int _olat  = 12;    //network latency/frequency

static int motion_compute (int lmtime)
{
  //handle network throttle
  static unsigned long llts = 0;
  unsigned long clts = get_millis ();
  unsigned long mdt = clts - llts;
  static unsigned long mpktt = 0;
  static unsigned long mpktd = 0;
  //pw_pitch at 40%
  //pw_pitch = get_map (pw_pitch, -32800, 32800, -16400, 16400);
  //ffb roll mapping: 10%
  //pf_roll = get_map (pf_roll, -130, 130, -3280, 3280);
  //ffb pitch mapping: 10%
  //pf_pitch = get_map (pf_pitch, -130, 130, -3280, 3280);
  //vib roll mapping: 10%
  //pv_roll = get_map (pv_roll, -130, 130, -3280, 3280);
  //vib pitch mapping: 10%
  //pv_pitch = get_map (pv_pitch, -130, 130, -3280, 3280);
  //
  //pl_roll  = pw_roll;
  //pl_pitch = pw_pitch;
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
  if (_cpkt)
  {
    //motion
    #if 1
    //pedals-based pitch
    _cpkt[MFC_PIPITCH] = -get_cmap (pw_pitch, -32800, 32800, -10000, 10000);
    //gear changes
    _cpkt[MFC_PISURGE] = -get_cmap (pf_pitch, -100, 100, -10000, 10000);
    //steering vibrations / 'boating': pv_pitch
    _cpkt[MFC_PIHEAVE] = -get_cmap (pv_pitch, -128, 128, -10000, 10000);
    //ffb roll
    _cpkt[MFC_PIROLL]  = get_cmap (pf_roll, -128, 128, -10000, 10000);
    //steering / body roll
    _cpkt[MFC_PISWAY]  = get_cmap (pw_roll, -16400, 16400, -10000, 10000);
    //steering direction
    _cpkt[MFC_PIYAW]   = get_cmap (pw_roll, -16400, 16400, -10000, 10000);
    //
    if (_odbg > 1)
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
      //roll: MFC_PIROLL + MFC_PISWAY
      if (_cpkt[MFC_PISWAY] > 0)
      {
        if (_cpkt[MFC_PIROLL] < 0)
          _cpkt[MFC_PIROLL] *= -1;
      }
      else
      {
        if (_cpkt[MFC_PIROLL] > 0)
          _cpkt[MFC_PIROLL] *= -1;
      }
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
    _cpkt[MFC_PISPEED] = 0;          //speed
  }
  #if 0
  //pw_roll = 0;
  //if (pv_roll)  //vibrations feel emphasis: 50%
    //pl_roll  = /*get_cmap (pw_roll, -16400, 16400, -2000, 2000) + get_map (pf_roll, -130, 130, -5000, 5000) + get_map (pv_roll, -550, 550, -5000, 5000);
    pl_roll  = get_cmap (pf_roll, pf_roll_max, -pf_roll_max, -pf_movfbk, pf_movfbk) + get_map (pv_roll, -150, 150, -pf_vibfbk, pf_vibfbk);
  //else
    //pl_roll  = get_cmap (pf_roll, pf_roll_max, -pf_roll_max, -10000, 10000);
  //
  //pl_roll = 0;
  //
  //if (pv_pitch) //vibrations feel emphasis: 50%
    //pl_pitch = get_map (pw_pitch, -32800, 32800, -2000, 2000) + get_map (pf_pitch, -130, 130, -3000, 3000) + get_map (pv_pitch, -550, 550, -5000, 5000);
    pl_pitch = get_cmap (pw_pitch, -32800, 32800, -pf_movfbk, pf_movfbk) + get_cmap (pv_pitch, -125, 125, -pf_vibfbk, pf_vibfbk);
  
    //pl_pitch = get_map (pv_pitch, 0, 0x0ff, -5000, 5000);
  //else
    //pl_pitch = get_map (pw_pitch, -32800, 32800, -5000, 5000) + get_map (pf_pitch, -130, 130, -5000, 5000);
    //pl_pitch = get_cmap (pw_pitch, -32800, 32800, -10000, 10000);
  //
  //pl_roll  = get_map (pw_roll, -32800, 32800, -4000, 4000) + get_map (pf_roll, -130, 130, -500, 500) + get_map (pv_roll, -130, 130, -500, 500);
  //pl_pitch = get_map (pw_pitch, -32800, 32800, -4000, 4000) + get_map (pf_pitch, -130, 130, -500, 500) + get_map (pv_pitch, -130, 130, -500, 500);
  //debug
  if (_odbg > 2)
    printf ("\n#i:%c%04lums roll %d \t pitch %d \t wroll %d \t wpitch %d \t froll %d \t fpitch %d \t vroll %d \t vpitch %d",
          _wd, lmtime, pl_roll, pl_pitch, pw_roll, pw_pitch, pf_roll, pf_pitch, pv_roll, pv_pitch);
  debug_printl (10, "\n#i:%c%04lums map roll %d \t pitch %d \t wroll %d \t wpitch %d \t froll %d \t fpitch %d \t vroll %d \t vpitch %d",
          _wd, lmtime, 
          pl_roll, pl_pitch, 
          get_map (pw_roll, -32800, 32800, -13110, 13110),
          get_map (pw_pitch, -32800, 32800, -13110, 13110),
          get_map (pf_roll, -130, 130, -3280, 3280),
          get_map (pf_pitch, -130, 130, -3280, 3280),
          get_map (pv_roll, -130, 130, -3280, 3280),
          get_map (pv_pitch, -130, 130, -3280, 3280));
  #endif
  //
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
*/
  }
  //set 2DOF targets
  if (_cpkt)
  {
    //profiling vars
    // val .. 100
    // X   .. var
    //
    mfc_bcast_send ();
    //[-10000 .. 10000]
    //dof2l = pdata[MFC_PIPITCH] + pdata[MFC_PISURGE] + pdata[MFC_PIHEAVE] + pdata[MFC_PIROLL] + pdata[MFC_PISWAY];
    //dof2r = pdata[MFC_PIPITCH] + pdata[MFC_PISURGE] + pdata[MFC_PIHEAVE] - pdata[MFC_PIROLL] - pdata[MFC_PISWAY];
    //
    if (1||_odbg)
      printf ("\n#i@%04lu.roll:% 6d (r: % 5d / s: % 5d) | pitch: % 6d \t(p: % 5d / s: % 5d / h: % 5d)",
        mdt, _cpkt[MFC_PIROLL] + _cpkt[MFC_PISWAY], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY],
        _cpkt[MFC_PIPITCH] + _cpkt[MFC_PISURGE] + _cpkt[MFC_PIHEAVE],
        _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE]);
    if (0 || _odbg > 1)
      printf ("\n#i:pitch \t%d \t%d \t%d \troll \t%d \t%d \t%d", 
        _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY], _cpkt[MFC_PIYAW]);
  }
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
      if (1 || _odbg)
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
      if (1 || _odbg)
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
        if (1 || _odbg)
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
        if (1||_odbg)
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
        if (1 || _odbg > 2)
          printf ("\n#RAW whl %d acc %d brk %d", pw_roll, lpacc, lpbrk);
        //
        if (lpbrk < -5)
          pw_pitch = lpbrk;  //cut off accel if brake pressed
        else
        {
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
            //_wd = 'F';
          }
          else
          {
            max_accel = lpacc;
            c_accel = lpacc;
            //fprintf (stderr, "\n#i:3 lpacc %d \t max_acc %d \t c_acc %d", lpacc, max_accel, c_accel);
            //_wd = 'F';
          }
          //
          pw_pitch = c_accel + pf_accspd; //it should stay nose-up while accelerating
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
    //reset forces and vibrations
    //pf_roll = pf_pitch = pv_roll = 0;
    pf_pitch = 0;
    pv_pitch = 0;
    pv_roll = 0;
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
 //<type>,<len>,  03    30    f8    13  B2:B1:R3:R2:R1:Y3:Y2:Y1  B3    D1    D2    D3
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

int motion_process_fanatec (char *report, int rlen, unsigned long dtime)
{
  //update delta time
  if (dtime == -1)
    dtime = 4;
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
      //
      _wd = 'F';
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_digits + 2), 5) == 0)
      {
        //nothing to do here: lights
        _wd = 'U';
      }
      else
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_lights2 + 2), 4) == 0)
      {
        //nothing to do here: lights
        _wd = 'U';
      }
      else
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_lights + 2), 5) == 0)
      {
        _wd = 'W'; //overwrite to wheel to avoid movement during standstill
        //lights: process max revs led
        if (report[8] & 0x01)
          pf_pitch = pf_shiftspd;
      }
      else
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_whlvib1 + 2), 4) == 0)
      {
        //TODO:vibrations?
        _wd = 'U';
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
         cwpos = report[6]; //normal_ffb2 (((int) report[6]), 0x080);
         pf_roll = (cwpos > lwpos)? (cwpos - lwpos):-(lwpos - cwpos);
         lwpos = cwpos;
         if (0||_odbg > 2)
           printf ("\n#d.WHLPOS1 ffb roll %d / %d", pf_roll, ((int) report[6]));
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
       cwpos = report[8]; //normal_ffb2 (((int) report[6]), 0x080);
       pf_roll = (cwpos > lwpos)? (cwpos - lwpos):-(lwpos - cwpos);
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
        if (1 || _odbg)
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
          //_wd = 'F';
        }
        else
        {
          max_accel = lpacc;
          c_accel = lpacc;
          //fprintf (stderr, "\n#i:3 lpacc %d \t max_acc %d \t c_acc %d", lpacc, max_accel, c_accel);
          //_wd = 'F';
        }
        //
        pw_pitch = c_accel + pf_accspd; //it should stay nose-up while accelerating
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
      }
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
  if (_mot == 0 && motion_compute (dtime))
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
          //_wd = 'F';
        }
        else
        {
          max_accel = lpacc;
          c_accel = lpacc;
          //fprintf (stderr, "\n#i:3 lpacc %d \t max_acc %d \t c_acc %d", lpacc, max_accel, c_accel);
          //_wd = 'F';
        }
        //
        pw_pitch = c_accel + pf_accspd; //it should stay nose-up while accelerating
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
      if (1 || _odbg > 2)
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
