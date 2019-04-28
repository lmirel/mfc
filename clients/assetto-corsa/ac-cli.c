/*
* codemasters f1 dashboard test

gcc -o /opt/mfcc-ac clients/assetto_corsa/ac-cli.c -lrt -std=c11
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

#include "../api_client.h"

//from https://docs.google.com/document/d/1KfkZiIluXZ6mMhLWfDX1qAGbvhGRC3ZUzjVIt5FQpp4/pub
#if 0
struct RTCarInfo
{
int identifier; //char identifier;
int size;
float speed_Kmh;
float speed_Mph;
float speed_Ms;
bool isAbsEnabled;
bool isAbsInAction;
bool isTcInAction;
bool isTcEnabled;
bool isInPit;
bool isEngineLimiterOn;
float accG_vertical;
float accG_horizontal;
float accG_frontal;
int lapTime;
int lastLap;
int bestLap;
int lapCount;
float gas;
float brake;
float clutch;
float engineRPM;
float steer;
int gear;
float cgHeight;
float wheelAngularSpeed[4];
float slipAngle[4];
float slipAngle_ContactPatch[4];
float slipRatio[4];
float tyreSlip[4];
float ndSlip[4];
float load[4];
float Dy[4];
float Mz[4];
float tyreDirtyLevel[4];
float camberRAD[4];
float tyreRadius[4];
float tyreLoadedRadius[4];
float suspensionHeight[4];
float carPositionNormalized;
float carSlope;
float carCoordinates[3];

}

struct RTLap
{
int carIdentifierNumber;
int lap;
char driverName[50];
char carName[50];
int time;

};

struct handshackerResponse{
char carName[50];
char driverName[50];
int identifier;
int version;
char trackName[50];
char trackConfig[50];

}
#endif

#define UDP_MAX_PACKETSIZE  328
#define UDP_PORT            9996

#define DEBUG 0
#define debug_print(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

int inet_aton(const char *cp, struct in_addr *inp);
//int usleep(long usec);

int ctime_ms(char pt)
{
    struct timeval te; 
    gettimeofday (&te, NULL); // get current time
    long long ms = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
//    ms = 999999;
    ms &= 0xffffffff;
    if (pt)
        printf("milliseconds: %lld\n", ms);
    return (int)ms;
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

long get_map(long x, long in_min, long in_max, long out_min, long out_max)
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

float get_map_f(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//+bcast network
int inet_aton(const char *cp, struct in_addr *inp);

/*
  int pkt_type;
  int pkt_dof_type;
  int data[6];    //{speed, roll, pitch, ..}
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
  fflush (stdout);
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
//-

char _done = 0;
void terminate (int sig)
{
  _done = 1;
}

enum acstat {
    sthandshake = 0,
    stupdate = 1,
    stspot = 2,
    stdismiss = 3
};

char *pp_ps4 = "192.168.2.111";
char _pp_ps4[50];
/**
A. right axis composition
a. g-force - longitudinal/pitch - overwrite
b. g-force - lateral - add

B. right axis composition
a. g-force - longitudinal/pitch - overwrite
b. g-force - lateral (invert) - add
*/
int _pitchprc = 100;
int _rollprc = 100;
int _yawprc = 100;
int _surgeprc = 100;
int _swayprc = 100;
int _heaveprc = 100;
char _odbg = 0;
int main (int argc, char **argv, char **envp)
{
  struct pollfd fdset[3];
  int nfds = 1;
  int gpio_fd, timeout, rc;
  unsigned int gpio;
  int len;
  int lport = UDP_PORT;
  int _rollprc = 100;
  int _pitchprc = 100;
  
  //roll and pitch percentage
  for (int i = 1; i < argc; i++)
  {
    if (argv[i][0] == '-')
      switch (argv[i][1])
      {
      	case 'c': //roll %
      		snprintf (_pp_ps4, 49, "%s", argv[i]+2);
      		pp_ps4 = _pp_ps4;
      	  break;
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
        case 'd': //debug enabled
          _odbg++;
          break;
        default:
          printf ("\n#w:unknown option -%c", argv[i][1]);
      }
  }
  //configuration summary 
  printf ("\n# ##");
  printf ("\n#MFC ASSETTO CORSA client");
  printf ("\n#running configuration:");
  printf ("\n#  pitch feedback %d%% (-p%d) range [0..100]%%", _pitchprc, _pitchprc);
  printf ("\n#   roll feedback %d%% (-r%d) range [0..100]%%", _rollprc, _rollprc);
  printf ("\n#    yaw feedback %d%% (-y%d) range [0..100]%%", _yawprc, _yawprc);
  printf ("\n#  surge feedback %d%% (-s%d) range [0..100]%%", _surgeprc, _surgeprc);
  printf ("\n#   sway feedback %d%% (-w%d) range [0..100]%%", _swayprc, _swayprc);
  printf ("\n#  heave feedback %d%% (-h%d) range [0..100]%%", _heaveprc, _heaveprc);
  //printf ("\n#     server port %d", MFCSVR_PORT);
  printf ("\n# verbosity level %d (-d%d)", _odbg, _odbg);
  //printf ("\n#     client port %d (-l%d)", lport, lport);
  printf ("\n#    server address %s (-c%s)", pp_ps4, pp_ps4);
  printf ("\n# ##");
  //
  int cs = mfc_bcast_prep ("127.0.0.1", 0);
  if (cs < 3)
  {
    printf ("\n#e:can't connect to MFC server on port %d", MFCSVR_PORT);
    exit(1);
  }
  printf ("\n#i:<%d:MFC server on port %d", cs, MFCSVR_PORT);
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
  struct sockaddr_in si_other, si_me;
  int s, i, slen = sizeof (si_other), pktk = 0;
  if ((s = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf ("socket\n");
    exit (1);
  }
  #if 0
  int broadcast=1;
  setsockopt(s, SOL_SOCKET, SO_BROADCAST,
          &broadcast, sizeof (broadcast));
  #endif
/* receiver */
  memset((char *) &si_me, 0, sizeof (si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons (9999);
  si_me.sin_addr.s_addr = htonl (INADDR_ANY);
  if (bind (s, (struct sockaddr*)&si_me, sizeof (si_me))==-1)
  {
    fprintf (stderr, "bind() failed\n");
    exit (1);
  }
/* sender */
  memset ((char *) &si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons (lport);
  pktk = 990;
  //TODO: add server IP param
  if (inet_aton (pp_ps4, &si_other.sin_addr) == 0)
  {
    fprintf (stderr, "inet_aton() failed\n");
    exit (1);
  }
  //
  printf ("\n#i:>%d:listening on port %d", s, lport);
  char ac_pkt[1024];
  //handshake
  int bs = connect (s, (struct sockaddr*)&si_other, sizeof (si_other));
  if (bs < 0)
    printf ("\n#ERR:connect");
  else
    printf ("\n#i:%d<%dB connected", s, bs);
  int hshk[3] = {1, 1, sthandshake};
  bs = send (s, &hshk, 12, 0);
  if (bs < 0)
    printf ("\n#ERR: send handshake");
  else
    printf ("\n#i:%d<%dB sent for handshake", s, bs);
  //get handshake
  bs = recv (s, ac_pkt, 1024, 0);
  if (bs < 0)
    printf ("\n#ERR: recv handshake");
  else
    printf ("\n#i:%d<%dB recvd handshake", s, bs);
  //subscribe
  int sscr[3] = {1, 1, stupdate};
  bs = send (s, &sscr, 12, 0);
  if (bs < 0)
    printf ("\n#ERR:subscribe");
  else
    printf ("\n#i:%d<%dB sent, subscribed", s, bs);
  //ctime_ms (1);
  //learning values
  float fv[MFC_PKT_SIZE];
  float lminl, lmaxl, lminr, lmaxr;
  lminl = lmaxl = lminr = lmaxr = 0.0f;
  //only send 3 PAUSEd packets
  (void) signal(SIGINT, terminate);
  (void) signal(SIGTERM, terminate);
  (void) signal(SIGHUP, terminate);
  //
  int ppkt = 0;
  long lts = ctime_ms (0);
  char packetBuffer[UDP_MAX_PACKETSIZE];
  //
  int wpkt[MFC_PKT_SIZE] = {0};
  int mpkt[MFC_PKT_SIZE] = {0};
  int Mpkt[MFC_PKT_SIZE] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
  //
  int *_cpkt = mfc_bcast_pktget ();
  int   pktl = mfc_bcast_pktlen ();
  //
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
      lminl = lmaxl = lminr = lmaxr = 0.0f;
      printf(".");
      fflush (stdout);
      sleep (1);
    }
              
    if (fdset[0].revents & POLLIN)
    {
      int rlen = 0, idx;
      //recvfrom(RecvSocket,  packetBuffer , SMS_UDP_MAX_PACKETSIZE, 0, (SOCKADDR *) & Sender, &SenderAddrSize);
      //if ((rlen = recvfrom (s, (void *)&packetBuffer, SMS_UDP_MAX_PACKETSIZE, 0, (struct sockaddr*)&si_other, &slen))==-1)
      if ((rlen = recvfrom (s, (void *)&packetBuffer, UDP_MAX_PACKETSIZE, 0, NULL, NULL))==-1)
      {
        printf("\n#w:recvfrom() failed.");
      }
      else
      {
        ppkt++;
        if ((ppkt % 500) == 0)
          printf ("\n#i:received %dpkts", ppkt);
        //
        if (1)
          printf("\r\n@%lums received %dB packet (vs %d) from %s:%d <", 
                  ctime_ms(0) - lts, rlen, UDP_MAX_PACKETSIZE, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
        lts = ctime_ms(0);
        printf ("\n#i:pkt:%c / %d rpm %f gear %d spd %f %f %f", (char)get_int (packetBuffer, 0), get_int (packetBuffer, 4), 
          get_float (packetBuffer, 68), get_int (packetBuffer, 76), get_float (packetBuffer, 8), get_float (packetBuffer, 12), get_float (packetBuffer, 16));
        printf ("\n#i:ffb:vert %f hori %f long%f", get_float (packetBuffer, 28), get_float (packetBuffer, 32), get_float (packetBuffer, 36));
        //
        
        if (_cpkt)
        {
          //motion data packet
          _cpkt[MFC_PITYPE] = PKTT_DATA;
          _cpkt[MFC_PIDOF]  = PKTT_2DOFN;
          //motion
          #if 1
          #define VERT_IDX 28
          #define HORI_IDX 32
          #define LONG_IDX 36
          //
          fv[MFC_PIPITCH] = 0;//-get_float (packetBuffer, VERT_IDX + 8);
          _cpkt[MFC_PIPITCH]  = (int)(fv[MFC_PIPITCH] * 1000.0f);
          //_cpkt[MFC_PIPITCH] = -get_cmap (pf_pitch, -100, 100, -1000, 1000);
          //
          fv[MFC_PISURGE] = -get_float (packetBuffer, LONG_IDX);
          _cpkt[MFC_PISURGE]  = (int)(fv[MFC_PISURGE] * 1000.0f);
          //_cpkt[MFC_PISURGE] = -get_cmap (pw_pitch, -32800, 32800, -7000, 7000);
          //
          fv[MFC_PIHEAVE] = -get_float (packetBuffer, VERT_IDX);
          _cpkt[MFC_PIHEAVE]  = (int)(fv[MFC_PIHEAVE] * 1000.0f);
          //_cpkt[MFC_PIHEAVE] = -get_cmap (pv_pitch, -128, 128, -2000, 2000);
          //
          fv[MFC_PIROLL] = 0;//-get_float (packetBuffer, HORI_IDX + 4);
          _cpkt[MFC_PIROLL]  = (int)(fv[MFC_PIROLL] * 1000.0f);
          //_cpkt[MFC_PIROLL]  = get_cmap (pf_roll, -128, 128, -7000, 7000);
          //
          fv[MFC_PISWAY] = -get_float (packetBuffer, HORI_IDX);
          _cpkt[MFC_PISWAY]  = (int)(fv[MFC_PISWAY] * 1000.0f);
          //_cpkt[MFC_PISWAY]  = get_cmap (get_float (packetBuffer, local_accel_idx)*100, 8200, 8200, -3000, 3000);
          //
          fv[MFC_PIYAW] = -get_float (packetBuffer, HORI_IDX); //TODO: compute direction
          _cpkt[MFC_PIYAW]  = (int)(fv[MFC_PIYAW] * 1000.0f);
          //_cpkt[MFC_PIYAW]   = get_cmap (pw_roll, -16400, 16400, -10000, 10000);
          //
          if (0)
            printf ("\n#i:telem %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f",
              fv[MFC_PIPITCH]>=0.0f?" +":" ", fv[MFC_PIPITCH], fv[MFC_PISURGE]>=0.0f?" +":" ", fv[MFC_PISURGE],
              fv[MFC_PIHEAVE]>=0.0f?" +":" ", fv[MFC_PIHEAVE], fv[MFC_PIROLL]>=0.0f?" +":" ", fv[MFC_PIROLL], 
              fv[MFC_PISWAY]>=0.0f?" +":" ", fv[MFC_PISWAY], fv[MFC_PIYAW]>=0.0f?" +":" ", fv[MFC_PIYAW]);
          if (1||_odbg)
            printf ("\n#i:pitch \t%d \t%d \t%d \troll \t%d \t%d \t%d", 
              _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY], _cpkt[MFC_PIYAW]);
          //
          if (_odbg)
            printf ("\n#i:pitch%% \t%d \t%d \t%d \troll%% \t%d \t%d \t%d", 
              _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY], _cpkt[MFC_PIYAW]);
          //
          int _newrange = 0;
          for (int i = MFC_PIDOF + 1; i < MFC_PIDOF + 7; ++i)
          {
            if (_cpkt[i] < mpkt[i])
            {
              mpkt[i] = _cpkt[i];
              _newrange = 1;
            }
            if (_cpkt[i] > Mpkt[i])
            {
              Mpkt[i] = _cpkt[i];
              _newrange = 1;
            }
            //
            if (0 && (_cpkt[i] < -550 || _cpkt[i] > 550))
            {
              printf ("\n#E:min value for %d is %d vs %d orig %.8f", i, mpkt[i], -550, fv[i]);
              printf ("\n#E:MAX value for %d is %d vs %d orig %.8f", i, Mpkt[i], 550, fv[i]);
              if (0)
                printf ("\n#i:telem %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f",
                  fv[MFC_PIPITCH]>=0.0f?" +":" ", fv[MFC_PIPITCH], fv[MFC_PISURGE]>=0.0f?" +":" ", fv[MFC_PISURGE],
                  fv[MFC_PIHEAVE]>=0.0f?" +":" ", fv[MFC_PIHEAVE], fv[MFC_PIROLL]>=0.0f?" +":" ", fv[MFC_PIROLL], 
                  fv[MFC_PISWAY]>=0.0f?" +":" ", fv[MFC_PISWAY], fv[MFC_PIYAW]>=0.0f?" +":" ", fv[MFC_PIYAW]);
              if (0||_odbg)
                printf ("\n#i:pitch \t%d \t%d \t%d \troll \t%d \t%d \t%d", 
                  _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY], _cpkt[MFC_PIYAW]);
            }
            //
            //if (_cpkt[i] < -550 || _cpkt[i] > 550)
            //  _newrange = 1;
          }
          if (_newrange)
            for (int i = MFC_PIDOF + 1; i < MFC_PIDOF + 7; ++i)
              printf ("\n#E:%d# [%d.. %d ..%d]", i, mpkt[i], _cpkt[i], Mpkt[i]);
          //
          _cpkt[MFC_PIPITCH] = get_cmap (_cpkt[MFC_PIPITCH], mpkt[MFC_PIPITCH], Mpkt[MFC_PIPITCH], -4000, 4000);
          _cpkt[MFC_PISURGE] = get_cmap (_cpkt[MFC_PISURGE], mpkt[MFC_PISURGE], Mpkt[MFC_PISURGE], -4000, 4000);
          _cpkt[MFC_PIHEAVE] = get_cmap (_cpkt[MFC_PIHEAVE], mpkt[MFC_PIHEAVE], Mpkt[MFC_PIHEAVE], -2000, 2000);
          _cpkt[MFC_PIROLL]  = get_cmap (_cpkt[MFC_PIROLL],  mpkt[MFC_PIROLL],  Mpkt[MFC_PIROLL], -5000, 5000);
          _cpkt[MFC_PISWAY]  = get_cmap (_cpkt[MFC_PISWAY],  mpkt[MFC_PISWAY],  Mpkt[MFC_PISWAY], -5000, 5000);
          _cpkt[MFC_PIYAW]   = get_cmap (_cpkt[MFC_PIYAW],   mpkt[MFC_PIYAW],   Mpkt[MFC_PIYAW], -10000, 10000);
          //adjust %
          _cpkt[MFC_PIPITCH] = _cpkt[MFC_PIPITCH] * _pitchprc / 100;
          _cpkt[MFC_PIROLL]  = _cpkt[MFC_PIROLL]  * _rollprc  / 100;
          _cpkt[MFC_PIYAW]   = _cpkt[MFC_PIYAW]   * _yawprc   / 100;
          _cpkt[MFC_PISURGE] = _cpkt[MFC_PISURGE] * _surgeprc / 100;
          _cpkt[MFC_PISWAY]  = _cpkt[MFC_PISWAY]  * _swayprc  / 100;
          _cpkt[MFC_PIHEAVE] = _cpkt[MFC_PIHEAVE] * _heaveprc / 100;
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
          //
          mfc_bcast_send ();
        }
      }
    }
    fflush (stdout);
  }
  //
  printf("\n#i:cleaning up.. ");
  int diss[3] = {1, 1, stdismiss};
  bs = send (s, &diss, 12, 0);
  if (bs < 0)
    printf ("\n#ERR:dismiss");
  else
    printf ("\n#i:%d<%dB sent, dismissed", s, bs);
  //
  close (s);
  mfc_bcast_close ();
  //
  printf("\n#i:done.\n");
  return 0;
}

#if 0
//from https://github.com/ottonello/AssettoCorsaTelemetry/blob/master/src/main/java/sample/corsa/TelemetryInterface.java
package sample.corsa;

import sample.StructWriter;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;

public class TelemetryInterface {
    DatagramSocket clientSocket;
    public static final int CORSA_PORT = 9996;
    private Status status = Status.init;
    InetAddress IPAddress;
    RTCarInfo telemetry;

    public void connect() {
        new Thread(new Runnable() {
            public void run() {
                try {
                    startHandshake();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }).start();

    }

    private void startHandshake() throws IOException {
        IPAddress = InetAddress.getByName("localhost");

        clientSocket = new DatagramSocket(9999);
        clientSocket.connect(IPAddress, CORSA_PORT);

        byte[] receiveData = new byte[1024];

        byte[] handShake = getHandshake();
        clientSocket.send(new DatagramPacket(handShake, handShake.length));

        DatagramPacket receivePacket = new DatagramPacket(receiveData, HandshakeResponse.SIZE);
        clientSocket.receive(receivePacket);

        HandshakeResponse handshakeResponse = new HandshakeResponse(receiveData);
        System.out.println("Handshake: " + handshakeResponse);

        status = Status.handshake;

        byte[] subscribe = getSubscribeUpdates();
        clientSocket.send(new DatagramPacket(subscribe, subscribe.length));

        System.out.println("Subscribed");
        status = Status.subscribed;

        while (!Status.dismissed.equals(status)) {
//            System.out.println("Receiving");

            DatagramPacket receivedLapData = new DatagramPacket(receiveData, 328);
            clientSocket.receive(receivedLapData);

            telemetry = new RTCarInfo(receiveData);
//            System.out.println("RECEIVED: " + telemetry);
        }
        System.out.println("Bye");

    }

    public RTCarInfo getTelemetry() {
        return telemetry;
    }

    private byte[] getHandshake() throws IOException {
        StructWriter structWriter = new StructWriter(12);
        structWriter.writeInt(1);
        structWriter.writeInt(1);
        structWriter.writeInt(OperationId.HANDSHAKE);
        return structWriter.toByteArray();
    }

    private byte[] getSubscribeUpdates() throws IOException {
        StructWriter structWriter = new StructWriter(12);
        structWriter.writeInt(1);
        structWriter.writeInt(1);
        structWriter.writeInt(OperationId.SUBSCRIBE_UPDATE);
        return structWriter.toByteArray();
    }

    private byte[] getSubscribeSpot() throws IOException {
        StructWriter structWriter = new StructWriter(12);
        structWriter.writeInt(1);
        structWriter.writeInt(1);
        structWriter.writeInt(OperationId.SUBSCRIBE_SPOT);
        return structWriter.toByteArray();
    }

    private byte[] getDismiss() throws IOException {
        StructWriter structWriter = new StructWriter(12);
        structWriter.writeInt(1);
        structWriter.writeInt(1);
        structWriter.writeInt(OperationId.DISMISS);
        return structWriter.toByteArray();
    }

    public void stop() throws IOException {
        System.out.println("Dismissing");

        byte[] dismiss = getDismiss();
        try {
            clientSocket.send(new DatagramPacket(dismiss, dismiss.length, IPAddress, CORSA_PORT));

        } catch (Exception e) {
            e.printStackTrace();
        }

        clientSocket.close();
        status = Status.dismissed;

    }
}
//
public enum Status {
    init,
    handshake,
    subscribed,
    dismissed
}
//
package sample.corsa;

import sample.StructReader;

import java.io.IOException;

public class HandshakeResponse {
    public static final int CAR_NAME_SIZE = 50;
    public static final int DRIVER_NAME_SIZE = 50;
    public static final int TRACK_NAME_SIZE = 50;
    public static final int TRACK_CONFIG_SIZE = 50;
    public static final int SIZE =
            CAR_NAME_SIZE *2 +
                    DRIVER_NAME_SIZE * 2 +
                    TRACK_NAME_SIZE * 2 +
                    TRACK_CONFIG_SIZE *2 +
                    4 +
                    4;

    String driverName;
    String carName;
    String trackName;
    String trackConfig;
    int identifier;
    int version;

    public HandshakeResponse(byte[] received) throws IOException {
        StructReader structReader = new StructReader(received);

        carName = structReader.readChars(CAR_NAME_SIZE);
        driverName =structReader.readChars(DRIVER_NAME_SIZE);
        identifier = structReader.readInt();
        version = structReader.readInt();
        trackName = structReader.readChars(TRACK_NAME_SIZE);
        trackConfig = structReader.readChars(TRACK_CONFIG_SIZE);
    }

    public String getDriverName() {
        return driverName;
    }

    public void setDriverName(String driverName) {
        this.driverName = driverName;
    }

    public String getCarName() {
        return carName;
    }

    public void setCarName(String carName) {
        this.carName = carName;
    }

    public String getTrackName() {
        return trackName;
    }

    public void setTrackName(String trackName) {
        this.trackName = trackName;
    }

    public String getTrackConfig() {
        return trackConfig;
    }

    public void setTrackConfig(String trackConfig) {
        this.trackConfig = trackConfig;
    }

    public int getIdentifier() {
        return identifier;
    }

    public void setIdentifier(int identifier) {
        this.identifier = identifier;
    }

    public int getVersion() {
        return version;
    }

    public void setVersion(int version) {
        this.version = version;
    }

    @Override
    public String toString() {
        return "HandshakeResponse{" +
                "carName='" + carName + '\'' +
                ", driverName='" + driverName + '\'' +
                ", identifier=" + identifier +
                ", version=" + version +
                ", trackName='" + trackName + '\'' +
                ", trackConfig='" + trackConfig + '\'' +
                '}';
    }
}
//
package sample.corsa;

public interface OperationId {
    public int HANDSHAKE = 0;
    public int SUBSCRIBE_UPDATE =1;
    public int SUBSCRIBE_SPOT = 2;
    public int DISMISS=3;
}
//
package sample.corsa;

import sample.StructReader;

import java.io.IOException;
import java.util.Arrays;

/**
 * Created by Marcos on 8/15/2015.
 */
public class RTCarInfo {

    int identifier;
    int size;

    float speed_Kmh;
    float speed_Mph;
    float speed_Ms;

    boolean isAbsEnabled;
    boolean isAbsInAction;
    boolean isTcInAction;
    boolean isTcEnabled;
    boolean isInPit;
    boolean isEngineLimiterOn;

    float accG_vertical;
    float accG_horizontal;
    float accG_frontal;

    int lapTime;
    int lastLap;
    int bestLap;
    int lapCount;

    float gas;
    float brake;
    float clutch;
    float engineRPM;
    float steer;
    int gear;
    float cgHeight;

    float wheelAngularSpeed[];
    float slipAngle[];
    float slipAngle_ContactPatch[];
    float slipRatio[];
    float tyreSlip[];
    float ndSlip[];
    float load[];
    float Dy[];
    float Mz[];
    float tyreDirtyLevel[];

    float camberRAD[];
    float tyreRadius[];
    float tyreLoadedRadius[];
    float suspensionHeight[];
    float carPositionNormalized;
    float carSlope;
    float carCoordinates[];

    public RTCarInfo(byte[] received) throws IOException {
        StructReader structReader = new StructReader(received);
        identifier = structReader.readInt();
        size = structReader.readInt();
        speed_Kmh = structReader.readFloat();
        speed_Mph = structReader.readFloat();
        speed_Ms = structReader.readFloat();
        isAbsEnabled = structReader.readBool();
        isAbsInAction = structReader.readBool();
        isTcInAction = structReader.readBool();
        isTcEnabled = structReader.readBool();
        isInPit = structReader.readBool();
        isEngineLimiterOn = structReader.readBool();

        accG_vertical = structReader.readFloat();
        accG_horizontal = structReader.readFloat();
        accG_frontal = structReader.readFloat();

        lapTime = structReader.readInt();
        lastLap = structReader.readInt();
        bestLap = structReader.readInt();
        lapCount = structReader.readInt();

        gas = structReader.readFloat();
        brake = structReader.readFloat();
        clutch = structReader.readFloat();
        engineRPM = structReader.readFloat();
        steer = structReader.readFloat();
        gear = structReader.readInt();
        cgHeight = structReader.readFloat();
        wheelAngularSpeed = structReader.readFloats(4);
        slipAngle = structReader.readFloats(4);
        slipAngle_ContactPatch = structReader.readFloats(4);
        slipRatio = structReader.readFloats(4);
        tyreSlip = structReader.readFloats(4);
        ndSlip = structReader.readFloats(4);
        load = structReader.readFloats(4);
        Dy = structReader.readFloats(4);
        Mz = structReader.readFloats(4);
        tyreDirtyLevel = structReader.readFloats(4);

        camberRAD = structReader.readFloats(4);
        tyreRadius = structReader.readFloats(4);
        tyreLoadedRadius = structReader.readFloats(4);
        suspensionHeight = structReader.readFloats(4);
        carPositionNormalized = structReader.readFloat();
        carSlope = structReader.readFloat();

        carCoordinates = structReader.readFloats(3);
    }

    public int getIdentifier() {
        return identifier;
    }

    public int getSize() {
        return size;
    }

    public float getSpeed_Kmh() {
        return speed_Kmh;
    }

    public float getSpeed_Mph() {
        return speed_Mph;
    }

    public float getSpeed_Ms() {
        return speed_Ms;
    }

    public boolean isAbsEnabled() {
        return isAbsEnabled;
    }

    public boolean isAbsInAction() {
        return isAbsInAction;
    }

    public boolean isTcInAction() {
        return isTcInAction;
    }

    public boolean isTcEnabled() {
        return isTcEnabled;
    }

    public boolean isInPit() {
        return isInPit;
    }

    public boolean isEngineLimiterOn() {
        return isEngineLimiterOn;
    }

    public float getAccG_vertical() {
        return accG_vertical;
    }

    public float getAccG_horizontal() {
        return accG_horizontal;
    }

    public float getAccG_frontal() {
        return accG_frontal;
    }

    public int getLapTime() {
        return lapTime;
    }

    public int getLastLap() {
        return lastLap;
    }

    public int getBestLap() {
        return bestLap;
    }

    public int getLapCount() {
        return lapCount;
    }

    public float getGas() {
        return gas;
    }

    public float getBrake() {
        return brake;
    }

    public float getClutch() {
        return clutch;
    }

    public float getEngineRPM() {
        return engineRPM;
    }

    @Override
    public String toString() {
        return "RTCarInfo{" +
                "identifier=" + identifier +
                ", size=" + size +
                ", speed_Kmh=" + speed_Kmh +
                ", speed_Mph=" + speed_Mph +
                ", speed_Ms=" + speed_Ms +
                ", isAbsEnabled=" + isAbsEnabled +
                ", isAbsInAction=" + isAbsInAction +
                ", isTcInAction=" + isTcInAction +
                ", isTcEnabled=" + isTcEnabled +
                ", isInPit=" + isInPit +
                ", isEngineLimiterOn=" + isEngineLimiterOn +
                ", accG_vertical=" + accG_vertical +
                ", accG_horizontal=" + accG_horizontal +
                ", accG_frontal=" + accG_frontal +
                ", lapTime=" + lapTime +
                ", lastLap=" + lastLap +
                ", bestLap=" + bestLap +
                ", lapCount=" + lapCount +
                ", gas=" + gas +
                ", brake=" + brake +
                ", clutch=" + clutch +
                ", engineRPM=" + engineRPM +
                ", steer=" + steer +
                ", gear=" + gear +
                ", cgHeight=" + cgHeight +
                ", wheelAngularSpeed=" + Arrays.toString(wheelAngularSpeed) +
                ", slipAngle=" + Arrays.toString(slipAngle) +
                ", slipAngle_ContactPatch=" + Arrays.toString(slipAngle_ContactPatch) +
                ", slipRatio=" + Arrays.toString(slipRatio) +
                ", tyreSlip=" + Arrays.toString(tyreSlip) +
                ", ndSlip=" + Arrays.toString(ndSlip) +
                ", load=" + Arrays.toString(load) +
                ", Dy=" + Arrays.toString(Dy) +
                ", Mz=" + Arrays.toString(Mz) +
                ", tyreDirtyLevel=" + Arrays.toString(tyreDirtyLevel) +
                ", camberRAD=" + Arrays.toString(camberRAD) +
                ", tyreRadius=" + Arrays.toString(tyreRadius) +
                ", tyreLoadedRadius=" + Arrays.toString(tyreLoadedRadius) +
                ", suspensionHeight=" + Arrays.toString(suspensionHeight) +
                ", carPositionNormalized=" + carPositionNormalized +
                ", carSlope=" + carSlope +
                ", carCoordinates=" + Arrays.toString(carCoordinates) +
                '}';
    }
}
//
package sample.corsa;

import sample.StructReader;

import java.io.IOException;
import java.util.Arrays;

/**
 * Created by Marcos on 8/15/2015.
 */
public class RTLap {
    int carIdentifierNumber;
    int lap;
    String driverName;
    String carName;
    int time;

    public RTLap(byte[] received) throws IOException {
        StructReader structReader = new StructReader(received);
        carIdentifierNumber = structReader.readInt();
        lap = structReader.readInt();
        driverName = structReader.readChars(50);
        carName = structReader.readChars(50);
        time = structReader.readInt();
    }

    public int getCarIdentifierNumber() {
        return carIdentifierNumber;
    }

    public int getLap() {
        return lap;
    }

    public String getDriverName() {
        return driverName;
    }

    public String getCarName() {
        return carName;
    }

    public int getTime() {
        return time;
    }

    @Override
    public String toString() {
        return "RTLap{" +
                "carIdentifierNumber=" + carIdentifierNumber +
                ", lap=" + lap +
                ", driverName='" + driverName + '\'' +
                ", carName='" + carName + '\'' +
                ", time=" + time +
                '}';
    }
}
//
#endif
