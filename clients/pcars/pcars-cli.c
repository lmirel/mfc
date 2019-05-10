/*
* pCars dashboard test

gcc -o /opt/mfcc-pcars2 clients/pcars/pcars-dash.c -lrt -std=c11
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

#include "SMS_UDP_Definitions_v2p5.hpp"

#define DEBUG 0
#define debug_print(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

int inet_aton(const char *cp, struct in_addr *inp);
int usleep(long usec);

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
#include "../api_client.h"
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
int main(int argc, char **argv, char **envp)
{
  struct pollfd fdset[3];
  int nfds = 1;
  int gpio_fd, timeout, rc;
  unsigned int gpio;
  int len;
  int lport = SMS_UDP_PORT; /* Project Cars sends to this port: 5605 */
  
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
        case 'd': //debug enabled
          _odbg++;
          break;
        default:
          printf ("\n#w:unknown option -%c", argv[i][1]);
      }
  }
  //configuration summary 
  printf ("\n# ##");
  printf ("\n#MFC PCARS2 client");
  printf ("\n#running configuration:");
  printf ("\n#  pitch feedback %d%% (-p%d) range [0..100]%%", _pitchprc, _pitchprc);
  printf ("\n#   roll feedback %d%% (-r%d) range [0..100]%%", _rollprc, _rollprc);
  printf ("\n#    yaw feedback %d%% (-y%d) range [0..100]%%", _yawprc, _yawprc);
  printf ("\n#  surge feedback %d%% (-s%d) range [0..100]%%", _surgeprc, _surgeprc);
  printf ("\n#   sway feedback %d%% (-w%d) range [0..100]%%", _swayprc, _swayprc);
  printf ("\n#  heave feedback %d%% (-h%d) range [0..100]%%", _heaveprc, _heaveprc);
  //printf ("\n#     server port %d", MFCSVR_PORT);
  printf ("\n# verbosity level %d (-d%d)", _odbg, _odbg);
  printf ("\n#     client port %d (-l%d)", lport, lport);
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
  if ((s = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
  {
    printf("socket\n");
    exit(1);
  }
  int broadcast = 1;
  setsockopt (s, SOL_SOCKET, SO_BROADCAST,
          &broadcast, sizeof (broadcast));
/* receiver */
  memset((char *) &si_me, 0, sizeof (si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons (lport);
  si_me.sin_addr.s_addr = htonl (INADDR_ANY);
  if (bind (s, (struct sockaddr*)&si_me, sizeof (si_me))==-1)
  {
    fprintf (stderr, "bind() failed\n");
    exit (1);
  }
/* sender */
  memset ((char *) &si_other, 0, sizeof (si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons (65001);
  pktk = 990;
  if (inet_aton ("127.0.0.1", &si_other.sin_addr) == 0) 
  {
    fprintf (stderr, "inet_aton() failed\n");
    exit(1);
  }

  printf ("\n#i:>%d:listening on port %d", s, lport);
  //ctime_ms (1);
  //learning values
  float fv[MFC_PKT_SIZE];
  float lminl, lmaxl, lminr, lmaxr, llv, lrv;
  lminl = lmaxl = lminr = lmaxr = 0.0f;
  //only send 3 PAUSEd packets
  (void) signal(SIGINT, terminate);
  (void) signal(SIGTERM, terminate);
  (void) signal(SIGHUP, terminate);
  //
  int rlen = 0, idx;
  int ppkt = 0;
  long lts = ctime_ms (0);
  char packetBuffer[SMS_UDP_MAX_PACKETSIZE];
  PacketBase packetHeader;
  memset(&packetHeader, 0, sizeof( PacketBase ));
  sParticipantsData pData;
  sParticipantsData pData2;
  sParticipantVehicleNamesData pVehicles;
  sParticipantVehicleNamesData pVehicles2;
  sVehicleClassNamesData pClasses;
  sGameStateData		stateData;
  sTelemetryData  sTelem;

  memset(&pData, 0, sizeof( sParticipantsData ));
  memset(&pData2, 0, sizeof( sParticipantsData ));
  memset(&stateData, 0, sizeof( sGameStateData ));
  memset(&pVehicles, 0, sizeof( sParticipantVehicleNamesData ));
  memset(&pVehicles2, 0, sizeof( sParticipantVehicleNamesData ));
  memset(&pClasses, 0, sizeof( sVehicleClassNamesData ));
  memset(&sTelem, 0, sizeof( sTelemetryData ));
  //int wpkt[MFC_PKT_SIZE] = {0};
  int mpkt[MFC_PKT_SIZE] = {0, 0, -1650, -1050, -2550, -1650, -1200, -1050, 0};
  int Mpkt[MFC_PKT_SIZE] = {1, 1,  2350,  3450,  2650,  2350,  1200,  3450, 1};
  //
  int *_cpkt = mfc_bcast_pktget ();
  int pktl = mfc_bcast_pktlen ();
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
      usleep (1000000);
    }
              
    if (fdset[0].revents & POLLIN)
    {
      char participantsReceived = 0;
      char participantsReceived2 = 0;
      char stateReceived = 0;
      char vehiclesReceived = 0;
      char vehiclesReceived2 = 0;
      char telemReceived = 0;
      //recvfrom(RecvSocket,  packetBuffer , SMS_UDP_MAX_PACKETSIZE, 0, (SOCKADDR *) & Sender, &SenderAddrSize);
      //if ((rlen = recvfrom (s, (void *)&packetBuffer, SMS_UDP_MAX_PACKETSIZE, 0, (struct sockaddr*)&si_other, &slen))==-1)
      if ((rlen = recvfrom (s, (void *)&packetBuffer, SMS_UDP_MAX_PACKETSIZE, 0, NULL, NULL))==-1)
      {
        printf("\n#w:recvfrom() failed.");
      }
      else
      {
        ++ppkt;
        if (0)
          printf("\r\n@%lums received %dB packet (vs %d) from %s:%d <", 
                  ctime_ms(0) - lts, rlen, SMS_UDP_MAX_PACKETSIZE, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
        lts = ctime_ms(0);
        memcpy(&packetHeader, packetBuffer, sizeof( PacketBase ));
        //
        if ((ppkt % 500) == 0)
          printf ("\n#i:received %dpkts", ppkt);
        //printf ("\n#pkt type %d", packetHeader.mPacketType);
        switch ( packetHeader.mPacketType )
        {
            case eCarPhysics :
            {
              //float							sSuspensionTravel[4];							// 312 16
              //~0.080 still
              //- 0 front LT:
              //- 1 front RT:
              //- 2  back LT:
              //- 3  back RT:
              //float							sOrientation[3];									// 52 12
              //float							sAngularVelocity[3];							// 88 12
              //- 1  roll: +LT
              //float							sLocalVelocity[3];								// 64 12
              //- 0  roll: +LT,  -RT
              //- 2 pitch: +ACC, -BRK
              //float							sLocalAcceleration[3];						// 100 12
              // tenths
              //- 0  roll: +LT,  -RT
              //- 2 pitch: -ACC, +BRK
              //#i:telem  -7.79411411     -1.03792381     -9.79688072     0.00000000
              //#i:telem  -6.75643492     -1.22993863     -10.73717594    0.00000000
              //#i:telem  +12.45860386    +0.02436667     +3.99445176     0.00000000
              //#i:telem  +14.37643242    +0.14723888     +4.68823385     0.00000000
              //#i:telem  +12.37826157    -0.06973051     +7.29721689     0.00000000
              //float							sSpeed;														// 36 4
#if 1
/*
Pitch is the tilt of the car forwards or backwards in [°]
Roll is how much the car is dipped to the left or right in [°]
Yaw is the heading of the car (north, east, south, west) in [°]

Surge means the acceleration of the car in longitudinal direction [g]
Sway means the acceleration of the car in lateral direction [g]
Heave means the acceleration up and down [g]
			float							sOrientation[3];									// 52 12
			float							sLocalVelocity[3];								// 64 12
			float							sWorldVelocity[3];								// 76 12
			float							sAngularVelocity[3];							// 88 12
			float							sLocalAcceleration[3];						// 100 12
			float							sWorldAcceleration[3];						// 112 12
* Yaw, roll and pitch inputs from the Orientation values
* Sway, Surge, and Heave inputs we get from the Local Acceleration values
* Traction loss happens when the car is not pointing in the direction of travel.
Most of the time it takes math to get this answer.
But with PCars, it provides this info in one of the the LocalVelocity outputs I believe
**Most likely what you will want to run the sim with is Sway and Surge.
As Pitch and Roll will just tip the sim with the current angles of the track.
Where Sway and Surge will let you feel the gforces produced by the car.
--
example from LFS V3_Dash\AdditionPlugin\Plugin.vb: 186
With MyOutsim_Internal
    Roll_Output = (.sngOrientation2 * 180 / 3.14159)
    Pitch_Output = (.sngOrientation1 * 180 / 3.14159) * -1
    Heave_Output = (System.Math.Cos(.sngOrientation2) * .sngAcceleration2)
    Yaw_Output = (.sngOrientation0 * 180 / 3.14159)
    Sway_Output = ((System.Math.Cos(.sngOrientation0) * .sngAcceleration0) + (System.Math.Sin(.sngOrientation0) * .sngAcceleration1))
    Surge_Output = ((-System.Math.Sin(.sngOrientation0) * .sngAcceleration0) + (System.Math.Cos(.sngOrientation0) * .sngAcceleration1))
    Extra1_Output = (((System.Math.Sin(.sngOrientation0) * .sngAcceleration0) + (System.Math.Sin(.sngOrientation0) * .sngAcceleration1)) * -1)
End With
*/
#if 0
              idx = 100;//float							sLocalAcceleration[3];						// 100 12
              fv[0] = get_float (packetBuffer, idx);  //roll
              fv[1] = 0.0f;//get_float (packetBuffer, idx + 4);
              //fv[1] = get_float (packetBuffer, idx + 4);
              //fv[2] = 0.0f;//get_float (packetBuffer, idx + 8);
              fv[2] = get_float (packetBuffer, idx + 8);  //pitch
              fv[3] = 0.0f;//get_float (packetBuffer, idx + 12);
              //fv[3] = get_float (packetBuffer, idx + 12);
#endif
  if (_cpkt)
  {
    //motion data packet
    _cpkt[MFC_PITYPE] = PKTT_DATA;
    _cpkt[MFC_PIDOF]  = PKTT_2DOFN;
    //motion
    #if 1
#define ORIENTATION_IDX 52
#define LOCAL_ACCEL_IDX 100
#define LOCAL_VELOC_IDX 64
    //
    fv[MFC_PIPITCH] = get_float (packetBuffer, ORIENTATION_IDX + 8);
    _cpkt[MFC_PIPITCH]  = (int)(fv[MFC_PIPITCH] * 100.0f);
    //_cpkt[MFC_PIPITCH] = -get_cmap (pf_pitch, -100, 100, -1000, 1000);
    //
    fv[MFC_PISURGE] = get_float (packetBuffer, LOCAL_ACCEL_IDX + 8);
    _cpkt[MFC_PISURGE]  = (int)(fv[MFC_PISURGE] * 100.0f);
    //_cpkt[MFC_PISURGE] = -get_cmap (pw_pitch, -32800, 32800, -7000, 7000);
    //
    fv[MFC_PIHEAVE] = get_float (packetBuffer, LOCAL_ACCEL_IDX + 4);
    _cpkt[MFC_PIHEAVE]  = (int)(fv[MFC_PIHEAVE] * 100.0f);
    //_cpkt[MFC_PIHEAVE] = -get_cmap (pv_pitch, -128, 128, -2000, 2000);
    //
    fv[MFC_PIROLL] = get_float (packetBuffer, ORIENTATION_IDX + 4);
    _cpkt[MFC_PIROLL]  = (int)(fv[MFC_PIROLL] * 100.0f);
    //_cpkt[MFC_PIROLL]  = get_cmap (pf_roll, -128, 128, -7000, 7000);
    //
    fv[MFC_PISWAY] = get_float (packetBuffer, LOCAL_VELOC_IDX);
    _cpkt[MFC_PISWAY]  = (int)(fv[MFC_PISWAY] * 100.0f);
    //_cpkt[MFC_PISWAY]  = get_cmap (get_float (packetBuffer, local_accel_idx)*100, 8200, 8200, -3000, 3000);
    //
    fv[MFC_PIYAW] = get_float (packetBuffer, ORIENTATION_IDX);
    _cpkt[MFC_PIYAW]  = (int)(fv[MFC_PIYAW] * 100.0f);
    //_cpkt[MFC_PIYAW]   = get_cmap (pw_roll, -16400, 16400, -10000, 10000);
    //
    if (0)
      printf ("\n#i:telem %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f",
        fv[MFC_PIPITCH]>=0.0f?" +":" ", fv[MFC_PIPITCH], fv[MFC_PISURGE]>=0.0f?" +":" ", fv[MFC_PISURGE],
        fv[MFC_PIHEAVE]>=0.0f?" +":" ", fv[MFC_PIHEAVE], fv[MFC_PIROLL]>=0.0f?" +":" ", fv[MFC_PIROLL], 
        fv[MFC_PISWAY]>=0.0f?" +":" ", fv[MFC_PISWAY], fv[MFC_PIYAW]>=0.0f?" +":" ", fv[MFC_PIYAW]);
    if (0||_odbg)
      printf ("\n#i:pitch \t%d \t%d \t%d \troll \t%d \t%d \t%d", 
        _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY], _cpkt[MFC_PIYAW]);
    //
    if (_odbg)
      printf ("\n#i:pitch%% \t%d \t%d \t%d \troll%% \t%d \t%d \t%d", 
        _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE], _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY], _cpkt[MFC_PIYAW]);
    //
    if (1)//disable automax
    {
      int _newrange = 0;
      for (int i = MFC_PIDOF + 1; i < MFC_PIDOF + 7; ++i)
      {
        if (_cpkt[i] < mpkt[i])
        {
          //mpkt[i] = _cpkt[i];
          //_newrange = 1;
          printf ("\n#E:%d# [%d.. %d ..%d]", i, mpkt[i], _cpkt[i], Mpkt[i]);
        }
        if (_cpkt[i] > Mpkt[i])
        {
          //Mpkt[i] = _cpkt[i];
          //_newrange = 1;
          printf ("\n#E:%d# [%d.. %d ..%d]", i, mpkt[i], _cpkt[i], Mpkt[i]);
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
    }
    //adjust here for a total max of 10000/-10000 for pitch and roll
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
/**
              //float							sLocalAcceleration[3];						// 100 12
              // tenths
              //- 0  roll: +LT,  -RT
              //- 2 pitch: -ACC, +BRK

              float roll_out, pitch_out, heave_out, yaw_out, sway_out, surge_out;
              float ori0, ori1, ori2; //orientation/direction of travel
              float acc0, acc1, acc2; //acceleration of travel
              roll_out  = (((cos (ori0) * acc0) + (sin (ori0) * acc1)) * -1);
              pitch_out = (((-sin (ori0) * acc0) + (cos (ori0) * acc1)) * -1);
              heave_out = (cos (ori2) * acc2);
              yaw_out   = (((sin (ori0) * acc0) + (sin (ori0) * acc1)) *-1);
              sway_out  = ((cos (ori0) * acc0) + (sin (ori0) * acc1));
              surge_out = ((-sin (ori0) * acc0) + (cos (ori0) * acc1));

              A. right axis composition
              a. g-force - longitudinal/pitch - overwrite
              b. g-force - lateral - add

              B. left axis composition
              a. g-force - longitudinal/pitch - overwrite
              b. g-force - lateral (invert) - add

*/
#if 0
              if (_odbg)
                printf ("\n#i:%04d:telem %s%.8f \t %s%.8f", 
                  ppkt, fv[2]>0?" +":" ", fv[2], fv[0]>0?" +":" ", fv[0]);
              //llv = fv[2] + fv[0]; //pitch + roll
              //lrv = fv[2] - fv[0]; //pitch - roll
              wpkt[MFC_PIPITCH] = (int)(fv[2] * 1000.0f);
              //printf (" \t %06d ", wpkt[MFC_PIPITCH]);
              //range: min .. Max
              if (wpkt[MFC_PIPITCH] < mpkt[MFC_PIPITCH])
                mpkt[MFC_PIPITCH] = wpkt[MFC_PIPITCH];
              if (wpkt[MFC_PIPITCH] > Mpkt[MFC_PIPITCH])
                Mpkt[MFC_PIPITCH] = wpkt[MFC_PIPITCH];
              wpkt[MFC_PIPITCH] = (int)get_map (wpkt[MFC_PIPITCH], mpkt[MFC_PIPITCH], Mpkt[MFC_PIPITCH], -10000, 10000);
              //
              wpkt[MFC_PIROLL]  = (int)(fv[0] * 1000.0f);
              //printf (" \t %06d ", wpkt[MFC_PIROLL]);
              //range: min .. Max
              if (wpkt[MFC_PIROLL] < mpkt[MFC_PIROLL])
                mpkt[MFC_PIROLL] = wpkt[MFC_PIROLL];
              if (wpkt[MFC_PIROLL] > Mpkt[MFC_PIROLL])
                Mpkt[MFC_PIROLL] = wpkt[MFC_PIROLL];
              wpkt[MFC_PIROLL]  = (int)get_map (wpkt[MFC_PIROLL], mpkt[MFC_PIROLL], Mpkt[MFC_PIROLL], -10000, 10000);
              //
              if (_odbg)
                printf ("\n#i:%04d:Mtelem [%06d \t %06d \t %06d] \t [%06d \t %06d \t %06d]", 
                  ppkt, 
                  mpkt[MFC_PIPITCH], wpkt[MFC_PIPITCH], Mpkt[MFC_PIPITCH],
                  mpkt[MFC_PIROLL],  wpkt[MFC_PIROLL],  Mpkt[MFC_PIROLL]);
#endif
#if 0
              //autoadjust min
              if (lminl > llv)
                lminl = llv;
              if (lmaxl < llv)
                lmaxl = llv;
              //autoadjust max
              if (lminr > lrv)
                lminr = lrv;
              if (lmaxr < lrv)
                lmaxr = lrv;
              //map min/max on actuator range
              llv = get_map_f (llv, lminl, lmaxl, -10000.0f, 0.0f);
              lrv = get_map_f (lrv, lminr, lmaxr, -10000.0f, 0.0f);
              //smooth curve
              int ld = 50;
              llv -= (int)llv%ld;
              lrv -= (int)lrv%ld;
              //
              if (_odbg)
                printf ("\n#i:%04d:telem %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f", 
                  ppkt, fv[0]>0?" +":" ", fv[0], fv[2]>0?" +":" ", fv[2], llv>0?" +":" ", llv, lrv>0?" +":" ", lrv);
#endif
#else //use suspention travel ;)
              idx = 312;//float							sSuspensionTravel[4];							// 312 16
              fv[0] = get_float (packetBuffer, idx);
              //fv[1] = 0.0f;//get_float (packetBuffer, idx + 4);
              fv[1] = get_float (packetBuffer, idx + 4);
              fv[2] = 0.0f;//get_float (packetBuffer, idx + 8);
              //fv[2] = get_float (packetBuffer, idx + 8);
              fv[3] = 0.0f;//get_float (packetBuffer, idx + 12);
              //fv[3] = get_float (packetBuffer, idx + 12);
              //printf ("\n#i:telem %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f", fv[0]>0?" +":" ", fv[0], fv[1]>0?" +":" ", fv[1], fv[2]>0?" +":" ", fv[2], fv[3]>0?" +":" ", fv[3]);
              //
              float llv = fv[0];
              float lrv = fv[1];
              llv -= 0.080f;
              lrv -= 0.080f;
              //
              llv *= 100000.0f;
              lrv *= 100000.0f;
              //
              llv *= (-1);
              lrv *= (-1);
              //autoadjust min
              if (lminl > llv)
                lminl = llv;
              if (lmaxl < llv)
                lmaxl = llv;
              //autoadjust max
              if (lminr > lrv)
                lminr = lrv;
              if (lmaxr < lrv)
                lmaxr = lrv;
              //map min/max on actuator range
              llv = get_map_f (llv, lminl, lmaxl, -10000.0f, 0.0f);
              lrv = get_map_f (lrv, lminr, lmaxr, -10000.0f, 0.0f);
              //smooth curve
              int ld = 10;
              llv -= (int)llv%ld;
              lrv -= (int)lrv%ld;
              //
              printf ("\n#i:telem %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f", fv[0]>0?" +":" ", fv[0], fv[1]>0?" +":" ", fv[1], llv>0?" +":" ", llv, lrv>0?" +":" ", lrv);
#endif
              //
              break;
            }
            #if 0
            case eParticipants :
            if (packetHeader.mPartialPacketIndex == 1)
            {
              memcpy(&pData, packetBuffer, sizeof( sParticipantsData ));
            }
            if (packetHeader.mPartialPacketIndex == 2)
            {
              memcpy(&pData2, packetBuffer, sizeof( sParticipantsData ));
              participantsReceived2 = 1;
            }
            if (packetHeader.mPartialPacketIndex == packetHeader.mPartialPacketNumber)
            {
              participantsReceived = 1;
            }

            break;
          case eGameState :
            memcpy(&stateData, packetBuffer, sizeof( sGameStateData ));
            stateReceived = 1;
            break;
          case eParticipantVehicleNames :
          {
            //last packet are always the vehicle class names
            if (packetHeader.mPartialPacketIndex == packetHeader.mPartialPacketNumber)
            {
              memcpy(&pClasses, packetBuffer, sizeof( sVehicleClassNamesData ));
              vehiclesReceived = 1;
            }
            else
            {
              if (packetHeader.mPartialPacketIndex == 1)
              {
                memcpy(&pVehicles, packetBuffer, sizeof( sParticipantVehicleNamesData ));
              }
              if (packetHeader.mPartialPacketIndex == 2)
              {
                memcpy(&pVehicles2, packetBuffer, sizeof( sParticipantVehicleNamesData ));
              }
            }
          }
          #endif
          default: break;
        }
        //
        #if 0
        if (telemReceived)
        {
          //printf ("\n#i:telem RPM %d", ntohs(sTelem.sRpm));
        }
        //
        if (stateReceived)
        {
          int gameState = stateData.mGameState & 7;
          int sessionState = stateData.mGameState >> 4;

          printf(" Game State %i, gameState  %i, sessionState %i \n", stateData.mGameState, gameState, sessionState );
          printf(" Race Participants  \n");
          if (participantsReceived)
          {
            for (int i=0;i<PARTICIPANTS_PER_PACKET;++i)
            {
              if (pData.sName[i][0] != '\0')
              {
                printf(" Name %S \n",pData.sName[i]);
              }
            }
            if (participantsReceived2)
            {
              for (int i=0;i<PARTICIPANTS_PER_PACKET;++i)
              {
                if (pData2.sName[i][0] != '\0')
                {
                  printf(" Name %S \n",pData2.sName[i]);
                }
              }
            }
          }
          if (vehiclesReceived)
          {
            printf("Vehicle Names\n");
            for (int i=0;i<VEHICLES_PER_PACKET;++i)
            {
              if (pVehicles.sVehicles[i].sName[0] != '\0')
              {
                printf("Vehicle Name %S, index %d, class %d \n",pVehicles.sVehicles[i].sName[i],pVehicles.sVehicles[i].sIndex, pVehicles.sVehicles[i].sClass);
              }
            }
            if (vehiclesReceived2)
            {
              for (int i=0;i<VEHICLES_PER_PACKET;++i)
              {
                if (pVehicles2.sVehicles[i].sName[0] != '\0')
                {
                  printf("Vehicle Name %S, index %d, class %d \n",pVehicles2.sVehicles[i].sName[i],pVehicles2.sVehicles[i].sIndex, pVehicles2.sVehicles[i].sClass);
                }
              }
            }
            printf("Class Names\n");
            for (int i=0;i<CLASSES_SUPPORTED_PER_PACKET;++i)
            {
              if (pClasses.sClasses[i].sName[0] != '\0')
              {
                printf("Class Name %S, index %d`\n",pClasses.sClasses[i].sName,pClasses.sClasses[i].sClassIndex);
              }
            }
          }
        }
        #endif
        //printf ("\r\n");
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
