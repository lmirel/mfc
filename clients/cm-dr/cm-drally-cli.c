/*
* codemasters f1 dashboard test

gcc -o /opt/mfcc-cmf1 clients/cm-f1/cmf1-dash.c -lrt -std=c11
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

struct UDPPacket
{
    float m_time;
    float m_lapTime;
    float m_lapDistance;
    float m_totalDistance;
    float m_x;      // World space position
    float m_y;      // World space position
    float m_z;      // World space position
    float m_speed;
    float m_xv;      // Velocity in world space
    float m_yv;      // Velocity in world space
    float m_zv;      // Velocity in world space
    float m_xr;      // World space right direction
    float m_yr;      // World space right direction
    float m_zr;      // World space right direction
    float m_xd;      // World space forward direction
    float m_yd;      // World space forward direction
    float m_zd;      // World space forward direction
    float m_susp_pos_bl;
    float m_susp_pos_br;
    float m_susp_pos_fl;
    float m_susp_pos_fr;
    float m_susp_vel_bl;
    float m_susp_vel_br;
    float m_susp_vel_fl;
    float m_susp_vel_fr;
    float m_wheel_speed_bl;
    float m_wheel_speed_br;
    float m_wheel_speed_fl;
    float m_wheel_speed_fr;
    float m_throttle;
    float m_steer;
    float m_brake;
    float m_clutch;
    float m_gear;
    float m_gforce_lat;
    float m_gforce_lon;
    float m_lap;
    float m_engineRate;
    float m_sli_pro_native_support; // SLI Pro support
    float m_car_position;   // car race position
    float m_kers_level;    // kers energy left
    float m_kers_max_level;   // kers maximum energy
    float m_drs;     // 0 = off, 1 = on
    float m_traction_control;  // 0 (off) - 2 (high)
    float m_anti_lock_brakes;  // 0 (off) - 1 (on)
    float m_fuel_in_tank;   // current fuel mass
    float m_fuel_capacity;   // fuel capacity
    float m_in_pits;    // 0 = none, 1 = pitting, 2 = in pit area
    float m_sector;     // 0 = sector1, 1 = sector2; 2 = sector3
    float m_sector1_time;   // time of sector1 (or 0)
    float m_sector2_time;   // time of sector2 (or 0)
    float m_brakes_temp[4];   // brakes temperature (centigrade)
    float m_wheels_pressure[4];  // wheels pressure PSI
    float m_team_info;    // team ID 
    float m_total_laps;    // total number of laps in this race
    float m_track_size;    // track size meters
    float m_last_lap_time;   // last lap time
    float m_max_rpm;    // cars max RPM, at which point the rev limiter will kick in
    float m_idle_rpm;    // cars idle RPM
    float m_max_gears;    // maximum number of gears
    float m_sessionType;   // 0 = unknown, 1 = practice, 2 = qualifying, 3 = race
    float m_drsAllowed;    // 0 = not allowed, 1 = allowed, -1 = invalid / unknown
    float m_track_number;   // -1 for unknown, 0-21 for tracks
    float m_vehicleFIAFlags;  // -1 = invalid/unknown, 0 = none, 1 = green, 2 = blue, 3 = yellow, 4 = red
 };

#define UDP_MAX_PACKETSIZE  256
#define UDP_PORT            20777

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
        //profiling params
        case 'r': //roll %
          _rollprc = atoi (argv[i]+2);
          if (_rollprc < 0 || _rollprc > 100)
            _rollprc = 100;
          break;
        case 'p': //pitch %
          _pitchprc = atoi (argv[i]+2);
          if (_pitchprc < 0 || _pitchprc > 100)
            _pitchprc = 100;
          break;
        case 'l': //listen port
          lport = atoi (argv[i]+2);
          break;
      }
  }
  //configuration summary
  printf ("\n# ##");
  printf ("\n#MFC CM Dirt Rally client");
  printf ("\n#running configuration:");
  printf ("\n#   roll feedback %d%% (-r%d) range [0..100]%%", _rollprc, _rollprc);
  printf ("\n#  pitch feedback %d%% (-p%d) range [0..100]%%", _pitchprc, _pitchprc);
  printf ("\n#     client port %d (-l%d)", lport, lport);
  //printf ("\n#     server port %d", MFCSVR_PORT);
  printf ("\n# verbosity level %d (-d%d)", _odbg, _odbg);
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
  int s, i, slen = sizeof(si_other), pktk = 0;
  if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
  {
    printf("socket\n");
    exit(1);
  }
  int broadcast=1;
  setsockopt(s, SOL_SOCKET, SO_BROADCAST,
          &broadcast, sizeof (broadcast));
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
/* sender */
  memset((char *) &si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons(65001);
  pktk = 990;
#if 0
  if (inet_aton("127.0.0.1", &si_other.sin_addr)==0) {
    fprintf(stderr, "inet_aton() failed\n");
    exit(1);
  }
#endif
  printf ("\n#i:>%d:listening on port %d", s, lport);
  //ctime_ms (1);
  //learning values
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
  int *pkt = mfc_bcast_pktget ();
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
        if (0)
          printf("\r\n@%lums received %dB packet (vs %d) from %s:%d <",
                  ctime_ms(0) - lts, rlen, UDP_MAX_PACKETSIZE, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
        lts = ctime_ms(0);
        //
        //float m_gforce_lat; //34x4 - roll
        //float m_gforce_lon; //35x4 - pitch
        float fv[4];
        idx = 136;
        fv[0] = get_float (packetBuffer, idx);
        //fv[1] = 0.0f;//get_float (packetBuffer, idx + 4);
        fv[1] = get_float (packetBuffer, idx + 4);
        fv[2] = 0.0f;//get_float (packetBuffer, idx + 8);
        //fv[2] = get_float (packetBuffer, idx + 8);
        fv[3] = 0.0f;//get_float (packetBuffer, idx + 12);
        //fv[3] = get_float (packetBuffer, idx + 12);
        if (0)
          printf ("\n#i:telem %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f",
            fv[0]>=0.0f?" +":" ", fv[0], fv[1]>=0.0f?" +":" ", fv[1], fv[2]>=0.0f?" +":" ", fv[2], fv[3]>=0.0f?" +":" ", fv[3]);
/**
A. right axis composition
a. g-force - longitudinal(pitch) - overwrite
b. g-force - lateral(roll) - add

B. right axis composition
a. g-force - longitudinal(pitch) - overwrite
b. g-force - lateral(roll) * invert - add

*/
        if (_odbg)
          printf ("\n#i:%04d:telem %s%.8f \t %s%.8f",
            ppkt, fv[1]>0?" +":" ", fv[1], fv[0]>0?" +":" ", fv[0]);
        //llv = fv[2] + fv[0]; //pitch + roll
        //lrv = fv[2] - fv[0]; //pitch - roll
        wpkt[MFC_PIPITCH] = (int)(fv[1] * 1000.0f);
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
#if 0
        float llv = (-1)*fv[1] + fv[0]; //(-)pitch + roll
        float lrv = (-1)*fv[1] - fv[0]; //(-)pitch - roll
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
          printf ("\n#i:telem %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f",
            fv[0]>0?" +":" ", fv[0], fv[1]>0?" +":" ", fv[1], llv>0?" +":" ", llv, lrv>0?" +":" ", lrv);
#endif
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
pkt[8] = (int)lrv;  //speed
*/
        //set 2DOF targets
        if (pkt)
        {
          //
          pkt[MFC_PITYPE] = PKTT_DATA;
          pkt[MFC_PIDOF]  = PKTT_2DOFN;
          //pkt[2] = -5000 + (int)llv;
          //pkt[3] = -5000 + (int)lrv;
          //motion
          pkt[MFC_PIPITCH] = wpkt[MFC_PIPITCH];  //pitch
          pkt[MFC_PIROLL]  = wpkt[MFC_PIROLL];   //roll
          pkt[MFC_PIYAW]   = 0;                  //yaw
          pkt[MFC_PISURGE] = 0;                   //surge
          pkt[MFC_PISWAY]  = 0;                   //sway
          pkt[MFC_PIHEAVE] = 0;                   //heave
          //speed
          pkt[MFC_PISPEED] = 0;                   //speed
          //profiling vars
          // val .. 100
          // X   .. var
          pkt[MFC_PIPITCH] = pkt[MFC_PIPITCH] * _pitchprc / 100;
          pkt[MFC_PIROLL]  = pkt[MFC_PIROLL]  * _rollprc  / 100;
          //
          mfc_bcast_send ();
        }
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
