/*
* pCars dashboard test

gcc -o drallyd drally-dash.c -lrt -std=c11
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

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
/*
struct m_drivers
{
char   d_name[25];
int    d_position;
int    d_lapNumber;
float  d_gapFrom1st; (important but computable)
float  d_lapDistance;
float  d_last_lap_time;
float  d_best_lap_time; (not so important)
enum   d_tyre; (U,X,S,M,H,I,W)
enum   d_status; (RUN, PIT, DNF, DSQ)
};
*/
float get_float (char *buf, int off)
{
  return (float)(buf[off+3]<<24|buf[off+2]<<16|buf[off+1]<<8|buf[off]);
}

unsigned int get_int (char *buf, int off)
{
  return (unsigned int)(buf[off+3]<<24|buf[off+2]<<16|buf[off+1]<<8|buf[off]);
}

unsigned short get_short (char *buf, int off)
{
  return (unsigned short)(buf[off+1]<<8|buf[off]);
}

#if 0

#include <math.h>
#include <inttypes.h>

FILE* fp;
float ctime_ms(char pt, FILE *fp)
{
  double uptime, idle_time;
  /* Read the system uptime and accumulated idle time from /proc/uptime.  */
  rewind (fp);
  fscanf (fp, "%lf %lf\n", &uptime, &idle_time);
  if (pt)
    printf("Uptime = %lf\n", uptime);
  return (uptime*100);
}
#endif

#if 0
#include <sys/sysinfo.h>

int ctime_ms(char pt, int fd)
{
  struct sysinfo info;
  sysinfo(&info);
  if (pt)
    printf("Uptime = %ld\n", info.uptime);
}
#endif

#include <sys/time.h>
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

#if 0
#include <time.h>
#include <math.h>
#include <inttypes.h>

long ctime_ms (char pt)
{
    long            ms; // Milliseconds
    time_t          s;  // Seconds
    struct timespec spec;

    clock_gettime(CLOCK_REALTIME, &spec);

    s  = spec.tv_sec;
    ms = round(spec.tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds
    if (pt)
        printf("Current time: %"PRIdMAX".%03ld seconds since the Epoch\n",
           (intmax_t)s, ms);
    return ms;
}
#endif

#include <sys/socket.h>
#include <netinet/in.h>

int main(int argc, char **argv, char **envp)
{
    struct pollfd fdset[3];
    int nfds = 1;
    int gpio_fd, timeout, rc;
    unsigned int gpio;
    int len;
    int lport = 20777; /* Dirt Rally sends to this port: 20777 */


    if (argc < 1) {
	printf("Usage: pcars-dash [<listen port>]\n\n");
	printf("Reads shared data from game\n");
	exit(-1);
    }
    if (argc > 1) {
	lport = atoi(argv[1]);
    }
    printf("Listening on port: %d\n", lport);
#define POLL_TIMEOUT 5

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
    if (inet_aton("192.168.2.220", &si_other.sin_addr)==0) {
      fprintf(stderr, "inet_aton() failed\n");
      exit(1);
    }
#if 0
        for (i=0; i<NPACK; i++) {
      printf("Sending packet %d\n", i);
      sprintf(buf, "This is packet %d\n", i);
      if (sendto(s, buf, BUFLEN, 0, &si_other, slen)==-1)
        diep("sendto()");
    }
#endif

    //ctime_ms (1);
    //only send 3 PAUSEd packets
    int ppkt = 1;
    long lts = ctime_ms (0);
    while (1) 
    {
      memset((void*)fdset, 0, sizeof(fdset));
      
      fdset[0].fd = s;
      fdset[0].events = POLLIN;

      rc = poll(fdset, nfds, timeout);      

      if (rc < 0) {
          printf("\npoll() failed!\n");
          return -1;
      }
          
      if (rc == 0) {
          //printf(".");
      }
                
      if (fdset[0].revents & POLLIN)
      {
#define BUFLEN 2048
        char buf [BUFLEN];
        int rlen = 0, i;
        if ((rlen = recvfrom(s, (void *)&buf, BUFLEN, 0, (struct sockaddr*)&si_other, &slen))==-1)
        {
          printf("recvfrom() failed\n");
        }
        else
        {
          printf("\r\n@%lums received %dB packet (vs %d) from %s:%d <", 
                ctime_ms(0) - lts, rlen, BUFLEN, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
          lts = ctime_ms(0);
        }
        fflush(stdout);
      }
    }

    close(s);
//    fclose (fp);
    return 0;
}
