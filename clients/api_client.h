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
//motion: all values range from -10000 to +10000 as they are mapped server-side
pkt[2] = pl_pitch;  //pitch
pkt[3] = pl_roll;   //roll
pkt[4] = 0;         //yaw
pkt[5] = 0;         //surge
pkt[6] = 0;         //sway
pkt[7] = 0;         //heave
//speed
pkt[8] = 0;         //speed
*/
#define MFC_PKT_SIZE  9
#define MFC_PITYPE    0
//motion
#define MFC_PIDOF     1
#define MFC_PIPITCH   2
#define MFC_PIROLL    3
#define MFC_PIYAW     4
#define MFC_PISURGE   5
#define MFC_PISWAY    6
#define MFC_PIHEAVE   7
//speed
#define MFC_PISPEED   8

//pkt_type
#define PKTT_CTRL   (0x00)
#define PKTT_DATA   (0x01)

//pkt_dof_type
//in 6DOF mode, all 6 forces are available discretly: wheel, ffb, vibration
//  wroll, wpitch, froll, fpitch, vroll, vpitch
#define PKTT_0DOF    (0x00)
#define PKTT_1DOF    (0x01)
#define PKTT_2DOF    (0x02)
#define PKTT_2DOFN   (0x82)
#define PKTT_3DOF    (0x03)
#define PKTT_4DOF    (0x04)
#define PKTT_5DOF    (0x04)
#define PKTT_6DOF    (0x06)
#define PKTT_6DOFN   (0x86)

#define MFCSVR_PORT (64401)

int mfc_bcast_prep (char *dst, int svr);
int mfc_bcast_send ();
int mfc_bcast_receive ();
int *mfc_bcast_pktget ();
int mfc_bcast_pktlen ();
void mfc_bcast_close ();

void bcast_prep ();
void bcast_send ();
void bcast_close ();

long get_map (long x, long in_min, long in_max, long out_min, long out_max);
long get_cmap (long x, long in_min, long in_max, long out_min, long out_max);
float get_map_f (float x, float in_min, float in_max, float out_min, float out_max);

float get_float (char *buf, int off);
int get_int (char *buf, int off);
unsigned int get_uint (char *buf, int off);
unsigned short get_ushort (char *buf, int off);
short get_short (char *buf, int off);

int normal_axis (int val, int max);
int normal_pedal (int val, int max);
int normal_brake (int val, int max);
int normal_accel (int val, int max);
int normal_ffb (int val, int mid);

unsigned long get_millis ();
