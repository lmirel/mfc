
//flags
#define FLGPIO_PFIN  0x10

#define SCNCMD_LEN		16

#define RDTMOC  50  //read timeout cycles, multiplied by sleep below
#define SLTMOC  20  //sleep ms between reads

#define SCNPOS_MIN  0       //full retract
#define SCNPOS_MAX  -10000  //full expand

#define DEFA_VCMD   0x0EA6  //read from 7C04 - velocity while moving: MAX 3750
#define DEFA_ACMD   0x0093  //read from 7C05
#define DEFA_OVCM   0x02EE  //read from 000A - velocity while homing: MIN 750

int scn_get_response (int fd, char *rsp);
int scn_get_pos (int fd);
int scn_set_pos (int fd, int pos);
int scn_set_home (int fd);
int scn_set_son (int fd);
int scn_set_soff (int fd);
int scn_set_vel (int fd, int vel, int acc);
int scn_get_status (int fd);
