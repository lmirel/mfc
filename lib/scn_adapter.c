/*
gcc -o scn6t scn_adapter.c -DTRUN -std=c11
*/

/*
travel distance is length x INP(default is 04h for SCN6)

1. use homing patern 07h. home position is 0 and extends negatively until -10000 (safe limit)
2. use absolute positioning between 0 and -10000
3. have fun!

N1: needs homing after each power off, otherwise might not react properly
N2: a movement response may respond with PFIN but the actual move PFIN needs to be verified from status
**N3: it seems that for position at -5000 (middle), the SCN times-out/doesn't respond

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

max extend
1:
#<cmd '0R40000740008F'
#>rsp(16b in 16k)U0R4\0000165B\57
#i:pos 0x0000165B / 5723
#i:run cmd '0mFFFF5BF00000'
#<cmd '0mFFFF5BF000FE'
#>rsp(16b in 16k)U0m0\70080900\46
#<cmd '0R40000740008F'
#>rsp(16b in 16k)U0R4\00000AE9\46
#i:pos 0x00000AE9 / 2793
#<cmd '0R40000740008F'
#>rsp(16b in 16k)U0R4\FFFFFE59\E4
#i:pos 0xFFFFFE59 / -423
#<cmd '0R40000740008F'
#>rsp(16b in 17k)U0R4\FFFFF1C9\EA
#i:pos 0xFFFFF1C9 / -3639
#<cmd '0R40000740008F'
#>rsp(16b in 16k)U0R4\FFFFEE1E\DD
#i:pos 0xFFFFEE1E / -4578
#<cmd '0R40000740008F'
#>rsp(16b in 16k)U0R4\FFFFEE1E\DD
#i:pos 0xFFFFEE1E / -4578

0:
#<cmd '0R40000740008F'
#>rsp(16b in 17k)U0R4\FFFFFF6C\D8
#i:pos 0xFFFFFF6C / -148
#i:run cmd '0mFFFFD8F00000'
#<cmd '0mFFFFD8F000F9'
#>rsp(16b in 16k)U0m0\70080900\46
#<cmd '0R40000740008F'
#>rsp(16b in 16k)U0R4\FFFFFF5E\D7
#i:pos 0xFFFFFF5E / -162

max retract
1:
#<cmd '0R40000740008F'
#>rsp(16b in 16k)U0R4\00001657\62
#i:pos 0x00001657 / 5719
#i:run cmd '0m0000EE1E0000'
#<cmd '0m0000EE1E0043'
#>rsp(16b in 16k)U0m0\70080800\47
#<cmd '0R40000740008F'
#>rsp(16b in 16k)U0R4\0000165D\55
#i:pos 0x0000165D / 5725

0:
#<cmd '0R40000740008F'
#>rsp(16b in 16k)U0R4\FFFFFF68\E3
#i:pos 0xFFFFFF68 / -152
#i:run cmd '0m00009E740000'
#<cmd '0m00009E74005A'
#>rsp(16b in 16k)U0m0\70080900\46
#<cmd '0R40000740008F'
#>rsp(16b in 16k)U0R4\00000AD4\4C
#i:pos 0x00000AD4 / 2772
#<cmd '0R40000740008F'
#>rsp(16b in 16k)U0R4\00001777\5F
#i:pos 0x00001777 / 6007
#<cmd '0R40000740008F'
#>rsp(16b in 17k)U0R4\00002417\67
#i:pos 0x00002417 / 9239
#<cmd '0R40000740008F'
#>rsp(16b in 16k)U0R4\0000279D\4F
#i:pos 0x0000279D / 10141
#<cmd '0R40000740008F'
#>rsp(16b in 16k)U0R4\0000279D\4F
#i:pos 0x0000279D / 10141


1) STX0T400000400094ETX This will write to add. H0400 (PCMD)
2) STX0W4FFFFE57D008ETX Change command position #1 to -6787 pulses
3) STX0T400000403091ETX This will write to add. H0403 (INP)
4) STX0W400000475085ETX Change pusher movement to 1141 pulses
5) STX0T40000040608EETX This will write to add. H0406 (SPOW)
6) STX0W40000005A07FETX Change pusher force command to 50%
7) STX0V501010000093ETX Write to memory for position area point #01
*/
//STX0T400000400094ETX

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include <termios.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>

#define DEBUG 0
#define debug_print(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

#include "scn_adapter.h"

//servo on/off
char cmd_on[] = {0x02, 0x30, 'q', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '7', 'E', 0x03, 0x00};
char cmd_of[] = {0x02, 0x30, 'q', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '7', 'F', 0x03, 0x00};
//increment
char cmd_in[] = {0x02, 0x30, 'm', '0', '0', '0', '0', '0', '3', 'E', '8', 0x30, 0x30, 0x30, 0x30, 0x03, 0x00};
char cmd_ou[] = {0x02, 0x30, 'm', '8', '0', '0', '0', '0', '3', 'E', '8', 0x30, 0x30, 0x30, 0x30, 0x03, 0x00};
//absolute positioning
char cmd_po[] = {0x02, 0x30, 'a', '0', '0', '0', '0', '0', '0', '0', '0', 0x30, 0x30, 0x30, 0x30, 0x03, 0x00};
//read status
char cmd_st[] = {0x02, 0x30, 'n', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '8', '2', 0x03, 0x00};
//emergency stop
char cmd_es[] = {0x02, 0x30, 'd', '0', '0', '0', '0', '0', '0', '0', '0', 0x30, 0x30, 0x30, 0x30, 0x03, 0x00};
//read position: 0n 00007400(monitor) or 00007C00 (exec)
char cmd_rp[] = {0x02, 0x30, 'R', '4', '0', '0', '0', '0', '7', '4', '0', '0', '0', '8', 'F', 0x03, 0x00};
//action command
char cmd_ac[] = {0x02, 0x30, '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 0x03, 0x00};
//speed/accel
//-param bank0
//#i:3<0R40000000E085 >VCMD - velocity
//#i:3>U0R40000.0EA6.49(16b in 320ms/16c)
//#i:3<0R40000000F084 >ACMD - acceleration
//#i:3>U0R40000.00DD.4D(16b in 340ms/17c) > 221
//#i:3<0R400000010099 >SPOW
//#i:3>U0R40000003C5F(16b in 320ms/16c)
//#i:3<0R40000000A089 >OVCM - velocity while homing
//#i:3>U0R40000.02EE.49(16b in 320ms/16c) >750
//-exec bank31
//#i:3<0R400007C0407C >VCMD
//#i:3>U0R40000.0EA6.49(16b in 320ms/16c) >3750
//#i:3<0R400007C0507B >ACMD
//#i:3>U0R40000.0093.69(16b in 320ms/16c) >147
char cmd_sa[] = {0x02, 0x30, 'v', '2', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 0x03, 0x00};
//jog extend - 50 units
char cmd_je[] = {0x02, 0x30, 'j', 'F', 'F', 'F', 'F', 'F', 'F', '3', '2', '0', '0', '0', '0', 0x03, 0x00};
//jog retract - 50 units
char cmd_jr[] = {0x02, 0x30, 'j', '0', '0', '0', '0', '0', '0', '3', '2', '0', '0', '0', '0', 0x03, 0x00};
//increment extend - 50 units
char cmd_me[] = {0x02, 0x30, 'm', 'F', 'F', 'F', 'F', 'F', 'F', '3', '2', '0', '0', '0', '0', 0x03, 0x00};
//increment retract - 50 units
char cmd_mr[] = {0x02, 0x30, 'm', '0', '0', '0', '0', '0', '0', '3', '2', '0', '0', '0', '0', 0x03, 0x00};
//homing
//#i:3<0o07000000007A
//#i:3>U0o070080F0037(16b in 320ms/16c)
char cmd_ho[] = {0x02, 0x30, 'o', '0', '7', '0', '0', '0', '0', '0', '0', '0', '0', '7', 'A', 0x03, 0x00};
//counter clockwise homing?!
//char cmd_ho[] = {0x02, 0x30, 'o', '0', '8', '0', '0', '0', '0', '0', '0', '0', '0', '7', 'A', 0x03, 0x00};

int cmdl = SCNCMD_LEN;

void cfmakeraw(struct termios *termios_p);
//int usleep(long usec);
int scn_test (int fd);

char s_port[] = "/dev/ttyUSB0";
char *mcmd = NULL;

#ifdef TRUN
int main (int argc, char **argv)
{
  //strcpy (s_port, "/dev/ttyUSB0");
  if (argc > 1)
    s_port[11] = argv[1][0];
  printf ("\r\n#i: using port '%s'", s_port);
  if (argc > 2)
  {
    mcmd = argv[2];
    printf ("\r\n#i:command '%s'", mcmd);
  }
  //
  int fd1 = open (s_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd1 == -1)
  {
    perror (s_port);
    return 1;
  }
  struct termios options;
  tcgetattr(fd1, &options);
  cfsetispeed(&options, B9600);
  cfsetospeed(&options, B9600);
  cfmakeraw(&options);
  if (tcsetattr(fd1, TCSANOW, &options) < 0)
  {
    printf("\n#can't set serial port options");
    close(fd1);
    return 2;
  }
  else
  {
    printf("\n#connected to '%s', on %d", s_port, fd1);
  }
  //
  scn_test (fd1);
  //
  close (fd1);
  printf ("\r\n#cleaning up, done.\r\n");
  //
  return 0;
}
#endif

int scn_get_crc (char *pl)
{
  int i;
  int s = 0;
  pl++;  //start counting from the 2nd byte
  for (i = 0; i < 12; i++)
  {
    s += *(pl + i);
  }
  //printf ("\r\n#i sum %x", s);
  s = s & 0xff;
  //printf ("\r\n#i del 8-11 %x", s);
  s = ~s; s = s & 0xff;
  //printf ("\r\n#i inv %x", s);
  s += 1;
  //printf ("\r\n#i plus 1 %x", s);
  //
  //char ccrc[5];
  //sprintf (ccrc, "%02X", s);
  //printf ("\r\n#i CRC is '%s'", ccrc);
  //
  return s;
}

/**
 * hex2int
 * take a hex string and convert it to a 32bit number (max 8 hex digits)
 */
int hex2int (char *hex) 
{
    int val = 0;
    for (int i = 0; i < 8; i++) {
        // get current character then increment
        char byte = *hex++; 
        // transform hex character to the 4bit equivalent number, using the ascii table indexes
        if (byte >= '0' && byte <= '9') byte = byte - '0';
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;    
        // shift 4 to make space for new digit, and add the 4 bits of the new digit 
        val = (val << 4) | (byte & 0xF);
    }
    //printf ("\n#i:hex to int 0x%08X/%d", val, val);
    return val;
}

/**
 * hex2byte
 * take a hex string and convert it to a byte number (max 2 hex digits)
 */
int hex2byte (char *hex) 
{
    int val = 0;
    for (int i = 0; i < 2; i++) {
        // get current character then increment
        char byte = *hex++; 
        // transform hex character to the 4bit equivalent number, using the ascii table indexes
        if (byte >= '0' && byte <= '9') byte = byte - '0';
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;    
        // shift 4 to make space for new digit, and add the 4 bits of the new digit 
        val = (val << 4) | (byte & 0xF);
    }
    val &= 0xff;
    //printf ("\n#i:hex2byte 0x%02X/%d", val, val);
    return val;
}

int scn_get_response (int fd, char *rsp)
{
  int rbl = 0;
  int rbk = 0;
  while (rbl < cmdl)
  {
    rbk++;
    int rbb = read (fd, rsp + rbl, cmdl - rbl);
    if (rbb > 0)
      rbl += rbb;
    //50 tries of 20ms wait
    if (rbk > RDTMOC)
    {
      debug_print ("\n#w:%d>TMO reading from SCN", fd);
      return 0;
    }
    usleep (SLTMOC * 1000);//wait 20ms
  }
  rsp[rbl] = 0;
  debug_print ("\n#i:%d>%s(%db in %dms/%dc)", fd, rsp + 1/*skip STX*/, rbl, rbk * SLTMOC, rbk);
  //
  return 1;
}

int scn_get_pfin (int fd)
{
  char resp[20];
  write (fd, cmd_st, cmdl);
  debug_print ("\n#i:%d<%s", fd, cmd_st);
  if (scn_get_response (fd, resp))
  {
    //#>rsp(16b in 320ms/16c)U0n0\70080900\45
    return (hex2byte (resp + 10) & FLGPIO_PFIN);
  }
  return 0;
}

int scn_get_status (int fd)
{
  char resp[20];
  write (fd, cmd_st, cmdl);
  debug_print ("\n#i:%d<%s", fd, cmd_st);
  if (scn_get_response (fd, resp))
  {
    //#>rsp(16b in 320ms/16c)U0n0\70080900\45
    return (hex2int (resp + 4));
  }
  return 0;
}

int scn_send_cmd (int fd, char *cmd, char *rsp)
{
  //write CRC
  char ccrc[5];
  sprintf (ccrc, "%02X", scn_get_crc (cmd));
  //set cmd crc
  cmd[13] = ccrc[0];
  cmd[14] = ccrc[1];
  //
  write (fd, cmd, cmdl);
  debug_print ("\n#i:%d<%s", fd, cmd);
  //
  if (scn_get_response (fd, rsp))
  {
    //#>rsp(16b in 320ms/16c)U0n0\70080900\45
    return (hex2int (rsp + 4));
  }
  return 0;
}

int scn_get_pos (int fd)
{
  int cp = -1;
  char rsp[20];
  debug_print ("\n#i:%d<%s", fd, cmd_rp);
  write (fd, cmd_rp, cmdl);
  //read position
  if (scn_get_response (fd, rsp))
    cp = hex2int (rsp + 5);
  debug_print (", p%d", cp);
  return cp;
}

int scn_set_pos (int fd, int pos)
{
  int cp = -1;
  char rsp[20], lcmd[20];
  //
  memcpy (lcmd, cmd_po, cmdl + 1);
  snprintf (rsp, 10, "%08X", pos);
  memcpy (lcmd + 3, rsp, 8);
  char ccrc[5];
  sprintf (ccrc, "%02X", scn_get_crc (lcmd));
  //set cmd crc
  lcmd[13] = ccrc[0];
  lcmd[14] = ccrc[1];
  debug_print ("\n#i:%d<%s, p%d", fd, lcmd, pos);
  //send command
  write (fd, lcmd, cmdl);
  //read position
  //if (scn_get_response (fd, rsp))
    //cp = hex2int (rsp + 5);
  //
  return cp;
}

int scn_set_vel (int fd, int vel, int acc)
{
  char rsp[20], lcmd[20];
  //
  memcpy (lcmd, cmd_sa, cmdl + 1);
  snprintf (rsp, 10, "%04X%04X", vel, acc);
  memcpy (lcmd + 4, rsp, 8);
  char ccrc[5];
  sprintf (ccrc, "%02X", scn_get_crc (lcmd));
  //set cmd crc
  lcmd[13] = ccrc[0];
  lcmd[14] = ccrc[1];
  debug_print ("\n#i:%d<%s", fd, lcmd);
  //send command
  write (fd, lcmd, cmdl);
  //read response
  //scn_get_response (fd, rsp);
  //
  return 1;
}

int scn_set_home (int fd)
{
  int cp = -1;
  char rsp[20];
  //
  debug_print ("\n#i:%d<%s", fd, cmd_ho);
  //send command
  write (fd, cmd_ho, cmdl);
  //read position
  if (scn_get_response (fd, rsp))
    cp = hex2int (rsp + 5);
  //
  return cp;
}

int scn_set_son (int fd)
{
  int cp = -1;
  char rsp[20];
  //
  debug_print ("\n#i:%d<%s", fd, cmd_on);
  //send command
  write (fd, cmd_on, cmdl);
  //
  scn_get_response (fd, rsp);
  //
  return cp;
}

int scn_set_soff (int fd)
{
  int cp = -1;
  char rsp[20];
  //
  debug_print ("\n#i:%d<%s", fd, cmd_of);
  //send command
  write (fd, cmd_of, cmdl);
  //
  scn_get_response (fd, rsp);
  //
  return cp;
}

int scn_test (int fd1)
{
  //set servo ON
  char buf[255];
  //position
  printf ("\n#i:current position <%d>", scn_get_pos (fd1));
  //power on
  scn_send_cmd (fd1, cmd_on, buf);
  //run command
  if (mcmd)
  {
    //command
    //cmd_ac[2] = mcmd[0];
    //cmd params: 8 bytes
    int j;
    for (j = 0; j < strlen (mcmd); j++)
      cmd_ac[2+j] = mcmd[j];
    //
    printf ("\n#i:run cmd '%s'", cmd_ac);
    scn_send_cmd (fd1, cmd_ac, buf);
    if (*mcmd == 'o')//homing?
    {
      printf ("\n#i:wait for homing to finish..");
      //wait for position to be reported as 0
      int mk = 10;
      while (scn_get_pos (fd1) && mk)
      {
        usleep (200000);
        mk--;
      }
      printf ("\n#i:done.");
    }
  }
  //if !PFIN flag, wait for it to finish
  //if (!(stat & FLGPIO_PFIN))
  {
    printf ("\n#i:wait for move to finish..");
    int mk = 5;
    while (!scn_get_pfin (fd1) && mk)
    {
      usleep (20000);
      mk--;
    }
  }
  printf ("\n#i:move finished.");
  //power off
  scn_send_cmd (fd1, cmd_of, buf);
  //position
  printf ("\n#i:current position <%d>", scn_get_pos (fd1));
  //get status
  scn_get_pfin (fd1);
  //
  return 0;
}
