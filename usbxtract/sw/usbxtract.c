/*
 Copyright (c) 2015 Mathieu Laurendeau <mat.lau@laposte.net>
 License: GPLv3

 *
 2019: Mirel Lazar mirel.t.lazar@gmail.com adapter for Motion Feedback Controller System
 *
 */

#include <proxy.h>
#include <signal.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <info.h>
#include <getopt.h>
#include <adapter.h>

static char * port = NULL;
static char * udev = NULL;
int vid = 0, pid = 0;

static void usage()
{
  printf("#usage: sudo usbxtract --tty /dev/ttyUSB0 --device 044f:b66d\n");
}

int args_read(int argc, char *argv[]) 
{

  int ret = 0, val;
  int c;

  struct option long_options[] = {
    /* These options don't set a flag. We distinguish them by their indices. */
    { "help",    no_argument,       0, 'h' },
    { "version", no_argument,       0, 'V' },
    { "tty",     required_argument, 0, 't' },
    { "device",  required_argument, 0, 'd' },
    { "capture", required_argument, 0, 'c' },
    { 0, 0, 0, 0 }
  };

  while (1) 
  {
    /* getopt_long stores the option index here. */
    int option_index = 0;

    c = getopt_long (argc, argv, "hd:t:d:V", long_options, &option_index);

    /* Detect the end of the options. */
    if (c == -1)
      break;

    switch (c) {

    case 'h':
      usage ();
      exit (0);
      break;

    case 't':
      port = optarg;
      ret++;
      break;

    case 'c':
      if (sscanf (optarg, "%d", &val) == 1)
        adapter_debug (val);
      break;

    case 'd':
      udev = optarg;
      ret++;
      break;

    case 'V':
      printf("usbxtract %s %s\n", INFO_VERSION, INFO_ARCH);
      exit(0);
      break;

    case '?':
      usage();
      exit(-1);
      break;

    default:
      printf("unrecognized option: %c\n", c);
      ret = -1;
      break;
    }
  }

  return ret;
}

static void terminate (int sig) 
{
  proxy_stop();
}

int main (int argc, char * argv[]) 
{

  (void) signal (SIGINT, terminate);
  (void) signal (SIGTERM, terminate);
  (void) signal (SIGHUP, terminate);

  int ret;
  ret = args_read (argc, argv);
  if (ret < 2)
  {
    usage ();
    return -1;
  }
  if (udev == NULL)
  {
    usage ();
    return -1;
  }
  if (sscanf (udev, "%04x:%04x", &vid, &pid) < 2)
  {
    printf ("invalid option: --device %s\n", udev);
    usage ();
    return -1;
  }
  if (vid == 0)
  {
    printf ("invalid option: --device %s\n", udev);
    usage ();
    return -1;
  }
  if (pid == 0)
  {
    printf ("invalid option: --device %s\n", udev);
    usage ();
    return -1;
  }
  printf ("#i:initializing USB proxy with device %s", udev);
  ret = proxy_init (vid, pid);  //T300RS

  if (ret == 0) 
  {
    printf ("\n#i:starting");
    ret = proxy_start (port);
  }
  printf ("\n#i:done\n");
  return ret;
}
