/*
 Copyright (c) 2016 Mathieu Laurendeau <mat.lau@laposte.net>
 License: GPLv3
 */

#include <async.h>

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

s_device devices[ASYNC_MAX_DEVICES] = { };

void async_init(void) __attribute__((constructor (101)));
void async_init(void)
{
    int i;
    for (i = 0; i < ASYNC_MAX_DEVICES; ++i) {
        devices[i].fd = -1;
    }
}

void async_clean(void) __attribute__((destructor (101)));
void async_clean(void)
{
    int i;
    for (i = 0; i < ASYNC_MAX_DEVICES; ++i) {
        if(devices[i].fd >= 0) {
            async_close(i);
        }
    }
}

void async_print_error(const char * file, int line, const char * msg) {
  
  fprintf(stderr, "%s:%d %s failed with error: %m\n", file, line, msg);
}

static int add_device(const char * path, int fd, int print) {
    int i;
    for (i = 0; i < ASYNC_MAX_DEVICES; ++i) {
        if(devices[i].path && !strcmp(devices[i].path, path)) {
            if(print) {
                fprintf(stderr, "%s:%d add_device %s: device already opened\n", __FILE__, __LINE__, path);
            }
            return -1;
        }
    }
    for (i = 0; i < ASYNC_MAX_DEVICES; ++i) {
        if(devices[i].fd == -1) {
            devices[i].path = strdup(path);
            if(devices[i].path != NULL) {
                devices[i].fd = fd;
                return i;
            }
            else {
                fprintf(stderr, "%s:%d add_device %s: can't duplicate path\n", __FILE__, __LINE__, path);
                return -1;
            }
        }
    }
    return -1;
}

int async_open_path(const char * path, int print) {
    int ret = -1;
    if(path != NULL) {
        int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if(fd != -1) {
            ret = add_device(path, fd, print);
            if(ret == -1) {
                close(fd);
            }
        }
        else {
            if(print) {
                ASYNC_PRINT_ERROR("open")
            }
        }
    }
    return ret;
}

int async_close(int device) {

    ASYNC_CHECK_DEVICE(device, -1)

    close(devices[device].fd);

    free(devices[device].path);
    free(devices[device].read.buf);

    memset(devices + device, 0x00, sizeof(*devices));

    devices[device].fd = -1;

    return 0;
}

int async_read_timeout(int device, void * buf, unsigned int count, unsigned int timeout) {

  ASYNC_CHECK_DEVICE(device, -1)

  unsigned int bread = 0;
  int res;

  fd_set readfds;

  time_t sec = timeout / 1000;
  __suseconds_t usec = (timeout - sec * 1000) * 1000;
  struct timeval tv = {.tv_sec = sec, .tv_usec = usec};

  while(bread != count)
  {
    FD_ZERO(&readfds);
    FD_SET(devices[device].fd, &readfds);
    int status = select(devices[device].fd+1, &readfds, NULL, NULL, &tv);
    if(status > 0)
    {
      if(FD_ISSET(devices[device].fd, &readfds))
      {
        res = read(devices[device].fd, buf+bread, count-bread);
        if(res > 0)
        {
          bread += res;
        }
      }
    }
    else if(status == EINTR)
    {
      continue;
    }
    else
    {
      break;
    }
  }

  return bread;
}

int async_write_timeout(int device, const void * buf, unsigned int count, unsigned int timeout) {

  ASYNC_CHECK_DEVICE(device, -1)

  unsigned int bwritten = 0;
  int res;

  fd_set writefds;

  time_t sec = timeout / 1000;
  __suseconds_t usec = (timeout - sec * 1000) * 1000;
  struct timeval tv = {.tv_sec = sec, .tv_usec = usec};

  while(bwritten != count)
  {
    FD_ZERO(&writefds);
    FD_SET(devices[device].fd, &writefds);
    int status = select(devices[device].fd+1, NULL, &writefds, NULL, &tv);
    if(status > 0)
    {
      if(FD_ISSET(devices[device].fd, &writefds))
      {
        res = write(devices[device].fd, buf+bwritten, count-bwritten);
        if(res > 0)
        {
          bwritten += res;
        }
      }
    }
    else if(status == EINTR)
    {
      continue;
    }
    else
    {
      break;
    }
  }

  return bwritten;
}

/*
 * This function is called on data reception.
 */
static int read_callback(int device) {

    ASYNC_CHECK_DEVICE(device, -1)
    
    int ret = read(devices[device].fd, devices[device].read.buf, devices[device].read.count);
    
    if(ret < 0) {
        ASYNC_PRINT_ERROR("read")
    }

    return devices[device].callback.fp_read(devices[device].callback.user, (const char *)devices[device].read.buf, ret);
}

/*
 * This function is called on failure.
 */
static int close_callback(int device) {

    ASYNC_CHECK_DEVICE(device, -1)

    return devices[device].callback.fp_close(devices[device].callback.user);
}

int async_set_read_size(int device, unsigned int size) {

    ASYNC_CHECK_DEVICE(device, -1)
    
    if(size > devices[device].read.size) {
        void * ptr = realloc(devices[device].read.buf, size);
        if(ptr == NULL) {
    	    fprintf(stderr, "%s:%d %s: can't allocate a buffer\n", __FILE__, __LINE__, __func__);
            return -1;
        }
        devices[device].read.buf = ptr;
        devices[device].read.size = size;
    }
    
    devices[device].read.count = size;
    
    return 0;
}

int async_register(int device, int user, ASYNC_READ_CALLBACK fp_read, ASYNC_WRITE_CALLBACK fp_write, ASYNC_CLOSE_CALLBACK fp_close, GPOLL_REGISTER_FD fp_register) {

    ASYNC_CHECK_DEVICE(device, -1)
    
    devices[device].callback.user = user;
    devices[device].callback.fp_read = fp_read;
    //fp_write is ignored
    devices[device].callback.fp_close = fp_close;

    return fp_register(devices[device].fd, device, read_callback, NULL, close_callback);
}

int async_write(int device, const void * buf, unsigned int count) {

    ASYNC_CHECK_DEVICE(device, -1)

    int ret = write(devices[device].fd, buf, count);
    if (ret == -1) {
        ASYNC_PRINT_ERROR("write")
    }
    else if((unsigned int) ret != count) {
        fprintf(stderr, "%s:%d write: only %u written (requested %u)\n", __FILE__, __LINE__, ret, count);
    }
    
    return ret;
}

