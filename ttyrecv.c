#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <math.h>

#define MAX_BUFFER_SIZE 512

int fd, s;

static void usage(const char * appname)
{
  printf("usage:\n");
  printf("\t%s dev\n", appname);
}


int open_serial(const char* devname)
{
    fd = open(devname, O_RDWR|O_NOCTTY|O_NDELAY);
    if(fd == -1)
    {
        perror("open serial port error!\n");
        return -1;
    }

    printf("open dev success:%s.\n", devname);
    return 0;
}

int main(int argc, char* argv[])
{
    char hd[MAX_BUFFER_SIZE], *rbuf;
    int flag_close, retv;
    struct termios opt;

    if(argc < 2)
    {
      usage(argv[0]);
      return -1;
    }

    retv = open_serial(argv[1]);
    if(retv < 0)
    {
        printf("Open serrial port error!\n");
        return -1;
    }

    tcgetattr(fd, &opt);
    cfmakeraw(&opt);
    cfsetispeed(&opt, B9600);
    cfsetospeed(&opt, B9600);
    tcsetattr(fd, TCSANOW, &opt);
    rbuf = hd;
    printf("Ready for receiving data...\n");

    while(1)
    {
        while((retv = read(fd, rbuf, 1)) > 0)
            printf( "%c ", *rbuf);
    }

    printf("\n");
    flag_close = close(fd);
    if(flag_close == -1)
        printf("Close the device failure!\n");

    return 0;
}
