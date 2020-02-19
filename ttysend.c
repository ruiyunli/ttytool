#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#define MAX_BUFFER_SIZE 512

int fd, flag_close;

static void usage(const char * appname)
{
  printf("usage:\n");
  printf("\t%s dev\n", appname);
}


int open_serial(const char* devname)
{
    fd = open(devname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd == -1)
    {
        perror("open serial port error\n");
        return -1;
    }

    printf("Open serial port success:%s\n", devname);
    return 0;
}

int main(int argc, char* argv[])
{
    char sbuf[] = {"Hello, this is a serial port test!\n"};
    int retv;
    struct termios option;

    // tbd
    if(argc < 2)
    {
      usage(argv[0]);
      return -1;
    }

    retv = open_serial(argv[1]);
    if(retv < 0)
    {
        perror("open serial port error!\n");
        return -1;
    }

    printf("Ready for sending data...\n");

    tcgetattr(fd, &option);
    cfmakeraw(&option);

    cfsetispeed(&option, B9600);
    cfsetospeed(&option, B9600);

    tcsetattr(fd, TCSANOW, &option);

    int length = sizeof(sbuf);

    retv = write(fd, sbuf, length);
    if(retv == -1)
    {
        perror("Write data error!\n");
        return -1;
    }

    printf("The number of char sent is %d\n", retv);
    return 0;
}
