/*
 ============================================================================
 Name        : grabber.c
 Author      : Diego
 Version     :
 Copyright   : Your copyright notice
 Description : Captures an image using the v4l2 driver over the /dev/video0 device, converts it to a yuv420p, puts a header at top and sends the result through stdout.
 ============================================================================
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <errno.h>
#include <signal.h>


#include <assert.h>
#include <sys/types.h>
#include <sys/time.h>


#define CLEAR(x) memset(&(x), 0, sizeof(x))

int width, height;

void yuyv422_to_yuv420p(int width, int height, uint8_t * b422, uint8_t * y, uint8_t *u, uint8_t * v)
{
	int b422width = width * 2;

	uint8_t * row = b422;
	uint8_t * max = row + height * b422width;

	int offset;

	uint8_t * cptr;
	uint8_t * mptr;

	int nrow;

	for(row = b422, nrow = 0; row < max; row += b422width, nrow++)
	{
		mptr = row + b422width;

		if(nrow % 2 == 0) //Nos guardamos "u" y "v"
		{
			for(cptr = row; cptr < mptr; cptr += 4)
			{
				*u = *(cptr + 1);
				*v = *(cptr + 3);
				u++;
				v++;
			}
		}
		//Nos guardamos "y" siempre
		for(cptr = row; cptr < mptr; cptr +=2)
		{
			*y = *cptr;
			y++;
		}
	}
}

int
writep9header(int w, int h)
{/*
	fputs("P9\n", stdout);
	fprintf(stdout, "%d %d\n", w ,h);
	fputs("255\n",stdout);
	return 0;
	*/
	char header[100];
	int n=sprintf(header, "P7\n%d %d\n255\n", w, h);
	write(1, header, n);

	return 0;
}


/**
 * Set a file descriptor to blocking or non-blocking mode.
 *
 * @param fd The file descriptor
 * @param blocking 0:non-blocking mode, 1:blocking mode
 *
 * @return 1:success, 0:failure.
 **/
int fd_set_blocking(int fd, int blocking) {
    /* Save the current flags */
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1)
        return 0;

    if (blocking)
        flags &= ~O_NONBLOCK;
    else
        flags |= O_NONBLOCK;
    return fcntl(fd, F_SETFL, flags) != -1;
}

void segfault_sigaction(int signal, siginfo_t *si, void *arg)
{
    printf("Caught segfault at address %p\n", si->si_addr);
    exit(0);
}

void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

uint8_t *buffer;
 
static int xioctl(int fd, int request, void *arg)
{
        int r;
 
        do r = ioctl (fd, request, arg);
        while (-1 == r && EINTR == errno);
 
        return r;
}
 
int print_caps(int fd)
{
        struct v4l2_capability caps = {};
        if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps))
        {
                perror("Querying Capabilities");
                return 1;
        }
 
        fprintf(stderr, "Driver Caps:\n"
                "  Driver: \"%s\"\n"
                "  Card: \"%s\"\n"
                "  Bus: \"%s\"\n"
                "  Version: %d.%d\n"
                "  Capabilities: %08x\n",
                caps.driver,
                caps.card,
                caps.bus_info,
                (caps.version>>16)&&0xff,
                (caps.version>>24)&&0xff,
                caps.capabilities);
 
 /*
        struct v4l2_cropcap cropcap = {0};
        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl (fd, VIDIOC_CROPCAP, &cropcap))
        {
                perror("Querying Cropping Capabilities");
                return 1;
        }
 
        fprintf(stderr, "Camera Cropping:\n"
                "  Bounds: %dx%d+%d+%d\n"
                "  Default: %dx%d+%d+%d\n"
                "  Aspect: %d/%d\n",
                cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
                cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
                cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);
*/ 
        int support_yuyv422 = 0;
 
        struct v4l2_fmtdesc fmtdesc = {0};
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        char fourcc[5] = {0};
        char c, e;
        fprintf(stderr,"  FMT : CE Desc\n--------------------\n");
        while (0 == xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc))
        {
                strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
                if (fmtdesc.pixelformat == V4L2_PIX_FMT_YUYV)
                    support_yuyv422 = 1;
                c = fmtdesc.flags & 1? 'C' : ' ';
                e = fmtdesc.flags & 2? 'E' : ' ';
                fprintf(stderr,"  %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
                fmtdesc.index++;
        }
 
        if (!support_yuyv422)
        {
            fprintf(stderr, "Doesn't support YUYV.\n");
            return 1;
        }
 
        struct v4l2_format fmt = {0};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
 
        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
        {
            perror("Setting Pixel Format");
            return 1;
        }
 
        strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
        fprintf(stderr, "Selected Camera Mode:\n"
                "  Width: %d\n"
                "  Height: %d\n"
                "  PixFmt: %s\n"
                "  Field: %d\n",
                fmt.fmt.pix.width,
                fmt.fmt.pix.height,
                fourcc,
                fmt.fmt.pix.field);
        return 0;
}
 
int init_mmap(int fd)
{
    struct v4l2_requestbuffers req = {0};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
 
    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        perror("Requesting Buffer");
        return 1;
    }
 
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
    {
        perror("Querying Buffer");
        return 1;
    }
 
    buffer = mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    fprintf(stderr, "Length: %d\nAddress: %p\n", buf.length, buffer);
    fprintf(stderr, "Image Length: %d\n", buf.bytesused);
 
    return 0;
}
 

int read_frame(int fd)
{
	struct v4l2_buffer buf;
	CLEAR(buf);

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
		switch (errno) {
			case EAGAIN:
				return 0;

			case EIO:
				/* Could ignore EIO, see spec. */

				/* fall through */
				return 0;

			default:
				return 0;
		}
	}

	if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
		return 0;

	return 1;
}

int start_capturing(int fd)
{
	struct v4l2_buffer buf = {0};
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = 0;
	if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
	{
		perror("Query Buffer");
		return 1;
	}

	if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
	{
		perror("Start Capture");
		return 1;
	}
	return 0;

}

int main(int argc, char ** argv)
{

	if(argc != 3)
	{
		perror("Incorrect number of parameters.\nUsage: program <WIDTH> <HEIGHT>");
		exit(1);
	}

	width = atoi(argv[1]);
	height = atoi(argv[2]);

	fd_set_blocking(0,1);

    	struct sigaction sa;

    	memset(&sa, 0, sizeof(sigaction));
    	sigemptyset(&sa.sa_mask);
    	sa.sa_sigaction = segfault_sigaction;
    	sa.sa_flags   = SA_SIGINFO;

    	sigaction(SIGSEGV, &sa, NULL);

	const char * v4linuxDevice = "/dev/video0";

        int fd;
 
        fd = open(v4linuxDevice, O_RDWR);
        if (fd == -1)
        {
                perror("Opening video device");
                return 1;
        }
 
        if(print_caps(fd))
            return 1;
 
        if(init_mmap(fd))
            return 1;

	long yuv420p_size = width*height + width*height/2;
	uint8_t * yuv420p = (uint8_t*) malloc(yuv420p_size); //for yuv420p

	uint8_t * y = yuv420p;
	uint8_t * u = y + width * height;
	uint8_t * v = u + width * height / 4;


	if(start_capturing(fd))
		return 1;
	fd_set fds;


	while(1)
	{


		fprintf(stderr, "GRABBER: capturando imagen...\n");
		FD_ZERO(&fds);
		FD_SET(fd, &fds);
		struct timeval tv = {0};
		tv.tv_sec = 2;
		int r = select(fd+1, &fds, NULL, NULL, &tv);
		
		int frameReady = 1;	
		
		if(-1 == r)
		{
			perror("Waiting for Frame");
			frameReady = 0;
		}

		
		if(frameReady)
		{

			if(read_frame(fd))
			{
				fprintf(stderr, "GRABBER: Convirtiendo imagen de yuyv422 a yuv420p\n");
				yuyv422_to_yuv420p(width, height, buffer, y, u, v);
				fprintf(stderr, "GRABBER: Añadido header a %ld bytes de imagen yuyv420\n", yuv420p_size);
				writep9header(width, height);
				fprintf(stderr, "GRABBER: Enviando imagen por la salida estandar\n");
				write(1, yuv420p, yuv420p_size);
			}
		}
	}


	return 0;
}
