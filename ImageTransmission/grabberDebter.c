#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>

#include <libdebt.h>

//old encode includes
#include <math.h>
#include <ctype.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <fcntl.h>

//extra grabber includes
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <errno.h>
#include <assert.h>
#include <sys/types.h>


#define CLEAR(x) memset(&(x), 0, sizeof(x))

int width_G, height_G;

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

uint8_t *buffer_G;
 
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
        fmt.fmt.pix.width = width_G;
        fmt.fmt.pix.height = height_G;
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
 
    buffer_G = mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    fprintf(stderr, "Length: %d\nAddress: %p\n", buf.length, buffer_G);
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

//GRABBER METHODS

static char *
pgmName(char *path)
{
	char *name = strrchr(path, '/');
	return name ? name + 1 : path;
}

static void
usage(char *pgmname, struct debtEncParam *e, struct debtDecParam *d)
{
	fprintf(stderr,
			"usage: %s (-d|-e)[options] < infile > outfile\n"
			"Options:\n"
			"\t-v [0|1]         - Use vector instructions if possible <%d>\n"
			"\n"
			"Decoding: -d\n"
			"\t-b [bias]        - Bias used when decoding (default, zero, mid, exp, midexp) <%s>\n"
			"\t-a [bias weight] - Bias weight to use with bias <%f>\n"
			"\t-c [size]        - Truncate input file to [size] bytes <%d>\n"
			"\nEncoding: -e\n"
			"\t-t transform     - Wavelet used (CDF-9/7) and (5/3, 9/7-M, 13/7-T, haar) <%s>\n"
			"\t-n nbands        - Number of transformations <%d>\n"
			"\t-k depth         - Decompose for depth <= otherwise partition <%d>\n"
			"\t-y val           - Dynamic decomposition/partition <%d>\n"
			"\t-r refmul        - Refinement mutliplier <%f>\n"
			"\t-q quality       - Quality value (0 = max) <%f>\n"
			"\t-u resolution    - Resolution order <%d>\n"
			"\t-x maxsize       - Truncate output at maxsize bytes <%d>\n"
			"\n"
			"\t-p pad           - Pad codes when mapping <%d>\n"
			"\t-m map           - Output map codes <%d>\n"
			"\t-s esc           - Escape 0xff on output stream <%d>\n"
			"\n"
			"\t-l left-shift    - ROI left shift <%d>\n"
			"\t-i x0,y0,x1,y1   - ROI rectangle (Top-Left and Bottom-Right) (repeat)\n"
			"\n"
			"\t-h               - Benchmark <%d>\n"
			"\t-f               - Print encoding statistics <%d>\n"
			"\t-I               - Specify an ASCII identifier for the image (it is like a header)\n"
			"\t-W               - Specify the width of the image\n"
			"\t-H               - Specify the height of the image\n",
			pgmname,
			e->vector,
			bias_name(d->bias), d->weight, d->bitlen >> 3,
			transform_name(e->transform), e->nbands, e->maxk, e->dynamic, e->refmul, e->quality, e->resolution, e->maxsize,
			e->pad, e->map, e->esc,
			e->uroi_shift,
			e->benchmark, e->timestats);
	exit(-1);
}

static int
optGetIntError(char *optarg, int *val, int min)
{
	int x = atoi(optarg);
	if (x < min)
		return 1;
	*val = x;
	return 0;
}

static float
optGetFloatError(char *optarg, float *val, float min)
{
	float x = atof(optarg);
	if (x < min)
		return 1;
	*val = x;
	return 0;
}

static char *
getInt(char *s, int *v, int *sign)
{
	*v = -1;
	if (*s == '+') {
		*sign = 1;
		++s;
	} else if (*s == '-') {
		*sign = -1;
		++s;
	} else {
		*sign = 0;
	}
	if (*s >= '0' && *s <= '9') {
		int x = *s++ - '0';
		while (*s >= '0' && *s <= '9') {
			x = x * 10 + (*s++ - '0');
		}
		*v = x;
	}
	return s;
}

int
optGetRect(struct debtEncParam *e, char *s)
{
	int error = 0;
	int sx0, sy0, sx1, sy1;
	int x0, y0, x1, y1;
	s = getInt(  s, &x0, &sx0); error += ((x0 == -1) || (*s != ','));
	s = getInt(++s, &y0, &sx1); error += ((y0 == -1) || (*s != ','));
	s = getInt(++s, &x1, &sy0); error += ((x1 == -1) || (*s != ','));
	s = getInt(++s, &y1, &sy1); error += ((y1 == -1) || (*s != '\0'));
	/* fix the rectangle */
	if (sx0 || sy0 || sx1 < 0 || sy1 < 0)
		++error;
	if ((sx1 > 0 && !x1) || (sy1 > 0 && !y1))
		++error;
	if (!error) {
		if (!e->uroi_list) {
			e->uroi_list = uroi_list_new();
		}
		if (e->uroi_list) {
			uroi_rect(e->uroi_list, x0, y0, x1, y1);
		}
	}
	return error;
}

/* -d to decode or -e to encode */
/* returns -1 to graph, 0 to decode, 1 to encode */
static int
getOptions(int argc, char *argv[], struct debtEncParam *e, struct debtDecParam *d, char** imId, int* imIdSize)
{
	int opt;
	int error = 0;
	int xi = 0;
	int dec = 0;
	int enc = 0;
	int graph = 0;

	while ((opt = getopt(argc, argv, "v:db:a:c:et:n:k:r:q:u:x:p:m:s:l:i:y:gf:h:I:W:H:")) != -1) {
		switch (opt) {
		case 'I':
			*imId = optarg;
			*imIdSize = strlen(optarg);
			break;
		case 'W':
			width_G = atoi(optarg);
			break;
		case 'H':
			height_G = atoi(optarg);
			break;
		case 'v':
			error += optGetIntError(optarg, &e->vector, 0);
			break;
		case 'h':
			error += optGetIntError(optarg, &e->benchmark, 0);
			break;
		case 'f':
			error += optGetIntError(optarg, &e->timestats, 0);
			break;
		case 'g':
			/* special hidden option to calculate rate x psnr curve */
			graph = 1;
			break;
		case 'd':
			++dec;
			break;
		case 'b':
			if (!optarg) {
				d->bias = BIAS_DEF;
			} else {
				int b = bias_code(optarg);
				if (b != -1)
					d->bias = b;
				else
					++error;
			}
			break;
		case 'a':
			error += optGetFloatError(optarg, &d->weight, 0.0);
			break;
		case 'c':
			if (optGetIntError(optarg, &xi, 0))
				++error;
			else
				d->bitlen = xi << 3;
			break;
		case 'e':
			++enc;
			break;
		case 't':
			if (!optarg) {
				e->transform = TRANSFORM_I_B_13x7T;
			} else {
				int t = transform_code(optarg);
				if (t != -1)
					e->transform = t;
				else
					++error;
			}
			break;
		case 'n':
			error += optGetIntError(optarg, &e->nbands, 0);
			break;
		case 'k':
			error += optGetIntError(optarg, &e->maxk, 0);
			break;
		case 'r':
			error += optGetFloatError(optarg, &e->refmul, 0.0);
			break;
		case 'q':
			error += optGetFloatError(optarg, &e->quality, 0.0);
			break;
		case 'u':
			error += optGetIntError(optarg, &e->resolution, 0);
			break;
		case 'x':
			error += optGetIntError(optarg, &e->maxsize, 0);
			break;
		case 'p':
			error += optGetIntError(optarg, &e->pad, 0);
			break;
		case 'm':
			error += optGetIntError(optarg, &e->map, 0);
			break;
		case 's':
			error += optGetIntError(optarg, &e->esc, 0);
			break;
		case 'l':
			error += optGetIntError(optarg, &e->uroi_shift, 0);
			break;
		case 'i':
			error += optGetRect(e, optarg);
			break;
		case 'y':
			error += optGetIntError(optarg, &e->dynamic, 0);
			break;
		default: /* '?' */
			usage(argv[0], e, d);
		}
	}

	if (error) {
		usage(pgmName(argv[0]), e, d);
	}
	if (optind < argc) {
		fprintf(stderr, "Unexpected argument(s) after options\n");
		usage(pgmName(argv[0]), e, d);
	}
	if (dec && (enc || graph)) {
		fprintf(stderr, "Decode and Encode flags cannot be used simultaneously\n");
		usage(pgmName(argv[0]), e, d);
	}
	return graph ? -1 : !dec;
}

static void
defaultParams(struct debtEncParam *e, struct debtDecParam *d)
{
	debtEncParam_init(e);
	e->vector = 1;
	debtDecParam_init(d);
}

#define MAXSIZE 	(64 * 1024 * 1024) // 64 megabytes is a HUGE BUFFER for a compressed image! Signal anything above it

void clearInputBuffer()
{
	fd_set_blocking(0,0);
	char c;
	while(fread(&c,1,1,stdin)>0){}
	fd_set_blocking(0,1);
}

static unsigned char *
read_img(int *size, FILE *f)
{
	int rsize = *size;
	/* silently limit the compressed image size to 64M */
	if (rsize <= 0 || rsize > MAXSIZE) {
		rsize = MAXSIZE;
	}
	/* allocate memory for it */
	unsigned char *cimg = malloc(rsize);
	if (cimg) {
		rsize = fread(cimg, 1, rsize, f);
		if (rsize > 0) {
			*size = rsize;
			return cimg;
		}
		fprintf(stderr, "Short read (%d)\n", rsize);
		free(cimg);
	} else {
		fprintf(stderr, "Failed to alloc memory (%d)\n", rsize);
	}
	return NULL;
}

/*
 * returns the number of bytes written or -1 in case of error or short write
 */
static int
write_img(struct imgBuffer *img, FILE *f)
{
	int color = colorFormatFmt(img->colorFormat);
	if (color != COLORFORMAT_FMT_GREY && color != COLORFORMAT_FMT_420) {
		fprintf(stderr, "Unsupported color format (%d)\n", color);
		return -1;
	}
	int ysize = img->width * img->height; // FIXME: assumes img->pitch == img->width
	int uvsize = img->uvwidth * img->uvheight; // FIXME: assumes img->uvpitch == img->uvwidth
	int yuvsize = ysize + (uvsize << 1);
	/* PGM header */
	//fprintf(stdout, "P%d\n%d %d\n255\n", color == COLORFORMAT_FMT_GREY ? 5 : 7, img->width, img->height);
//	int n = fwrite(img->buffer, 1, yuvsize, f);
	int n = write(1, img->buffer, yuvsize); //parece que write flushea después de escribir, en cambio fwrite no
	if (n != yuvsize) {
		fprintf(stderr, "Short write (%d out of %d)\n", n, yuvsize);
		n = -1;
	}
	return n;
}

static int
decode(struct debtDecParam *d)
{
	int retval = -1;
	/* read the image up to size */
	int debtimgsize = ((d->bitlen + 7) >> 3);
	unsigned char *debtimg = read_img(&debtimgsize, stdin);
	if (debtimg) {
		/* prepare an imgBuffer for decoding */
		struct imgBuffer *dst = imgBuffer_new();
		if (dst) {

			fprintf(stderr, "DECODER: Intentando descomprimir!!!\n");
			if (debt_decode_imgbuffer(dst, debtimg, debtimgsize, d, NULL, NULL)) {
				fprintf(stderr, "DECODER: Descomprimiendo la imagen!!!\n");
				retval = write_img(dst, stdout);
			}
			else
				fprintf(stderr, "DECODER: ERROR al descomprimir!!!\n");
			imgBuffer_del(dst);
		}
		free(debtimg);
		clearInputBuffer();
	}
	return retval;
}

static int
readint(int *x, int *next, FILE *f)
{
	int error = -1;
	int v = 0;
	int c;
	if (isdigit(c = getc(f))) {
		v = c - '0';
		while (isdigit(c = getc(f))) {
			v = v * 10 + (c - '0');
		}
		*x = v;
		error = 0;
	}
	*next = c;
	return error;
}

static int
readwh(int *w, int *h, FILE *f)
{
	int next;
	if (readint(w, &next, f) || next != ' ')
		return -1;
	if (readint(h, &next, f) || next != '\n')
		return -1;
	return 0;
}

static int
readmax(int *max, FILE *f)
{
	int next;
	return (readint(max, &next, f) || next != '\n') ? -1 : 0;
}

static int
read_header(int *w, int *h, FILE *f)
{
	if (getc(f) != 'P')
		return -1;
	int c = getc(f);
	if (c != '5' && c != '7')
		return -1;
	if (getc(f) != '\n')
		return -1;
	if (readwh(w, h, f))
		return -1;
	int max;
	if (readmax(&max, f) || max != 255)
		return -1;
	return (c == '5') ? COLORFORMAT_FMT_GREY : COLORFORMAT_FMT_420;
}

static struct imgBuffer *
read_src_img(FILE *f)
{
        struct imgBuffer *img = NULL;
        int w, h, color;
        color = read_header(&w, &h, f);
        if (color != -1) {
                /* prepare an image buffer for the input image */
                if ((img = imgBuffer_aligned_new(1))) {
                        if (imgBuffer_reinit(img, w, h, color)) {
                                int yuvsize = (img->width * img->height) + ((img->uvwidth * img->uvheight) << 1);
                                int n = fread(img->buffer, 1, yuvsize, f);
                                if (n == yuvsize) {
                                        return img;
                                }
                                fprintf(stderr, "Short read (%d from %d)\n", n, yuvsize);
                        }
                        imgBuffer_del(img);
                        img = NULL;
                }
        }
        return img;
}

static struct imgBuffer * buildImgBuffer(unsigned char * ci, int color, int w, int h, int yuvsize)
{
	struct imgBuffer *img = NULL;
	/* prepare an image buffer for the input image */
	if ((img = imgBuffer_aligned_new(1))) {
		if (imgBuffer_reinit(img, w, h, color)) 
		{
			memcpy(img->buffer, ci, yuvsize);
			return img;
		}
		imgBuffer_del(img);
		img = NULL;
	}
	return img;
}

static int
encode(struct debtEncParam *e, unsigned char * buffer, int buflen, int fixed, char* imId, int imIdSize, struct imgBuffer * src)
{
	struct timeval start;
	int imgsize = 0;
	
	fprintf(stderr, "ENCODER: Comprobando imagen...\n");
	if (src) {

		fprintf(stderr, "ENCODER: Imagen encontrada!\n");
		/* encode it! */
		int blen = debt_encode_imgbuffer(src, &buffer, &buflen, fixed, e);
		if (blen) {
			gettimeofday(&start, NULL);
			
			fprintf(stderr, "ENCODER: Escribiendo identificador de la imagen (%d bytes)\n", imIdSize);
			write(1, imId, imIdSize);

			fprintf(stderr, "ENCODER: Escribiendo timeval actual (%ld bytes)\n", sizeof(struct timeval));
			write(1, &start, sizeof(struct timeval));

			fprintf(stderr, "ENCODER: Comprimiendo...\n");

			/* negative result means valid but truncated output (the user has probably specified a max size or no more memory */
			if (blen < 0) {
				blen = -blen;
			}
			int n = (blen + 7) >> 3;
			//imgsize = fwrite(buffer, 1, n, stdout);
			imgsize = write(1, buffer, n);
			if (n != imgsize) {
				fprintf(stderr, "Short write (%d out of %d)\n", imgsize, n);
			}
		}
		//fprintf(stderr, "ENCODER: Limpiando buffer...\n");
		//clearInputBuffer();
	}
	return imgsize;
}

static int
graph(struct debtEncParam *e, struct debtDecParam *d)
{
	int ret = -1;
	/* read the source image */
	struct imgBuffer *src = read_src_img(stdin);
	if (src) {
		/* compress it once with the parameters that were supplied */
		/* setup the output buffer */
		unsigned char *dbtbuf = NULL;
		int dbtbuflen = 0;
		int fixed = 0;
		if (e->maxsize) {
			dbtbuf = malloc(dbtbuflen = e->maxsize);
			fixed = 1;
		}
		/* encode it! */
		int blen = debt_encode_imgbuffer(src, &dbtbuf, &dbtbuflen, fixed, e);
		if (blen) {
			/* negative result means valid but truncated output (the user has probably specified a max size or no more memory */
			if (blen < 0) {
				blen = -blen;
			}
			int dbtlen = (blen + 7) >> 3;
			struct imgBuffer *dst = imgBuffer_aligned_new(1);
			if (dst) {
				if (debt_decode_imgbuffer(dst, dbtbuf, dbtlen, d, e, src)) { // this will overwrite e with the transmitted parameters (should be the same!)
					ret = 0; // success
				}
				imgBuffer_del(dst);
			}
		}
		free(dbtbuf);
		dbtbuflen = 0;
		imgBuffer_del(src);
	}
	return ret;
}



int isEnc(int * e)
{
	*e = 1;
	return 0;
}

int isDec(int * d)
{
	*d = 1;
	return 0;
}

int
main(int argc, char *argv[])
{
	struct debtEncParam e;
	struct debtDecParam d;

	defaultParams(&e, &d);

	char * imId;
	int imIdSize;

	int isEncoder = 0, isDecoder = 0;
	int act = getOptions(argc, argv, &e, &d, &imId, &imIdSize);

	int ret = (act == -1) ? graph(&e, &d) : (!act ? isDec(&isDecoder) : isEnc(&isEncoder));


	if(isEncoder || isDecoder)
	{

		if(isEncoder)
		{
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

			struct imgBuffer *img = NULL;
			/* prepare an image buffer for the input image */
			if ((img = imgBuffer_aligned_new(1))) {
				if (!imgBuffer_reinit(img, width_G, height_G, COLORFORMAT_FMT_420)) 
					return 1;
			}
			else return 1;

			long yuv420p_size = width_G*height_G + width_G*height_G/2;
			
			uint8_t * y = img->buffer;
			uint8_t * u = y + width_G * height_G;
			uint8_t * v = u + width_G * height_G / 4;

			if(start_capturing(fd))
				return 1;
			fd_set fds;

			/* setup the output buffer */
			unsigned char *buffer = NULL;
			int buflen = 0;
			int fixed = 0;
			if (e.maxsize) {
				buffer = malloc(buflen = e.maxsize);
				fixed = 1;
			}

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
						yuyv422_to_yuv420p(width_G, height_G, buffer_G, y, u, v);
						
						encode(&e, buffer, buflen, fixed, imId, imIdSize, img);
					}
				}

			}
		}
		else
		{
			while(1)
			{

				decode(&d);
			}
		}
	}

	debtDecParam_destroy(&d);
	debtEncParam_destroy(&e);

	return ret;
}
