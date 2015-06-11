/*
 * main.cpp
 *
 *  Created on: Feb 16, 2015
 *      Author: diego
 */

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <sys/time.h> /*para timeout*/
#include <sys/ioctl.h>

#include <sys/types.h>
#include <sys/select.h>
#include <stdint.h>
/*
 * 'open_port()' - Open serial port 1.
 *
 * Returns the file descriptor on success or -1 on error.
 */
#define FLUSH_IN   0
#define FLUSH_OUT  1
#define FLUSH_BOTH 2
struct termios options;

uint32_t
rc_crc32(uint32_t crc, const char *buf, size_t len)
{
	static uint32_t table[256];
	static int have_table = 0;
	uint32_t rem;
	uint16_t octet;
	uint16_t i, j;
	const uint8_t *p, *q;

	/* This check is not thread safe; there is no mutex. */
	printf("uint32_t Checksum::crc32_tab[258] = {");
	if (have_table == 0) {
		/* Calculate CRC table. */
		for (i = 0; i < 256; i++) {

			rem = i << 24;  /* remainder from polynomial division */
			for (j = 0; j < 8; j++) {
				if (rem & 0x80000000) {
					rem <<= 1;
					rem ^= 0x04C11DB7;
				} else
					rem <<= 1;
			}
			if(i % 5 == 0)
				printf("\n");

			if(i < 255)
				printf("0x%8.8X, ",rem);
			else
				printf("0x%8.8X\n",rem);

			table[i] = rem;
		}
		printf("};\n");
		have_table = 1;
	}

	crc = 0;//~crc;
	q = buf + len;
	for (p = buf; p < q; p++) {
		octet = *p;  /* Cast to unsigned octet. */
		crc = (crc << 8) ^ table[(crc >> 24) ^ octet];
	}
	return crc;//~crc;
}

uint32_t crc32_tab[258] = {
0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC,
0x17C56B6B, 0x1A864DB2, 0x1E475005, 0x2608EDB8, 0x22C9F00F,
0x2F8AD6D6, 0x2B4BCB61, 0x350C9B64, 0x31CD86D3, 0x3C8EA00A,
0x384FBDBD, 0x4C11DB70, 0x48D0C6C7, 0x4593E01E, 0x4152FDA9,
0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75, 0x6A1936C8,
0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3,
0x709F7B7A, 0x745E66CD, 0x9823B6E0, 0x9CE2AB57, 0x91A18D8E,
0x95609039, 0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5,
0xBE2B5B58, 0xBAEA46EF, 0xB7A96036, 0xB3687D81, 0xAD2F2D84,
0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D, 0xD4326D90, 0xD0F37027,
0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB, 0xCEB42022,
0xCA753D95, 0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1,
0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D, 0x34867077,
0x30476DC0, 0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C,
0x2E003DC5, 0x2AC12072, 0x128E9DCF, 0x164F8078, 0x1B0CA6A1,
0x1FCDBB16, 0x018AEB13, 0x054BF6A4, 0x0808D07D, 0x0CC9CDCA,
0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE, 0x6B93DDDB,
0x6F52C06C, 0x6211E6B5, 0x66D0FB02, 0x5E9F46BF, 0x5A5E5B08,
0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D,
0x40D816BA, 0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E,
0xBFA1B04B, 0xBB60ADFC, 0xB6238B25, 0xB2E29692, 0x8AAD2B2F,
0x8E6C3698, 0x832F1041, 0x87EE0DF6, 0x99A95DF3, 0x9D684044,
0x902B669D, 0x94EA7B2A, 0xE0B41DE7, 0xE4750050, 0xE9362689,
0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683,
0xD1799B34, 0xDC3ABDED, 0xD8FBA05A, 0x690CE0EE, 0x6DCDFD59,
0x608EDB80, 0x644FC637, 0x7A089632, 0x7EC98B85, 0x738AAD5C,
0x774BB0EB, 0x4F040D56, 0x4BC510E1, 0x46863638, 0x42472B8F,
0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53, 0x251D3B9E,
0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5,
0x3F9B762C, 0x3B5A6B9B, 0x0315D626, 0x07D4CB91, 0x0A97ED48,
0x0E56F0FF, 0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623,
0xF12F560E, 0xF5EE4BB9, 0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2,
0xE6EA3D65, 0xEBA91BBC, 0xEF68060B, 0xD727BBB6, 0xD3E6A601,
0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD, 0xCDA1F604,
0xC960EBB3, 0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7,
0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B, 0x9B3660C6,
0x9FF77D71, 0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD,
0x81B02D74, 0x857130C3, 0x5D8A9099, 0x594B8D2E, 0x5408ABF7,
0x50C9B640, 0x4E8EE645, 0x4A4FFBF2, 0x470CDD2B, 0x43CDC09C,
0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8, 0x68860BFD,
0x6C47164A, 0x61043093, 0x65C52D24, 0x119B4BE9, 0x155A565E,
0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B,
0x0FDC1BEC, 0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088,
0x2497D08D, 0x2056CD3A, 0x2D15EBE3, 0x29D4F654, 0xC5A92679,
0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0, 0xD6AD50A5, 0xD26C4D12,
0xDF2F6BCB, 0xDBEE767C, 0xE3A1CBC1, 0xE760D676, 0xEA23F0AF,
0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5,
0x9E7D9662, 0x933EB0BB, 0x97FFAD0C, 0xAFB010B1, 0xAB710D06,
0xA6322BDF, 0xA2F33668, 0xBCB4666D, 0xB8757BDA, 0xB5365D03,
0xB1F740B4
};

uint32_t crc32(const uint8_t *buf, size_t len, uint32_t crc)
{
	uint32_t rem;
	uint16_t octet;
	uint16_t i, j;
	const uint8_t *p, *q;

	q = buf + len;
	for (p = buf; p < q; p++) {
		octet = *p;  /* Cast to unsigned octet. */
		crc = (crc << 8) ^ crc32_tab[(crc >> 24) ^ octet];
	}
	return crc;
}

void test0()
{
	char cad[] = "123456789";
	uint8_t c = 0xff;
	//uint32_t crc = rc_crc32(0, cad, strlen(cad));
	//uint32_t crc = rc_crc32(0, &c, 1);
	uint32_t crc = crc32(&c, 1, 0);
	printf("0x%X\n", crc);
	crc = crc32(cad, strlen(cad), 0);
	printf("0x%X\n", crc);
}


int esperarOk(int d, unsigned long long m) //s en milliseconds
{
	struct timeval time0, time1;
	gettimeofday(&time0, NULL);
	unsigned long long t0 = time0.tv_sec*1000 + time0.tv_usec/1000;
	unsigned long long t1 = t0;
	char ok[] = "OK";
	char * max = ok + strlen(ok);
	char * ptr = ok;
	char c;
	int n;
	while(t1 - t0 < m)
	{
		ioctl(d, FIONREAD, &n);
		if(n>0)
		{
			read(d,&c,1);
			if(c == *ptr)
				ptr++;
			else
				ptr = ok;
		}
		if(ptr == max)
			return 0;
		gettimeofday(&time1, NULL);
		t1 = time1.tv_sec*1000 + time1.tv_usec/1000;

	}
	return -1;

}
int open_port2()
{
	int serialFileDescriptor;
	serialFileDescriptor = open(
		"/dev/ttyACM0",
		O_RDWR |
		O_NOCTTY |
		O_NONBLOCK );

	// block non-root users from using this port
	ioctl(serialFileDescriptor, TIOCEXCL);

	// clear the O_NONBLOCK flag, so that read() will
	//   block and wait for data.
	fcntl(serialFileDescriptor, F_SETFL, 0);

	// grab the options for the serial port
	tcgetattr(serialFileDescriptor, &options);

	// setting raw-mode allows the use of tcsetattr() and ioctl()
	cfmakeraw(&options);

	// specify any arbitrary baud rate
	//ioctl(serialFileDescriptor, IOSSIOSPEED, &baudRate);

	return serialFileDescriptor;
}
int
open_port(void)
{


	int fd; /* File descriptor for the port */
	char msg[200];
	const char *sp[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3",
			"/dev/ttyACM4", "/dev/ttyACM5", "/dev/ttyACM6", "/dev/ttyACM7"};

	int i;
	for(i = 0; i <7; i++)
	{
		fd = open(sp[i], O_RDWR |
				O_NOCTTY |
				O_NONBLOCK );
		if (fd == -1)
		{
			sprintf(msg, "open_port: Unable to open %s\n",sp[i]);
			perror(msg);
		}
		else
		{
			fcntl(fd, F_SETFL, FNDELAY);
			//fcntl(fd, F_SETFL, 0);
			/*
			 * Get the current options for the port...
			 */
			tcgetattr(fd, &options);
			/*
			 * Set the baud rates to 19200...
			 */
			cfsetispeed(&options, B9600);
			cfsetospeed(&options, B9600);
			/*
			 * Enable the receiver and set local mode...
			 */

			options.c_cflag |= (CLOCAL | CREAD);
			/*
			 * Set the new options for the port...
			 */
			//No parity (8N1):
			options.c_cflag &= ~PARENB;
			options.c_cflag &= ~CSTOPB;
			options.c_cflag &= ~CSIZE;
			options.c_cflag |= CS8;


			//disable hardware flow control:
			options.c_cflag &= ~CRTSCTS;

			//disable software flow controls:
			options.c_iflag &= ~(IXON | IXOFF | IXANY);

			//Choosing Raw Input
			options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

			//Choosing Raw Output
			options.c_oflag &= ~OPOST;

			//per si acas...
			cfmakeraw(&options);

			//options.c_cc[VMIN]  = 0;
			//options.c_cc[VMIN] = 1500;
			tcsetattr(fd, TCSANOW, &options);



			//ioctl(fd,TCFLSH, FLUSH_BOTH);
			//TODO: Averiguamos si es nuestro dispositivo si contesta con un "Soc de les Coves" a la pregunta "Quien eres"
			if(checkDevice(fd, "Hello\n",
					"Vinroma", 20000) == 0)
			//checkDevice(fd, "Hello, device, where are you from?\n",
			//					"Hello, I'm from Les Coves de Vinroma", 4000);
			return (fd);

		}

	}
	return -1;


}
int checkDevice(int fd, const char * hello, const char * reply, unsigned long long m)
{
	unsigned int replySize = strlen(reply);
	//char* buffer = malloc(replySize);
	char* max = reply + replySize;
	char* ptr = reply;
	struct timeval time0, time1;
	gettimeofday(&time0, NULL);
	unsigned long long t0 = time0.tv_sec*1000 + time0.tv_usec/1000;
	unsigned long long t1 = t0;
	unsigned int n;
	char c;
	int helloSize = strlen(hello);
	sleep(3);
	tcflush(fd, TCIOFLUSH);
	while(t1 - t0 < m)
	{
		write(fd, hello, helloSize);
		tcflush(fd, TCOFLUSH);
		ioctl(fd, FIONREAD, &n);
		printf("Chequeando... %d\n", n);
		if(n>0)
		{
			read(fd,&c,1);
			printf("letra %c\n", c);
			if(c == *ptr)
				ptr++;
			else
				ptr = reply;
		}
		if(ptr == max)
			return 0;
		gettimeofday(&time1, NULL);
		t1 = time1.tv_sec*1000 + time1.tv_usec/1000;

	}
	char msg[200];
	sprintf(msg, "ERROR AL LEER %d\n",fd);
	perror(msg);
	close(fd);
	return -1;
}

void test1()
{
	int fd = open_port();
	while(fd < 0)
	{
		open_port();
	}
	printf("Por obert: %d\n", fd);
	int i;
	int n;


	unsigned char c[5000];
	//options.c_cc[VMIN] = 1500;
	//tcsetattr(fd, TCSANOW, &options);
	fcntl(fd, F_SETFL, 0);

	tcflush(fd, TCIOFLUSH);
	while(1)
	{
		//tcflush(fd, TCIOFLUSH);
		//ioctl(fd, FIONREAD, &n);

		n = read(fd, c, 1500);
		printf("Leidos %d\n", n);
//		if(n > 0)
//		{
//			printf("Hay caracteres en bufer! %d\n", n);
//			n = read(fd, c, n);
//
//		//	printf("byte: %c",c);
//			//ioctl(0,TCFLSH, FLUSH_BOTH);
//		}

	}


	fputs("Bye!", stdout);
	close(fd);
}

int main(void)
{
test0();
}

void test2()
{
	int    fd1;  /* input sources 1 and 2 */
	fd_set readfs;    /* file descriptor set */
	int    maxfd;     /* maximum file desciptor used */
	int    loop=1;    /* loop while TRUE */

	/* open_input_source opens a device, sets the port correctly, and
	   returns a file descriptor */
	fd1 = open_port();   /* COM2 */
	if (fd1<0) exit(0);

	maxfd = fd1+1;  /* maximum bit entry (fd) to test */

	/* loop for input */
	while (loop) {
	  FD_SET(fd1, &readfs);  /* set testing for source 1 */
	  /* block until input becomes available */
	  select(maxfd, &readfs, NULL, NULL, NULL);
	  if (FD_ISSET(fd1, &readfs))         /* input from source 1 available */
		{
		  printf("disp.");
		  char c;
		  read(fd1, &c, 1);
		  printf("%c",c);
		  //tcflush(fd1, TCIOFLUSH);
		}
	}
	return 0;
}
/*

		ioctl(0,TCFLSH, FLUSH_BOTH);

		printf("Enviando mensaje...\n");
		n = write(fd, "pepito", 6);
		n = write(fd, buf, 1500);
		if(n < 0)
		{
			fd = open_port();
		}
		ioctl(fd,TCFLSH, FLUSH_OUT);
		printf("Mensaje enviado\n");

		ioctl(fd, FIONREAD, &n);
		if(esperarOk(fd,3000) != 0)
			printf("Timeout!\n");
		else
			{
				printf("ACK recibido!\n");
				//ioctl(fd,TCFLSH, FLUSH_IN);
			}
		ioctl(fd,TCFLSH, FLUSH_IN);
 */


/*
int fd = open_port();
sleep(5);
int i;
int n, b;
unsigned char buf[5000];
char s;
ioctl(0,TCFLSH, FLUSH_BOTH);
for(i = 0; i < 4000 ; i++)
{
	while (1)
	{

		ioctl(fd, FIONREAD, &n);
		if(n>0)
		{
		//	printf("Disponibles: %d\n", n);
			b = n > 5000 ? 5000: n;
			read(fd, buf, b);
			//printf("Leidos %d\n", b);
			break;
		}
		//else printf("No hay datos\n");
	}

	n = write(fd, "1", 1);
	if (n < 0)
	  fputs("write() of 4 bytes failed!\n", stderr);


	//printf("Iteración: %d, Escritos: %d\n", i,n);

}
ioctl(fd,TCFLSH, FLUSH_OUT);
//fputs("Bye!", stdout);
close(fd);*/

//			printf("Iteracion %d ->0x%02X", i,(uint8_t)*((&rem)));
//			printf("%02X", i,(uint8_t)*((&rem)+1));
//			printf("%02X", i,(uint8_t)*((&rem)+2));
//			printf("%02X\n", i,(uint8_t)*((&rem)+3));
