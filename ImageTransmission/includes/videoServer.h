#ifndef VIDEOSERVER_H
#define VIDEOSERVER_H

#include<stdio.h>
#include<string.h>    
#include<stdlib.h>   
#include<sys/socket.h>
#include<arpa/inet.h> 
#include<unistd.h>    
#include<ctype.h>
#include<signal.h>

struct videoTransmissionConfig
{
	unsigned int frameSize;
	unsigned int maxPacketLength;
	unsigned int delayBetweenPackets;
	unsigned int maxFrameAge;
	unsigned int width;
	unsigned int height;

} typedef videoTransmissionConfig;

int
readint(int fd, unsigned int *x, char next)
{
	int error = -1;
	int v = 0;
	char c;
	int n = read(fd, &c, 1);
	if (n > 0 && isdigit(c)) {
		v = c - '0';
		n = read(fd, &c, 1);
		while (n > 0 && isdigit(c)) {
			v = v * 10 + (c - '0');
			n = read(fd, &c, 1);
		}
		*x = v;
		if(c == next)
			error = 0;
	}
	return error;
}

int waitFor(int fd, char* str)
{
	unsigned int length = strlen(str);

	char * ptr = str;
	int nc = 0;
	
	char c=' ';
	
	while(nc < length)
	{
		if(read(fd, &c, 1)>0)
		{
			if(c != str[nc])
				return -1;
			nc += 1;
		}
		else
			return -1;
	}
	return 0;
}

int getParameter(int fd, const char *par, unsigned int *value, char end)
{
	char buffer[30];
	sprintf(buffer, "%s:",par);	
	if(waitFor(fd, buffer)==-1)return -1;
	return readint(fd, value, end);

	return 0;
}

int getVideoTransmissionConfig(int sock,
	videoTransmissionConfig *config)
{
	if(getParameter(sock, "frameSize", &config->frameSize, ',')==-1)return -1;
	if(getParameter(sock, "maxPacketLength", &config->maxPacketLength, ',')==-1)return -1;
	if(getParameter(sock, "delayBetweenPackets", &config->delayBetweenPackets, ',')==-1)return -1;
	return getParameter(sock, "maxFrameAge", &config->maxFrameAge, '}');
	return 0;
}

int buildVideoTransmissionConfigMsg(char *buffer, videoTransmissionConfig *config)
{
	int n;
	n = sprintf(buffer, "{frameSize:%d,maxPacketLength:%d,delayBetweenPackets:%d,maxFrameAge:%d}",
			config->frameSize,
			config->maxPacketLength,
			config->delayBetweenPackets,
			config->maxFrameAge);

	return n;
}

#endif
