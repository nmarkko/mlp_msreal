#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>

#include "images.h"

#define MAX_PKT_SIZE (24644*4)

//comment to send pixels as commands via regular write function of char driver
//leave uncommented to write directly to memory (faster)
#define MMAP

int main(void)
{
	char classified[20]= "";
	// set start reg to 1
	fp = fopen("/dev/mlp", "w");
	if(fp == NULL)
	{
		printf("Cannot open /dev/mlp for write\n");
		return -1;
	}
	fprintf(fp,"%d",1); 
	fclose(fp);
	if(fp == NULL)
	{
		printf("Cannot close /dev/mlp\n");
		return -1;
	}
	//reset start reg
	fp = fopen("/dev/mlp", "w");
	if(fp == NULL)
	{
		printf("Cannot open /dev/mlp for write\n");
		return -1;
	}
	fprintf(fp,"%d",0);
	fclose(fp);
	if(fp == NULL)
	{
		printf("Cannot close /dev/mlp\n");
		return -1;
	}
	
	// send image and params directly via mmap
	int fd;
	int *p;
	fd = open("/dev/image", O_RDWR|O_NDELAY);
	if (fd < 0)
	{
		printf("Cannot open /dev/image for write\n");
		return -1;
	}
	p=(int*)mmap(0,24644*4, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0); // pixels + mlp parameters
	memcpy(p, image, MAX_PKT_SIZE);
	munmap(p, MAX_PKT_SIZE);
	close(fd);
	if (fd < 0)
	{
		printf("Cannot close /dev/image for write\n");
		return -1;
	}
	
	// read classification result
	fp = fopen("/dev/mlp", "r");
	if(fp == NULL)
	{
		printf("Cannot open /dev/mlp for read\n");
		return -1;
	}
	fscanf(fp,"%s",classified);
	fclose(fp);
	if(fp == NULL)
	{
		printf("Cannot close /dev/mlp\n");
		return -1;
	}

	printf("%s", classified);

	return 0;
}

