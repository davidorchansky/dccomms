#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <cstdio>

//Temporal
#include <chrono>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int clip(int v)
{
	cout << "clipping value: " << v << endl;
	return v;
}

int
writePPMHeader(FILE* file, int w, int h)
{
	char header[100];
	int n=sprintf(header, "P6\n%d %d\n255\n", w, h);
	fwrite(header, 1, n, file);

	return 0;
}
void getRGB(int y, int u, int v, int * R, int *G, int *B)
{
	int C = y - 16;
	int D = u - 128;
	int E = v - 128;

	/*
	*R = clip(( 298 * C           + 409 * E + 128) >> 8);
	*G = clip(( 298 * C - 100 * D - 208 * E + 128) >> 8);
	*B = clip(( 298 * C + 516 * D           + 128) >> 8);
	*/
	*R = ( 298 * C           + 409 * E + 128) >> 8;
	*G = ( 298 * C - 100 * D - 208 * E + 128) >> 8;
	*B = ( 298 * C + 516 * D           + 128) >> 8;

}

void yuv420p_to_rgb(int width, int height, uint8_t * y, uint8_t *u, uint8_t *v, void * buffer)
{
	int milis = 1000;
	int uvwidth = width / 2;
	int uvheight = height / 2;
	int uvrow;
	int uvcol;
	int max=1, min=1;
	float m, b;

	//int _buffer[width*height*3];
//	int * ptr = _buffer;

	int16_t * ptr = (int16_t*) buffer;
	for(int row = 0; row < height; row++)
	{

		uvrow = row / 2;

		for(int col = 0; col < width; col++)
		{
			int R,G,B;
			uint8_t * yptr, *uptr, *vptr;
			yptr = y + row * width + col;
			uvcol = col / 2;

			//std::cout << "R : G : B en ("<<row<<","<<col<<"): ";
//			std::cout << "uvrow: " << uvrow << " uvcol: " << uvcol << std::endl;
			int uvoffset = uvrow * uvwidth + uvcol;
			uptr = u + uvoffset;
			vptr = v + uvoffset;

			getRGB(*yptr,*uptr,*vptr, &R, &G, &B);
//			std::cout << R << " : " << G << " : " << B << std::endl;
			if(R < min)min = R;
			if(G < min)min = G;
			if(B < min)min = B;
			if(R > max)max = R;
			if(G > max)max = G;
			if(B > max)max = B;
			*ptr = R;
			*(ptr+1) = G;
			*(ptr+2) = B;
			ptr+=3;
//			std::this_thread::sleep_for(std::chrono::milliseconds(milis));

		}
	}
	m = 255.0 / (max-min);
	b = 0-m*min;
	int length = width*height*3;
	std::cout << length << std::endl;

	ptr = (int16_t*) buffer;
	uint8_t * dst = (uint8_t *) buffer;
	for(int e = 0; e < length; e++)
	{
	   dst[e] = m*ptr[e]+b;
	  // std::cout << (int)*ptr << std::endl;
	//	std::this_thread::sleep_for(std::chrono::milliseconds(milis));
	}

	//std::cout << min << " " << max << " " << m << " " << b  <<std::endl;
	
}
int main(int argc, char ** argv)
{
	if(argc != 5)
	{
		std::cerr << "Numero de argumentos incorrecto" << std::endl;
		std::cout << "Usage:\n\targs: <Width> <Height> <inputFile> <outputFile>" << std::endl;
		exit(1);
	}

	int width = atoi(argv[1]);
	int height = atoi(argv[2]);
	char * inputFileName = argv[3];
	char * outputFileName = argv[4];	

	uint8_t inputBuffer[1280*720+1280*720/2];

	FILE * inputFile = fopen(inputFileName, "rb");
	
	int ysize = width*height;
	int usize = width*height/4;
	int vsize = usize;

	int rawSize = ysize + usize + vsize;

	cout << rawSize << endl;

	uint8_t * y = inputBuffer;
	uint8_t * u = y + ysize;
	uint8_t * v = u + usize;


	int bytesRead = fread(y, 1, ysize, inputFile);
	bytesRead += fread(u, 1, usize, inputFile);
	bytesRead += fread(v, 1, vsize, inputFile);

	fclose(inputFile);

	cout << "Bytes read: " << bytesRead << endl;

	int imSize = width * height * 3 * 15;
	uint8_t  * resBuffer = (uint8_t*)malloc(imSize);

	yuv420p_to_rgb(width, height, y, u, v, resBuffer);

	FILE * outputFile = fopen(outputFileName, "wb");

	writePPMHeader(outputFile, width, height);
	fwrite(resBuffer, 1, imSize, outputFile);

	fclose(outputFile);

		//std::this_thread::sleep_for(std::chrono::milliseconds(2000));
/*	Mat image;
	image = imread(outputFileName, CV_LOAD_IMAGE_COLOR);   // Read the file

	if(! image.data )                              // Check for invalid input
	{
		cout <<  "Could not open or find the image" << std::endl ;
		return -1;
	}
*/
//	namedWindow( "IRS ROV Camera", WINDOW_AUTOSIZE );// Create a window for display.
//imshow( "IRS ROV Camera", image );                   // Show our image inside it.

//Mat image2 = imread("res2.ppm", CV_LOAD_IMAGE_COLOR);
//imshow("IRS ROV Camera", image2);
//	int i = 0;
//	while(1){
//i++;
//	waitKey(1);
//	std::cout << "MERDA "<<i << std::endl;
//	};
	return 0;
}
