#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <cstdio>
#include <sys/time.h>
#include <iostream>
using namespace cv;
using namespace std;
/// Global variables

Mat src, src_gray;
Mat dst, detected_edges, nnoise;


int edgeThresh = 1;
int lowThreshold = 40;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;

struct timeval t0,t1;
/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
static void mostrarTiempo(const char * info, struct timeval *t0, struct timeval *t1)
{
	long long elapsed = (t1->tv_sec-t0->tv_sec)*1000000LL + t1->tv_usec-t0->tv_usec;

	fprintf(stdout, "%s:\t%lld us\n",info, elapsed);

//	std::cout << "Elapsed: "<<  elapsed << std::endl;
	
}
void CannyThreshold(int, void*)
{

  /// Reduce noise with a kernel 3x3

  gettimeofday(&t0, NULL);
  GaussianBlur( src_gray, nnoise, Size(5,5),1 ,1);
  gettimeofday(&t1, NULL);
  mostrarTiempo("Noise",&t0,&t1); 

  gettimeofday(&t0, NULL);
  /// Canny detector
  Canny( nnoise, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
  gettimeofday(&t1, NULL);
  mostrarTiempo("Canny",&t0,&t1); 

  /// Using Canny's output as a mask, we display our result
  //dst = Scalar::all(0);

  //src.copyTo( dst, detected_edges);
  //imshow( window_name, dst );
 }


/** @function main */
int main( int argc, char** argv )
{

  gettimeofday(&t0, NULL);
  /// Load an image
  src = imread( argv[1] );
  gettimeofday(&t1, NULL);
  mostrarTiempo("Leer imagen",&t0,&t1); 

  if( !src.data )
  { return -1; }

  /// Create a matrix of the same type and size as src (for dst)
  dst.create( src.size(), src.type() );

  gettimeofday(&t0, NULL);
  /// Convert the image to grayscale
  cvtColor( src, src_gray, CV_BGR2GRAY );
  gettimeofday(&t1, NULL);
  mostrarTiempo("Convertir a GS",&t0,&t1); 


  /// Show the image
  CannyThreshold(0, 0);

  imwrite("no-noise.pgm",nnoise);
  imwrite("edges.pgm", detected_edges);
  return 0;
 }
