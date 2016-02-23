#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <cstdio>
#include <sys/time.h>
#include <iostream>
using namespace cv;

/// Global variables

Mat src, src_gray;
Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Edge Map";

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

struct timeval t0,t1;
  gettimeofday(&t0, NULL);
  /// Reduce noise with a kernel 3x3
  blur( src_gray, detected_edges, Size(5,5) );

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  gettimeofday(&t1, NULL);
	mostrarTiempo("Elapsed: ",&t0,&t1); 

  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);

  src.copyTo( dst, detected_edges);
  imshow( window_name, dst );
 }


/** @function main */
int main( int argc, char** argv )
{
  /// Load an image
  src = imread( argv[1] );

  if( !src.data )
  { return -1; }

  /// Create a matrix of the same type and size as src (for dst)
  dst.create( src.size(), src.type() );

  /// Convert the image to grayscale
  cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Create a window
  namedWindow( window_name, CV_WINDOW_AUTOSIZE );

  /// Create a Trackbar for user to enter threshold
  createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

  /// Show the image
  CannyThreshold(0, 0);

  /// Wait until user exit program by pressing a key
  waitKey(0);

  return 0;
  }
