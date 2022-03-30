#pragma once
#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <iterator>
//#include <Windows.h>
//#include <direct.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <omp.h>
#include <time.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "KinectCapture.h"
#include "DirectoryConfig.h"

// MSVC defines this in winsock2.h!?
;typedef struct timeval_ {
	long tv_sec;
	long tv_usec;
} timeval_;


class Capture {
public:
	Capture(const int img_width, const int img_height);
	~Capture();

	void sysLoop();

private:

	const int img_width, img_height;
	Kinect kinect;

	

	

};

