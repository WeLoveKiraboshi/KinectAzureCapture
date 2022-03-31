#include "../include/captureRGBD.h"
#include "../include/Vector.h"

#include <string>
#include <sys/time.h>

//use Linux for only mnkdir
#include <sys/stat.h>
#include <sys/types.h>


using namespace std;

#define CROPPING

Capture::Capture(const int img_width, const int img_height)
	: img_width(img_width),
	img_height(img_height)
{
	


}


Capture::~Capture()
{
	
}

typedef struct {
	double r, g, b;
} COLOUR;

COLOUR GetColour(double v, double vmin, double vmax)
{
	COLOUR c = { 1.0,1.0,1.0 }; // white
	double dv;

	if (v < vmin)
		v = vmin;
	if (v > vmax)
		v = vmax;
	dv = vmax - vmin;

	if (v < (vmin + 0.25 * dv)) {
		c.r = 0;
		c.g = 4 * (v - vmin) / dv;
	}
	else if (v < (vmin + 0.5 * dv)) {
		c.r = 0;
		c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
	}
	else if (v < (vmin + 0.75 * dv)) {
		c.r = 4 * (v - vmin - 0.5 * dv) / dv;
		c.b = 0;
	}
	else {
		c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
		c.b = 0;
	}

	return(c);
}

double interpolate(double val, double y0, double x0, double y1, double x1) {
	return (val - x0)*(y1 - y0) / (x1 - x0) + y0;
}

double base(double val) {
	if (val <= -0.75) return 0;
	else if (val <= -0.25) return interpolate(val, 0.0, -0.75, 1.0, -0.25);
	else if (val <= 0.25) return 1.0;
	else if (val <= 0.75) return interpolate(val, 1.0, 0.25, 0.0, 0.75);
	else return 0.0;
}

double red(double gray) {
	return base(gray - 0.5);
}
double green(double gray) {
	return base(gray);
}
double blue(double gray) {
	return base(gray + 0.5);
}

cv::Mat cvtJetColor(cv::Mat input)
{
	cv::Mat scale_mat;
	input.convertTo(scale_mat, CV_8U, -255.0 / 10000.0, 255.0);

	/*double min, max;
	cv::minMaxLoc(scale_mat, &min, &max);*/
	double min = 0.0, max = 255.0;

	cv::Mat colorMat = cv::Mat(scale_mat.size(), CV_8UC3);
	for (int irow = 0; irow < scale_mat.rows; ++irow)
	{
		unsigned char *gray = scale_mat.ptr<unsigned char>(irow);
		cv::Vec3b* color = colorMat.ptr<cv::Vec3b>(irow);
		for (int icol = 0; icol < scale_mat.cols; ++icol)
		{
			unsigned char grayPtr = gray[icol];
			if ((int)grayPtr == 255)
			{
				cv::Vec3b colorVal = cv::Vec3b(0, 0, 0);
				color[icol] = colorVal;
			}
			else
			{

				double grayVal = ((double)grayPtr - min) / (max - min) * 2.0 - 1.0;

				cv::Vec3b colorVal;
				colorVal[2] = uint8_t(blue(grayVal) * 0xff);
				colorVal[1] = uint8_t(green(grayVal) * 0xff);
				colorVal[0] = uint8_t(red(grayVal) * 0xff);

				color[icol] = colorVal;
			}

		}
	}
	return colorMat;

}

//struct timezone
//{
//	int  tz_minuteswest; /* minutes W of Greenwich */
//	int  tz_dsttime;     /* type of dst correction */
//};


//int gettimeofday(struct timeval_ * tp, struct timezone * tzp)
//{
//	// Note: some broken versions only have 8 trailing zero's, the correct epoch has 9 trailing zero's
//	// This magic number is the number of 100 nanosecond intervals since January 1, 1601 (UTC)
//	// until 00:00:00 January 1, 1970
//	static const uint64_t EPOCH = ((uint64_t)116444736000000000ULL);
//
//	SYSTEMTIME  system_time;
//	FILETIME    file_time;
//	uint64_t    time;
//
//	GetSystemTime(&system_time);
//	SystemTimeToFileTime(&system_time, &file_time);
//	time = ((uint64_t)file_time.dwLowDateTime);
//	time += ((uint64_t)file_time.dwHighDateTime) << 32;
//
//	tp->tv_sec = (long)((time - EPOCH) / 10000000L);
//	tp->tv_usec = (long)(system_time.wMilliseconds * 1000);
//	return 0;
//}
//


void Capture::sysLoop()
{
		
	std::string folderPath = "";
	std::string colorFolderPath = "";
	std::string depthFolderPath = "";
    std::string gravityFolderPath = "";
    std::string imuFolderPath = "";
	std::vector<cv::Mat> vColorMat, vDepthMat;
	std::vector<k4a::Vector> vIMUVec, vGravityVec;
	std::vector<std::string> vColorName, vDepthName, vIMUName, vGravityName;

	bool saveFrames = false;
	bool saveImages = false;
	int idx = 0;
	std::string windowName = "frame";

	while (true) 
	{
		double timer = (double)cv::getTickCount();
		time_t sec_time;
		time_t millisec_time;
        //Windows use this
		//		timeval_ tv;
        //		gettimeofday(&tv, NULL);

		//Ubuntu use this
        struct timeval tv;
        gettimeofday(&tv, NULL);


		sec_time = tv.tv_sec;
		millisec_time = tv.tv_usec;


		cv::Mat colorOri = cv::Mat(cv::Size(img_width, img_height), CV_8UC3, cv::Scalar::all(0));
		cv::Mat color_8UC4 = cv::Mat(cv::Size(img_width, img_height), CV_8UC4, cv::Scalar::all(0));
		cv::Mat depthOri = cv::Mat(cv::Size(img_width, img_height), CV_16UC1, cv::Scalar::all(0));
			
		kinect.run();
		kinect.getColor(color_8UC4);
		kinect.getDepth(depthOri);
        k4a::Vector IMU_vec;
        k4a::Vector grav_vec;
        //kinect.getIMU(IMU_vec);
        //kinect.getGravity(grav_vec);
		
		cv::cvtColor(color_8UC4, colorOri, CV_BGRA2BGR);
		color_8UC4.release();
		
		cv::Mat color, depth;
		colorOri(cv::Rect((colorOri.cols - img_width) / 2, (colorOri.rows - img_height) / 2, img_width, img_height)).copyTo(color);
		depthOri(cv::Rect((colorOri.cols - img_width) / 2, (colorOri.rows - img_height) / 2, img_width, img_height)).copyTo(depth);

		if (saveFrames && !saveImages)
		{
			vColorMat.push_back(color.clone());
			vDepthMat.push_back(depth.clone());
            vIMUVec.push_back(IMU_vec);
            vGravityVec.push_back(grav_vec);

			char colorName[256], depthName[256],IMUName[256],gravityName[256];
			sprintf(colorName, "/%10d.%06d.png", sec_time, millisec_time);
			sprintf(depthName, "/%10d.%06d.png", sec_time, millisec_time);
            sprintf(IMUName, "/%10d.%06d.txt", sec_time, millisec_time);
            sprintf(gravityName, "/%10d.%06d.txt", sec_time, millisec_time);
			std::string colorPath(colorName);
			std::string depthPath(depthName);
            std::string imuPath(IMUName);
            std::string gravityPath(gravityName);
			vColorName.push_back(colorPath);
			vDepthName.push_back(depthPath);
			vIMUName.push_back(imuPath);
			vGravityName.push_back(gravityPath);

			idx++;
		}

		cv::Mat showPallet;
		cv::Mat jet = cvtJetColor(depth.clone());
		float fps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);
		std::ostringstream fps_ss;
		fps_ss << fps;
		std::string fps_s(fps_ss.str());
		std::ostringstream idx_ss;
		idx_ss << idx;
		std::string idx_s(idx_ss.str());
		cv::hconcat(color.clone(), jet, showPallet);
		putText(showPallet, "FPS        : " + fps_s, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 170, 50), 2);
		putText(showPallet, "SAVE FRAME : " + idx_s, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 170, 50), 2);
		if (saveFrames && !saveImages)
		{
			putText(showPallet, "MODE       : Video Capture", cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 170, 50), 2);
		}
		else if (saveImages && !saveFrames)
		{
			putText(showPallet, "MODE       : Image Capture", cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 170, 50), 2);
		}
		else
		{
			putText(showPallet, "MODE       : ", cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 170, 50), 2);
		}

		cv::imshow(windowName, showPallet);
		int key = cv::waitKey(1);
		
		if (key == 27) {
			break;
		}
		else if (key == 's' && !saveFrames && !saveImages)
		{
			char folderName[256];
			time_t now = time(NULL);
			struct tm *pnow = localtime(&now);
			sprintf(folderName, "video%02d%02d%02d%02d", pnow->tm_mon + 1, pnow->tm_mday,
				pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
			std::string str(folderName);
			folderPath = strData + str;
			mkdir(folderPath.c_str(), 0775);

			char folderNameColor[256];
			char folderNameDepth[256];
            char folderNameGravity[256];
            char folderNameIMU[256];
			sprintf(folderNameColor, "/rgb");
			sprintf(folderNameDepth, "/depth");
            sprintf(folderNameGravity, "/gravity");
            sprintf(folderNameIMU, "/imu");
			std::string tmpPathColor(folderNameColor);
			std::string tmpPathDepth(folderNameDepth);
            std::string tmpPathGravity(folderNameGravity);
            std::string tmpPathIMU(folderNameIMU);
			colorFolderPath = strData + str + tmpPathColor;
			depthFolderPath = strData + str + tmpPathDepth;
            gravityFolderPath = strData + str + tmpPathGravity;
            imuFolderPath = strData + str + tmpPathIMU;
			mkdir(colorFolderPath.c_str(), 0775);
			mkdir(depthFolderPath.c_str(), 0775);
            mkdir(gravityFolderPath.c_str(), 0775);
            mkdir(imuFolderPath.c_str(), 0775);

			saveFrames = true;
            std::string calibrationFilePath = strData + str + "/calibration.txt";
            kinect.save_calibration_file(calibrationFilePath);
		}
		
		color.release();
		depth.release();

	}

	if (saveFrames)
	{
        std::ofstream rgb_file_writer;
        std::ofstream depth_file_writer;
        std::ofstream associate_file_writer;
        rgb_file_writer.open(folderPath+"/rgb.txt", std::ios::out);
        depth_file_writer.open(folderPath+"/depth.txt", std::ios::out);
        associate_file_writer.open(folderPath+"/associate.txt", std::ios::out);

		std::cout << "Save images" << std::endl;
		for (int i = 0; i < idx; ++i)
		{

			cv::Mat color = vColorMat[i];
			cv::Mat depth = vDepthMat[i];
			k4a::Vector imu_vec = vIMUVec[i];
			k4a::Vector gravity_vec = vGravityVec[i];
			cv::Mat jet = cvtJetColor(depth.clone());

			cv::Mat showPallet;
			cv::hconcat(color.clone(), jet, showPallet);
			std::ostringstream idx_ss;
			idx_ss << i;
			std::string idx_s(idx_ss.str());
			putText(showPallet, "FPS        : ", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 170, 50), 2);
			putText(showPallet, "SAVE FRAME : " + idx_s, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 170, 50), 2);
			putText(showPallet, "MODE       : Saving", cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 170, 50), 2);
			cv::imshow(windowName, showPallet);
			cv::waitKey(1);

			std::string colorName = vColorName[i];
			std::string depthName = vDepthName[i];
            std::string imuName = vIMUName[i];
            std::string gravityName = vGravityName[i];
			cv::imwrite(colorFolderPath + colorName, color);
			cv::imwrite(depthFolderPath + depthName, depth);

            std::ofstream ofs_imu(imuFolderPath+imuName);
            ofs_imu << imu_vec.X << std::endl;
            ofs_imu << imu_vec.Y << std::endl;
            ofs_imu << imu_vec.Z << std::endl;


            std::ofstream ofs_gravity(gravityFolderPath+gravityName);
            ofs_gravity << gravity_vec.X << std::endl;
            ofs_gravity << gravity_vec.Y << std::endl;
            ofs_gravity << gravity_vec.Z << std::endl;

            std::string timestamp = colorName.substr(1, 17);
            rgb_file_writer << timestamp << " " << "rgb"+colorName << std::endl;
            depth_file_writer << timestamp << " " << "depth"+depthName << std::endl;
            associate_file_writer << timestamp << " " << "rgb"+colorName << " " << timestamp << " " << "depth"+depthName << std::endl;
		}

        rgb_file_writer.close();
        depth_file_writer.close();
        associate_file_writer.close();
	}


	cv::destroyAllWindows();
	kinect.~Kinect();

	return;
}

