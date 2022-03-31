#ifndef __APP__
#define __APP__
#pragma once
//#include <Windows.h>
#include <k4a/k4a.h>
#include <opencv2/opencv.hpp>
#include "Vector.h"

//#include <vector>
//#include <wrl/client.h>
//using namespace Microsoft::WRL;



class Kinect
{
public:
	// Constructor
	Kinect();

	// Destructor
	~Kinect();


	void run();

	void getColor(cv::Mat& color);

	void getDepth(cv::Mat& depth);

	void getIMU(k4a::Vector& vec);

    void getGravity(k4a::Vector& vec);

    void save_calibration_file(std::string save_path);


private:
    cv::Mat imBGRA;
    //cv::Mat imBGR, imIr;
    cv::Mat imD;
    //cv::Mat im_transformed_color_image;
    cv::Mat im_transformed_depth_image;

    int returnCode = 1;
    k4a_device_t device = NULL;
    const int32_t TIMEOUT_IN_MS = 1000;
    k4a_transformation_t transformation = NULL;

    int captureFrameCount;
    k4a_capture_t capture = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    k4a_image_t ir_image = NULL;
    k4a_image_t transformed_depth_image = NULL;

    bool continueornot = true;

    k4a_calibration_t calibration;


    k4a_imu_sample_t imu_sample;
    k4a::Vector imu_Acc;
    k4a::Vector imu_grav;




	// Initialize
	void initialize();

	// Initialize Sensor
	inline void initializeSensor();

	// Finalize
	void finalize();

	// Update Data
	void update();

	// Update Color
	inline void updateColor();

	// Update Depth
	inline void updateDepth();


	//update IMU
    inline void updateIMU();
    inline k4a::Vector extract_gravity_from_imu(k4a::Vector& imuAcc);

	//transform Depth to Color
	inline void transform_depth_to_color();


	// Show Data
	void show();

	// Show Color
	inline void showColor();

	// Show Depth
	inline void showDepth();

	void reflectImage(const cv::Mat image, cv::Mat& reflectMat);

    // Processing
    static std::string get_serial(k4a_device_t device);


};

#endif // __APP__