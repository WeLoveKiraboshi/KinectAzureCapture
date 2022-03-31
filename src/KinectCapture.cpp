#include "../include/KinectCapture.h"
#include "../include/util.h"

#include <thread>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>


// Choose Resolution
#define COLOR
// #define DEPTH


using namespace std;

//template<typename T> cv::Mat create_mat_from_buffer(T *data, int width, int height, int channels = 1)
//{
//    cv::Mat mat(height, width, CV_MAKETYPE(DataType<T>::type, channels));
//    memcpy(mat.data, data, width * height * channels * sizeof(T));
//    return mat;
//}


// Constructor
Kinect::Kinect()
{
	// Initialize
	initialize();
}

// Destructor
Kinect::~Kinect()
{
	// Finalize
	finalize();
}

// Processing
void Kinect::run()
{
	// Update Data
	update();

	// Show Data
	//show();
}

// Initialize
void Kinect::initialize()
{
	cv::setUseOptimized(true);
    initializeSensor();

	// Wait a Few Seconds until begins to Retrieve Data from Sensor ( about 2000-[ms] )
	std::this_thread::sleep_for(std::chrono::seconds(2));
}

// Initialize Sensor
inline void Kinect::initializeSensor()
{
    uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0)
    {
        cout << "No K4A devices found" << endl;
        exit(0);
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        cout << "No K4A devices found" << endl;
        if (device != NULL)
        {
            k4a_device_close(device);
        }
    }
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.synchronized_images_only = true;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;//examples/transformation
    config.color_resolution = K4A_COLOR_RESOLUTION_1536P; //K4A_COLOR_RESOLUTION_720P;
    //config.depth_format = K4A_IMAGE_FORMAT_DEPTH16;//https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/

    //config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;//640x576.
    config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;//1024x1024.
    config.camera_fps = K4A_FRAMES_PER_SECOND_15; //30
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        cout << "Failed to get calibration" << endl;
        exit(0);
    }
    // const int width = calibration.depth_camera_calibration.resolution_width;
    // const int height = calibration.depth_camera_calibration.resolution_height;
    k4a_calibration_camera_t calib = calibration.depth_camera_calibration;
    cout << "\n===== Device "  << ": " << get_serial(device) << " =====\n";
    cout << "depth camera resolution width: " << calib.resolution_width << endl;
    cout << "depth camera resolution height: " << calib.resolution_height << endl;
    cout << "depth camera principal point x: " << calib.intrinsics.parameters.param.cx << endl;
    cout << "depth camera principal point y: " << calib.intrinsics.parameters.param.cy << endl;
    cout << "depth camera focal length x: " << calib.intrinsics.parameters.param.fx << endl;
    cout << "depth camera focal length y: " << calib.intrinsics.parameters.param.fy << endl;
    cout << "depth camera radial distortion coefficients:" << endl;
    cout << "depth camera k1: " << calib.intrinsics.parameters.param.k1 << endl;
    cout << "depth camera k2: " << calib.intrinsics.parameters.param.k2 << endl;
    cout << "depth camera k3: " << calib.intrinsics.parameters.param.k3 << endl;
    cout << "depth camera k4: " << calib.intrinsics.parameters.param.k4 << endl;
    cout << "depth camera k5: " << calib.intrinsics.parameters.param.k5 << endl;
    cout << "depth camera k6: " << calib.intrinsics.parameters.param.k6 << endl;
    cout << "depth camera center of distortion in Z=1 plane, x: " << calib.intrinsics.parameters.param.codx << endl;
    cout << "depth camera center of distortion in Z=1 plane, y: " << calib.intrinsics.parameters.param.cody << endl;
    cout << "depth camera tangential distortion coefficient x: " << calib.intrinsics.parameters.param.p1 << endl;
    cout << "depth camera tangential distortion coefficient y: " << calib.intrinsics.parameters.param.p2 << endl;
    cout << "depth camera metric radius: " << calib.intrinsics.parameters.param.metric_radius << endl;

    k4a_calibration_camera_t calib1 = calibration.color_camera_calibration;
    cout << "color camera resolution width: " << calib1.resolution_width << endl;
    cout << "color camera resolution height: " << calib1.resolution_height << endl;
    cout << "color camera principal point x: " << calib1.intrinsics.parameters.param.cx << endl;
    cout << "color camera principal point y: " << calib1.intrinsics.parameters.param.cy << endl;
    cout << "color camera focal length x: " << calib1.intrinsics.parameters.param.fx << endl;
    cout << "color camera focal length y: " << calib1.intrinsics.parameters.param.fy << endl;
    cout << "color camera radial distortion coefficients:" << endl;
    cout << "color camera k1: " << calib1.intrinsics.parameters.param.k1 << endl;
    cout << "color camera k2: " << calib1.intrinsics.parameters.param.k2 << endl;
    cout << "color camera k3: " << calib1.intrinsics.parameters.param.k3 << endl;
    cout << "color camera k4: " << calib1.intrinsics.parameters.param.k4 << endl;
    cout << "color camera k5: " << calib1.intrinsics.parameters.param.k5 << endl;
    cout << "color camera k6: " << calib1.intrinsics.parameters.param.k6 << endl;
    cout << "color camera center of distortion in Z=1 plane, x: " << calib1.intrinsics.parameters.param.codx << endl;
    cout << "color camera center of distortion in Z=1 plane, y: " << calib1.intrinsics.parameters.param.cody << endl;
    cout << "color camera tangential distortion coefficient x: " << calib1.intrinsics.parameters.param.p1 << endl;
    cout << "color camera tangential distortion coefficient y: " << calib1.intrinsics.parameters.param.p2 << endl;
    cout << "color camera metric radius: " << calib1.intrinsics.parameters.param.metric_radius << endl;

    transformation = k4a_transformation_create(&calibration);
    //transformation_color_IMU_R = calibration.extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_COLOR].rotation;

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        cout << "Failed to start cameras" << endl;
        exit(0);
    }
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_imu(device))
    {
        cout <<"Failed to get imu data from capture" << endl;
        exit(0);
    }
}

// get device serial num
inline string Kinect::get_serial(k4a_device_t device)
{
    size_t serial_number_length = 0;

    if (K4A_BUFFER_RESULT_TOO_SMALL != k4a_device_get_serialnum(device, NULL, &serial_number_length))
    {
        cout << "Failed to get serial number length" << endl;
        k4a_device_close(device);
        exit(-1);
    }

    char *serial_number = new (std::nothrow) char[serial_number_length];
    if (serial_number == NULL)
    {
        cout << "Failed to allocate memory for serial number (" << serial_number_length << " bytes)" << endl;
        k4a_device_close(device);
        exit(-1);
    }

    if (K4A_BUFFER_RESULT_SUCCEEDED != k4a_device_get_serialnum(device, serial_number, &serial_number_length))
    {
        cout << "Failed to get serial number" << endl;
        delete[] serial_number;
        serial_number = NULL;
        k4a_device_close(device);
        exit(-1);
    }

    string s(serial_number);
    delete[] serial_number;
    serial_number = NULL;
    return s;
}


// Finalize
void Kinect::finalize()
{
	cv::destroyAllWindows();

	// Close Sensor
    k4a_device_close(device);
}


void Kinect::save_calibration_file(std::string save_path)
{
    std::ofstream writing_file;
    writing_file.open(save_path, std::ios::out);
    writing_file << "\n===== Device "  << ": " << get_serial(device) << " =====\n";
    k4a_calibration_camera_t calib = calibration.depth_camera_calibration;
    writing_file << "depth camera resolution width: " << calib.resolution_width << endl;
    writing_file << "depth camera resolution height: " << calib.resolution_height << endl;
    writing_file << "depth camera principal point x: " << calib.intrinsics.parameters.param.cx << endl;
    writing_file << "depth camera principal point y: " << calib.intrinsics.parameters.param.cy << endl;
    writing_file << "depth camera focal length x: " << calib.intrinsics.parameters.param.fx << endl;
    writing_file << "depth camera focal length y: " << calib.intrinsics.parameters.param.fy << endl;
    writing_file << "depth camera radial distortion coefficients:" << endl;
    writing_file << "depth camera k1: " << calib.intrinsics.parameters.param.k1 << endl;
    writing_file << "depth camera k2: " << calib.intrinsics.parameters.param.k2 << endl;
    writing_file << "depth camera k3: " << calib.intrinsics.parameters.param.k3 << endl;
    writing_file << "depth camera k4: " << calib.intrinsics.parameters.param.k4 << endl;
    writing_file << "depth camera k5: " << calib.intrinsics.parameters.param.k5 << endl;
    writing_file << "depth camera k6: " << calib.intrinsics.parameters.param.k6 << endl;
    writing_file << "depth camera center of distortion in Z=1 plane, x: " << calib.intrinsics.parameters.param.codx << endl;
    writing_file << "depth camera center of distortion in Z=1 plane, y: " << calib.intrinsics.parameters.param.cody << endl;
    writing_file << "depth camera tangential distortion coefficient x: " << calib.intrinsics.parameters.param.p1 << endl;
    writing_file << "depth camera tangential distortion coefficient y: " << calib.intrinsics.parameters.param.p2 << endl;
    writing_file << "depth camera metric radius: " << calib.intrinsics.parameters.param.metric_radius << endl;

    k4a_calibration_camera_t calib1 = calibration.color_camera_calibration;
    writing_file << "color camera resolution width: " << calib1.resolution_width << endl;
    writing_file << "color camera resolution height: " << calib1.resolution_height << endl;
    writing_file << "color camera principal point x: " << calib1.intrinsics.parameters.param.cx << endl;
    writing_file << "color camera principal point y: " << calib1.intrinsics.parameters.param.cy << endl;
    writing_file << "color camera focal length x: " << calib1.intrinsics.parameters.param.fx << endl;
    writing_file << "color camera focal length y: " << calib1.intrinsics.parameters.param.fy << endl;
    writing_file << "color camera radial distortion coefficients:" << endl;
    writing_file << "color camera k1: " << calib1.intrinsics.parameters.param.k1 << endl;
    writing_file << "color camera k2: " << calib1.intrinsics.parameters.param.k2 << endl;
    writing_file << "color camera k3: " << calib1.intrinsics.parameters.param.k3 << endl;
    writing_file << "color camera k4: " << calib1.intrinsics.parameters.param.k4 << endl;
    writing_file << "color camera k5: " << calib1.intrinsics.parameters.param.k5 << endl;
    writing_file << "color camera k6: " << calib1.intrinsics.parameters.param.k6 << endl;
    writing_file << "color camera center of distortion in Z=1 plane, x: " << calib1.intrinsics.parameters.param.codx << endl;
    writing_file << "color camera center of distortion in Z=1 plane, y: " << calib1.intrinsics.parameters.param.cody << endl;
    writing_file << "color camera tangential distortion coefficient x: " << calib1.intrinsics.parameters.param.p1 << endl;
    writing_file << "color camera tangential distortion coefficient y: " << calib1.intrinsics.parameters.param.p2 << endl;
    writing_file << "color camera metric radius: " << calib1.intrinsics.parameters.param.metric_radius << endl;

    writing_file.close();
}



// Update Data
void Kinect::update()
{
    //Get a capture
    switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
    {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            std::cout << "Timed out waiting for a capture" << std::endl;
        case K4A_WAIT_RESULT_FAILED:
            std::cout << "Failed to read a capture" << std::endl;
    }

	// Update Color
	updateColor();

    // Update Depth
	updateDepth();

	//transform Depth to Color Frame
    transform_depth_to_color();

    //update IMU data
    updateIMU();

    // Release the capture
    k4a_image_release(color_image);
    k4a_image_release(depth_image);
    k4a_image_release(transformed_depth_image);
    k4a_capture_release(capture);

}

// Update Color
inline void Kinect::updateColor()
{
    // Get a color image
    color_image = k4a_capture_get_color_image(capture);

    if (color_image == 0)
    {
        std::cout << "Failed to get color image from capture" << std::endl;
    }
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    imBGRA = cv::Mat(color_image_height_pixels, color_image_width_pixels, CV_8UC4, (void*)k4a_image_get_buffer(color_image)).clone();
}

// Update Depth(depth_image & imD)
inline void Kinect::updateDepth()
{
    // Get a depth image
    depth_image = k4a_capture_get_depth_image(capture);
    if (depth_image == 0)
    {
        std::cout <<"Failed to get depth image from capture" << std::endl;
    }
}


inline void Kinect::updateIMU()
{
    // Capture a imu sample
    //std::chrono::milliseconds timeout = 10;
    int32_t imu_timeout_ms = 60; //internal::clamp_cast<int32_t>(timeout.count());
    switch (k4a_device_get_imu_sample(device, &imu_sample, imu_timeout_ms))
    {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a imu sample\n");
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a imu sample\n");
    }
    // Access the accelerometer readings
    imu_Acc = imu_sample.acc_sample;
    imu_grav = extract_gravity_from_imu(imu_Acc);
}


inline k4a::Vector Kinect::extract_gravity_from_imu(k4a::Vector& imuAcc)
{
    //const k4a_calibration_t& sensorCalibration;
    const float* transformation_color_IMU_R = calibration.extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_COLOR].rotation;
    k4a::Vector Rx = { transformation_color_IMU_R[0], transformation_color_IMU_R[1], transformation_color_IMU_R[2] };
    k4a::Vector Ry = { transformation_color_IMU_R[3], transformation_color_IMU_R[4], transformation_color_IMU_R[5] };
    k4a::Vector Rz = { transformation_color_IMU_R[6], transformation_color_IMU_R[7], transformation_color_IMU_R[8] };

    k4a::Vector depthAcc = { Rx.Dot(imuAcc), Ry.Dot(imuAcc) , Rz.Dot(imuAcc) };
    // The acceleration due to gravity, g, is in a direction toward the ground.
    // However an accelerometer at rest in a gravity field reports upward acceleration
    // relative to the local inertial frame (the frame of a freely falling object).

    k4a::Vector depthGravity = depthAcc.Normalized() * -1;
    //depthGravity.Y = depthGravity.Y * -1;

    return depthGravity;
}


void Kinect::transform_depth_to_color()
{
    //tramnsfrorm depthint
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * (int)sizeof(uint16_t),
                                                 &transformed_depth_image))
    {
        cout << "Failed to create transformed depth image" << endl;
    }
    if(K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(transformation, depth_image, transformed_depth_image))
    {
        cout << "Failed to compute transformed depth image" << endl;
    }

    uint8_t *buffer = k4a_image_get_buffer(transformed_depth_image);
    uint16_t *depth_buffer = reinterpret_cast<uint16_t *>(buffer);
    cv::Mat mat(color_image_height_pixels, color_image_width_pixels, CV_16U);//CV_MAKETYPE(DataType<uint16_t>::type, 1));
    memcpy(mat.data, depth_buffer, color_image_height_pixels * color_image_width_pixels * 1 * sizeof(uint16_t));
    mat.copyTo(im_transformed_depth_image);
    //create_mat_from_buffer<uint16_t>(depth_buffer, color_image_width_pixels, color_image_width_pixels).copyTo(im_transformed_depth_image);

    if(im_transformed_depth_image.empty())
        cout << "im_transformed_depth_image is empty" << endl;
}




// Show Data
void Kinect::show()
{
	// Show Color
	showColor();

	// Show Depth
	showDepth();
}

// Show Color
inline void Kinect::showColor()
{
	if (imBGRA.empty()) {
		return;
	}

#ifdef COLOR
	// Resize Image
	cv::Mat resizeMat;
	const double scale = 0.5;
	cv::resize(imBGRA, resizeMat, cv::Size(), scale, scale);

	// Show Image
	cv::imshow("Color", resizeMat);
#else
	// Show Image
	cv::imshow("Color", colorMat);
#endif
}

// Show Depth
inline void Kinect::showDepth()
{
	if (im_transformed_depth_image.empty()) {
		return;
	}
	// Scaling ( 0-8000 -> 255-0 )
	cv::Mat scaleMat;
    im_transformed_depth_image.convertTo(scaleMat, CV_8U, -255.0 / 8000.0, 255.0);
	cv::applyColorMap( scaleMat, scaleMat, cv::COLORMAP_BONE );

#ifdef COLOR
	// Resize Image
	cv::Mat resizeMat;
	const double scale = 0.5;
	cv::resize(scaleMat, resizeMat, cv::Size(), scale, scale);

	// Show Image
	cv::imshow("Depth", resizeMat);
#else
	// Show Image
	cv::imshow("Depth", scaleMat);
#endif
}

void Kinect::getColor(cv::Mat& color)
{
	if(imBGRA.empty()) {
		return;
	}
    //std::cout << imBGRA.rows << " " << imBGRA.cols << std::endl; // (720 1280) => (480, 640)
    //std::cout << color.rows << " " << color.cols << std::endl; // (720 1280) => (480, 640)
	//Resize Image
    cv::Mat resizeMat;//(color.rows, color.cols, imBGRA.type());
	cv::resize(imBGRA, resizeMat, cv::Size(), 640./(float)imBGRA.cols, 480./(float)imBGRA.rows);
    //std::cout << resizeMat.rows << " " << resizeMat.cols << std::endl; // (720 1280) => (480, 640)
    resizeMat.copyTo(color);
}

void Kinect::getDepth(cv::Mat& depth)
{
	if (im_transformed_depth_image.empty()) {
		return;
	}
    //std::cout << im_transformed_depth_image.rows << " " << im_transformed_depth_image.cols << std::endl; // (720 1280) => (480, 640)
    //std::cout << depth.rows << " " << depth.cols << std::endl; // (720 1280) => (480, 640)
    //Resize Image
    cv::Mat resizeMat;//(color.rows, color.cols, imBGRA.type());
    cv::resize(im_transformed_depth_image, resizeMat, cv::Size(), 640./(float)im_transformed_depth_image.cols, 480./(float)im_transformed_depth_image.rows);
    //std::cout << resizeMat.rows << " " << resizeMat.cols << std::endl; // (720 1280) => (480, 640)

    resizeMat.copyTo(depth);
}

void Kinect::getIMU(k4a::Vector& vec)
{
    vec = imu_Acc;//.Normalized();
    //std::cout << "IMU Acc : " << imu_Acc.X << " " << imu_Acc.Y << " " << imu_Acc.Z << std::endl;
    //std::cout << "After : " << vec.X << " " << vec.Y << " " << vec.Z << std::endl;
}

void Kinect::getGravity(k4a::Vector& vec)
{
    vec = imu_grav;//.Normalized();
    //std::cout << "Before : " << imu_grav.X << " " << imu_grav.Y << " " << imu_grav.Z << std::endl;
    //std::cout << "After : " << vec.X << " " << vec.Y << " " << vec.Z << std::endl;

    //k4a::Vector angle = vec.Angle(k4a::Vector(0,1.0,0));
    //std::cout << "After : " << vec.X << " " << vec.Y << " " << vec.Z << std::endl;

}




void Kinect::reflectImage(const cv::Mat image, cv::Mat& reflectMat)
{
	if (image.empty())
		return;
	cv::flip(image, reflectMat, 1);

	return;
}
