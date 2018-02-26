#include <iostream>

#include <signal.h>
#include <cstdlib>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <opencv2/opencv.hpp>

bool exiting = false;

void sigintHandler(int _signo) {
	exiting = true;
}

int main() {
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;

	if (freenect2.enumerateDevices() == 0) {
		std::cerr << "No Kinect found. Make sure the device is connected." << std::endl;
		std::exit(1);
	}

	pipeline = new libfreenect2::OpenGLPacketPipeline();

	std::string serialNumber = "";
	serialNumber = freenect2.getDefaultDeviceSerialNumber();
	std::cout << "Device serial number is " << serialNumber << std::endl;

	pipeline = new libfreenect2::OpenGLPacketPipeline();
	std::cout << "Initializing OpenGL packet pipeline..." << std::endl;

	dev = freenect2.openDevice(serialNumber, pipeline);
	if (!dev) {
		std::cerr << "Error opening the device." << std::endl;
		std::exit(1);
	}

	signal(SIGTERM, sigintHandler);
	signal(SIGINT, sigintHandler);

	int frameTypes = libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
	libfreenect2::SyncMultiFrameListener listener(frameTypes);
	libfreenect2::FrameMap frames;

	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);

	std::cout << "Starting device..." << std::endl;

	if (!dev->start()) {
		std::cerr << "Failed to start device!" << std::endl;
		std::exit(1);
	}

	libfreenect2::Registration *registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
	libfreenect2::Frame *rgb, *ir, *depth;
	unsigned int frameNo = 0;
	cv::Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd;
	char fName[64];

	while (!exiting) {
		if (!listener.waitForNewFrame(frames, 10 * 1000)) {
			std::cerr << "Timeout while waiting for frame. Exiting..." << std::endl;
			std::exit(1);
		}

		rgb = frames[libfreenect2::Frame::Color];
		ir = frames[libfreenect2::Frame::Ir];
		depth = frames[libfreenect2::Frame::Depth];

		registration->apply(rgb, depth, &undistorted, &registered);

		snprintf(fName, 32, "rgb_%09u.png", frameNo);
		cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
		cv::imwrite(fName, rgbmat);

		snprintf(fName, 32, "ir_%09u.png", frameNo);
		cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
		cv::imwrite(fName, irmat);

		snprintf(fName, 32, "depth_%09u.png", frameNo);
		cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
		cv::imwrite(fName, depthmat);

		snprintf(fName, 32, "depthundistorted_%09u.png", frameNo);
		cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
		cv::imwrite(fName, depthmatUndistorted);

		snprintf(fName, 32, "rbgdregistered_%09u.png", frameNo);
		cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
		cv::imwrite(fName, rgbd);

		listener.release(frames);
		++frameNo;
	}

	dev->stop();
	dev->close();

	delete registration;
	return 0;
}