


#ifndef DETECTOR_H
#define DETECTOR_H

#include <string>
#include <thread>

#include <fstream>
#include <sstream>
#include <iostream>


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <vector>
#include <cstdio>
#include <boost/thread.hpp>

#include <mutex>

#include "Tracking.h"
#include <opencv2/opencv.hpp>	

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn/dnn.hpp>


namespace ORB_SLAM2
{

class Tracking;

class Detector
{

public:

	Detector();
	Tracking *mpTracker;

	void Run();
	void SetTracker(Tracking *pTracker);
	bool isNewImgArrived();
	void SetDetectionFlag();
	void Detect();

	bool isFinished();
	void RequestFinish();

	cv::Mat mImg;	
	std::mutex mMutexNewImgDetection;
	std::mutex mMutexGetNewImg;
	bool mbNewImgFlag;

	bool mbFinishRequested;
	std::mutex mMutexFinish;

	double aprox_area;	
	double fig_area;

	int fig_param = 640*480;
	int fig_factor;

	//------------------------------------------------------------

	// YOLO parameters
	cv::dnn::Net net;
	
	float confThreshold; // Confidence threshold
	float nmsThreshold;  // Non-maximum suppression threshold
	
	int inpWidth;  // Width of network's input image 
	int inpHeight; // Height of network's input image
	std::vector<std::string> classes;

	// OUTPUT
	std::vector<cv::Rect> people_boxes;	

	
	// YOLO functions
	std::vector<cv::String> getOutsNames(const cv::dnn::Net& net);
	void postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs, std::vector<int>& classIds, std::vector<int> &centersX, std::vector<int> &centersY, std::vector<cv::Rect>& boxes);
    void drawPred (int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);

};

}// namespace ORB_SLAM2

#endif // DETECTOR_H
