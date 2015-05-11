/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "live_slam_wrapper.h"
#include <vector>
#include "util/sophus_util.h"
#include "util/snprintf.h"
#include "util/global_funcs.h"

#include "slam_system.h"

#include "io_wrapper/image_display.h"
#include "io_wrapper/output_3d_wrapper.h"
#include "io_wrapper/input_image_stream.h"
#include "util/global_funcs.h"

#include <iostream>
#include <opencv/highgui.h>

namespace lsd_slam
{


LiveSLAMWrapper::LiveSLAMWrapper(InputImageStream* imageStream, Output3DWrapper* outputWrapper)
{
	log("LiveSLAMWrapper::LiveSLAMWrapper CONSTRUCTOR ------ START ----------------------------------------------------------------------  BBBBBBBBBBBBBBBBBB", 2);
	this->imageStream = imageStream;
	this->outputWrapper = outputWrapper;
	imageStream->getBuffer()->setReceiver(this);

	fx = imageStream->fx();
	fy = imageStream->fy();
	cx = imageStream->cx();
	cy = imageStream->cy();
	width = imageStream->width();
	height = imageStream->height();

	outFileName = packagePath+"estimated_poses.txt";
	log("LiveSLAMWrapper::LiveSLAMWrapper CONSTRUCTOR ------ ----------------------------------------------------------------------  BBBBBBBBBBBBBBBBBB outFileName", 2);
	log(outFileName, 2);
	log("LiveSLAMWrapper::LiveSLAMWrapper CONSTRUCTOR ------ ----------------------------------------------------------------------  BBBBBBBBBBBBBBBBBB outFileName", 2);

	isInitialized = false;


	Sophus::Matrix3f K_sophus;
	K_sophus << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

	outFile = nullptr;


	// make Odometry
	monoOdometry = new SlamSystem(width, height, K_sophus, doSlam);

	log("LiveSLAMWrapper::LiveSLAMWrapper CONSTRUCTOR ------ ----------------------------------------------------------------------  BBBBBBBBBBBBBBBBBB monoOdometry initialized", 2);

	monoOdometry->setVisualization(outputWrapper);

	log("LiveSLAMWrapper::LiveSLAMWrapper CONSTRUCTOR ------ ----------------------------------------------------------------------  BBBBBBBBBBBBBBBBBB monoOdometry setVisualization DONE", 2);

	imageSeqNumber = 0;

	log("LiveSLAMWrapper::LiveSLAMWrapper CONSTRUCTOR ------ END ----------------------------------------------------------------------  BBBBBBBBBBBBBBBBBB", 2);
}


LiveSLAMWrapper::~LiveSLAMWrapper()
{
	if(monoOdometry != 0)
		delete monoOdometry;
	if(outFile != 0)
	{
		outFile->flush();
		outFile->close();
		delete outFile;
	}
}

void LiveSLAMWrapper::Loop()
{
	while (true) {
		boost::unique_lock<boost::recursive_mutex> waitLock(imageStream->getBuffer()->getMutex());
		while (!fullResetRequested && !(imageStream->getBuffer()->size() > 0)) {
			notifyCondition.wait(waitLock);
		}
		waitLock.unlock();
		
		
		if(fullResetRequested)
		{
			resetAll();
			fullResetRequested = false;
			if (!(imageStream->getBuffer()->size() > 0))
				continue;
		}
		
		TimestampedMat image = imageStream->getBuffer()->first();
		imageStream->getBuffer()->popFront();
		
		log("LiveSLAMWrapper::Loop --- Util.displayImage to MyVideo  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<-----------", 2);
		// process image
		Util::displayImage("MyVideo", image.data);
		newImageCallback(image.data, image.timestamp);

		log("LiveSLAMWrapper::Loop --- Util.displayImage AFTER newImageCallback  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<-----------", 2);

		auto key = cvWaitKey(10); //Capture Keyboard stroke
		if (char(key) == 27){
			break; //If you hit ESC key loop will break.
		}
		log("LiveSLAMWrapper::Loop --- Util.displayImage ENDk  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<----------- *******", 2);
		
	}
}


void LiveSLAMWrapper::newImageCallback(const cv::Mat& img, Timestamp imgTime)
{
	log("debug: =============== LiveSLAMWrapper::newImageCallback ---- START -------------------------- STEP1 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", 2);
	++ imageSeqNumber;

	// Convert image to grayscale, if necessary
	cv::Mat grayImg;
	if (img.channels() == 1)
		grayImg = img;
	else
		cvtColor(img, grayImg, CV_RGB2GRAY);
	

	log("debug: =============== LiveSLAMWrapper::newImageCallback ---- -------------------------- STEP2 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", 2);

	// Assert that we work with 8 bit images
	assert(grayImg.elemSize() == 1);
	assert(fx != 0 || fy != 0);

	log("debug: =============== LiveSLAMWrapper::newImageCallback ---- -------------------------- STEP3 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", 2);

	// need to initialize
	if(!isInitialized)
	{
		log("debug: =============== LiveSLAMWrapper::newImageCallback ---- -------------------------- STEP3 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", 2);
		monoOdometry->randomInit(grayImg.data, imgTime.toSec(), 1);
		log("debug: =============== LiveSLAMWrapper::newImageCallback ---- -------------------------- STEP3.1 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", 2);

		isInitialized = true;
	}
	else if(isInitialized && monoOdometry != nullptr)
	{
		log("debug: =============== LiveSLAMWrapper::newImageCallback ---- -------------------------- STEP4 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX ----------------------------------------------->>>>", 2);
////RR		monoOdometry->trackFrame(grayImg.data,imageSeqNumber,false,imgTime.toSec());
	}

//	log("debug: =============== LiveSLAMWrapper::newImageCallback ---- SLEEPING 2 seconds  -------------------------- STEP5 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX SLEEPING SLEEPING", 2);
//	sleep (2);
	log("debug: =============== LiveSLAMWrapper::newImageCallback ---- END -------------------------- STEP5 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", 2);

}

void LiveSLAMWrapper::logCameraPose(const SE3& camToWorld, double time)
{
	Sophus::Quaternionf quat = camToWorld.unit_quaternion().cast<float>();
	Eigen::Vector3f trans = camToWorld.translation().cast<float>();

	char buffer[1000];
	int num = snprintf(buffer, 1000, "%f %f %f %f %f %f %f %f\n",
			time,
			trans[0],
			trans[1],
			trans[2],
			quat.x(),
			quat.y(),
			quat.z(),
			quat.w());

	if(outFile == 0)
		outFile = new std::ofstream(outFileName.c_str());
	outFile->write(buffer,num);
	outFile->flush();
}

void LiveSLAMWrapper::requestReset()
{
	fullResetRequested = true;
	notifyCondition.notify_all();
}

void LiveSLAMWrapper::resetAll()
{
	if(monoOdometry != nullptr)
	{
		delete monoOdometry;
		printf("Deleted SlamSystem Object!\n");

		Sophus::Matrix3f K;
		K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
		monoOdometry = new SlamSystem(width,height,K, doSlam);
		monoOdometry->setVisualization(outputWrapper);

	}
	imageSeqNumber = 0;
	isInitialized = false;

	Util::closeAllWindows();

}

}
