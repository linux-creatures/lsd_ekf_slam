/**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
 
 
 
#include "LSDWrapper.h"
#include <cvd/gl_helpers.h>
#include <gvars3/instances.h>
#include "../HelperFunctions.h"
#include "Predictor.h"
#include "DroneKalmanFilter.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "GLWindow2.h"
#include "EstimationNode.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include "LSD-SLAM/slam_system.h"
#include "LSD-SLAM/io_wrapper/ROS/ROSOutput3DWrapper.h"
#include "LSD-SLAM/io_wrapper/ROS/rosReconfigure.h"
#include "LSD-SLAM/util/sophus_util.h"
#include "LSD-SLAM/util/global_funcs.h"

pthread_mutex_t LSDWrapper::navInfoQueueCS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t LSDWrapper::shallowMapCS = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t LSDWrapper::logScalePairs_CS = PTHREAD_MUTEX_INITIALIZER;

namespace lsd_slam{
LSDWrapper::LSDWrapper(DroneKalmanFilter* f, EstimationNode* nde, Output3DWrapper* outputWrapper)
{
	filter = f;
	node = nde;

	predConvert = 0;
	predIMUOnlyForScale = 0;
	mpCamera = 0;
	newImageAvailable = false;
	
	mapPointsTransformed = std::vector<tvec3>();
	keyFramesTransformed = std::vector<tse3>();

	
	predConvert = new Predictor();
	predIMUOnlyForScale = new Predictor();
	imuOnlyPred = new Predictor();

	drawUI = UI_PRES;
	frameWidth = frameHeight = 0;

	logfileScalePairs = 0;

	this->outputWrapper = outputWrapper;


	isInitialized = false;

/***********************************************************relok this*/
	std::ifstream fleH (file.c_str());
	Sophus::Matrix3f K_sophus;
	TooN::Vector<5> camPar;
	fleH >> camPar[0] >> camPar[1] >> camPar[2] >> camPar[3] >> camPar[4];
	fleH.close();
	std::cout<< "Set Camera Paramerer to: " << camPar[0] << " " << camPar[1] << " " << camPar[2] << " " << camPar[3] << " " << camPar[4] << camPar[5] << " " << camPar[6] << " " << camPar[7] << std::endl;
	
	K_sophus << camPar[0], 0.0, camPar[1], 0.0, camPar[2], camPar[3], 0.0, 0.0, 1.0;

	outFile = nullptr;
	frameWidth = camPar[4];
	frameHeight = camPar[5];

	// make Odometry
	monoOdometry = new SlamSystem(frameWidth, frameHeight, K_sophus, doSlam);

	monoOdometry->setVisualization(outputWrapper);

	imageSeqNumber = 0;
}



void LSDWrapper::ResetInternal()
{


	// read camera calibration (yes, its done here)
	std::string calibFile;
	while(node->arDroneVersion == 0)
	{
		std::cout << "Waiting for first navdata to determine drone version!" << std::endl;
		usleep(250000);
	}
	if(file.size()==0)
	{
		if(node->arDroneVersion == 1)
			file = node->packagePath + "/camcalib/ardrone1_default.txt";
		else if(node->arDroneVersion == 2)
			file = node->packagePath + "/camcalib/ardrone2_default.txt";
	}


	outFileName = node->packagePath+"estimated_poses.txt";
	//Specify width and height for the monoOdometry module

	if(monoOdometry != nullptr)
	{
		delete monoOdometry;
		printf("Deleted SlamSystem Object!\n");

		K_sophus << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
		monoOdometry = new SlamSystem(frameWidth, frameWidth, K_sophus, doSlam);
		monoOdometry->setVisualization(outputWrapper);

	}

	//Setting the filestream
	std::ifstream fleH (file.c_str());
	inputStream->setCalibration(fileH);
	fleH.close();

	inputStream->run();
	outputWrapper = new ROSOutput3DWrapper(inputStream->width(), inputStream->height());
	LiveSLAMWrapper lsdTracker(inputStream, outputWrapper);

	predConvert->setPosRPY(0,0,0,0,0,0);
	predIMUOnlyForScale->setPosRPY(0,0,0,0,0,0);

	resetLSDRequested = false;
	isGoodCount = 0;
	lastAnimSentClock = 0;
	LSDInitializedClock = 0;
	lastLSDMessage = "";

	flushMapKeypoints = false;

	node->publishCommand("u l LSD has been reset.");
	imageSeqNumber = 0;
	isInitialized = false;

	Util::closeAllWindows();
}


LSDWrapper::~LSDWrapper(void)
{
	if(predConvert != 0) delete predConvert;
	if(predIMUOnlyForScale != 0) delete predIMUOnlyForScale;
	if(imuOnlyPred != 0) delete imuOnlyPred;

	if(monoOdometry != 0)
		delete monoOdometry;

	if(outFile != 0)
	{
		outFile->flush();
		outFile->close();
		delete outFile;
	}

}


void LSDWrapper::startSystem()
{
	keepRunning = true;
	changeSizeNextRender = false;
	start();
}

void LSDWrapper::stopSystem()
{
	keepRunning = false;
	new_frame_signal.notify_all();
	join();
}


void LSDWrapper::run()
{
	std::cout << "Waiting for Video" << std::endl;

	// wait for firsst image
	while(!newImageAvailable)
		usleep(100000);	// sleep 100ms
	newImageAvailable = false;
	while(!newImageAvailable)
		usleep(100000);	// sleep 100ms

	// read image height and width**temporarily*** 
	frameWidth = 640;
	frameHeight = 480;

	ResetInternal();


	snprintf(charBuf,200,"Video resolution: %d x %d",frameWidth,frameHeight);
	ROS_INFO(charBuf);
	node->publishCommand(std::string("u l ")+charBuf);

	// create window
	Util::displayImage("LSD SLAM Drone Camera Feed", image.data);

	// Framewidth size removed
	changeSizeNextRender = true;

	boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);

	while(keepRunning)
	{
		if(newImageAvailable)
		{
			newImageAvailable = false;

			mimFrameBW_workingCopy.copy_from(mimFrameBW);

			// release lock and do the work-intensive stuff.....
			lock.unlock();

			HandleFrame();


			if(changeSizeNextRender)
			{
				myGLWindow->set_size(desiredWindowSize);
				changeSizeNextRender = false;
			}

			// get lock again
			lock.lock();
		}
		else
			new_frame_signal.wait(lock);
	}

	lock.unlock();
	delete myGLWindow;
}



// called every time a new frame is available.
// needs to be able to 
// - (finally) roll forward filter
// - query it's state 
// - add a PTAM observation to filter.
void LSDWrapper::HandleFrame()
{
	//printf("tracking Frame at ms=%d (from %d)\n",getMS(ros::Time::now()),mimFrameTime-filter->delayVideo);


	// prep data
	msg = "";
	ros::Time startedFunc = ros::Time::now();

	// reset?
	if(resetLSDRequested)
		ResetInternal();



	// make filter thread-safe.
	// --------------------------- ROLL FORWARD TIL FRAME. This is ONLY done here. ---------------------------
	pthread_mutex_lock( &filter->filter_CS );
	//filter->predictUpTo(mimFrameTime,true, true);
	TooN::Vector<10> filterPosePreLSD = filter->getPoseAtAsVec(mimFrameTime_workingCopy-filter->delayVideo,true);
	pthread_mutex_unlock( &filter->filter_CS );

	// ------------------------ do PTAM -------------------------
	myGLWindow->SetupViewport();
	myGLWindow->SetupVideoOrtho();
	myGLWindow->SetupVideoRasterPosAndZoom();



	// 1. transform with filter
	TooN::Vector<6> LSDPoseGuess = filter->backTransformLSDObservation(filterPosePreLSD.slice<0,6>());
	// 2. convert to se3
	predConvert->setPosRPY(LSDPoseGuess[0], LSDPoseGuess[1], LSDPoseGuess[2], LSDPoseGuess[3], LSDPoseGuess[4], LSDPoseGuess[5]);
	// 3. multiply with rotation matrix	
	TooN::SE3<> LSDPoseGuessSE3 = predConvert->droneToFrontNT * predConvert->globaltoDrone;


	boost::unique_lock<boost::recursive_mutex> waitLock(imageStream->getBuffer()->getMutex());
		while (!fullResetRequested && !(imageStream->getBuffer()->size() > 0)) {
			notifyCondition.wait(waitLock);
		}
		waitLock.unlock();
		
	
	// Track image and get current pose estimate
	/***Note: image is of type imagedata from lsd-slam change it***/
	TooN::SE3<> LSDResultSE3;
	newImageCallback(mimFrameBW_workingCopy.data, mimFrameBW_workingCopy.timestamp);

	LSDResultSE3 = monoOdometry->getCurrentPoseEstimate();
	
	ros::Duration timeLSD= ros::Time::now() - startedLSD;

	TooN::Vector<6> LSDResultSE3TwistOrg = LSDResultSE3.ln();

	node->publishTf(LSDResultSE3, mimFrameTimeRos_workingCopy, mimFrameSEQ_workingCopy,"cam_front");


	// 1. multiply from left by frontToDroneNT.
	// 2. convert to xyz,rpy
	predConvert->setPosSE3_globalToDrone(predConvert->frontToDroneNT * LSDResultSE3);
	TooN::Vector<6> LSDResult = TooN::makeVector(predConvert->x, predConvert->y, predConvert->z, predConvert->roll, predConvert->pitch, predConvert->yaw);

	// 3. transform with filter
	TooN::Vector<6> LSDResultTransformed = filter->transformLSDObservation(LSDResult);

	//Init failed code removed



	// --------------------------- assess result ------------------------------
	bool isGood = true;
	bool diverged = false;
	
	// calculate absolute differences.
	TooN::Vector<6> diffs = LSDResultTransformed - filterPosePreLSD.slice<0,6>();
	for(int i=0;1<1;i++) diffs[i] = abs(diffs[i]);

	if(filter->getNumGoodLSDObservations() < 10 && monoOdometry->IsGood())
	{
		isGood = true;
	}

	//If the last tracking step result is lost
	else if(lsdTracker->lastStepResult == LOST)
		isGood = false;
		diverged = true;

	else
	{

		// if yaw difference too big: something certainly is wrong.
		// maximum difference is 5 + 2*(number of seconds since PTAM observation).
		double maxYawDiff = 10.0 + (getMS()-lastGoodYawClock)*0.002;
		if(maxYawDiff > 20) maxYawDiff = 1000;
		if(false && diffs[5] > maxYawDiff) 
			isGood = false;

		if(diffs[5] < 10) 
			lastGoodYawClock = getMS();

		if(diffs[5] > 4.0) 
			isGood = false;

		// if rp difference too big: something certainly is wrong.
		if(diffs[3] > 20 || diffs[4] > 20)
			isGood = false;

		if(diffs[3] > 3 || diffs[4] > 3 || dodgy)
			isGood = false;
	}

	if(isGood)
	{
		if(isGoodCount < 0) isGoodCount = 0;
		isGoodCount++;
	}
	else
	{
		if(isGoodCount > 0) isGoodCount = 0;
		isGoodCount--;
		
		if(mpTracker->lastStepResult == mpTracker->T_RECOVERED_DODGY)
			isGoodCount = std::max(isGoodCount,-2);
		if(mpTracker->lastStepResult == mpTracker->T_RECOVERED_GOOD)
			isGoodCount = std::max(isGoodCount,-5);

	}





	TooN::Vector<10> filterPosePostLSD;
	// --------------------------- scale estimation & update filter (REDONE) -----------------------------
	// interval length is always between 1s and 2s, to enshure approx. same variances.
	// if interval contained height inconsistency, z-distances are simply both set to zero, which does not generate a bias.
	// otherwise distances are scaled such that height is weighted more.
	// if isGood>=3 && framesIncludedForScale < 0			===> START INTERVAL
	// if 18 <= framesIncludedForScale <= 36 AND isGood>=3	===> ADD INTERVAL, START INTERVAL
	// if framesIncludedForScale > 36						===> set framesIncludedForScale=-1 

	// include!

	// TODO: make shure filter is handled properly with permanent roll-forwards.
	pthread_mutex_lock( &filter->filter_CS );
	if(filter->useLSD)
	{
		filter->addLSDObservation(LSDResult,mimFrameTime_workingCopy-filter->delayVideo);
	}
	else
		filter->addFakeLSDObservation(mimFrameTime_workingCopy-filter->delayVideo);

	filterPosePostLSD = filter->getCurrentPoseSpeedAsVec();
	pthread_mutex_unlock( &filter->filter_CS );

	TooN::Vector<6> filterPosePostLSDBackTransformed = filter->backTransformLSDObservation(filterPosePostLSD.slice<0,6>());


	// if interval is started: add one step.
	int includedTime = mimFrameTime_workingCopy - lsdPositionForScaleTakenTimestamp;
	if(framesIncludedForScaleXYZ >= 0) framesIncludedForScaleXYZ++;

	// if interval is overdue: reset & dont add
	if(includedTime > 3000) 
	{
		framesIncludedForScaleXYZ = -1;
	}

	if(isGoodCount >= 3)
	{
		// filter stuff
		lastScaleEKFtimestamp = mimFrameTime_workingCopy;

		if(includedTime >= 2000 && framesIncludedForScaleXYZ > 1)	// ADD! (if too many, was resetted before...)
		{
			TooN::Vector<3> diffLSD = filterPosePostLSDBackTransformed.slice<0,3>() - LSDPositionForScale;
			bool zCorrupted, allCorrupted;
			float pressureStart = 0, pressureEnd = 0;
			TooN::Vector<3> diffIMU = evalNavQue(lsdPositionForScaleTakenTimestamp - filter->delayVideo + filter->delayXYZ,mimFrameTime_workingCopy - filter->delayVideo + filter->delayXYZ,&zCorrupted, &allCorrupted, &pressureStart, &pressureEnd);

			pthread_mutex_lock(&logScalePairs_CS);
			if(logfileScalePairs != 0)
				(*logfileScalePairs) <<
						pressureStart << " " <<
						pressureEnd << " " <<
						diffIMU[2] << " " <<
						diffLSD[2] << std::endl;
			pthread_mutex_unlock(&logScalePairs_CS);


			if(!allCorrupted)
			{
				// filtering: z more weight, but only if not corrupted.
				double xyFactor = 0.05;
				double zFactor = zCorrupted ? 0 : 3;
			
				diffLSD.slice<0,2>() *= xyFactor; diffLSD[2] *= zFactor;
				diffIMU.slice<0,2>() *= xyFactor; diffIMU[2] *= zFactor;

				filter->updateScaleXYZ(diffLSD, diffIMU, LSDResult.slice<0,3>());
				//currentkeyframe scale set here.
				currentKeyFrame->getScaledCamToWorld().scale() = filter->getCurrentScales()[0];
			}
			framesIncludedForScaleXYZ = -1;	// causing reset afterwards
		}

		if(framesIncludedForScaleXYZ == -1)	// RESET!
		{
			framesIncludedForScaleXYZ = 0;
			LSDPositionForScale = filterPosePostLSDBackTransformed.slice<0,3>();
			//predIMUOnlyForScale->resetPos();	// also resetting z corrupted flag etc. (NOT REquired as reset is done in eval)
			lsdPositionForScaleTakenTimestamp = mimFrameTime_workingCopy;
		}
	}
	
	//Map locking removed


	 
	// ----------------------------- update shallow map for LSD SLAM-----------------
	//Map locking removed
	if(rand()%5==0)
	{
		pthread_mutex_lock(&shallowMapCS);
		// Should convert both of them to LSD
		mapPointsTransformed.clear();
		keyFramesTransformed.clear();
		for(unsigned int i=0;i<keyFrameGraph->allFramePoses.size();i++)
		{
			predConvert->setPosSE3_globalToDrone(predConvert->frontToDroneNT * mpMap->vpKeyFrames[i]->se3CfromW);
			TooN::Vector<6> CamPos = TooN::makeVector(predConvert->x, predConvert->y, predConvert->z, predConvert->roll, predConvert->pitch, predConvert->yaw);
			CamPos = filter->transformPTAMObservation(CamPos);
			predConvert->setPosRPY(CamPos[0], CamPos[1], CamPos[2], CamPos[3], CamPos[4], CamPos[5]);
			keyFramesTransformed.push_back(predConvert->droneToGlobal);
		}
		TooN::Vector<3> LSDScales = filter->getCurrentScales();
		TooN::Vector<3> LSDOffsets = filter->getCurrentOffsets().slice<0,3>();
		//Converting the local points by LSD to the world perspective by multiplying by the scaleng
		//A little confusion b/w using keyframegraph or currentkeyframe
		for(unsigned int i=0;i<keyFrameGraph->numPoints;i++)
		{
			TooN::Vector<3> pos = (mpMap->vpPoints)[i]->v3WorldPos;
			pos[0] *= LSDScales[0];
			pos[1] *= LSDScales[1];
			pos[2] *= LSDScales[2];
			pos += PTAMOffsets;
			mapPointsTransformed.push_back(pos);
		}
		// flush map keypoints
		if(flushMapKeypoints)
		{
			std::ofstream* fle = new std::ofstream();
			fle->open("pointcloud.txt");
			for(unsigned int i=0;i<mapPointsTransformed.size();i++)
			{
				(*fle) << mapPointsTransformed[i][0] << " "
					   << mapPointsTransformed[i][1] << " "
					   << mapPointsTransformed[i][2] << std::endl;
			}
			fle->flush();
			fle->close();
			printf("FLUSHED %d KEYPOINTS to file pointcloud.txt\n\n",mapPointsTransformed.size());
			flushMapKeypoints = false;
		}
		pthread_mutex_unlock(&shallowMapCS);
	}
*/

	// ---------------------- output and render! ---------------------------
	ros::Duration timeALL = ros::Time::now() - startedFunc;
	else if(isGood) snprintf(charBuf,1000,"\nQuality: good           ");
	else snprintf(charBuf,1000,"\nQuality: lost                       ");
	
	snprintf(charBuf+20,800, "scale: %.3f (acc: %.3f)                            ",filter->getCurrentScales()[0],(double)filter->getScaleAccuracy());
	snprintf(charBuf+50,800, "LSD time: %i ms                            ",(int)(1000*timeALL.toSec()));
	snprintf(charBuf+68,800, "(%i ms total)  ",(int)(1000*timeALL.toSec()));
	if() snprintf(charBuf+83,800, "m.l. ");
	else snprintf(charBuf+83,800, "     ");
	if(filter->allSyncLocked) snprintf(charBuf+88,800, "s.l. ");
	else snprintf(charBuf+88,800, "     ");


	msg += charBuf;

	if()
	{
		if(drawUI == UI_DEBUG)
		{
			snprintf(charBuf,1000,"\nLSD Diffs:              ");
			snprintf(charBuf+13,800, "x: %.3f                          ",diffs[0]);
			snprintf(charBuf+23,800, "y: %.3f                          ",diffs[1]);
			snprintf(charBuf+33,800, "z: %.3f                          ",diffs[2]);
			snprintf(charBuf+43,800, "r: %.2f                          ",diffs[3]);
			snprintf(charBuf+53,800, "p: %.2f                          ",diffs[4]);
			snprintf(charBuf+63,800, "y: %.2f",diffs[5]);
			msg += charBuf;


			snprintf(charBuf,1000,"\nLSD Pose:              ");
			snprintf(charBuf+13,800, "x: %.3f                          ",LSDResultTransformed[0]);
			snprintf(charBuf+23,800, "y: %.3f                          ",LSDResultTransformed[1]);
			snprintf(charBuf+33,800, "z: %.3f                          ",LSDResultTransformed[2]);
			snprintf(charBuf+43,800, "r: %.2f                          ",LSDResultTransformed[3]);
			snprintf(charBuf+53,800, "p: %.2f                          ",LSDResultTransformed[4]);
			snprintf(charBuf+63,800, "y: %.2f",LSDResultTransformed[5]);
			msg += charBuf;

		}
	}

	if(drawUI != UI_NONE)
	{
		// render grid
		predConvert->setPosRPY(filterPosePostLSD[0], filterPosePostLSD[1], filterPosePostLSD[2], filterPosePostLSD[3], filterPosePostLSD[4], filterPosePostLSD[5]);

		//renderGrid(predConvert->droneToFrontNT * predConvert->globaltoDrone);
		//renderGrid(PTAMResultSE3);


		// draw HUD
		//if(mod->getControlSystem()->isControlling())
		{
			myGLWindow->SetupViewport();
			myGLWindow->SetupVideoOrtho();
			myGLWindow->SetupVideoRasterPosAndZoom();

			//glDisable(GL_LINE_SMOOTH);
			glLineWidth(2);
			glBegin(GL_LINES);
			glColor3f(0,0,1);

			glVertex2f(0,frameHeight/2);
			glVertex2f(frameWidth,frameHeight/2);

			glVertex2f(frameWidth/2,0);
			glVertex2f(frameWidth/2,frameHeight);

			// 1m lines
			glVertex2f(0.25*frameWidth,0.48*frameHeight);
			glVertex2f(0.25*frameWidth,0.52*frameHeight);
			glVertex2f(0.75*frameWidth,0.48*frameHeight);
			glVertex2f(0.75*frameWidth,0.52*frameHeight);
			glVertex2f(0.48*frameWidth,0.25*frameHeight);
			glVertex2f(0.52*frameWidth,0.25*frameHeight);
			glVertex2f(0.48*frameWidth,0.75*frameHeight);
			glVertex2f(0.52*frameWidth,0.75*frameHeight);

			glEnd();
		}


		myGLWindow->DrawCaption(msg);
	}

	lastLSDResultRaw = LSDResultSE3; 
	// ------------------------ LOG --------------------------------------
	// log!
	if(node->logfileLSD != NULL)
	{
		TooN::Vector<3> scales = filter->getCurrentScalesForLog();
		TooN::Vector<3> sums = TooN::makeVector(0,0,0);
		TooN::Vector<6> offsets = filter->getCurrentOffsets();
		pthread_mutex_lock(&(node->logLSD_CS));
		
		//****Look at this.... time may not be proper**/
		logCameraPose(camToWorld, time);

		pthread_mutex_unlock(&(node->logLSD_CS));
	}

	myGLWindow->swap_buffers();
	myGLWindow->HandlePendingEvents();

}




void LSDWrapper::newImageCallback(const cv::Mat& img, Timestamp imgTime)
{

	TooN::SE3<> LSDResultSE3;
	++ imageSeqNumber;

	// Convert image to grayscale, if necessary
	cv::Mat grayImg;
	if (img.channels() == 1)
		grayImg = img;
	else
		cvtColor(img, grayImg, CV_RGB2GRAY);
	

	// Assert that we work with 8 bit images
	assert(grayImg.elemSize() == 1);
	assert(fx != 0 || fy != 0);


	// need to initialize
	if(!isInitialized)
	{
		monoOdometry->randomInit(grayImg.data, imgTime.toSec(), 1);
		isInitialized = true;
	}
	else if(isInitialized && monoOdometry != nullptr)
	{
		monoOdometry->trackFrame(grayImg.data,imageSeqNumber,false,imgTime.toSec());
	}

}


void LSDWrapper::logCameraPose(const SE3& camToWorld, double time)
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




/************************************************************************/

// Draw the reference grid to give the user an idea of wether tracking is OK or not.
void LSDWrapper::renderGrid(TooN::SE3<> camFromWorld)
{
	myGLWindow->SetupViewport();
	myGLWindow->SetupVideoOrtho();
	myGLWindow->SetupVideoRasterPosAndZoom();

	camFromWorld.get_translation() *= 1;

	// The colour of the ref grid shows if the coarse stage of tracking was used
	// (it's turned off when the camera is sitting still to reduce jitter.)
	glColor4f(0,0,0,0.6);
  
	// The grid is projected manually, i.e. GL receives projected 2D coords to draw.
	int nHalfCells = 5;
	int nTot = nHalfCells * 2 + 1;
	CVD::Image<Vector<2> >  imVertices(CVD::ImageRef(nTot,nTot));
	for(int i=0; i<nTot; i++)
		for(int j=0; j<nTot; j++)
		{
			Vector<3> v3;
			v3[0] = (i - nHalfCells) * 1;
			v3[1] = (j - nHalfCells) * 1;
			v3[2] = 0.0;
			Vector<3> v3Cam = camFromWorld * v3;
			//v3Cam[2] *= 100;
			if(v3Cam[2] < 0.001)
				v3Cam = TooN::makeVector(100000*v3Cam[0],100000*v3Cam[1],0.0001);

			imVertices[i][j] = mpCamera->Project(TooN::project(v3Cam))*0.5;
		}

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glLineWidth(2);
	for(int i=0; i<nTot; i++)
	{
		glBegin(GL_LINE_STRIP);
		for(int j=0; j<nTot; j++)
		CVD::glVertex(imVertices[i][j]);
		glEnd();
      
		glBegin(GL_LINE_STRIP);
		for(int j=0; j<nTot; j++)
		CVD::glVertex(imVertices[j][i]);
		glEnd();
	};
  
  glLineWidth(1);
  glColor3f(1,0,0);



}

TooN::Vector<3> LSDWrapper::evalNavQue(unsigned int from, unsigned int to, bool* zCorrupted, bool* allCorrupted, float* out_start_pressure, float* out_end_pressure)
{
	predIMUOnlyForScale->resetPos();

	int firstAdded = 0, lastAdded = 0;
	pthread_mutex_lock(&navInfoQueueCS);
	int skipped=0;
	int used = 0;
	int firstZ = 0;

	float sum_first=0, num_first=0, sum_last=0, num_last=0;
	int pressureAverageRange = 100;


	for(std::deque<ardrone_autonomy::Navdata>::iterator cur = navInfoQueue.begin();
			cur != navInfoQueue.end();
			)
	{
		int curStampMs = getMS(cur->header.stamp);

		if(curStampMs < (int)from-pressureAverageRange)
			cur = navInfoQueue.erase(cur);
		else
		{
			if(curStampMs >= (int)from-pressureAverageRange && curStampMs <= (int)from+pressureAverageRange)
			{
				sum_first += cur->pressure;
				num_first++;
			}

			if(curStampMs >= (int)to-pressureAverageRange && curStampMs <= (int)to+pressureAverageRange)
			{
				sum_last += cur->pressure;
				num_last++;
			}
			cur++;
		}
	}

	for(std::deque<ardrone_autonomy::Navdata>::iterator cur = navInfoQueue.begin();
			cur != navInfoQueue.end();
			cur++
			)
	{
		int frontStamp = getMS(cur->header.stamp);
		if(frontStamp < from)		// packages before: delete
		{
			//navInfoQueue.pop_front();
			skipped++;
		}
		else if(frontStamp >= from && frontStamp <= to)
		{
			if(firstAdded == 0) 
			{
				firstAdded = frontStamp;
				firstZ = cur->altd;
				predIMUOnlyForScale->z = firstZ*0.001;	// avoid height check initially!
			}
			lastAdded = frontStamp;
			// add
			predIMUOnlyForScale->predictOneStep(&(*cur));
			// pop
			//navInfoQueue.pop_front();
			used++;
		}
		else
			break;

	}
	//printf("QueEval: before: %i; skipped: %i, used: %i, left: %i\n", totSize, skipped, used, navInfoQueue.size());
	predIMUOnlyForScale->z -= firstZ*0.001;	// make height to height-diff

	*zCorrupted = predIMUOnlyForScale->zCorrupted;
	*allCorrupted = abs(firstAdded - (int)from) + abs(lastAdded - (int)to) > 80;
	pthread_mutex_unlock(&navInfoQueueCS);

	if(*allCorrupted)
		printf("scalePackage corrupted (imu data gap for %ims)\n",abs(firstAdded - (int)from) + abs(lastAdded - (int)to));
	else if(*zCorrupted)
		printf("scalePackage z corrupted (jump in meters: %.3f)!\n",predIMUOnlyForScale->zCorruptedJump);

	printf("first: %f (%f); last: %f (%f)=> diff: %f (z alt diff: %f)\n",
			sum_first/num_first,
			num_first,
			sum_last/num_last,
			num_last,
			sum_last/num_last - sum_first/num_first,
			predIMUOnlyForScale->z
	);


	*out_end_pressure = sum_last/num_last;
	*out_start_pressure = sum_first/num_first;

	return TooN::makeVector(predIMUOnlyForScale->x,predIMUOnlyForScale->y,predIMUOnlyForScale->z);
}



//Input function to takein the navdata from ardrone driver

void LSDWrapper::newNavdata(ardrone_autonomy::Navdata* nav)
{
	lastNavinfoReceived = *nav;

	if(getMS(lastNavinfoReceived.header.stamp) > 2000000)
	{
		printf("LSDSystem: ignoring navdata package with timestamp %f\n", lastNavinfoReceived.tm);
		return;
	}
	if(lastNavinfoReceived.header.seq > 2000000 || lastNavinfoReceived.header.seq < 0)
	{
		printf("LSDSystem: ignoring navdata package with ID %i\n", lastNavinfoReceived.header.seq);
		return;
	}

	// correct yaw with filter-yaw (!):
	lastNavinfoReceived.rotZ = filter->getCurrentPose()[5];

	pthread_mutex_lock( &navInfoQueueCS );
	navInfoQueue.push_back(lastNavinfoReceived);

	if(navInfoQueue.size() > 1000)	// respective 5s
	{
		navInfoQueue.pop_front();
		if(!navQueueOverflown)
			printf("NavQue Overflow detected!\n");
		navQueueOverflown = true;
	}
	pthread_mutex_unlock( &navInfoQueueCS );

	//filter->setPing(nav->pingNav, nav->pingVid);

	imuOnlyPred->yaw = filter->getCurrentPose()[5];
	imuOnlyPred->predictOneStep(&lastNavinfoReceived);
}


//newImage modified for the lsd integration
void LSDWrapper::newImage(sensor_msgs::ImageConstPtr img)
{

	// convert to CVImage
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);


	boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);

	// copy to internal image, convert to bw, set flag.
	if(ros::Time::now() - img->header.stamp > ros::Duration(30.0))
		mimFrameTimeRos = (ros::Time::now()-ros::Duration(0.001));
	else
		mimFrameTimeRos = (img->header.stamp);

	mimFrameTime = getMS(mimFrameTimeRos);

	//mimFrameTime = getMS(img->header.stamp);
	mimFrameSEQ = img->header.seq;
	mimFrameBW.data = cv->image;
	mimFrameBW.timestamp = Timestamp(img->header.stamp.toSec());

	newImageAvailable = true;

	lock.unlock();
	new_frame_signal.notify_all();
}




void LSDWrapper::on_key_down(int key)
{
	if(key == 114) // r
	{
		node->publishCommand("p reset");
	}
	if(key == 117) // u
	{
		node->publishCommand("p toggleUI");
	}
	if(key == 32) // Space
	{
		node->publishCommand("p space");
	}
	if(key == 107) // k
	{
		node->publishCommand("p keyframe");
	}
	if(key == 108) // l
	{
		node->publishCommand("toggleLog");
	}
	if(key == 115) // s
	{
		pthread_mutex_lock(&logScalePairs_CS);
		if(logfileScalePairs == 0)
		{
			logfileScalePairs = new std::ofstream();
			logfileScalePairs->open ("logScalePairs.txt");
			printf("\nSTART logging scale pairs\n\n");
		}
		else
		{
			logfileScalePairs->flush();
			logfileScalePairs->close();
			delete logfileScalePairs;
			logfileScalePairs = NULL;
			printf("\nEND logging scale pairs\n\n");
		}
		pthread_mutex_unlock(&logScalePairs_CS);
	}

	if(key == 109) // m
	{
		node->publishCommand("p toggleLockMap");
	}

	if(key == 110) // n
	{

		node->publishCommand("p toggleLockSync");
	}

	if(key == 116) // t
	{

		flushMapKeypoints = true;
	}

}


// Handle commands should be completely changed for LSD but of a low priority

// reached by typing "df p COMMAND" into console

bool LSDWrapper::handleCommand(std::string s)
{
	if(s.length() == 5 && s.substr(0,5) == "space")
	{
  		mpTracker->pressSpacebar();
	}

	// ptam reset: resets only PTAM, keeps filter state.
	if(s.length() == 5 && s.substr(0,5) == "reset")
	{
		//filter->clearPTAM();
  		Reset();

	}

	if(s.length() == 8 && s.substr(0,8) == "keyframe")
	{
		forceKF = true;
	}

	if(s.length() == 8 && s.substr(0,8) == "toggleUI")
	{
		if(drawUI == UI_NONE) drawUI = UI_DEBUG;
		else if(drawUI == UI_DEBUG) drawUI = UI_PRES;
		else if(drawUI == UI_PRES) drawUI = UI_NONE;
		else drawUI = UI_PRES;
	}

	if(s.length() == 11 && s.substr(0,11) == "lockScaleFP")
	{
		lockNextFrame = true;
	}


	if(s.length() == 14 && s.substr(0,14) == "toggleLockSync")
	{
		filter->allSyncLocked = !filter->allSyncLocked;


		if(filter->allSyncLocked)
		{
			printf("\n\nSYNC LOCKED!\n\n\n");
			node->publishCommand("u l PTAM sync locked.");
		}
		else
		{
			printf("\n\nSYNC UNLOCKED!\n\n\n");
			node->publishCommand("u l PTAM sync UNlocked.");
		}
	}

	return true;
}

void LSDWrapper::on_mouse_down(CVD::ImageRef where, int state, int button)
{
	double x = 4*(where.x/(double)this->myGLWindow->size().x - 0.5);
	double y = -4*(where.y/(double)this->myGLWindow->size().y - 0.5);
	char bf[100];


	node->publishCommand("c clearCommands");
	node->publishCommand("c lockScaleFP");

	if(button == 1)
		snprintf(bf,100,"c moveByRel %.3f %.3f 0 0",x,y);
	else
		snprintf(bf,100,"c moveByRel 0 0 %.3f %.3f",y,x*45);

	node->publishCommand(bf);
}
}