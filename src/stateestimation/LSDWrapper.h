#pragma once
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
#ifndef __LSDWRAPPER_H
#define __LSDWRAPPER_H

#include "GLWindow2.h"
#include "TooN/se3.h"
#include <deque>
#include <chrono>
#include "sensor_msgs/Image.h"
#include "ardrone_autonomy/Navdata.h"
#include "cvd/thread.h"
#include "cvd/image.h"
#include "cvd/byte.h"
#include "MouseKeyHandler.h"
#include "boost/thread.hpp"
#include "LSD-SLAM/io_wrapper/timestamp.h"
#include "LSD-SLAM/io_wrapper/notify_buffer.h"
#include "LSD-SLAM/io_wrapper/timestamped_object.h"
#include "util/sophus_util.h"


namespace cv {
	class Mat;
}

namespace lsd_slam
{
class SlamSystem;
class Predictor;
class DroneKalmanFilter;
class DroneFlightModule;
class EstimationNode;
class KeyFrameGraph;
class LiveSLAMWrapperROS;
class Output3DWrapper;

typedef TooN::Vector<3> tvec3;
typedef TooN::SE3<> tse3;

// this is a wrapper around PTAM, doing the scale estimation etc.
// it runs in its own thread, and has its own window (Camera Image + PTAM points etc.)
// the thread re-renders old images, or (via callback) performs ROS-message handeling (i.e. tracking).
// it then updates the DroneKalmanFilter.

class LSDWrapper : private CVD::Thread, private MouseKeyHandler
{
private:
	// base window
	GLWindow2* myGLWindow;

	//Change this to Utils equivalent
	CVD::ImageRef desiredWindowSize;		// size the window scould get changed to if [changeSizeNextRender]
	CVD::ImageRef defaultWindowSize;		// size the window gets opened with
	bool changeSizeNextRender;


	// the associated thread's run function.
	// calls HandleFrame() every time a new frame is available.
	void run();

	void HandleFrame();

	// references to filter.
	DroneKalmanFilter* filter;
	EstimationNode* node;
	Output3DWrapper* outputWrapper;

	// -------------------- LSD Related stuff --------------------------------
	char charBuf[1000];
	std::string msg;

	Timestamped mimFrameBW;
	Timestamped mimFrameBW_workingCopy;
	int mimFrameTime;
	int mimFrameTime_workingCopy;
	unsigned int mimFrameSEQ;
	unsigned int mimFrameSEQ_workingCopy;
	ros::Time mimFrameTimeRos;
	ros::Time mimFrameTimeRos_workingCopy;
	int frameWidth, frameHeight;


	// Map is in my global Coordinate system. keyframes give the front-cam-position, i.e.
	// CFromW is "GlobalToFront". this is achieved by aligning the global coordinate systems in the very beginning.
	
	LiveSLAMWrapper* lsdTracker;
	Predictor* predConvert;			// used ONLY to convert from rpy to se3 and back, i.e. never kept in some state.
	Predictor* predIMUOnlyForScale;	// used for scale calculation. needs to be updated with every new navinfo...

	bool isInitialized;

	SlamSystem* monoOdometry;

	std::string outFileName;
	std::ofstream* outFile;
	
	float fx, fy, cx, cy;
	int width, height;


	int imageSeqNumber;

	Predictor* imuOnlyPred;	
	int lastScaleEKFtimestamp;
	
	bool resetLSDRequested;
	enum {UI_NONE = 0, UI_DEBUG = 1, UI_PRES = 2} drawUI;

	bool flushMapKeypoints;

	int lastAnimSentClock;
	enum {ANIM_NONE, ANIM_TOOKKF, ANIM_GOOD, ANIM_INIT, ANIM_LOST, ANIM_FALSEPOS} lastAnimSent;

	//int lastGoodPTAM;	/// approx. timestamp of last good ptam observation... inaccurate!
	int lastGoodYawClock;
	int isGoodCount;	// number of succ. tracked frames in a row.

	TooN::Vector<3> LSDPositionForScale;
	int lsdPositionForScaleTakenTimestamp;
	int framesIncludedForScaleXYZ;
	std::deque<ardrone_autonomy::Navdata> navInfoQueue;
	bool navQueueOverflown;
	TooN::Vector<3> evalNavQue(unsigned int from, unsigned int to, bool* zCorrupted, bool* allCorrupted, float* out_start_pressure, float* out_end_pressure);
	

	// keep Running; locknext frame removed
	bool keepRunning;

	boost::condition_variable  new_frame_signal;
	boost::mutex new_frame_signal_mutex;


	// resets PTAM tracking
	void ResetInternal();

	void renderGrid(TooN::SE3<> camFromWorld);

	int videoFramePing;

	std::ofstream* logfileScalePairs;
	static pthread_mutex_t logScalePairs_CS; //pthread_mutex_lock( &cs_mutex );

public:

	LSDWrapper(DroneKalmanFilter* dkf, EstimationNode* nde);
	~LSDWrapper(void);

	// ROS exclusive: called by external thread if a new image/navdata is received.
	// takes care of sync etc.
	void newImage(sensor_msgs::ImageConstPtr img);
	void newNavdata(ardrone_autonomy::Navdata* nav);
	bool newImageAvailable;

	bool handleCommand(std::string s);
	bool mapLocked;
	int maxKF;
	static pthread_mutex_t navInfoQueueCS; //pthread_mutex_lock( &cs_mutex );
	static pthread_mutex_t shallowMapCS; //pthread_mutex_lock( &cs_mutex );

	// Event handling routines.
	// get called by the myGLWindow on respective event.
	virtual void on_key_down(int key);
	//virtual void on_mouse_move(CVD::ImageRef where, int state);
	virtual void on_mouse_down(CVD::ImageRef where, int state, int button);
	//virtual void on_event(int event);
	
	// resets PTAM tracking
	inline void Reset() {resetLSDRequested = true;};

	/** Runs the main processing loop. Will never return. */
	void Loop();

	/** Requests a reset from a different thread. */
	void requestReset();

	/** Callback function for new RGB images. */
	void newImageCallback(const cv::Mat& img, Timestamp imgTime);

	/** Writes the given time and pose to the outFile. */
	void logCameraPose(const SE3& camToWorld, double time);
	
	
	inline SlamSystem* getSlamSystem() {return monoOdometry;}

	// start and stop system and respective thread.
	void startSystem();
	void stopSystem();

	enum {LSD_IDLE = 0, LSD_LOST = 1, LSD_GOOD = 2, LSD_TOOKKF = 3} LSDStatus;

	TooN::SE3<> lastLSDResultRaw;
	std::string lastLSDMessage;

	// for map rendering: shallow clone
	std::vector<tvec3> mapPointsTransformed;
	std::vector<tse3> keyFramesTransformed;

	ardrone_autonomy::Navdata lastNavinfoReceived;

	int LSDInitializedClock;

};

#endif /* __LSDWRAPPER_H */
}