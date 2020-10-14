/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/





#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "util/settings.h"
#include "util/DatasetReader.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"
#include "dso_ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>


#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>


int start=0;
int end=100000;
bool preload=false;
std::string calib = "";
std::string vignetteFile = "";
std::string gammaFile = "";
std::string source = "";
std::string saveFile = "";
float playbackSpeed=0;
int mode=0;
bool useSampleOutput=false;

using namespace dso;

void parseArgument(char* arg)
{
	int option;
	char buf[1000];

	if(1==sscanf(arg,"sampleoutput=%d",&option))
	{
		if(option==1)
		{
			useSampleOutput = true;
			printf("USING SAMPLE OUTPUT WRAPPER!\n");
		}
		return;
	}
	if(1==sscanf(arg,"files=%s",buf))
	{
		source = buf;
		printf("loading data from %s!\n", source.c_str());
		return;
	}
  if(1==sscanf(arg,"quiet=%d",&option))
  {
    if(option==1)
    {
      setting_debugout_runquiet = true;
      printf("QUIET MODE, I'll shut up!\n");
    }
    return;
  }


  if(1==sscanf(arg,"nolog=%d",&option))
  {
    if(option==1)
    {
      setting_logStuff = false;
      printf("DISABLE LOGGING!\n");
    }
    return;
  }

	if(1==sscanf(arg,"nogui=%d",&option))
	{
		if(option==1)
		{
			disableAllDisplay = true;
			printf("NO GUI!\n");
		}
		return;
	}
	if(1==sscanf(arg,"nomt=%d",&option))
	{
		if(option==1)
		{
			multiThreading = false;
			printf("NO MultiThreading!\n");
		}
		return;
	}
	if(1==sscanf(arg,"preset=%d",&option))
	{
		printf("preset set to 0!\n");
		return;
	}

	if(1==sscanf(arg,"mode=%d",&option))
	{

		mode = option;
		if(option==0)
		{
			printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
		}
		if(option==1)
		{
			printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
			setting_photometricCalibration = 0;
			setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
			setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
		}
		if(option==2)
		{
			printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
			setting_photometricCalibration = 0;
			setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
			setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_minGradHistAdd=3;
		}
		return;
	}

	if(1==sscanf(arg,"calib=%s",buf))
	{
		calib = buf;
		printf("loading calibration from %s!\n", calib.c_str());
		return;
	}
	if(1==sscanf(arg,"vignette=%s",buf))
	{
		vignetteFile = buf;
		printf("loading vignette from %s!\n", vignetteFile.c_str());
		return;
	}

	if(1==sscanf(arg,"gamma=%s",buf))
	{
		gammaFile = buf;
		printf("loading gammaCalib from %s!\n", gammaFile.c_str());
		return;
	}
	
	if(1==sscanf(arg,"savefile=%s",buf))
	{
		saveFile = buf;
		printf("saving to %s on finish!\n", saveFile.c_str());
		return;
	}
	printf("could not parse argument \"%s\"!!\n", arg);
}




FullSystem* fullSystem = 0;
Undistort* undistorter = 0;
int frameID = 0;

//void vidCb(const sensor_msgs::ImageConstPtr img)
//{
//	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
//	assert(cv_ptr->image.type() == CV_8U);
//	assert(cv_ptr->image.channels() == 1);


//	if(setting_fullResetRequested)
//	{
//		std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
//		delete fullSystem;
//		for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
//		fullSystem = new FullSystem();
//		fullSystem->linearizeOperation=false;
//		fullSystem->outputWrapper = wraps;
//	    if(undistorter->photometricUndist != 0)
//	    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
//		setting_fullResetRequested=false;
//	}

//	MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows,(unsigned char*)cv_ptr->image.data);
//	ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1,0, 1.0f);
//	undistImg->timestamp=img->header.stamp.toSec(); // relay the timestamp to dso
//	fullSystem->addActiveFrame(undistImg, frameID);
//	frameID++;
//	delete undistImg;

//}


bool sortevents (const dvs_msgs::Event& ev1, const dvs_msgs::Event& ev2)
{
   return ev1.ts.toSec() < ev2.ts.toSec();
}


int main( int argc, char** argv )
{
	ros::init(argc, argv, "dso_ros");
	ros::NodeHandle nodehandle_;
	rosbag::Bag bag;
	std::string bagfilename = "/media/jhlee/4TBHDD/vivid/exp2/indoor/indoor_robust_global.bag";
    std::cout << "Loading bag file from : " << bagfilename << std::endl;
	bag.open( bagfilename , rosbag::bagmode::Read );
    std::vector<std::string> topics;
	topics.push_back(std::string("/dvs/image_raw"));
	topics.push_back(std::string("/dvs/events"));
	topics.push_back(std::string("/imu/data"));

	rosbag::View view(bag, rosbag::TopicQuery(topics));
	std::vector<dvs_msgs::Event> events_array = {};
	std::vector<sensor_msgs::Image::ConstPtr> images_array = {};
  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    const dvs_msgs::EventArrayConstPtr& dvsptr = m.instantiate<dvs_msgs::EventArray>();
    if (dvsptr != nullptr)
      for ( int i=0 ; i < (int)dvsptr->events.size() ; i++ )
        events_array.push_back(dvsptr->events[i]);
  }
  std::sort(events_array.begin(),events_array.end(),sortevents);

  int eventiter = 0;
  std::vector<uint8_t> evdata(240*180,0);
  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    const sensor_msgs::Image::ConstPtr& imgptr = m.instantiate<sensor_msgs::Image>();

    if ((imgptr != nullptr) and (imgptr->encoding == "mono8"))
    {
      sensor_msgs::Image::Ptr event_image = boost::make_shared<sensor_msgs::Image>();
      event_image->header = imgptr->header;
      event_image->height = imgptr->height;
      event_image->width = imgptr->width;
      event_image->encoding = imgptr->encoding;
      event_image->is_bigendian = imgptr->is_bigendian;
      event_image->step = imgptr->step;

      double timenow = imgptr->header.stamp.toSec();
      double eventt = events_array[0].ts.toSec();
      while (eventiter+1 < events_array.size() && timenow > eventt)
      {
        eventt = events_array[eventiter].ts.toSec();
        eventiter++;
      }
      int e_start = eventiter;
      int e_end = eventiter + 1e4;
      memset(evdata.data(),0,sizeof(uint8_t)*240*180);
      for (int i = e_start ; i < e_end ; i++)
      {
        evdata[(int)events_array[i].y * imgptr->width  + (int)events_array[i].x] = 255;
      }
      event_image->data = evdata;
      images_array.push_back(event_image); //use only things from dvs sensor
    }
  }
  std::cout << "Finished Loading bag file!" <<std::endl;
  std::cout << "loaded Event length : " << events_array.size() << std::endl;
  std::cout << "loaded Image length : " << images_array.size() << std::endl;
	bag.close();

	for(int i=1; i<argc;i++) parseArgument(argv[i]);


	playbackSpeed = 0;
	setting_desiredImmatureDensity = 500;
	setting_desiredPointDensity = 1000;
	setting_minFrames = 10;
	setting_maxFrames = 14;
	setting_maxOptIterations=10;
	setting_minOptIterations=5;
	setting_logStuff = false;
	setting_kfGlobalWeight = 1.3;
  setting_debugout_runquiet = true;

//    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

//    setGlobalCalib(
//            (int)undistorter->getSize()[0],
//            (int)undistorter->getSize()[1],
//            undistorter->getK().cast<float>());

  ImageFolderReader* reader = new ImageFolderReader(events_array,images_array,gammaFile,vignetteFile);
//	ImageFolderReader* reader = new ImageFolderReader(source,calib,gammaFile,vignetteFile);

	reader->setGlobalCalibration();

	int lstart=start;
	int lend = end;
	int linc = 1;
    fullSystem = new FullSystem();
	fullSystem->setGammaFunction(reader->getPhotometricGamma());
    fullSystem->linearizeOperation=(playbackSpeed==0);

    IOWrap::PangolinDSOViewer* viewer = 0;

	if(!disableAllDisplay)
    {
        viewer = new IOWrap::PangolinDSOViewer(wG[0],hG[0], false);
        fullSystem->outputWrapper.push_back(viewer);
    }


    if(useSampleOutput)
        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());

//    if(undistorter->photometricUndist != 0)
//    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

//    ros::Subscriber imgSub = nh.subscribe("image", 1, &vidCb);

//    fullSystem->printResult(saveFile);
	
    std::thread runthread([&]() {
        std::vector<int> idsToPlay;
        std::vector<double> timesToPlayAt;
        for(int i=lstart;i>= 0 && i< reader->getNumEventImages() && linc*i < linc*lend;i+=linc)
        {
            idsToPlay.push_back(i);
            if(timesToPlayAt.size() == 0)
            {
                timesToPlayAt.push_back((double)0);
            }
            else
            {
                double tsThis = reader->getTimestamp(idsToPlay[idsToPlay.size()-1]);
                double tsPrev = reader->getTimestamp(idsToPlay[idsToPlay.size()-2]);
                timesToPlayAt.push_back(timesToPlayAt.back() +  fabs(tsThis-tsPrev)/playbackSpeed);
            }
        }

        timeval tv_start;
        gettimeofday(&tv_start, NULL);
        clock_t started = clock();
        double sInitializerOffset=0;

        for(int ii=0;ii<(int)idsToPlay.size(); ii++)
        {
            if(!fullSystem->initialized)	// if not initialized: reset start time.
            {
                gettimeofday(&tv_start, NULL);
                started = clock();
                sInitializerOffset = timesToPlayAt[ii];
            }

            int i = idsToPlay[ii];

            ImageAndExposure* img;
            img = reader->getEventImage(i);

            fullSystem->addActiveFrame(img, i);

            delete img;

            if(fullSystem->initFailed || setting_fullResetRequested)
            {
                if(ii < 250 || setting_fullResetRequested)
                {
                    printf("RESETTING!\n");

                    std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
                    delete fullSystem;

                    for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();

                    fullSystem = new FullSystem();
                    fullSystem->setGammaFunction(reader->getPhotometricGamma());
                    fullSystem->linearizeOperation = (playbackSpeed==0);


                    fullSystem->outputWrapper = wraps;

                    setting_fullResetRequested=false;
                }
            }

            if(fullSystem->isLost)
            {
                    printf("LOST!!\n");
                    break;
            }

        }
        fullSystem->blockUntilMappingIsFinished();
        clock_t ended = clock();
        struct timeval tv_end;
        gettimeofday(&tv_end, NULL);

        int numFramesProcessed = abs(idsToPlay[0]-idsToPlay.back());
        double numSecondsProcessed = fabs(reader->getTimestamp(idsToPlay[0])-reader->getTimestamp(idsToPlay.back()));
        double MilliSecondsTakenSingle = 1000.0f*(ended-started)/(float)(CLOCKS_PER_SEC);
        double MilliSecondsTakenMT = sInitializerOffset + ((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
        printf("\n======================"
                "\n%d Frames (%.1f fps)"
                "\n%.2fms per frame (single core); "
                "\n%.2fms per frame (multi core); "
                "\n%.3fx (single core); "
                "\n%.3fx (multi core); "
                "\n======================\n\n",
                numFramesProcessed, numFramesProcessed/numSecondsProcessed,
                MilliSecondsTakenSingle/numFramesProcessed,
                MilliSecondsTakenMT / (float)numFramesProcessed,
                1000 / (MilliSecondsTakenSingle/numSecondsProcessed),
                1000 / (MilliSecondsTakenMT / numSecondsProcessed));
        //fullSystem->printFrameLifetimes();
        if(setting_logStuff)
        {
            std::ofstream tmlog;
            tmlog.open("logs/time.txt", std::ios::trunc | std::ios::out);
            tmlog << 1000.0f*(ended-started)/(float)(CLOCKS_PER_SEC*reader->getNumImages()) << " "
                  << ((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f) / (float)reader->getNumImages() << "\n";
            tmlog.flush();
            tmlog.close();
        }
    });

    if(viewer != 0)
        viewer->run();

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
        delete ow;
    }

	runthread.join();
	ros::AsyncSpinner spinner(12);
	spinner.start();

//    delete undistorter;
    delete fullSystem;
	delete reader;

	return 0;
}

