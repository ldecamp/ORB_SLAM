/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <opencv2/core/core.hpp>

#include "Tracking.h"
#include "FramePublisher.h"
#include "Map.h"
#include "MapPublisher.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
//#include "ORBVocabulary.h"
#include "Surf64Vocabulary.h"
#include "Statistics.h"
#include "CommandHandle.h"
#include "Converter.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ORB_SLAM");
    ros::start();

    cout << endl << "ORB-SLAM Copyright (C) 2014 Raul Mur-Artal" << endl <<
         "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
         "This is free software, and you are welcome to redistribute it" << endl <<
         "under certain conditions. See LICENSE.txt." << endl;

    if (argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM ORB_SLAM path_to_vocabulary path_to_settings (absolute or relative to package directory)" << endl;
        ros::shutdown();
        return 1;
    }

    //Load ORB Vocabulary
    string strSettingsFile;
    if (boost::starts_with(argv[2], "/"))
        strSettingsFile = argv[2];
    else
        strSettingsFile = ros::package::getPath("ORB_SLAM") + "/" + argv[2];

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        ROS_ERROR("Wrong path to settings. Path must be absolut or relative to ORB_SLAM package directory.");
        ros::shutdown();
        return 1;
    }

    //Create Frame Publisher for image_view
    ORB_SLAM::FramePublisher FramePub;

    //Load ORB Vocabulary
    string strVocFile;
    if (boost::starts_with(argv[1], "/"))
        strVocFile = argv[1];
    else
        strVocFile = ros::package::getPath("ORB_SLAM") + "/" + argv[1];

    cout << endl << "Loading ORB Vocabulary. This could take a while." << endl;
    cv::FileStorage fsVoc(strVocFile.c_str(), cv::FileStorage::READ);
    if (!fsVoc.isOpened())
    {
        cerr << endl << "Wrong path to vocabulary. Path must be absolut or relative to ORB_SLAM package directory." << endl;
        ros::shutdown();
        return 1;
    }

    //ORB_SLAM::ORBVocabulary Vocabulary;
    ORB_SLAM::Surf64Vocabulary Vocabulary;
    Vocabulary.load(fsVoc);
    cout << "Vocabulary loaded!" << endl << endl;

    //Initialize statistics gatherer
    const string dbPath = fsSettings["Statistics.databasePath"];
    int statsActive = fsSettings["Statistics.active"];
    int saveMap = fsSettings["Statistics.saveMap"];
    string mapPtsPath = fsSettings["Statistics.saveMapAt"];
    ORB_SLAM::Stats Statistics(dbPath.c_str(), 
        (statsActive == 0 ? false : true), 
        (saveMap == 0 ? false : true),
        mapPtsPath);

    //Create KeyFrame Database
    ORB_SLAM::KeyFrameDatabase Database(Vocabulary);

    //Create the map
    ORB_SLAM::Map World;

    FramePub.SetMap(&World);

    //Create Map Publisher for Rviz
    ORB_SLAM::MapPublisher MapPub(&World);

    //Initialize the Tracking Thread and launch
    ORB_SLAM::Tracking Tracker(&Vocabulary, &FramePub, &MapPub, &World, &Statistics, strSettingsFile);
    boost::thread trackingThread(&ORB_SLAM::Tracking::Run, &Tracker);

    Tracker.SetKeyFrameDatabase(&Database);

    //Initialize the Local Mapping Thread and launch
    ORB_SLAM::LocalMapping LocalMapper(&World);
    boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run, &LocalMapper);

    //Initialize the Loop Closing Thread and launch
    ORB_SLAM::LoopClosing LoopCloser(&World, &Database, &Vocabulary, &Statistics);
    boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, &LoopCloser);

    //Set pointers between threads
    Tracker.SetLocalMapper(&LocalMapper);
    Tracker.SetLoopClosing(&LoopCloser);

    LocalMapper.SetTracker(&Tracker);
    LocalMapper.SetLoopCloser(&LoopCloser);

    LoopCloser.SetTracker(&Tracker);
    LoopCloser.SetLocalMapper(&LocalMapper);

    //This "main" thread will show the current processed frame and publish the map
    float fps = fsSettings["Camera.fps"];
    if (fps == 0)
        fps = 30;

    ros::Rate r(fps);

    ORB_SLAM::CommandsHandle CommandsManager(&Statistics, &Tracker);

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/orb_slam/commands", 1,
                                       &ORB_SLAM::CommandsHandle::Commands_Callback,
                                       &CommandsManager);

    while (ros::ok())
    {
        FramePub.Refresh();
        MapPub.Refresh();
        Tracker.CheckResetByPublishers();
        r.sleep();
        ros::spinOnce();
    }

    // Tracker.Exit();//ensure lost stats ok

    ros::shutdown();

    return 0;
}
