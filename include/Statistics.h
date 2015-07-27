/* Add Static helper to store statistics along a run */
#ifndef __STATISTICS_H
#define __STATISTICS_H

#include <sqlite3.h>
#include <string>
#include "KeyFrame.h"

namespace ORB_SLAM {

class Stats {
public:
	//Sets the id for ant and route A{antID}R{RouteId} and the filepath for the database
	Stats(const char *dbpath, bool _active, bool _saveMap, std::string _mapPathPrefix);
	~Stats();
	void setId(std::string _id);
	void setInitialisationTime(double t);
	void saveFrameInfo(ORB_SLAM::Frame &frame, bool isKeyFrame); //Save a frame info
	void saveFrameInfo(ORB_SLAM::KeyFrame &frame); //Save a frame info
	void updateTrackerLostInfo(long t); //when tracker lost update count
	void saveGlobalInfo(long video_duration); //save global stats
	void saveMap(vector<ORB_SLAM::MapPoint*> mapPoints);
	void setLostAt(long t){
		lostAt.push_back(t*1000);//convert to ms
	}
	bool active;
	string getId(){
		return id;
	}
	void reset();

protected:
	long initialisedAt;
	int timesLost;
	long totalTimeLost;
	long maxTimeLost;
	bool dosaveMap;
	std::string id;
	std::string mapPathPrefix;
	sqlite3* db;
	std::vector<long> lostAt;
};

}
#endif