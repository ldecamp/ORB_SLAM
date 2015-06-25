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
	Stats(const char *dbpath, bool _active);
	~Stats();
	void setId(std::string _id);
	void setInitialisationTime(double timeMs);
	void saveFrameInfo(ORB_SLAM::Frame frame, bool isKeyFrame); //Save a frame info
	void updateTrackerLostInfo(long time); //when tracker lost update count
	void saveGlobalInfo(); //save global stats

protected:
	bool active;
	long initialisedAt;
	int timesLost;
	long totalTimeLost;
	long maxTimeLost;
	std::string id;
	sqlite3* db;
};

}
#endif