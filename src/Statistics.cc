#include <boost/format.hpp>
#include <iostream>
#include <stdlib.h>

#include "Converter.h"
#include "Statistics.h"

using namespace std;
using namespace ORB_SLAM;

// Define SQL query templates for database.
const char *CREATE_LOG_TABLE = "create table if not exists %s(timestamp bigint primary key asc,"
                               "isKF integer not null,"
                               "x float not null, y float not null, z float not null,"
                               "yaw float, pitch float, roll float);";
const char *DELETE_LOG_TABLE = "drop table if exists %s;"; //reset every run otherwise too expensive
const char *INSERT_LOG = "insert or replace into %s"
                         "(timestamp, isKF, x, y, z, yaw, pitch, roll)"
                         "values (%s, %s, %s, %s, %s, %s, %s, %s);";
const char *UPDATE_GLOBAL = "insert or replace into Global"
                            "(id, delay, timesLost, totalTimeLost, maxTimeLost, sequenceLength, lostDetails)"
                            "values (\"%s\", %s, %s, %s, %s, %s, \"%s\");";

static int db_callback(void *NotUsed, int argc, char **argv, char **azColName) {
	return 0;
}

Stats::Stats(const char *dbpath, bool _active, bool _saveMap, std::string _mapPathPrefix) {
	active = _active;
	dosaveMap = _saveMap;
	mapPathPrefix = _mapPathPrefix;
	if (!active) return;
	int rc = sqlite3_open(dbpath, &db);
	if (rc) {
		cout << "Could not initialise stats db" << endl;
		sqlite3_close(db);
		active = false;
		return;
	}
}

Stats::~Stats() {
	if (!active) return;
	sqlite3_close(db);
}


void Stats::setId(std::string _id) {
	if (!active || _id.empty()) return;
	id = _id;
	char *zErrMsg = 0;
	boost::format sql_del(DELETE_LOG_TABLE);
	sql_del % id;
	//delete log table if exists
	int rc = sqlite3_exec(db, sql_del.str().c_str(), db_callback, 0, &zErrMsg);
	if ( rc != SQLITE_OK ) {
		active = false;
		return;
	}
	boost::format sql_create(CREATE_LOG_TABLE);
	sql_create % id;
	//Recreate table
	rc = sqlite3_exec(db, sql_create.str().c_str(), db_callback, 0, &zErrMsg);
	if ( rc != SQLITE_OK ) {
		active = false;
		return;
	}
}

void Stats::updateTrackerLostInfo(long timeLost) {
	timesLost++;
	timeLost = timeLost * 1000; //convert to ms
	if (maxTimeLost < timeLost)
		maxTimeLost = timeLost;
	totalTimeLost += timeLost;
}

void Stats::setInitialisationTime(double t) {
	initialisedAt = (long)t * 1000;
}

void Stats::saveFrameInfo(ORB_SLAM::Frame &frame, bool isKeyFrame) {
	if (!active || id.empty()) return;
	char *zErrMsg = 0;
	boost::format qry(INSERT_LOG);
	double msTimeStamp = frame.mTimeStamp * 1000;
	qry % id % msTimeStamp % (isKeyFrame ? 1 : 0);


	cv::Mat Rcw = frame.mTcw.rowRange(0, 3).colRange(0, 3).t();
	cv::Mat tcw = frame.mTcw.rowRange(0, 3).col(3);
	//Get and add full pose
	cv::Mat ow = -Rcw * tcw;
	//Set X,Y,Z
	qry % ow.at<float>(0) % ow.at<float>(1) % ow.at<float>(2);

	//Add Yaw/Pitch/Roll
	vector<float> q = ORB_SLAM::Converter::toYawPitchRoll(Rcw);
	qry % q[0] % q[1] % q[2];

	//add log frame into db
	int rc = sqlite3_exec(db, qry.str().c_str(), db_callback, 0, &zErrMsg);
	if ( rc != SQLITE_OK ) {
		cout << "There was an error saving frame info. " << endl;
		cout << "Error msg: " << zErrMsg << endl;
	}
}

void Stats::saveFrameInfo(ORB_SLAM::KeyFrame& frame) {
	if (!active || id.empty()) return;
	char *zErrMsg = 0;
	boost::format qry(INSERT_LOG);
	double msTimeStamp = frame.mTimeStamp * 1000;
	qry % id % msTimeStamp % 1;

	cv::Mat Pwc = frame.GetPoseInverse();


	cv::Mat Rot = Pwc.rowRange(0, 3).colRange(0, 3);
	cv::Mat ow = Pwc.rowRange(0, 3).col(3);
	//Set X,Y,Z
	qry % ow.at<float>(0) % ow.at<float>(1) % ow.at<float>(2);

	//Add Yaw/Pitch/Roll
	vector<float> q = ORB_SLAM::Converter::toYawPitchRoll(Rot);
	qry % q[0] % q[1] % q[2];

	//add log frame into db
	int rc = sqlite3_exec(db, qry.str().c_str(), db_callback, 0, &zErrMsg);
	if ( rc != SQLITE_OK ) {
		cout << "There was an error saving frame info. " << endl;
		cout << "Error msg: " << zErrMsg << endl;
	}
}

void Stats::saveGlobalInfo(long video_duration) {
	if (!active || id.empty()) return;
	char *zErrMsg = 0;
	boost::format qry(UPDATE_GLOBAL);

	stringstream lostDetails;
	unsigned int N = lostAt.size();
	for (unsigned int i = 0; i < N; ++i)
	{
		if(lostAt[i]==0.0L)
			continue;
		lostDetails << lostAt[i];
		if (i != N - 1)
			lostDetails << ";";
	}

	qry % id
	% initialisedAt
	% timesLost
	% totalTimeLost
	% maxTimeLost
	% (video_duration * 1000L)
	% lostDetails.str();
	cout << "sql: " << qry.str() << endl;
	//update global config for entry
	int rc = sqlite3_exec(db, qry.str().c_str(), db_callback, 0, &zErrMsg);
	if ( rc != SQLITE_OK ) {
		cout << "There was an error saving global config. " << endl;
	}
	reset();
}

void Stats::saveMap(vector<ORB_SLAM::MapPoint*> mapPoints) {
	if (dosaveMap) {
		ofstream f;

		cout << "Saving map points to ASCII file" << endl;
		string mapPtsPath = mapPathPrefix + "/" + id + ".csv";
		f.open(mapPtsPath.c_str());
		f << fixed;

		for (size_t i = 0; i < mapPoints.size(); i++)
		{
			ORB_SLAM::MapPoint* mPt = mapPoints[i];
			cv::Mat wC = mPt->GetWorldPos();
			f << wC.at<float>(0) << ", " << wC.at<float>(1) << ", " << wC.at<float>(2) << endl;
		}
		f.close();
	}
}

void Stats::reset() {
	//reset counter once saved
	lostAt.clear();
	lostAt.resize(0);
	timesLost = 0;
	totalTimeLost = 0;
	maxTimeLost = 0;
}