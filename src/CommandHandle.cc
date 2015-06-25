#include "CommandHandle.h"
#include <boost/algorithm/string.hpp>
#include <vector>

using namespace ORB_SLAM;
using namespace boost;
using namespace std;

CommandsHandle::CommandsHandle(Stats* statistics, Tracking* tracker) {
	Statistics = statistics;
	Tracker = tracker;
}

void CommandsHandle::Commands_Callback(const std_msgs::String::ConstPtr& msg) {
	vector<string> result;
	cout << "Received message: " << msg->data << endl;
	split( result, msg->data, is_any_of(":"), token_compress_on );

	if (iequals(result[0], "SetVideoKey")) {
		string id = result[1];
		Statistics->setId(id);
		return;
	}

	if (iequals(result[0], "ResetMap")) {
		Tracker->Reset();
		return;
	}

	if (iequals(result[0], "SaveAndResetMap")) {
		Statistics->saveGlobalInfo();
		Tracker->Reset();
		return;
	}

}
