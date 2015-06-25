#ifndef __COMMANDS_HANDLER_H
#define __COMMANDS_HANDLER_H

#include "Statistics.h"
#include "Tracking.h"
#include "std_msgs/String.h"

namespace ORB_SLAM {
class CommandsHandle {
public:
	CommandsHandle(Stats* statistics, Tracking* tracker);
	void Commands_Callback(const std_msgs::String::ConstPtr& msg);

protected:
	Stats* Statistics;
	Tracking* Tracker;
};
}

#endif