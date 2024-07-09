#ifndef CONTROLLER_HELPERS
#define CONTROLLER_HELPERS

// General includes
#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

// ROS includes
#include <ros/console.h>

using std::minus;
using std::vector;
using std::pow;
using std::sqrt;
using std::transform;


// Logging pragmas
#define CONTROLLER_INFO(msg) \
	if (enable_debug_){ROS_INFO_STREAM("[CONTROLLER]: " << msg);}

#define CONTROLLER_WARN(msg) \
	if (enable_debug_){ROS_WARN_STREAM("[CONTROLLER]: " << msg);}

#define CONTROLLER_ERROR(msg) ROS_ERROR_STREAM("[CONTROLLER]: " << msg)

#define CONTROLLER_INFO_STREAM(msg) \
	if (enable_debug_){ROS_INFO_STREAM("[CONTROLLER]: " << msg);}

#define CONTROLLER_WARN_STREAM(msg) \
	if (enable_debug_){ROS_WARN_STREAM("[CONTROLLER]: " << msg);}

#define CONTROLLER_ERROR_STREAM(msg) ROS_ERROR_STREAM("[CONTROLLER]: " << msg)

#define CONTROLLER_INFO_ONCE(msg) \
	if (enable_debug_){ROS_INFO_STREAM_ONCE("[CONTROLLER]: " << msg);}

#define CONTROLLER_WARN_ONCE(msg) \
	if (enable_debug_){ROS_WARN_STREAM_ONCE("[CONTROLLER]: " << msg);}

#define CONTROLLER_ERROR_ONCE(msg) ROS_ERROR_STREAM_ONCE("[CONTROLLER]: " << msg)

#define CONTROLLER_INFO_ALWAYS(msg) ROS_INFO_STREAM("[CONTROLLER]: " << msg)
#define CONTROLLER_WARN_ALWAYS(msg) ROS_WARN_STREAM("[CONTROLLER]: " << msg)

double getEuclideanDistance3d(vector<double>& a, vector<double>& b)
{
	vector<double> diff{vector<double>(3,0)};
    transform(a.begin(), a.end(), b.begin(), diff.begin(), minus<double>());
    return (double) sqrt(pow(diff[0],2) + pow(diff[1],2) + pow(diff[2],2));
}


#endif //CONTROLLER_HELPERS
