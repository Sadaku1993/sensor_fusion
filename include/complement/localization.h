#ifndef __LOCALIZATION_H__
#include <time_util/stopwatch.h>
#include <sensor_msgs/PointCloud.h>

using namespace std;

class Localization{
public:
	double x,y,pitch,roll,yaw,v,w,d_yaw;
    double z;
    double zz;
    double time;

    double dt;


	Stopwatch sw;
	
	Localization();
	~Localization();
	void showState();
	void gettime();
	void altering();
	void altering2();
	void altering3();
	void altering4();
	void alter_dyaw();
	void start();
};

#include "impl/localization.hpp"
#endif
