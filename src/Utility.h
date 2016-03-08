
#ifndef UTILITY_H
#define UTILITY_H

#include <ros/ros.h>
#include <ros/network.h>
#include <iostream>
#include <map>
#include <utility>
#include <iterator>
#include <vector>
#include <ctime>
#include <algorithm>
#include <cstdlib>
#include <math.h>
#include <algorithm>
#include <new>
#include <opencv2/opencv.hpp>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include "std_msgs/builtin_int16.h"
#include <sstream>
#include <pthread.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include "../msg_gen/cpp/include/explorationISL/imu.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <drrobot_jaguarV2_player/MotorInfo.h>
#include <drrobot_jaguarV2_player/MotorInfoArray.h>
#include <drrobot_jaguarV2_player/RangeArray.h>
#include <drrobot_jaguarV2_player/Range.h>
#include <drrobot_jaguarV2_player/PowerInfo.h>
#include <drrobot_jaguarV2_player/StandardSensor.h>
#include <drrobot_jaguarV2_player/CustomSensor.h>
#include <std_msgs/Header.h>
#include <dynamic_reconfigure/server.h>
#include <explorationISL/ExplorationISLConfig.h>

const double PI = 3.14159265359;
const double encoder_coeff = 640;
const double overrun = 0.0;
const double turn_coeff = 0.8;
const double gyro_coeff = 813;
const double gyro_drift_velocity = -14.266;
const double gyro_drift_acceleration = 0;
const double k_p = 0.2;           //trajectory correction rate
const double cliff_treshold = 50;
const double arm_vel = 0.3;
const double distanceThr = 1.9;
const double robot_width = 1;
const double critical_turn_threshold = 0.3;
const double critical_range_threshold = 3;

const bool print_data = true;
const bool gyro_active = true;
const bool tracker_active = true;
const bool use_graph = true;

enum State{
    DeterminingDirection = 1,
    Turning = 2,
    Translation = 3,
    MovingBack = 4
};

struct Position{
    double x;
    double y;
    double th;

    Position(double x, double y, double th):x(x),y(y),th(th){}
};


using namespace std;
using namespace cv;

class Graph{

private:

    int v;

    vector< map<int,double> > vertexList;

public:

    int current_place;

    vector<bool> isFinished;

    vector<bool> isPlaceFinished;

    vector<bool> isCritical;

    vector<int> placeIDList;

    vector<Position> poseList;

    vector< sensor_msgs::LaserScan > scanList;

    vector< sensor_msgs::Image > imageList;

    Graph();

    Graph(int v);

    vector< map<int,double> > getGraph();

    bool is_connected(bool *graph[], int size);

    bool isAdj(int x, int y);

    int getSize();

    int test();

    void addVertex();

    void addEdge(int a, int b, double distance);

    double shortestPath(int source, int destination);

    int findBackTrace(int index, int id);

    double prob();

    ~Graph();

};

class Conncomp1d{

public:

    int binaryImage[10000];

    double compwidth[10000];

    double centroid[10000];

    int compNumber;

    int count;

    int i;

    int cx;

    int findcompNumber();

    void findcompwidth();

    void findcentroid();

    int SetValues1(int *p1, int count);

};

class StdDeviation
{

private:
    int count;

    double value[10000];

    double mean;

public:

    double CalculateMean();

    double CalculateVariane();

    double CalculateSampleVariane();

    int SetValues(double *p, int count);

    double GetStandardDeviation();

    double GetSampleStandardDeviation();

};

#endif
