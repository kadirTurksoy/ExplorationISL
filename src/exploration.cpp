/*
 * File:   main.cpp
 * Author: macbookpro
 *
 * Created on June 18, 2013, 9:56 PM
 */
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <algorithm>
#include <new>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

/*#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>*/


#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>

#include <pthread.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

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


//#include "boost/math/special_functions/factorials.hpp"

bool active = false;
bool stop_flag = true;

double *direction;
int numberOfDirections;
const double PI = 3.14;
double velocity;
double translation_amount;

int move_back_control=0;
int move_back_counter[1000]={-1,-1};
double positionc [1000][2]={0,0};

int node[1000]={0};
int nd=1;

int position_control=0;

double directionF [1000];
double direction1;
double totalOrientation [1000]={0};
double ThresholdLimit=1;
int ind;
int recorded_moment;

double distanceThr=1.9;

//  general wariables

double robot_width=5;

double position_threshold;



void paramCallback(explorationISL::ExplorationISLConfig params, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %f %f %s",
              params.max_velocity,params.translation_interval,
              params.active?"True":"False");

    velocity = params.max_velocity;
    translation_amount = params.translation_interval;
    active = params.active;
    if(!active) stop_flag = true;
}


using namespace std;
//using namespace boost::math;
using namespace cv;


class conncomp1d

{
public:
    int binaryImage[10000];
    int compNumber;
    double compwidth[10000];
    double compwidth1[100];
    Mat widthA;
    double centroid[10000];;
    int maxe;
    int i;
    int cx;


public:
    int findcompNumber() {
        compNumber=0;
        for (i=0; i<maxe; i++){


            if(i==0){

                if (binaryImage[i]==1)
                    compNumber=compNumber+1;

            }




            if(i!=0) {


                if (binaryImage[i]==1 && binaryImage[i-1]==1) {


                }

                if (binaryImage[i]==1 && binaryImage[i-1]==0) {
                    compNumber=compNumber+1;
                    //compwidth[compNumber]=1;

                }


            }



        }



        return compNumber;

    }


    void findcompwidth(int segmentNumber) {

        //double *compwidth;
        //compwidth=new double[segmentNumber];
        cx=-1;

        for (i=0; i<maxe; i++){



            if(i==0){ if (binaryImage[0]==1) {
                    cx=cx+1;
                    compwidth[cx]=1;

                }


            }

            else {

                if (binaryImage[i]==1 && binaryImage[i-1]==1) {
                    compwidth[cx]=compwidth[cx]+1;

                    //cout << compwidth[cx];

                }

                if (binaryImage[i]==0 && binaryImage[i-1]==1) {
                    cx=cx+1;
                    compwidth[cx]=0;



                }


                if (binaryImage[i]==1 && binaryImage[i-1]==0) {

                    if (cx==-1) {
                        cx=0;}
                    compwidth[cx]=1;
                    //cout << compwidth[cx];



                }



            }



        }




        //delete compwidth;


    }










    void findcentroid(int segmentNumber) {


        //double cmpwidth=findcompwidth();

        //centroid=new double[segmentNumber];
        cx=-1;
        double  width_counter=0;

        for (i=0; i<maxe; i++){

            if(i==0){ if (binaryImage[0]==1) {
                    cx=cx+1;
                    centroid[cx]=0;

                }


            }

            else {

                if (binaryImage[i]==1 && binaryImage[i-1]==1) {

                    //width_counter=width_counter+1;

                    centroid[cx]=centroid[cx]+0.5;

                }

                if (binaryImage[i]==1 && binaryImage[i-1]==0) {
                    cx=cx+1;
                    //centroid[cx-1]=centroid[cx-1]+width_counter/2;
                    centroid[cx]=i;



                }


            }



        }




        //delete centroid;


    }




    int SetValues1(int *p1, int count)
    {
        if(count > 2000)
            return -1;
        maxe = count;
        for(int i = 0; i < count; i++)
            binaryImage[i] = p1[i];
        return 0;
    }

};

class StdDeviation
{

private:
    int max;
    double value[10000];

    double mean;

public:

    double CalculateMean()
    {
        double sum = 0;
        for(int i = 0; i < max; i++)
            sum += value[i];
        return (sum / max);
    }

    double CalculateVariane()
    {
        mean = CalculateMean();

        double temp = 0;
        for(int i = 0; i < max; i++)
        {
            temp += (value[i] - mean) * (value[i] - mean) ;
        }
        return temp / max;
    }

    double CalculateSampleVariane()
    {
        mean = CalculateMean();

        double temp = 0;
        for(int i = 0; i < max; i++)
        {
            temp += (value[i] - mean) * (value[i] - mean) ;
        }
        return temp / (max - 1);
    }

    int SetValues(double *p, int count)
    {
        if(count > 10000)
            return -1;
        max = count;
        for(int i = 0; i < count; i++)
            value[i] = p[i];
        return 0;
    }

    double GetStandardDeviation()
    {
        return sqrt(CalculateVariane());
    }

    double GetSampleStandardDeviation()
    {
        return sqrt(CalculateSampleVariane());
    }

};


void printarray (int arg[], int length) {
    for (int n=0; n<length; n++)
        cout << arg[n] << " ";
    cout << "\n";
}


void printarray1 (double arg[], int length) {
    for (int n=0; n<length; n++)
        cout << arg[n] << " ";
    cout << "\n";
}


void printarray2 (double arg[], int length) {
    for (int n=0; n<length; n++)
        cout << arg[n] << " ";
    cout << "\n";
}



double finddirection(double bubble [], int bubbleLength) {

    int panNumber=bubbleLength;

    bubble[0]=0;

    //double angle_norm[27]={-13 ,-12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};


    double *angle_norm;
    angle_norm=new double[panNumber];

    double angle_delta=PI/panNumber;


    for(int j=0;j<panNumber;j++){

        angle_norm[j]=(-PI/2)+j*angle_delta;
    }


    double positionThreshold=0.2;
    int position_control=0;



    /*

         for(int j=0;j<panNumber;j++){

             if (bubble[j]<ThresholdLimit) {

    bubble[j]=0;

             }
    }

    */

    //    int panNumber=sizeof(bubble)/sizeof(bubble[0]);

    StdDeviation sd;

    sd.SetValues(bubble, bubbleLength);

    // bubble processing

    double mean = sd.CalculateMean();
    double stdev = sd.GetStandardDeviation();

    double thr=mean+stdev;




    int c=0;
    int i;
    for(i=0; i<panNumber; i++){

        if (bubble[i]>thr){

            c=c+1;


        }   }

    int *indx1;



    indx1=new int[c];

    int c1=0;



    for(i=0; i<panNumber; i++){



        if (bubble[i]>thr){


            indx1[c1]=i;
            c1=c1+1;


        }   }


    int* indx11;
    indx11=indx1;

    int *indx2;
    int szindx1=c1;




    indx2=new int[panNumber];
    for(i=0; i<panNumber; i++ ) {


        if (i<szindx1)
            indx2[i]=indx1[i];

        else
            indx2[i]=0;


    }



    int *panGroupBit;



    panGroupBit=new int[panNumber];

    int cx2=0;



    for (i=0; i<panNumber; i++){





        int h=indx2[cx2];


        if(h==i){
            panGroupBit[i]=1;

            cx2=cx2+1;



        }

        else
            panGroupBit[i]=0;



    }



    //int *panGroupBitFull;




    // connecting components



    conncomp1d conn;

    conn.SetValues1(panGroupBit,panNumber);

    int numberofSegments=conn.findcompNumber();

    //printarray(panGroupBit,panNumber);




    conn.findcompwidth(numberofSegments);

    conn.findcentroid(numberofSegments);

    //    conn.findcompwidthM(numberofSegments);

    //printarray1(conn.compwidth,3);
    // printarray(,30);





    Mat A(1,numberofSegments,CV_32F,Scalar(0)),areas_sorted,areas_indx;


    cv::Mat areas(1,numberofSegments , CV_64F,conn.compwidth);


    cv::Mat centroidM(1,numberofSegments , CV_64F,conn.centroid);




    cv::sort(areas, areas_sorted, CV_SORT_EVERY_ROW + CV_SORT_DESCENDING);
    cv::sortIdx(areas, areas_indx, CV_SORT_EVERY_ROW + CV_SORT_DESCENDING);


    double *centroid_sort;
    centroid_sort=new double[numberofSegments];

    double *areas_sort;
    areas_sort=new double[numberofSegments];

    // arrange orinetations related to their widths
    for(i=0 ;i<numberofSegments;i++){

        int test;
        test = areas_indx.at<int>(i);

        //cout<< test;
        centroid_sort[i]=conn.centroid[test];
        areas_sort[i]=conn.compwidth[test];

    }




    cv::Mat centroid_sortM(1,numberofSegments , CV_64F,centroid_sort);



    // detecting small components that robot can not enter

    int real_segment=0;
    for(i=0 ;i<numberofSegments;i++){

        if (areas_sort[i]>1) {
            real_segment=real_segment+1;

        }


    }


    //    int real_segment=numberofSegments ;

    // generating the utility function == depth*width
    double *real_areas;
    real_areas=new double[real_segment];

    double *real_centroids;
    real_centroids=new double[real_segment];

    int *real_centroids_round;
    real_centroids_round=new int[real_segment];

    double *depth_areas;
    depth_areas=new double[real_segment];

    double *target_function;
    target_function=new double[real_segment];

    int c11=-1;

    for(i=0 ;i<numberofSegments;i++){

        if (areas_sort[i]>robot_width) {
            c11=c11+1;
            real_areas[c11]=areas_sort[i];
            real_centroids[c11]=centroid_sort[i];

            int kr=cvRound(real_centroids[c11]);

            real_centroids_round[c11]=kr;
            double cr=0;

            for (i=kr-5;i<kr+5;i++)
            {

                cr=cr+bubble[i];

            }
            depth_areas[c11]=cr/11;
            target_function[c11]=depth_areas[c11]*real_areas[c11];

        }


    }



    // sorting the utilty values
    cv::Mat target_functionM(1,real_segment , CV_64F,target_function),target_indx;



    cv::sortIdx(target_functionM, target_indx, CV_SORT_EVERY_ROW + CV_SORT_DESCENDING);





    double *target_function_sort;
    target_function_sort=new double[real_segment];

    double *depth_areas_sort;
    depth_areas_sort=new double[real_segment];

    double *centroid_sort2;
    centroid_sort2=new double[real_segment];

    double *real_orientations;
    real_orientations=new double[real_segment];

    for(i=0 ;i<real_segment;i++){

        int test1;
        test1 = target_indx.at<int>(i);


        centroid_sort2[i]=centroid_sort[test1];
        target_function_sort[i]=target_function[test1];
        depth_areas_sort[i]=depth_areas[test1];
        int kr11=cvRound(centroid_sort2[i]);

        real_orientations[i]=angle_norm[kr11];   /////////////// change 36
    }


    direction1=real_orientations[0];

    if (depth_areas_sort[0]<distanceThr)
    {


        //direction1=PI;

    }



    for(i=0 ;i<real_segment;i++){

        if (depth_areas_sort[i]<distanceThr)
        {


            real_orientations[i]=PI;

        }

    }


    direction=new double[real_segment];
    direction=real_orientations;

    numberOfDirections=real_segment;

    return (depth_areas_sort[0]);
    delete[] angle_norm;
    delete[] indx1;
    delete[] indx2;
    delete[] panGroupBit;
    delete[] centroid_sort;
    delete[] areas_sort;
    delete[] real_areas;
    delete[] real_centroids;
    delete[] real_centroids_round;
    delete[] depth_areas;
    delete[] target_function;

    delete[] target_function_sort;
    delete[] depth_areas_sort;
    delete[] centroid_sort2;
    delete[] real_orientations;
    delete[] direction;


}




int encoderRight=1;
int encoderLeft=1;



float *bubble_scan;

int bubble_size;

int test=0;

void motorSensorCallback(const drrobot_jaguarV2_player::MotorInfoArray::ConstPtr& msg)
{
    // logSensorMessage("drrobot_motor", *msg, STYPE_DRROBOT_PLAYER );
    //motorMessageWriteToFile(msg,STYPE_DRROBOT_PLAYER_TXT,"drrobot_motor");

    int msgSize = msg->motorInfos.capacity();





    if (msgSize == 6)
    {


        encoderRight= msg->motorInfos[3].encoder_pos;
        encoderLeft=msg->motorInfos[4].encoder_pos;
        /*
                ROS_INFO("Motor Encoder Pos: [%d, %d, %d, %d, %d, %d]", msg->motorInfos[0].encoder_pos, msg->motorInfos[1].encoder_pos, msg->motorInfos[2].encoder_pos
                         , msg->motorInfos[3].encoder_pos, msg->motorInfos[4].encoder_pos, msg->motorInfos[5].encoder_pos);
                ROS_INFO("Motor Encoder Vel: [%d, %d, %d, %d, %d, %d]", msg->motorInfos[0].encoder_vel, msg->motorInfos[1].encoder_vel, msg->motorInfos[2].encoder_vel
                         , msg->motorInfos[3].encoder_vel, msg->motorInfos[4].encoder_vel, msg->motorInfos[5].encoder_vel);
                ROS_INFO("Motor Encoder Dir: [%d, %d, %d, %d, %d, %d]", msg->motorInfos[0].encoder_dir, msg->motorInfos[1].encoder_dir, msg->motorInfos[2].encoder_dir
                         , msg->motorInfos[3].encoder_dir, msg->motorInfos[4].encoder_dir, msg->motorInfos[5].encoder_dir);
                ROS_INFO("Motor Motor Current: [%2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f]", msg->motorInfos[0].motor_current, msg->motorInfos[1].motor_current, msg->motorInfos[2].motor_current
                         , msg->motorInfos[3].motor_current, msg->motorInfos[4].motor_current, msg->motorInfos[5].motor_current);
                ROS_INFO("Motor Motor_PWM: [%d, %d, %d, %d, %d, %d]", msg->motorInfos[0].motor_pwm, msg->motorInfos[1].motor_pwm, msg->motorInfos[2].motor_pwm
                         , msg->motorInfos[3].motor_pwm, msg->motorInfos[4].motor_pwm, msg->motorInfos[5].motor_pwm);

*/
    }


}
// mustafa
float bubblex[1080]; // scan size is 1080

double bubbleFinal[1080];




void callback_scan(const sensor_msgs::LaserScanConstPtr& scan)
{


    float *bubble_scan = (float*)scan->ranges.data();



    bubble_size=scan->ranges.size();

    for (int i=0;i<bubble_size;i++)
    {

        bubblex[i]=*bubble_scan;

        bubble_scan++;




    }

    double bubblex2[1080];
    for (int i=0;i<bubble_size;i++)
    {
        bubblex2[i]=(double)bubblex[i];

        if (isnan(bubblex2[i])==1)
        {
            bubblex[i]=0.1;
        }

        if (isinf(bubblex2[i])==1)
        {
            if(i>0) {
                bubblex[i]=bubblex[i-1]; }
            else {
                bubblex[i]=5;
            }
        }





        bubbleFinal[i]=(double)bubblex[i];

    }

    // end mustafa
    //scan->angle_min;
    //scan->angle_increment;

    //logSensorMessage("scan", *scan, STYPE_LASER );
    //laserScanWriteToFile(scan,STYPE_LASER_TXT,"scan");
    //Q_EMIT scanSignal(*scan);

    //  Q_EMIT gpsSignal_raw("SCAN");
}



int main(int argc, char** argv) {





    int encoderRightInitial;
    int encoderLeftInitial;

    int encoderRightInitial2;
    int encoderLeftInitial2;

    int encoderDifference1;
    int encoderDifference2;



    int encoderRotation=290; // encoder value for 90 degree turning

    int encoderTranslation=640; // encoder value for 1m translation

    double turning_amount;



    int turningFinished=0;

    double impactControl;
    int impactRange=50;



    ros::init(argc, argv, "explorationISL");

    dynamic_reconfigure::Server<explorationISL::ExplorationISLConfig> server;
    dynamic_reconfigure::Server<explorationISL::ExplorationISLConfig>::CallbackType f;

    f = boost::bind(&paramCallback, _1, _2);
    server.setCallback(f);


    //if(!ros::init(argc,argv,"explorationISL"))return;


    int flag=1; // 1: determining direction 2: turning 3:translation
    int MovementFlag=0;

    double position [1000][2]={0,0};
    position_threshold=translation_amount*0.8;

    ros::NodeHandle n;

    ros::Subscriber laserData = n.subscribe<sensor_msgs::LaserScan>("/scan", 10,callback_scan);

    ros::Subscriber encoderData =  n.subscribe("/drrobot_player1/drrobot_motor", 1,motorSensorCallback);

    ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("drrobot_player1/drrobot_cmd_vel",1);

    geometry_msgs::Twist cmdvel_;
    geometry_msgs::Twist stop;

    stop.linear.x = 0.0;
    stop.linear.y = 0.0;
    stop.linear.z = 0.0;
    stop.angular.z = 0.0;

    ros::Rate loop(10);

    while(ros::ok())

    {
        ros::spinOnce();

        if(active)
            velocityPublisher.publish(cmdvel_);
        else{
            if(stop_flag){
                velocityPublisher.publish(stop);
                stop_flag=false;
            }
        }

        //ROS_INFO("ENCODERright: %d",encoderRight);

        //ROS_INFO("ENCODERleft: %d",encoderLeft);




        impactControl=0;

        for (int i=0;i<impactRange;i++) {
            impactControl=impactControl+bubbleFinal[(-impactRange/2)+(bubble_size/2)+i];


        }

        impactControl=impactControl/impactRange;




        if(flag==1) {

            cmdvel_.linear.x = 0;
            cmdvel_.angular.z = 0;

            encoderRightInitial=encoderRight;
            encoderLeftInitial=encoderLeft;








            double depthinfo1=finddirection(bubbleFinal,bubble_size);

            // loop closure

            int l=0;
            int q=0;





            while (l<=nd-1)
            {

                if (position_control==1) {
                    break; }


                turning_amount=direction[q];


                totalOrientation[nd]=totalOrientation[nd-1]+turning_amount;

                position[nd][0]= position[nd-1][0]+translation_amount*cos(totalOrientation[nd]);
                position[nd][1]= position[nd-1][1]+translation_amount*sin(totalOrientation[nd]);

                double positiondifference1=position[nd][0]-position[l][0];
                double positiondifference2=position[nd][1]-position[l][1];

                double normPositionDifference=sqrt(pow(positiondifference1,2)+pow(positiondifference2,2));

                if (nd>1 && position_control==0)


                {

                    if(normPositionDifference<position_threshold) {
                        q=q+1;
                        l=0;



                        if (q>numberOfDirections-1)
                        {
                            position_control=1;
                            move_back_control=1;


                            break;


                        }


                    }
                }

                l=l+1;

            }


            // end of loop closure

            int detected_node;

            int smallest;

            int targetIndex;


            if (move_back_control==1)
            {

                node[nd]=1000;
                move_back_counter[nd]=move_back_counter[nd-1]+1;
            }
            else
            {
                node[nd]=nd;
                move_back_counter[nd]=-1;

            }


            if (move_back_control==1)
            {
                if (move_back_counter[nd]==0)
                {
                    turning_amount=PI;
                    totalOrientation[nd]=totalOrientation[nd-1]+turning_amount;

                    position[nd][0]= position[nd-1][0]+translation_amount*cos(totalOrientation[nd]);
                    position[nd][1]= position[nd-1][1]+translation_amount*sin(totalOrientation[nd]);
                    node[nd]=node[nd-2];
                }

                // int* nodec;
                //nodec=new int[numberOfDirections];

                if (move_back_counter[nd]>0){
                    int movebackTarget=node[nd-1];
                    for (int d1=0;d1<nd;d1++) {
                        if(node[d1]==movebackTarget) {

                            detected_node=d1;

                            // ROS_INFO("det %d",node[detected_node]);

                            break;

                        }

                    }

                    targetIndex=detected_node-1;

                    int nodec[1000];
                    int kx=-1;
                    for (int i=0;i<numberOfDirections;i++)
                    {    turning_amount=direction[i];

                        totalOrientation[nd]=totalOrientation[nd-1]+turning_amount;

                        positionc[i][0]= position[nd-1][0]+translation_amount*cos(totalOrientation[nd]);
                        positionc[i][1]= position[nd-1][1]+translation_amount*sin(totalOrientation[nd]);


                        for (int d=0;d<nd;d++) {

                            double positiondifference1c=positionc[i][0]-position[d][0];
                            double positiondifference2c=position[i][1]-position[d][1];

                            double normPositionDifferencec=sqrt(pow(positiondifference1c,2)+pow(positiondifference2c,2));

                            if (normPositionDifferencec<position_threshold)
                            {  kx=kx+1;
                                nodec[kx]=node[d];
                                ROS_INFO("index %d",targetIndex);
                                ROS_INFO("candidate %d",nodec[kx]);
                                ROS_INFO("real node %d",node[targetIndex]);

                            }


                        }


                    }

                    int smallIndx=0;


                    for (int i=0;i<kx+1;i++)
                    {
                        if(node[targetIndex]==nodec[i])
                        {smallest=nodec[i];
                            smallIndx=i;
                            turning_amount=direction[smallIndx];
                            node[nd]=nodec[i];
                            ROS_INFO("True2");
                            ROS_INFO("real nodeeeeeeee %d",node[nd]);

                            break;
                        }

                    }


                    //delete[] nodec;

                    turning_amount=direction[smallIndx];

                }
            }

            // for fixing bugs
            if (node[nd]==1000) {
                totalOrientation[nd]=totalOrientation[nd-1]+turning_amount;
                position[nd][0]= position[nd-1][0]+ translation_amount*cos(totalOrientation[nd]);
                position[nd][1]= position[nd-1][1]+ translation_amount*sin(totalOrientation[nd]);

                for (int i=0;i<nd;i++) {
                    double positiondifference1cc=positionc[nd][0]-position[i][0];
                    double positiondifference2cc=position[nd][1]-position[i][1];

                    double normPositionDifferencecc=sqrt(pow(positiondifference1cc,2)+pow(positiondifference2cc,2));

                    if (normPositionDifferencecc<position_threshold)
                    { node[nd]=node[i];

                        break;
                    }

                }

            }

            MovementFlag=MovementFlag+1;

            //ROS_INFO("DEPTHHHHH %f",depthinfo);

            //ROS_INFO("Direction %f",direction1);


            //ros::Duration d=ros::Duration(1,0);
            //        d.sleep();
            turningFinished=0;


        }
        int desiredEncoderRot;

        if (MovementFlag>10) {

            flag=2;
            if (flag==2 &&   turningFinished==0) {

                encoderDifference1=(encoderLeft-encoderLeftInitial);  // turn right or left
                encoderDifference2=(encoderLeftInitial-encoderLeft);

                if (encoderDifference1<0)
                {
                    encoderDifference1=encoderDifference1+32768;

                }


                if (encoderDifference2<0)
                {
                    encoderDifference2=encoderDifference2+32768;

                }



                if (turning_amount<0) {

                    if (turning_amount>-0.15) {
                        //desiredEncoderRot=-cvRound(encoderRotation*(turning_amount+0.1)/(PI/2)); // buna bir bak
                        desiredEncoderRot=0;
                    }

                    else {   desiredEncoderRot=-cvRound(encoderRotation*(turning_amount)/(PI/2)); //

                    }



                    if(encoderDifference1<desiredEncoderRot) {





                        cmdvel_.linear.x = 0;
                        cmdvel_.angular.z = -velocity;
                    }

                    else {


                        flag=3;
                        encoderRightInitial2=encoderRight;
                        encoderLeftInitial2=encoderLeft;
                        turningFinished=1;
                        cmdvel_.linear.x = 0;
                        cmdvel_.angular.z = 0;


                    }

                }

                if (turning_amount>=0) {

                    if (turning_amount<0.15) {
                        //desiredEncoderRot=cvRound(encoderRotation*(turning_amount-0.1)/(PI/2));
                        desiredEncoderRot=0;
                    }
                    else {
                        desiredEncoderRot=cvRound(encoderRotation*(turning_amount)/(PI/2));
                    }



                    if(encoderDifference2<desiredEncoderRot) {

                        cmdvel_.linear.x = 0;
                        cmdvel_.angular.z = velocity;



                        // turn
                    }

                    else {


                        flag=3;
                        turningFinished=1;
                        cmdvel_.linear.x = 0;
                        cmdvel_.angular.z = 0;


                        encoderRightInitial2=encoderRight;
                        encoderLeftInitial2=encoderLeft;


                    }


                }



            }





            if (flag==3 || turningFinished==1) {
                //translation



                double impactLimit=0.5;
                int encoderDifference3=encoderRight-encoderRightInitial2;






                int desiredEncoderTrans=encoderTranslation;


                //ROS_INFO("encoderDifference: %d",encoderDifference3);

                if (encoderDifference3<-60)
                {
                    encoderDifference3=encoderDifference3+32768;

                }


                // ROS_INFO("ENCODERDIFF222: %d",desiredEncoderTrans);
                //  ROS_INFO("sensor: %f",impactControl);


                if(encoderDifference3<desiredEncoderTrans && impactControl>impactLimit ) {



                    cmdvel_.linear.x = velocity;
                    cmdvel_.angular.z = 0;
                }

                else
                {
                    cmdvel_.linear.x = 0;
                    cmdvel_.angular.z = 0;

                    MovementFlag=0;
                    turningFinished=0;

                    flag=1;
                    totalOrientation[nd]=totalOrientation[nd-1]+turning_amount;

                    position[nd][0]= position[nd-1][0]+ translation_amount*cos(totalOrientation[nd]);
                    position[nd][1]= position[nd-1][1]+ translation_amount*sin(totalOrientation[nd]);


                    ROS_INFO("POS1: %f",position[nd][0]);
                    ROS_INFO("POS2: %f",position[nd][1]);
                    ROS_INFO("direction: %f",turning_amount);

                    ROS_INFO("nodeee: %d",node[nd]);
                    ROS_INFO("move_back_count: %d",move_back_counter[nd]);
                    ROS_INFO("move_back_control: %d",move_back_control);
                    move_back_control=0;

                    nd++;
                    position_control=0;




                }




            }  }








        // algorithm();
        loop.sleep();



    }







    return 0;
}
