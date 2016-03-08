#include "Utility.h"

double speed = 0.0;
double yaw_rate = 0.0;
double yaw = 0.0;
int arm_pos = 0;
bool turnback_dir = 0;

bool active = false;
bool stop_flag = true;

double *direction;
int numberOfDirections;
double pose_x = 0.0;
double pose_y = 0.0;
double th = 0.0;
double velocity;
double translation_amount;
double cliff_check = 0.0;
double cliff_temp_x = 0.0;
double cliff_temp_y = 0.0;
bool cliff = false;

int move_back_control=0;
int move_back_counter[1000]={-1,-1};
double positionc [1000][2]={0,0};

int node[1000] = {0, 0};
int nd = 1;
int back_trace = 0; //make 1 for non-graph
bool node_finished[1000] = {false};
bool edge_like[1000] = {false};

int position_control=0;

double directionF [1000];
double totalOrientation [1000]={0};
int ind;
int recorded_moment;
double position_threshold;

sensor_msgs::LaserScanConstPtr temp;    //to store laser scan data
int placeID;                //to store place

void imuCallback(const explorationISL::imu::ConstPtr& msg){
    yaw_rate = msg->gyroz;
}

void yawCallback(const std_msgs::Float64::ConstPtr& msg){
    yaw = msg->data;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr & msg){
    pose_x = msg->pose.pose.position.x;
    pose_y = msg->pose.pose.position.y;
}

void paramCallback(explorationISL::ExplorationISLConfig params, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %f %f %s",
              params.max_velocity,params.translation_interval,
              params.active?"True":"False");

    velocity = params.max_velocity;

    translation_amount = params.translation_interval;
    position_threshold = translation_amount * 0.8;

    active = params.active;
    if(!active) stop_flag = true;
}

void placeCallback(std_msgs::Int16 msg){
        placeID = (int)msg.data;
}

int sgn(double x){
    if(x < 0) return -1;
    if(x > 0) return 1;
    return 0;
}

double normAngle(double x){
    x = (x - 2 * PI * std::floor(std::abs(x) / (2 * PI)) * sgn(x));
    if (std::abs(x) < PI)
        return x;
    return (- 2 * sgn(x) * PI + x);
}

inline double prob(){
    return (static_cast<double>(rand()) / RAND_MAX);
}

double nodeDistance(double *x, double *y){
    return sqrt((x[0]-y[0])*(x[0]-y[0]) + (x[1]-y[1])*(x[1]-y[1]));
}

template<class T>
void printarray (T arg[], int length) {
    for (int n=0; n<length; n++)
        cout << arg[n] << " ";
    cout << "\n";
}

void finddirection(double bubble[], int bubbleLength) {
    int panNumber = bubbleLength;

    double *angle_norm;
    angle_norm = new double[panNumber];

    double angle_delta = (PI * 3 / 2) / panNumber;

    for(int j = 0; j < panNumber; j++)
        angle_norm[j]= (-(PI * 3 / 2) / 2) + j * angle_delta;

    StdDeviation sd;

    sd.SetValues(bubble, bubbleLength);

    // bubble processing

    double mean = sd.CalculateMean();
    double stdev = sd.GetStandardDeviation();

    double thr = mean + stdev;

    int *panGroupBit;
    panGroupBit = new int[panNumber];
    int i;
    for (i = 0; i < panNumber; i++){
        if(bubble[i] > thr)
            panGroupBit[i] = 1;
        else
            panGroupBit[i] = 0;
    }

    /// ignore small(0.25-0.5 degrees) barriers(probably noise on laser)
    /*for (i = 1; i < panNumber; i++)
        if(panGroupBit[i - 1] == 1 && panGroupBit[i] == 0 && (panGroupBit[i + 1] == 1 || panGroupBit[i + 2] == 1))
            panGroupBit[i] = 1;
    //printarray(panGroupBit,panNumber);*/

    // connecting components

    Conncomp1d conn;

    conn.SetValues1(panGroupBit,panNumber);

    int numberofSegments = conn.findcompNumber();
    conn.findcompwidth();
    conn.findcentroid();

    //printarray(conn.compwidth, numberofSegments);
    //printarray(conn.centroid, numberofSegments);

    Mat areas_indx;

    cv::Mat areas(1, numberofSegments, CV_64F, conn.compwidth);

    cv::sortIdx(areas, areas_indx, CV_SORT_EVERY_ROW + CV_SORT_DESCENDING);

    double *centroid_sort;
    centroid_sort = new double[numberofSegments];

    double *areas_sort;
    areas_sort = new double[numberofSegments];

    // arrange orinetations related to their widths
    for(i = 0; i < numberofSegments; i++){
        int test;
        test = areas_indx.at<int>(i);

        centroid_sort[i] = conn.centroid[test];
        areas_sort[i] = conn.compwidth[test];
    }

    // detecting small components that robot can not enter

    double angle_robot_width = asin(robot_width / (2 * thr)) * 2 / PI * 180.0;

    int real_segment=0;
    for(i = 0; i < numberofSegments; i++)
        if (areas_sort[i] > angle_robot_width * 4) // *4 because each laser scan is made by 0.25 degree difference
            real_segment = real_segment + 1;

    //    int real_segment=numberofSegments ;

    // generating the utility function == depth*width
    double *real_areas;
    real_areas = new double[real_segment];

    double *real_centroids;
    real_centroids = new double[real_segment];

    int *real_centroids_round;
    real_centroids_round = new int[real_segment];

    double *depth_areas;
    depth_areas = new double[real_segment];

    double *target_function;
    target_function = new double[real_segment];

    int c11 = -1;
    for(i = 0; i < numberofSegments; i++){
        if (areas_sort[i] > angle_robot_width * 4)  // *4 because each laser scan is made by 0.25 degree difference
        {
            c11 = c11 + 1;
            real_areas[c11] = areas_sort[i];
            real_centroids[c11] = centroid_sort[i];

            int kr = cvRound(real_centroids[c11]);
            real_centroids_round[c11] = kr;

            double cr = 0;
            for (int j = kr - 5; j < kr + 5; j++)
            {
                cr = cr + bubble[j];
            }
            depth_areas[c11] = cr / 11;
            target_function[c11] = depth_areas[c11] * real_areas[c11];
        }
    }



    // sorting the utilty values
    cv::Mat target_functionM(1, real_segment, CV_64F, target_function), target_indx;

    cv::sortIdx(target_functionM, target_indx, CV_SORT_EVERY_ROW + CV_SORT_DESCENDING);

    double *target_function_sort;
    target_function_sort = new double[real_segment];

    double *depth_areas_sort;
    depth_areas_sort = new double[real_segment];

    double *centroid_sort2;
    centroid_sort2 = new double[real_segment];

    double *real_orientations;
    real_orientations = new double[real_segment];

    for(i = 0; i < real_segment; i++){
        int test1;
        test1 = target_indx.at<int>(i);

        centroid_sort2[i] = centroid_sort[test1];
        target_function_sort[i] = target_function[test1];
        depth_areas_sort[i] = depth_areas[test1];
        int kr11 = cvRound(centroid_sort2[i]);

        real_orientations[i] = angle_norm[kr11];
    }

    for(i = 0; i < real_segment; i++)
        if (depth_areas_sort[i] < distanceThr)
            real_orientations[i] = PI;


    direction = new double[real_segment];
    direction = real_orientations;

    numberOfDirections = real_segment;

    return;
    delete[] angle_norm;
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

int bubble_size = -1;

int test=0;

void motorSensorCallback(const drrobot_jaguarV2_player::MotorInfoArray::ConstPtr& msg)
{
    int msgSize = msg->motorInfos.capacity();

    if (msgSize == 6)
    {
        encoderRight = msg->motorInfos[3].encoder_pos;
        encoderLeft = msg->motorInfos[4].encoder_pos;
        arm_pos = msg->motorInfos[0].encoder_pos;
        speed= msg -> motorInfos[3].encoder_vel * (msg-> motorInfos[3].encoder_dir -0.5) -
               msg -> motorInfos[4].encoder_vel * (msg-> motorInfos[4].encoder_dir -0.5);
        speed = speed / encoder_coeff;
    }
}
// mustafa
float bubblex[1080]; // scan size is 1080

double bubbleFinal[1080];

void callback_scan(const sensor_msgs::LaserScanConstPtr& scan)
{
    // scan returns a vector which consist of a distance value for each
	//laser scan. For example jaguar robot's laser returns 270 degree
	//* 4 value for each degree.
    float *bubble_scan = (float*)scan->ranges.data();

    bubble_size = scan->ranges.size();

    temp = scan;

    for (int i = 0; i < bubble_size; i++)
    {
        bubblex[i] = *bubble_scan;

        bubble_scan++;
    }

    // clean bubble from nan and inf
    for (int i = 0; i < bubble_size; i++)
    {
        double yedek = (double)bubblex[i];

        if (isnan(yedek) == 1)
            bubblex[i]=0.1;

        if (isinf(yedek) == 1)
        {
            if(i > 0)
                bubblex[i] = bubblex[i-1];
            else
                bubblex[i] = 5;
        }

        bubbleFinal[i]=(double)bubblex[i];
    }
}

visualization_msgs::MarkerArray arrows;

ostream& operator<<(ostream& out, Graph& gr){
    vector<map<int,double> > graph1 = gr.getGraph();

    for(int i=0;i<graph1.size();i++){
        if(graph1[i].size()!=0)
            out << i << " -> ";
        else
            out << i << endl;
        for(map<int,double>::iterator itr = graph1[i].begin(); itr!=graph1[i].end(); itr++){
            if(distance(itr, graph1[i].end()) != 1)
                out << itr->first << "<" << itr->second << ">" << " - ";
            else
                out << itr->first << "<" << itr->second << ">" << endl;
        }
    }

    return out;
}

int main(int argc, char** argv) {

    int encoderLeftInitial;
    int encoderRightInitial2;

    int encoderDifference1;
    int encoderDifference2;

    int encoderRotation = 290; // encoder value for 90 degree turning
    int encoderTranslation = 640; // encoder value for 1m translation

    double turning_amount;
    double x_init = 0.0;
    double y_init = 0.0;
    double yaw_init = 0.0;
    double target_x = 0.0;
    double target_y = 0.0;
    double distance_to_target = 0.0;
    double total_translation = 0.0;
    double target_distance = 0.0;

    int turningFinished = 0;

    double impactControl;
    int impactRange = 100;

    ros::init(argc, argv, "explorationISL");

    dynamic_reconfigure::Server<explorationISL::ExplorationISLConfig> server;
    dynamic_reconfigure::Server<explorationISL::ExplorationISLConfig>::CallbackType f;

    f = boost::bind(&paramCallback, _1, _2);
    server.setCallback(f);


    enum State flag = DeterminingDirection;
    int MovementFlag = 0;

    double position [1000][2] = {0,0};
    position_threshold = translation_amount * 0.8;

    ros::NodeHandle n;


    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Subscriber imu = n.subscribe("/imu",1,imuCallback);

    ros::Subscriber odom = n.subscribe("/odom",1,odomCallback);

    ros::Subscriber laserData = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, callback_scan);

    ros::Subscriber encoderData =  n.subscribe("/drrobot_player1/drrobot_motor", 1, motorSensorCallback);

    ros::Subscriber yaw_pub = n.subscribe("/yaw", 1, yawCallback);

    ros::Subscriber place = n.subscribe("/createBDSTISL/PlaceID",1, placeCallback);

    ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("drrobot_player1/drrobot_cmd_vel",1);

    ros::Publisher pointPublisher = n.advertise<visualization_msgs::Marker>("points", 10);

    ros::Publisher arrowPublisher = n.advertise<visualization_msgs::MarkerArray>("arrows", 10);

    geometry_msgs::Twist cmdvel_;
    geometry_msgs::Twist stop;


    //Initializations

    Graph g(1);
    int critical_count = 0;

    sensor_msgs::LaserScan init_scan;

    if (temp == 0){
        g.scanList[0] = init_scan;
        ROS_INFO("enter!!");
    }
    else{
        g.scanList[0] = *temp;
    }


    stop.linear.x = 0.0;
    stop.linear.y = 0.0;
    stop.linear.z = 0.0;
    stop.angular.z = 0.0;

    ros::Rate loop(30); //default 10

    while(ros::ok()){
        ros::spinOnce();

        if(active)
            velocityPublisher.publish(cmdvel_);
        else{
            if(stop_flag){
                velocityPublisher.publish(stop);
                stop_flag=false;
            }
            loop.sleep();
            continue;
        }

        current_time = ros::Time::now();
        /*
        if(gyro_active){
            double dt = (current_time - last_time).toSec();
            yaw += (yaw_rate - gyro_drift_velocity) * dt / gyro_coeff;
        }*/

        if(tracker_active){
            total_translation += speed * (current_time - last_time).toSec();
        }

        last_time = current_time;

		if (bubble_size == -1) { // Laser scan hasn't been made yet
			loop.sleep();
			continue;
		}

        if(flag == DeterminingDirection) {
            cmdvel_.linear.x = 0.0;
            cmdvel_.linear.y = 0.0;
            cmdvel_.angular.z = 0.0;

            if(!cliff){
                finddirection(bubbleFinal, bubble_size);

                // loop closure
                int l = 0;
                int q = 0;

                for (int i = 0; i < numberOfDirections; i++)
                    if(print_data){
                        ROS_INFO("direction[%d]: %lf", i, direction[i]);
                    }

                //arrows for tracking
                for(int i = 0; i < numberOfDirections; i++){
                    visualization_msgs::Marker arrow;
                    arrow.id = nd + i * 10000;
                    arrow.header.frame_id = "/odom";
                    arrow.header.stamp = ros::Time::now();
                    arrow.ns = "arrows";
                    arrow.action = visualization_msgs::Marker::ADD;
                    arrow.type = visualization_msgs::Marker::ARROW;
                    arrow.scale.x = 0.02;
                    arrow.scale.y = 0.05;
                    arrow.color.r = 1.0f;
                    arrow.color.a = 1.0;
                    geometry_msgs::Point p1;
                    geometry_msgs::Point p2;
                    if (tracker_active){
                        p1.x = pose_x;
                        p1.y = pose_y;
                        double orientation;
                        if (gyro_active)
                            orientation = yaw + direction[i];
                        else
                            orientation = totalOrientation[nd - 1] + direction[i];

                        p2.x = pose_x + translation_amount * cos(orientation);
                        p2.y = pose_y + translation_amount * sin(orientation);
                    }
                    else{
                        p1.x = position[nd - 1][0];
                        p1.y = position[nd - 1][1];
                        double orientation;
                        if (gyro_active)
                            orientation = yaw + direction[i];
                        else
                            orientation = totalOrientation[nd - 1] + direction[i];
                        p2.x = position[nd - 1][0] + translation_amount * cos(orientation);
                        p2.y = position[nd - 1][1] + translation_amount * sin(orientation);
                    }
                    arrow.points.push_back(p1);
                    arrow.points.push_back(p2);

                    arrows.markers.push_back(arrow);
                }
                arrowPublisher.publish(arrows);


                while (l <= nd - 1)
                {
                    if (position_control == 1)
                        break;

                    if (numberOfDirections == 0){
                        move_back_control = 1;
                        position_control = 1;

                        break;
                    }

                    //finish the current place
                    if(use_graph){
                        if(g.current_place != placeID){
                            move_back_control = 1;
                            position_control = 1;

                            //transition nodes are critical
                            g.isCritical[nd-1] = true;
                            int prev_back_trace = g.findBackTrace(nd-1,g.current_place);
                            g.isCritical[prev_back_trace] = true;


                            break;
                        }
                    }

                    turning_amount = direction[q];

                    //turning greater than a threshold creates critical node
                    if(turning_amount > critical_turn_threshold){
                        g.isCritical[nd-1] = true;
                    }

                    double normPositionDifference;

                    if(tracker_active){
                        if (gyro_active)
                            totalOrientation[nd] = yaw + turning_amount;
                        else
                            totalOrientation[nd] = totalOrientation[nd - 1] + turning_amount;
                        normPositionDifference = 0;


                        position[nd][0] = pose_x + translation_amount * cos(totalOrientation[nd]);
                        position[nd][1] = pose_y + translation_amount * sin(totalOrientation[nd]);
                    }
                    else{
                        if (gyro_active)
                            totalOrientation[nd] = yaw + turning_amount;
                        else
                            totalOrientation[nd] = totalOrientation[nd - 1] + turning_amount;
                        normPositionDifference = 0;

                        position[nd][0] = position[nd - 1][0] + translation_amount * cos(totalOrientation[nd]);
                        position[nd][1] = position[nd - 1][1] + translation_amount * sin(totalOrientation[nd]);
                    }

                    double positiondifference1 = position[nd][0] - position[l][0];
                    double positiondifference2 = position[nd][1] - position[l][1];

                    normPositionDifference = sqrt(pow(positiondifference1, 2) + pow(positiondifference2, 2));

                    l = l + 1;

                    if (position_control == 0) ///????????????????????????????????is it nd > 0?????????????
                    {
                        if (normPositionDifference < position_threshold) {
                            q = q + 1;
                            g.addEdge((l-1),nd,nodeDistance(&position[l-1][0],&position[nd][0]));
                            l = 0;

                            if (q > numberOfDirections - 1)
                            {
                                position_control = 1;
                                move_back_control = 1;

                                break;
                            }
                        }
                    }
                }

                if (move_back_control != 1){
                    node_finished[back_trace] = false;
                    g.isFinished[back_trace] = false;
                }

                if (numberOfDirections == 1 && move_back_control != 1) {
                    edge_like[nd - 1] = true;
                    g.isCritical[nd - 1] = false;
                }

                //nodes having more than one possible direction is critical
                if (numberOfDirections > 1 && move_back_control != 1) {
                    g.isCritical[nd - 1] = true;
                }

                if(print_data){
                    ROS_INFO("turning_amount: %lf", turning_amount);
                    ROS_INFO("position_control: %d", position_control);
                    ROS_INFO("move_back_control: %d", move_back_control);
                    ROS_INFO("end of loop closure");
                    // end of loop closure
                }
            }
            else{
                move_back_control = 1;
                position_control = 1;
            }

            int detected_node; // for move_back operation

            //int smallest;

            int targetIndex;

            if (tracker_active){
                if (move_back_control == 1){
                    //check for possible directions out of view

                    if(!node_finished[back_trace]){
                        node_finished[back_trace] = true;
                        g.isFinished[back_trace] = true;
                        if(!edge_like[back_trace]){
                            target_distance = 0.0;
                            turning_amount = PI * 2 * (turnback_dir - 0.5);
                        }
                        else{
                            target_distance = 0.0;
                            turning_amount = 0.0;
                        }
                        MovementFlag = 0;
                        flag = MovingBack;
                        ROS_INFO("first turn: %lf", turning_amount);
                    }

                    else{
                         //find the previous node
                        if(use_graph){
                            back_trace = g.findBackTrace(back_trace,g.placeIDList[back_trace]);
                        }
                        else{
                            while(node_finished[back_trace]){
                                back_trace--;

                                //all the nodes are finished
                                if(back_trace < 0){
                                    ROS_INFO("Exploration Finished!");
                                    active = false;
                                    char ch;
                                    std::cin>>ch;
                                    return 0;
                                }
                            }
                        }

                        //all the nodes are finished
                        if(back_trace < 0){
                            ROS_INFO("Exploration Finished!");
                            active = false;
                            char ch;
                            std::cin>>ch;
                            return 0;
                        }

                        target_distance = sqrt(( - pose_x + position[back_trace][0]) * ( - pose_x + position[back_trace][0]) +
                                               ( - pose_y + position[back_trace][1]) * ( - pose_y + position[back_trace][1]));
                        double target_yaw = atan2(( position[back_trace][1] - pose_y), ( position[back_trace][0] - pose_x));

                        turning_amount = normAngle(target_yaw - normAngle(yaw));
                        MovementFlag = 0;
                        flag = MovingBack;
                        if(print_data){
                            ROS_INFO("move back turn: %lf %lf %lf %lf %lf ", turning_amount, yaw, normAngle(yaw), target_yaw, normAngle(target_yaw));
                            ROS_INFO("backtrace: %d %lf %lf %lf %lf ", back_trace, pose_x, position[back_trace][0], pose_y, position[back_trace][1]);
                        }
                    }
                }
            }

            else{
                if (move_back_control == 1)
                {
                    node[nd] = 1000;
                    move_back_counter[nd] = move_back_counter[nd - 1] + 1;
                }
                else
                {
                    node[nd] = nd;
                    move_back_counter[nd] = -1;
                }


                if (move_back_control == 1) // return to the previous node
                {
                    if (move_back_counter[nd] == 0)
                    {
                        turning_amount = PI;
                        totalOrientation[nd] = totalOrientation[nd - 1] + turning_amount;

                        position[nd][0] = position[nd - 1][0] + translation_amount * cos(totalOrientation[nd]);
                        position[nd][1] = position[nd - 1][1] + translation_amount * sin(totalOrientation[nd]);
                        node[nd] = node[nd - 2];
                    }


                    if (move_back_counter[nd] > 0){
                        int movebackTarget = node[nd - 1];
                        for (int d1 = 0; d1 < nd; d1++) {
                            if(node[d1] == movebackTarget) {
                                detected_node=d1;

                                // ROS_INFO("det %d",node[detected_node]);

                                break;
                            }
                        }

                        targetIndex = detected_node - 1;

                        int nodec[1000];
                        int kx = -1;
                        for (int i = 0; i < numberOfDirections; i++)
                        {
                            turning_amount = direction[i];


                            totalOrientation[nd] = totalOrientation[nd - 1] + turning_amount;

                            positionc[i][0] = position[nd - 1][0] + translation_amount * cos(totalOrientation[nd]);
                            positionc[i][1] = position[nd - 1][1] + translation_amount * sin(totalOrientation[nd]);


                            for (int d = 0; d < nd; d++) {

                                double positiondifference1c = positionc[i][0] - position[d][0];
                                double positiondifference2c = positionc[i][1] - position[d][1];

                                double normPositionDifferencec = sqrt(pow(positiondifference1c, 2) + pow(positiondifference2c, 2));

                                if (normPositionDifferencec < position_threshold)
                                {
                                    kx = kx + 1;
                                    nodec[kx] = node[d];
                                    ROS_INFO("index %d", targetIndex);
                                    ROS_INFO("candidate %d", nodec[kx]);
                                    ROS_INFO("real node %d", node[targetIndex]);
                                }
                            }
                        }

                        int smallIndx=0;

                        for (int i = 0; i <= kx; i++)
                        {
                            if(node[targetIndex] == nodec[i])
                            {
                                //smallest = nodec[i];
                                smallIndx = i;
                                turning_amount = direction[smallIndx];
                                node[nd] = nodec[i];

                                ROS_INFO("True2");
                                ROS_INFO("real nodeeeeeeee %d", node[nd]);

                                break;
                            }
                        }
                        //delete[] nodec;

                        turning_amount = direction[smallIndx];
                    }
                }
            }


            // for fixing bugs
            if (node[nd] == 1000 && !tracker_active) {
                totalOrientation[nd] = totalOrientation[nd - 1] + turning_amount;
                position[nd][0] = position[nd - 1][0] + translation_amount * cos(totalOrientation[nd]);
                position[nd][1] = position[nd - 1][1] + translation_amount * sin(totalOrientation[nd]);

                for (int i = 0; i < nd; i++) {
                    double positiondifference1cc = positionc[nd][0] - position[i][0];
                    double positiondifference2cc = positionc[nd][1] - position[i][1];

                    double normPositionDifferencecc = sqrt(pow(positiondifference1cc, 2) + pow(positiondifference2cc, 2));

                    if (normPositionDifferencecc < position_threshold)
                    { 
						node[nd] = node[i];
                        break;
                    }
                }
            }

            MovementFlag = MovementFlag + 1;
            if(print_data){
                ROS_INFO("MovementFlag: %d", MovementFlag);
            }
            
            //Initializations

            encoderLeftInitial = encoderLeft;
            x_init = pose_x;
            y_init = pose_y;
            yaw_init = yaw;
            target_x = x_init + translation_amount * cos(yaw_init);
            target_y = y_init + translation_amount * sin(yaw_init);
            total_translation = 0;
            turningFinished = 0;
        }
        
        int desiredEncoderRot;

        if (MovementFlag > 10) { ///?????????????????????????????????????????????why wait for 10???????????
            flag = Turning;

            if(tracker_active){
                if (flag == Turning && turningFinished == 0){
                    if(print_data){
                            ROS_INFO("arm_pos %d ", arm_pos);
                            ROS_INFO("yaw %lf", yaw);
                            ROS_INFO("turning_amount %lf", turning_amount);
                            ROS_INFO("angular.z %lf", velocity * (turning_amount > 0 ? 1 : -1));
                    }
                    if( abs(yaw - yaw_init) < abs(turning_amount)){
                        cmdvel_.linear.x = 0.0;
                        if ((arm_pos > 32720) || (arm_pos < 1000)){
                            cmdvel_.linear.y = arm_vel;
                            cmdvel_.angular.z = 0.0;
                        }
                        else{
                            cmdvel_.linear.y = 0.0;
                            cmdvel_.angular.z = velocity * turn_coeff * (turning_amount > 0 ? 1 : -1);
                        }
                    }
                    else{
                        if(print_data){
                            ROS_INFO("DONE!!! %lf", abs(yaw) - abs(turning_amount));
                        }
                        totalOrientation[nd] += yaw - yaw_init - turning_amount;
                        flag = Translation;
                        //yaw = 0.0;
                        turningFinished = 1 ;
                        cmdvel_.linear.x = 0;
                        cmdvel_.linear.y = 0.0;
                        cmdvel_.angular.z = 0;
                        x_init = pose_x;
                        y_init = pose_y;
                        total_translation = 0;

                    }
                }

                if (flag == Translation || turningFinished == 1) {
                    // translation
                    double impactLimit = 0.9;


                    impactControl = 0;
                    int order = 8;
                    bool impact = false;

                    int impact_count = 0;
                    for (int i = 0; i < impactRange; i++){
                        impactControl = bubbleFinal[(-impactRange / 2) + (bubble_size / 2) + i];
                        if(impactControl > impactLimit){
                            impact_count = 0;
                            continue;
                        }
                        double angle = ((-impactRange / 2) + i) / 4.0 * PI / 180.0;
                        double width = 2 * sin(angle) * impactControl;
                        if(width < robot_width) impact_count++;
                        else impact_count = 0;

                        if(impact_count >= order){
                            impact = true;
                            break;
                        }
                    }
                    if(print_data){
                        ROS_INFO("impactControl: %f", impactControl);
                    }

                    if (total_translation < translation_amount && !impact && !cliff) {
                        cmdvel_.linear.x = velocity;
                        if((arm_pos > 10000) || (arm_pos < 5)){
                            cmdvel_.linear.y = -arm_vel;
                        }
                        else{
                            cmdvel_.linear.y = 0.0;
                        }
                        cmdvel_.angular.z = 0;

                        distance_to_target = sqrt((target_x - pose_x) * (target_x - pose_x) +
                                                  (target_y - pose_y) * (target_y - pose_y));
/*
                        double estimated_x = pose_x + distance_to_target * cos(yaw);
                        double estimated_y = pose_y + distance_to_target * sin(yaw);

                        cmdvel_.angular.z = velocity * k_p * sgn((target_y - y_init) * (estimated_x - x_init) -
                                                              (estimated_y - y_init) * (target_x - x_init));*/
                        if(print_data){
                            ROS_INFO("%lf %lf %lf", total_translation, distance_to_target, speed);
                        }
                    }

                    else
                    {
                        cmdvel_.linear.x = 0.0;
                        cmdvel_.linear.y = 0.0;
                        cmdvel_.angular.z = 0.0;
                        //double k = 1.0 * total_translation / translation_amount;

                        MovementFlag = 0;
                        turningFinished = 0;


                        flag = DeterminingDirection;
                        //totalOrientation[nd] = totalOrientation[nd - 1] + turning_amount;

                        position[nd][0] = pose_x;
                        position[nd][1] = pose_y;



                        visualization_msgs::Marker points;
                        points.id = 0;
                        points.header.frame_id = "/odom";
                        points.header.stamp = ros::Time::now();
                        points.ns = "points";
                        points.action = visualization_msgs::Marker::ADD;
                        points.type = visualization_msgs::Marker::POINTS;
                        points.scale.x = points.scale.y = 0.05;
                        points.color.g = 1.0f;
                        points.color.a = 1.0;
                        geometry_msgs::Point p;
                        for(int i=0;i<=nd;i++){
                            p.x = position[i][0];
                            p.y = position[i][1];
                            points.points.push_back(p);
                        }
                        pointPublisher.publish(points);
                        if(print_data){
                            ROS_INFO("POS1: %f", position[nd][0]);
                            ROS_INFO("POS2: %f", position[nd][1]);
                            ROS_INFO("direction: %f", turning_amount);

                            ROS_INFO("nodeee: %d", node[nd]);
                            ROS_INFO("move_back_count: %d", move_back_counter[nd]);
                            ROS_INFO("move_back_control: %d", move_back_control);
                        }
                        move_back_control = 0;

                        if(cliff){
                            node_finished[nd] = true;
                            g.isFinished[nd] = true;
                            edge_like[nd] = true;
                            g.isCritical[nd] = false;
                            cmdvel_.linear.x = -velocity;
                        }

                        g.addVertex();
                        g.addEdge(nd,back_trace,nodeDistance(&position[nd][0],&position[back_trace][0]));

                        g.scanList.push_back(*temp);
                        g.isFinished.push_back(false);
                        g.isCritical.push_back(true);
                        g.placeIDList.push_back(placeID);
                        g.poseList.push_back(Position(pose_x,pose_y, yaw));

                        //at least 1 critical node for range greater than a threshold
                        critical_count++;
                        if(critical_count > critical_range_threshold){
                            g.isCritical[nd] = true;
                            critical_count = 0;
                        }

                        back_trace = nd;
                        nd++;

                        position_control = 0;
                    }
                }

            }
            else{
                if (flag == Turning && turningFinished == 0) {

                    if(gyro_active){
                        if(print_data){
                            ROS_INFO("yaw %lf", yaw);
                            ROS_INFO("turning_amount %lf", turning_amount);
                            ROS_INFO("angular.z %lf", velocity * (turning_amount > 0 ? 1 : -1));
                        }
                        if( abs(yaw - yaw_init) < abs(turning_amount)){
                            cmdvel_.linear.x = 0.0;
                            cmdvel_.angular.z = velocity * turn_coeff * (turning_amount > 0 ? 1 : -1);
                        }
                        else{
                            if(print_data){
                                ROS_INFO("DONE!!! %lf", abs(yaw) - abs(turning_amount));
                            }
                            totalOrientation[nd] += yaw - turning_amount;
                            flag = Translation;
                            yaw = 0.0;
                            turningFinished = 1 ;
                            cmdvel_.linear.x = 0;
                            cmdvel_.angular.z = 0;
                            encoderRightInitial2 = encoderRight;
                        }
                    }

                    else{
                        encoderDifference1 = (encoderLeft - encoderLeftInitial);  // turn right or left
                        encoderDifference2 = (encoderLeftInitial - encoderLeft);

                        ROS_INFO("encoderDifference1: %d", encoderDifference1);
                        ROS_INFO("encoderDifference2: %d", encoderDifference2);

                        if (encoderDifference1 < 0)
                            encoderDifference1 = encoderDifference1 + 32768;


                        if (encoderDifference2 < 0)
                            encoderDifference2 = encoderDifference2 + 32768;

                        ROS_INFO("encoderDifference1: %d", encoderDifference1);
                        ROS_INFO("encoderDifference2: %d", encoderDifference2);

                        if (turning_amount < 0) {
                            if (turning_amount > -0.15) {
                                //desiredEncoderRot=-cvRound(encoderRotation*(turning_amount+0.1)/(PI/2)); // buna bir bak
                                desiredEncoderRot=0;
                            }

                            else
                                desiredEncoderRot = -cvRound(encoderRotation * (turning_amount) / (PI / 2)); //

                            ROS_INFO("desiredEncoderRot: %d", desiredEncoderRot);
                            ROS_INFO("encoderDifference1: %d", encoderDifference1);

                            if(encoderDifference1 < desiredEncoderRot) {
                                cmdvel_.linear.x = 0;
                                cmdvel_.angular.z = -velocity;
                            }

                            else {
                                flag = Translation;
                                encoderRightInitial2 = encoderRight;
                                turningFinished = 1;
                                cmdvel_.linear.x = 0;
                                cmdvel_.angular.z = 0;
                            }
                        }

                        if (turning_amount >= 0) {

                            if (turning_amount < 0.15) {
                                //desiredEncoderRot=cvRound(encoderRotation*(turning_amount-0.1)/(PI/2));
                                desiredEncoderRot = 0;
                            }
                            else
                                desiredEncoderRot = cvRound(encoderRotation * (turning_amount) / (PI / 2));

                            ROS_INFO("desiredEncoderRot: %d", desiredEncoderRot);
                            ROS_INFO("encoderDifference2: %d", encoderDifference2);

                            if(encoderDifference2 < desiredEncoderRot) {
                                cmdvel_.linear.x = 0;
                                cmdvel_.angular.z = velocity;
                            }

                            else {
                                flag = Translation;
                                encoderRightInitial2 = encoderRight;
                                turningFinished = 1;
                                cmdvel_.linear.x = 0;
                                cmdvel_.angular.z = 0;
                            }
                        }
                    }
                }
                if (flag == Translation || turningFinished == 1) {
                    // translation
                    double impactLimit = 0.9;
                    int encoderDifference3 = encoderRight - encoderRightInitial2;

                    int desiredEncoderTrans = encoderTranslation * translation_amount;


                    ROS_INFO("encoderDifference3: %d",encoderDifference3);

                    if (encoderDifference3 < -60) ///?????????????????????????????????????????why -60??????????
                        encoderDifference3 = encoderDifference3 + 32768; ///??????????????????why 32768????????

                    ROS_INFO("desiredEncoderTrans: %d", desiredEncoderTrans);
                    ROS_INFO("encoderDifference3: %d", encoderDifference3);

                    impactControl = 0;
                    int order = 8;
                    bool impact = false;
                    /*for (int i = 0; i < order; i++)
                        impactControl += bubbleFinal[(-impactRange / 2) + (bubble_size / 2) + i] / order;
                    if(impactControl > impactLimit){
                        for (int i = order; i < impactRange; i++){
                            impactControl += bubbleFinal[(-impactRange / 2) + (bubble_size / 2) + i] / order;
                            impactControl -= bubbleFinal[(-impactRange / 2) + (bubble_size / 2) + i - order] / order;

                            if(impactControl <= impactLimit){
                                impact = true;
                                break;
                            }
                        }
                    }
                    else impact = true;*/
                    int impact_count = 0;
                    for (int i = 0; i < impactRange; i++){
                        impactControl = bubbleFinal[(-impactRange / 2) + (bubble_size / 2) + i];
                        if(impactControl > impactLimit){
                            impact_count = 0;
                            continue;
                        }
                        double angle = ((-impactRange / 2) + i) / 4.0 * PI / 180.0;
                        double width = 2 * sin(angle) * impactControl;
                        if(width < robot_width) impact_count++;
                        else impact_count = 0;

                        if(impact_count >= order){
                            impact = true;
                            break;
                        }
                    }

                    ROS_INFO("impactControl: %f", impactControl);

                    if (encoderDifference3 < desiredEncoderTrans && !impact ) {
                        cmdvel_.linear.x = velocity;
                        cmdvel_.angular.z = 0;
                    }

                    else
                    {
                        cmdvel_.linear.x = 0;
                        cmdvel_.angular.z = 0;
                        double k = 1.0 * encoderDifference3 / desiredEncoderTrans;

                        MovementFlag = 0;
                        turningFinished = 0;


                        flag = DeterminingDirection;
                        totalOrientation[nd] = totalOrientation[nd - 1] + turning_amount;

                        position[nd][0] = position[nd - 1][0] + k * translation_amount * cos(totalOrientation[nd]);
                        position[nd][1] = position[nd - 1][1] + k * translation_amount * sin(totalOrientation[nd]);

                        visualization_msgs::Marker points;
                        points.id = 0;
                        points.header.frame_id = "/base_link";
                        points.header.stamp = ros::Time::now();
                        points.ns = "points";
                        points.action = visualization_msgs::Marker::ADD;
                        points.type = visualization_msgs::Marker::POINTS;
                        points.scale.x = points.scale.y = 0.05;
                        points.color.g = 1.0f;
                        points.color.a = 1.0;
                        geometry_msgs::Point p;
                        for(int i=0;i<=nd;i++){
                            p.x = position[i][0];
                            p.y = position[i][1];
                            points.points.push_back(p);
                        }
                        pointPublisher.publish(points);

                        ROS_INFO("POS1: %f", position[nd][0]);
                        ROS_INFO("POS2: %f", position[nd][1]);
                        ROS_INFO("direction: %f", turning_amount);

                        ROS_INFO("nodeee: %d", node[nd]);
                        ROS_INFO("move_back_count: %d", move_back_counter[nd]);
                        ROS_INFO("move_back_control: %d", move_back_control);
                        move_back_control = 0;

                        g.addVertex();
                        g.addEdge(nd,back_trace,nodeDistance(&position[nd][0],&position[back_trace][0]));

                        g.scanList.push_back(*temp);
                        g.isFinished.push_back(false);
                        g.isCritical.push_back(true);
                        g.poseList.push_back(Position(pose_x,pose_y, yaw));

                        back_trace = nd;
                        nd++;

                        position_control = 0;
                    }
                }
            }
        }

        if(flag == MovingBack && !cliff){
            double impactLimit = 0.9;
            //backtracing
            if(move_back_control == 1){

                //impact control (will be merged in a safety node)

                impactControl = 0;
                int order = 8;
                bool impact = false;

                int impact_count = 0;
                for (int i = 0; i < impactRange; i++){
                    impactControl = bubbleFinal[(-impactRange / 2) + (bubble_size / 2) + i];
                    if(impactControl > impactLimit){
                        impact_count = 0;
                        continue;
                    }
                    double angle = ((-impactRange / 2) + i) / 4.0 * PI / 180.0;
                    double width = 2 * sin(angle) * impactControl;
                    if(width < robot_width) impact_count++;
                    else impact_count = 0;

                    if(impact_count >= order){
                        impact = true;
                        break;
                    }
                }

                //go to previous unfinished node

                if (abs(yaw_init - yaw) < abs(turning_amount)){

                    cmdvel_.linear.x = 0.0;
                    if ((arm_pos > 32720) || (arm_pos < 1000)){
                        cmdvel_.linear.y = arm_vel;
                        cmdvel_.angular.z = 0.0;
                    }
                    else{
                        cmdvel_.linear.y = 0.0;
                        cmdvel_.angular.z = velocity * turn_coeff * sgn(turning_amount);
                    }
                    if(print_data){
                        ROS_INFO("arm_pos %d ", arm_pos);
                        ROS_INFO("yaw %lf yaw_init %lf", yaw, yaw_init);
                        ROS_INFO("turning_amount %lf", turning_amount);
                        ROS_INFO("angular.z %lf", velocity * (turning_amount > 0 ? 1 : -1));
                    }
                }
                else{//change
                    cmdvel_.angular.z = 0.0;
                    cmdvel_.linear.y = 0.0;
                    double dist_to_start = sqrt(( pose_x - x_init) * ( pose_x - x_init) +
                                                ( pose_y - y_init) * ( pose_y - y_init));

                    if (dist_to_start < target_distance){
                        if(!impact && !cliff){
                            if((arm_pos > 10000) || (arm_pos < 5)){
                                cmdvel_.linear.y = -arm_vel;
                            }
                            else{
                                cmdvel_.linear.y = 0.0;
                            }

                            cmdvel_.linear.x = velocity;                            
                            cmdvel_.angular.z = 0.0;
                        }
                        else{
                            cmdvel_.linear.x = 0.0;
                            cmdvel_.linear.y = 0.0;
                            cmdvel_.angular.z = 0.0;
                        }
                    }
                    else{
                        cmdvel_.linear.x = 0.0;
                        cmdvel_.linear.y = 0.0;
                        flag = DeterminingDirection;
                        move_back_control = 0;
                        position_control = 0;
                    }
                }
            }

        }

        //cliff checking will be merged with impact control etc.


        cliff_check = sqrt((pose_x - cliff_temp_x) * (pose_x - cliff_temp_x) +
                            (pose_y - cliff_temp_y) * (pose_y - cliff_temp_y));

        //Partial Check
        /*
        if(cliff_check > 0.2){
            if(cmdvel_.linear.y < 0.01 && cmdvel_.angular.z == 0.0){
                cmdvel_.linear.y = -arm_vel;
            }
            if((cliff_check > 0.3) ){
                cliff_check = 0.0;
                cmdvel_.linear.y = 0.0;
                cliff_temp_x = pose_x;
                cliff_temp_y = pose_y;
            }
        }
        else {
            //cmdvel_.linear.y = 0.0;
        }*/

        if(abs(cmdvel_.linear.x) > 0.01 && !cliff){
            cmdvel_.linear.y = -arm_vel;
        }


        if((arm_pos > cliff_treshold) && (arm_pos <10000)){
            cliff = true;
            cliff_check = 0.0;
            cmdvel_.linear.x = -velocity;
            cmdvel_.linear.y = 0.0;


            //cliff during backtrace will be implemented later
            if (flag == MovingBack){
            }
            else if (flag == Translation){
            }
            else if (flag == Turning){
                cliff_check = 0.0;
            }
        }
        else{
            cliff = false;
        }
        //ROS_INFO("angular.z %lf", cmdvel_.angular.z);
        loop.sleep();
    }
    return 0;
}
