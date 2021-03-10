
#include <sstream>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "elisa3-lib.h"
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <sensor_msgs/LaserScan.h>

#include <string>

#define DEBUG_ROS_PARAMS 1
#define DEBUG_UPDATE_SENSORS_DATA 0
#define DEBUG_ODOMETRY 0
#define DEBUG_ACCELEROMETER 0
#define DEBUG_SPEED_RECEIVED 0
#define DEBUG_RANGE_SENSORS 0

#define SENSORS_NUM 4
#define ACCELEROMETER 0
#define FLOOR 1
#define PROXIMITY 2
#define MOTOR_POSITION 3

#define ACTUATORS_NUM 4
#define MOTORS 0
#define GREEN_LEDS 1
#define RGB_LED 2
#define IR_TX 3

#define WHEEL_DISTANCE 0.041		// Distance between wheels in meters (axis length).
#define ROBOT_RADIUS 0.025			// meters.

// CHANGED
int robotAddress[2];
//

bool enabledSensors[SENSORS_NUM];
bool changedActuators_0[ACTUATORS_NUM];
bool changedActuators_1[ACTUATORS_NUM];
// CHANGED
// int speedLeft_0 = 0, speedRight_0 = 0, speedLeft_1 = 0, speedRight_1 = 0;
unsigned char ledNum = 0, ledState = 0;
std::string elisa3Name;
std::string baseTag;
std::string nodeName;
//struct timeval currentTime2, lastTime2;
//struct timeval currentTime3, lastTime3;

//signed int accData[3];
//unsigned int floorData[4];
//unsigned int proxData[8];

//signed int robTheta=0, robXPos=0, robYPos=0;

ros::Publisher proxPublisher[8];
sensor_msgs::Range proxMsg[8];
ros::Publisher laserPublisher;
sensor_msgs::LaserScan laserMsg;

//ros::Publisher odomPublisher;
//nav_msgs::Odometry odomMsg;

std::map<int, ros::Publisher> odomPublishers;
//std::map<std::string, ros::Publisher> odomPublishers;

ros::Publisher accelPublisher;
sensor_msgs::Imu accelMsg;
ros::Publisher floorPublisher;
visualization_msgs::Marker floorMsg;

std::map<int, ros::Subscriber> VelSubscribers;
//std::map<std::string, ros::Subscriber> VelSubscribers;

std::map<int, ros::Subscriber> GreenLedSubscribers;
//std::map<std::string, ros::Subscriber> GreenLedSubscribers;

//double xPos, yPos, theta;
//double robXPosPrev, robYPosPrev, robThetaPrev, robDeltaX, robDeltaY, robDeltaTheta;
//double deltaXCorr, deltaYCorr;
//double xPosCorr, yPosCorr;
//double robDistTraveled, robDistTraveledPrev, robDeltaDistTraveled;
//ros::Time currentTime, lastTime;

//ros::Time currentTimeMap, lastTimeMap;

class Robot{
	public:
	double speedLeft, speedRight;
	int greenLed;
	int count, address;
	std::string tag;
	bool changedActuators[ACTUATORS_NUM];
	int robTheta, robXPos, robYPos;

	double xPos, yPos, theta;
	double robXPosPrev, robYPosPrev, robThetaPrev, robDeltaX, robDeltaY, robDeltaTheta;
	double deltaXCorr, deltaYCorr;
	double xPosCorr, yPosCorr;
	double robDistTraveled, robDistTraveledPrev, robDeltaDistTraveled;
	//double robDeltaDistTraveled;

	ros::Time currentTime, lastTime;

	nav_msgs::Odometry odomMsg;

	//double robTargetDistTraveled, robStepDistTraveled;
	//double robTargetTheta, robStepTheta;

};

std::map<int, Robot> robots_dict;
//std::map<std::string, Robot> robots_dict;

void updateSensorsData() {
	//robXPos = getOdomXpos(robotAddress[0]);
	//robYPos = getOdomYpos(robotAddress[0]);
	//robTheta = getOdomTheta(robotAddress[0]);

	std::map<int, Robot>::iterator it;
	for (it = robots_dict.begin(); it != robots_dict.end(); it++){
		robots_dict[it->first].robXPos = getOdomXpos(robots_dict[it->first].address);
		robots_dict[it->first].robYPos = getOdomYpos(robots_dict[it->first].address);
		robots_dict[it->first].robTheta = getOdomTheta(robots_dict[it->first].address);
	}
}

void updateRosInfo() {
	std::map<int, Robot>::iterator it;
	for (it = robots_dict.begin(); it != robots_dict.end(); it++){
		robots_dict[it->first].robDeltaX = robots_dict[it->first].robXPos - robots_dict[it->first].robXPosPrev;
		robots_dict[it->first].robDeltaY = robots_dict[it->first].robYPos - robots_dict[it->first].robYPosPrev;
		robots_dict[it->first].robXPosPrev = robots_dict[it->first].robXPos;
		robots_dict[it->first].robYPosPrev = robots_dict[it->first].robYPos;	

		robots_dict[it->first].theta = robots_dict[it->first].robTheta*M_PI/180;    // Expressed in radiant.
		
		// We noticed from field tests on a vertical wall that there is a difference in the measured distance between
		// a route traveled toward bottom and a route traveled toward top. For this reason we adjust the distance 
		// traveled based on the angle.
		//robots_dict[it->first].robDeltaDistTraveled = sqrt(robots_dict[it->first].robDeltaX*robots_dict[it->first].robDeltaX + robots_dict[it->first].robDeltaY*robots_dict[it->first].robDeltaY);

		if(robots_dict[it->first].robTheta <= 180 && robots_dict[it->first].robTheta >= 0) {
			robots_dict[it->first].robDistTraveled = sqrt(robots_dict[it->first].robDeltaX*robots_dict[it->first].robDeltaX + robots_dict[it->first].robDeltaY*robots_dict[it->first].robDeltaY);
			robots_dict[it->first].deltaXCorr = robots_dict[it->first].robDistTraveled*2/3*cos(robots_dict[it->first].theta);	
			// 2/3 is the magical factor found from filed tests, probably you'll need to adapt it to your surface.
			robots_dict[it->first].deltaYCorr = robots_dict[it->first].robDistTraveled*2/3*sin(robots_dict[it->first].theta);

			//robots_dict[it->first].deltaXCorr = robots_dict[it->first].robDeltaDistTraveled*2/3*cos(robots_dict[it->first].theta);
			robots_dict[it->first].deltaYCorr = robots_dict[it->first].robDeltaDistTraveled*2/3*sin(robots_dict[it->first].theta);

		} else {
			robots_dict[it->first].deltaXCorr = robots_dict[it->first].robDeltaX;
			robots_dict[it->first].deltaYCorr = robots_dict[it->first].robDeltaY;
		}
		//robots_dict[it->first].xPos += robots_dict[it->first].deltaXCorr/1000.0;	// Expressed in meters.
		//robots_dict[it->first].yPos += robots_dict[it->first].deltaYCorr/1000.0;	// Expressed in meters.

		robots_dict[it->first].xPos = robots_dict[it->first].robXPos/1000.0;   // Expressed in meters.
		robots_dict[it->first].yPos = robots_dict[it->first].robYPos/1000.0;   // Expressed in meters.

		
		robots_dict[it->first].robDeltaTheta = (robots_dict[it->first].robTheta - robots_dict[it->first].robThetaPrev)*M_PI/180;
		robots_dict[it->first].robThetaPrev = robots_dict[it->first].robTheta;
		robots_dict[it->first].robDeltaDistTraveled = (robots_dict[it->first].robDistTraveled - robots_dict[it->first].robDistTraveledPrev)/1000.0;
		robots_dict[it->first].robDistTraveledPrev = robots_dict[it->first].robDistTraveled;


		// Publish the odometry message over ROS.
		robots_dict[it->first].odomMsg.header.stamp = ros::Time::now();

		robots_dict[it->first].odomMsg.pose.pose.position.x = robots_dict[it->first].xPos; 
		robots_dict[it->first].odomMsg.pose.pose.position.y = robots_dict[it->first].yPos;
		robots_dict[it->first].odomMsg.pose.pose.position.z = 0;

		// Since all odometry is 6DOF we'll need a quaternion created from yaw.
		geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(robots_dict[it->first].theta);
		robots_dict[it->first].odomMsg.pose.pose.orientation = odomQuat;

		robots_dict[it->first].currentTime = ros::Time::now();
		robots_dict[it->first].odomMsg.twist.twist.linear.x = robots_dict[it->first].robDeltaDistTraveled / ((robots_dict[it->first].currentTime-robots_dict[it->first].lastTime).toSec());   // "robDeltaDistTraveled" is the linear distance covered in meters from the last update (delta distance);
		// the time from the last update is measured in seconds thus to get m/s we multiply them.
		robots_dict[it->first].odomMsg.twist.twist.angular.z = robots_dict[it->first].robDeltaTheta / ((robots_dict[it->first].currentTime-robots_dict[it->first].lastTime).toSec());  // "robDeltaTheta" is the angular distance covered in radiant from the last update (delta angle);
		// the time from the last update is measured in seconds thus to get rad/s we multiply them.

		robots_dict[it->first].lastTime = ros::Time::now();

		odomPublishers[it->first].publish(robots_dict[it->first].odomMsg);

	}

      
// //		OLD
	//robDeltaX = robXPos - robXPosPrev;
	//robDeltaY = robYPos - robYPosPrev;
	//robXPosPrev = robXPos;
	//robYPosPrev = robYPos;				
	//theta = robTheta*M_PI/180;    // Expressed in radiant.
	//// We noticed from field tests on a vertical wall that there is a difference in the measured distance between
	//// a route traveled toward bottom and a route traveled toward top. For this reason we adjust the distance 
	//// traveled based on the angle.
	//if(robTheta <= 180 && robTheta >= 0) {
		//robDistTraveled = sqrt(robDeltaX*robDeltaX + robDeltaY*robDeltaY);
		//deltaXCorr = robDistTraveled*2/3*cos(theta);	// 2/3 is the magical factor found from filed tests, probably you'll need to adapt it to your surface.
		//deltaYCorr = robDistTraveled*2/3*sin(theta);

	//} else {
		//deltaXCorr = robDeltaX;
		//deltaYCorr = robDeltaY;

	//}
	//xPos += deltaXCorr/1000.0;	// Expressed in meters.
	//yPos += deltaYCorr/1000.0;	// Expressed in meters.

	//xPos = robXPos/1000.0;   // Expressed in meters.
	//yPos = robYPos/1000.0;   // Expressed in meters.
    
	//robDeltaTheta = (robTheta - robThetaPrev)*M_PI/180;
	//robThetaPrev = robTheta;
	//robDeltaDistTraveled = (robDistTraveled - robDistTraveledPrev)/1000.0;
	//robDistTraveledPrev = robDistTraveled;

    // Publish the odometry message over ROS.
    //odomMsg.header.stamp = ros::Time::now();
    //odomMsg.header.frame_id = "odom";
    //std::stringstream ss;
    //ss << elisa3Name << "/base_link";
    //odomMsg.child_frame_id = ss.str();
    //odomMsg.pose.pose.position.x = xPos;       
    //odomMsg.pose.pose.position.y = yPos;
    //odomMsg.pose.pose.position.z = 0;
    //// Since all odometry is 6DOF we'll need a quaternion created from yaw.
    //geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(theta);
    //odomMsg.pose.pose.orientation = odomQuat;
    //currentTime = ros::Time::now();
    //odomMsg.twist.twist.linear.x = robDeltaDistTraveled / ((currentTime-lastTime).toSec());   // "robDeltaDistTraveled" is the linear distance covered in meters from the last update (delta distance);
    //// the time from the last update is measured in seconds thus to get m/s we multiply them.
    //odomMsg.twist.twist.angular.z = robDeltaTheta / ((currentTime-lastTime).toSec());  // "robDeltaTheta" is the angular distance covered in radiant from the last update (delta angle);
    //// the time from the last update is measured in seconds thus to get rad/s we multiply them.

    //lastTime = ros::Time::now();

   // odomPublisher.publish(odomMsg);
}

void handlerVelocity(const std_msgs::Float64MultiArray::ConstPtr& msg, int tag) {
//void handlerVelocity(const std_msgs::Float64MultiArray::ConstPtr& msg, std::string tag) {
	//int tag;
	double	speedLeft;
	double speedRight;
	//double targetDist;
	//double targetTheta;
	//tag = int(msg->data[0]);
	speedLeft = double(msg->data[0]);
	speedRight = double(msg->data[1]);
	//targetDist = double(msg -> data[2]);
	//targetTheta = double(msg -> data[3]);

	if(speedLeft>127) {
		speedLeft = 127;
	}
	if(speedRight>127){
		speedRight=127;
	}
	if(speedLeft<-127){
		speedLeft=-127;
	}
	if(speedRight<-127){
		speedRight=-127;
	}

	robots_dict[tag].changedActuators[MOTORS] = true;
	robots_dict[tag].speedLeft = speedLeft;
	robots_dict[tag].speedRight = speedRight;

	//robots_dict[tag].robTargetDistTraveled = targetDist;
	//robots_dict[tag].robTargetTheta = targetTheta;

	std::cout << "[" << nodeName << "] " << "[" << baseTag + "_" << tag << "] " << "speedLeft: " << speedLeft << std::endl;
	std::cout << "[" << nodeName << "] " << "[" << baseTag + "_" << tag << "] " << "speedRight: " << speedRight << std::endl;
	//std::cout << "[" << nodeName << "] " << "[" << baseTag + "_" << tag << "] " << "targetDist: " << targetDist << std::endl;
	//std::cout << "[" << nodeName << "] " << "[" << baseTag + "_" << tag << "] " << "targetTheta: " << targetTheta << std::endl;
}

void handlerGreenLed(const std_msgs::Float64MultiArray::ConstPtr& msg, int tag) {
//void handlerGreenLed(const std_msgs::Float64MultiArray::ConstPtr& msg, std::string tag) {
	//int tag;
	int	intensity;
	//tag = int(msg->data[0]);
	intensity = int(msg->data[0]);
	robots_dict[tag].changedActuators[GREEN_LEDS] = true;
	robots_dict[tag].greenLed = intensity;
}

void updateActuators() {
    
    char buff[6];

	std::map<int, Robot>::iterator it;

	for (it = robots_dict.begin(); it != robots_dict.end(); it++){

		// CHECK MOTORS
		if(robots_dict[it->first].changedActuators[MOTORS]){
			//robots_dict[it->first].robStepDistTraveled += robots_dict[it->first].robDeltaDistTraveled;
			//robots_dict[it->first].robStepTheta += robots_dict[it->first].robDeltaTheta;

			//if (robots_dict[it->first].robStepDistTraveled < robots_dict[it->first].robTargetDistTraveled) {
			//	setLeftSpeed(robots_dict[it->first].address, robots_dict[it->first].speedLeft);
			//	setRightSpeed(robots_dict[it->first].address, robots_dict[it->first].speedRight);
			//} else if (robots_dict[it->first].robStepTheta < robots_dict[it->first].robTargetTheta - 0.5) {
			//	setLeftSpeed(robots_dict[it->first].address, robots_dict[it->first].speedLeft);
			//	setRightSpeed(robots_dict[it->first].address, 0);
			//} else if (robots_dict[it->first].robStepTheta > robots_dict[it->first].robTargetTheta + 0.5) {
			//	setLeftSpeed(robots_dict[it->first].address, robots_dict[it->first].speedLeft);
			//	setRightSpeed(0, robots_dict[it->first].speedLeft);
			//} else {
			//	std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].count << "]" << "check " << std::endl;

			//	std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].count << "]" << "robStepTheta: " << robots_dict[it->first].robStepTheta << std::endl;
				//std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].count << "]" << "robStepDistTraveled: " << robots_dict[it->first].robStepDistTraveled << std::endl;

				//robots_dict[it->first].changedActuators[MOTORS] = false;
				//setLeftSpeed(robots_dict[it->first].address, 0);
				//setRightSpeed(robots_dict[it->first].address, 0);
				//robots_dict[it->first].robStepDistTraveled = 0.;
				//robots_dict[it->first].robStepTheta = 0.;
			//}

			robots_dict[it->first].changedActuators[MOTORS] = false;
			//std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].count << "]" << "updated leftSpeed: " << robots_dict[it->first].speedLeft << std::endl;
			//std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].count << "]" << "updated rightSpeed: " << robots_dict[it->first].speedRight << std::endl;

			setLeftSpeed(robots_dict[it->first].address, robots_dict[it->first].speedLeft);
			setRightSpeed(robots_dict[it->first].address, robots_dict[it->first].speedRight);

		}
		// CHECK GREEN LED
		if(robots_dict[it->first].changedActuators[GREEN_LEDS]){
			robots_dict[it->first].changedActuators[GREEN_LEDS] = false;
			std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].tag << "]" << "updated green leds: " << robots_dict[it->first].greenLed << std::endl;

			setGreen(robots_dict[it->first].address, robots_dict[it->first].greenLed);

		}
	}
}


int main(int argc,char *argv[]) {
		double init_xpos, init_ypos, init_theta;   
		unsigned char sensorsEnabled = 0;
		int i = 0;
   
    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line.
    * For programmatic remappings you can use a different version of init() which takes
    * remappings directly, but for most command-line programs, passing argc and argv is
    * the easiest way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
    ros::init(argc, argv, "elisa3_node_cpp");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle np("~"); // Private.
    ros::NodeHandle n; // Public.

	np.param<std::string>("base_tag", elisa3Name, "elisa3");

	np.param<std::string>("base_tag", baseTag, "elisa3");
	np.param<std::string>("name", nodeName, "elisa3");

	XmlRpc::XmlRpcValue body_list;
    np.param("robots", body_list,body_list);

	int N_robots = body_list.size();
	int robot_addresses[N_robots];
	std::cout << "[" << nodeName << "] " << "number of robots: " << N_robots << std::endl;

	//cmdGreenLed = n.subscribe("elisa3_robot/green_led",10,handlerGreenLed);
	//cmdVelSubscriber = n.subscribe("elisa3_robot/mobile_base/cmd_vel",10,handlerVelocity);

    if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct && body_list.size() > 0){
		XmlRpc::XmlRpcValue::iterator i;
		
		int count = 0;
		for (i = body_list.begin(); i != body_list.end(); ++i) {
			std::string address = (i->second).toXml();
			address.erase(0,7);
			address.erase(4,13); 

			Robot class_inst;
			class_inst.speedLeft =0;
			class_inst.speedRight = 0;
			class_inst.greenLed=0;
			class_inst.changedActuators[MOTORS] = false;
			class_inst.tag = (i->first);
			class_inst.count = count;
			class_inst.address = atoi(address.c_str());
			class_inst.robTheta=0;
			class_inst.robXPos=0;
 			class_inst.robYPos=0;
			class_inst.robThetaPrev=0;
			class_inst.robXPosPrev=0;
 			class_inst.robYPosPrev=0;
			class_inst.currentTime = ros::Time::now();
			class_inst.lastTime = ros::Time::now();

			//class_inst.robStepDistTraveled = 0;
			//class_inst.robStepTheta = 0;

			class_inst.odomMsg.header.frame_id = "elisa3_robot_"+ class_inst.tag +"/odom";
			class_inst.odomMsg.child_frame_id = "elisa3_robot_"+ class_inst.tag +"/base_link";

			robots_dict[count] = class_inst;
			//robots_dict[class_inst.tag] = class_inst;
	
			robotAddress[count] = atoi(address.c_str());
			robot_addresses[count] = atoi(address.c_str());

			boost::function<void (const std_msgs::Float64MultiArray::ConstPtr&)> f = boost::bind(handlerVelocity, _1, count);
			//boost::function<void (const std_msgs::Float64MultiArray::ConstPtr&)> f = boost::bind(handlerVelocity, _1, class_inst.tag);
			VelSubscribers[count] = n.subscribe("elisa3_robot_"+ class_inst.tag +"/mobile_base/cmd_vel", 10, f);
			//VelSubscribers[class_inst.tag] = n.subscribe("elisa3_robot_"+ class_inst.tag +"/mobile_base/cmd_vel", 10, f);

			boost::function<void (const std_msgs::Float64MultiArray::ConstPtr&)> g = boost::bind(handlerGreenLed, _1, count);
			//boost::function<void (const std_msgs::Float64MultiArray::ConstPtr&)> g = boost::bind(handlerGreenLed, _1, class_inst.tag);
			GreenLedSubscribers[count] = n.subscribe("elisa3_robot_"+ class_inst.tag +"/green_led", 10, g);
			//GreenLedSubscribers[class_inst.tag] = n.subscribe("elisa3_robot_"+ class_inst.tag +"/green_led", 10, g);

			odomPublishers[count] = n.advertise<nav_msgs::Odometry>("elisa3_robot_"+ class_inst.tag +"/odom", 10);
			//odomPublishers[class_inst.tag] = n.advertise<nav_msgs::Odometry>("elisa3_robot_"+ class_inst.tag +"/odom", 10);

			// LOG
			std::cout << "[" << nodeName << "] " << "[robot_" << class_inst.tag << "] " << "address: " << class_inst.address << "count: " << count << std::endl;

			count += 1;
		}
	}

    np.param("xpos", init_xpos, 0.0);
    np.param("ypos", init_ypos, 0.0);
    np.param("theta", init_theta, 0.0);
    np.param("accelerometer", enabledSensors[ACCELEROMETER], false);
    np.param("floor", enabledSensors[FLOOR], false);
    np.param("proximity", enabledSensors[PROXIMITY], false);
    np.param("motor_position", enabledSensors[MOTOR_POSITION], false); 


    //if(DEBUG_ROS_PARAMS) {
    //    std::cout << "[" << nodeName << "] " << "init pose: " << init_xpos << ", " << init_ypos << ", " << theta << std::endl;
     //   std::cout << "[" << nodeName << "] " << "accelerometer enabled: " << enabledSensors[ACCELEROMETER] << std::endl;
     //   std::cout << "[" << nodeName << "] " << "floor enabled: " << enabledSensors[FLOOR] << std::endl;
     //   std::cout << "[" << nodeName << "] " << "proximity enabled: " << enabledSensors[PROXIMITY] << std::endl;
     //   std::cout << "[" << nodeName << "] " << "motor position enabled: " << enabledSensors[MOTOR_POSITION] << std::endl;
    //}

		//startCommunication(robotAddress, 2);
		startCommunication(robot_addresses, N_robots);
    

    //if(enabledSensors[ACCELEROMETER]) {
	//			sensorsEnabled++;
     //   accelPublisher = n.advertise<sensor_msgs::Imu>("accel", 10);
    //}
    //if(enabledSensors[FLOOR]) {
		//		sensorsEnabled++;
        //floorPublisher = n.advertise<visualization_msgs::Marker>("floor", 10);
    //}
    //if(enabledSensors[PROXIMITY]) {
		//		sensorsEnabled++;
        //for(i=0; i<8; i++) {
            /**
            * The advertise() function is how you tell ROS that you want to
            * publish on a given topic name. This invokes a call to the ROS
            * master node, which keeps a registry of who is publishing and who
            * is subscribing. After this advertise() call is made, the master
            * node will notify anyone who is trying to subscribe to this topic name,
            * and they will in turn negotiate a peer-to-peer connection with this
            * node.  advertise() returns a Publisher object which allows you to
            * publish messages on that topic through a call to publish().  Once
            * all copies of the returned Publisher object are destroyed, the topic
            * will be automatically unadvertised.
            *
            * The second parameter to advertise() is the size of the message queue
            * used for publishing messages.  If messages are published more quickly
            * than we can send them, the number here specifies how many messages to
            * buffer up before throwing some away.
            */
           // std::stringstream ss;
           // ss.str("");
           // ss << "proximity" << i;
           // proxPublisher[i] = n.advertise<sensor_msgs::Range>(ss.str(), 10);
           // //proxMsg[i] = new sensor_msgs::Range();
           // proxMsg[i].radiation_type = sensor_msgs::Range::INFRARED;
           // ss.str("");
           // ss << elisa3Name << "/base_prox" << i;
           // proxMsg[i].header.frame_id =  ss.str();
           // proxMsg[i].field_of_view = 0.26;    // About 15 degrees...to be checked!
          //  proxMsg[i].min_range = 0.005;       // 0.5 cm.
          //  proxMsg[i].max_range = 0.05;        // 5 cm.                    
      // }       
        
       // laserPublisher = n.advertise<sensor_msgs::LaserScan>("scan", 10);
   // }
    //if(enabledSensors[MOTOR_POSITION]) {
	//			sensorsEnabled++;
       // odomPublisher = n.advertise<nav_msgs::Odometry>("odom", 10);
        //currentTime = ros::Time::now();
        //lastTime = ros::Time::now();        
    //}

   // if(sensorsEnabled == 0) {
    //    std::cerr << "[" << elisa3Name << "] " << "No sensors enabled!" << std::endl;
	//			stopCommunication();
    //    return -1;
   // }
       
    /**
    * The subscribe() call is how you tell ROS that you want to receive messages
    * on a given topic.  This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing.  Messages are passed to a callback function, here
    * called handlerVelocity.  subscribe() returns a Subscriber object that you
    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
    * object go out of scope, this callback will automatically be unsubscribed from
    * this topic.
    *
    * The second parameter to the subscribe() function is the size of the message
    * queue.  If messages are arriving faster than they are being processed, this
    * is the number of messages that will be buffered up before beginning to throw
    * away the oldest ones.
    */

    //cmdVelSubscriber_0 = n.subscribe("elisa3_robot_1/mobile_base/cmd_vel", 10, handlerVelocity_0);    
    //cmdVelSubscriber_1 = n.subscribe("elisa3_robot_2/mobile_base/cmd_vel", 10, handlerVelocity_1); 

	//cmdGreenLed_0 = n.subscribe("elisa3_robot_1/green_led",10,handlerGreenLed_0);
	//cmdGreenLed_1 = n.subscribe("elisa3_robot_2/green_led",10,handlerGreenLed_1);

	

    //theta = init_theta;
    //xPos = init_xpos;
    //yPos = init_ypos;

		// CHANGED
		//enableObstacleAvoidance(robotAddress[0]);
		//enableObstacleAvoidance(robotAddress[1]);
		//enableTVRemote(robotAddress[1]);

    while (ros::ok()) {
        updateSensorsData();
        updateRosInfo();
        updateActuators();
        ros::spinOnce();
				//if(waitForUpdate(robotAddress[0], 10000000)) { // Wait for at most 10 seconds.
					//	break; // We have connection problems, stop here.
				//}
    }

		stopCommunication();
}



