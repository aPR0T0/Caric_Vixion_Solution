/*
 This Programs helps to isolate the bounding box from the set of bounding boxe coordinate
 It publishes the 
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point32.h>
#include <vixion_solution/bbox.h>
#include <nav_msgs/Odometry.h>
#include <bits/stdc++.h>
#include <cmath>

// Global Message variable
vixion_solution::bbox bbox_isolator;

// Global Odometry variable
nav_msgs::Odometry odo_jur1;
nav_msgs::Odometry odo_raf1;

// Visited Array
std::vector<bool> visited;

// Angles for the rotation of the UAV
float target_yaw_jur = 0;
float target_yaw_raf = 0;

// Odometry Callback
void OdoCallbackJurong(const nav_msgs::Odometry::ConstPtr& msg){
	odo_jur1 = *msg;
}

void OdoCallbackRaffles(const nav_msgs::Odometry::ConstPtr& msg){
	odo_raf1 = *msg;
}

void bboxCallback(const sensor_msgs::PointCloud::ConstPtr& msg){

	for ( int i = 0; i < (msg->points.size()/4); i++){
		geometry_msgs::Point32 point;
		point.x = (msg->points[i].x + msg->points[i+1].x + msg->points[i+2].x + msg->points[i+3].x)/4;
		point.y = (msg->points[i].y + msg->points[i+1].y + msg->points[i+2].y + msg->points[i+3].y)/4;
		point.z = (msg->points[i].z + msg->points[i+1].z + msg->points[i+2].z + msg->points[i+3].z)/4;
		bbox_isolator.midpoints.push_back(point);
		visited.push_back(false);
	}

	bbox_isolator.box = msg->points.size()/12;

	for ( int i = 0; i < (msg->points.size() / 2); i++){
		int distance = sqrt(pow(msg->points[i+1].x-msg->points[i].x,2) + pow(msg->points[i+1].y-msg->points[i].y,2) + pow(msg->points[i+1].z-msg->points[i].z,2));
		bbox_isolator.length.push_back(distance);
	}
}

int main(int argc, char **argv){
	ros::init( argc, argv, "cmd_explorer");
	ros::NodeHandle nh;

	// Bounding Box Publisher
	ros::Publisher pub = nh.advertise<vixion_solution::bbox>("bbox_isolator", 1000);

	// Explorer Drone Publisher
	ros::Publisher drone_cmd_jur = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/jurong/command/trajectory", 1000);
	ros::Publisher drone_cmd_raf = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/raffles/command/trajectory", 1000);
	ros::Rate loop_rate(10);

	// Bounding Box Subscriber
	ros::Subscriber sub 	= nh.subscribe("/gcs/bounding_box_vertices", 1000, bboxCallback);
	ros::Subscriber odo_jur = nh.subscribe("/jurong/ground_truth/odometry", 1000, OdoCallbackJurong);
	ros::Subscriber odo_raf = nh.subscribe("/raffles/ground_truth/odometry", 1000, OdoCallbackRaffles);

	// Making an array of distances from midpoints of each face to the current odometry readings
	std::vector<float> distance_jur;
	std::vector<float> distance_raf;

	for (int i = 0 ; i < bbox_isolator.midpoints.size() ; i++){
		distance_jur.push_back(sqrt(pow(bbox_isolator.midpoints[i].x-odo_jur1.pose.pose.position.x,2) + pow(bbox_isolator.midpoints[i].y-odo_jur1.pose.pose.position.y,2) + pow(bbox_isolator.midpoints[i].z-odo_jur1.pose.pose.position.z,2)));
		distance_raf.push_back(sqrt(pow(bbox_isolator.midpoints[i].x-odo_raf1.pose.pose.position.x,2) + pow(bbox_isolator.midpoints[i].y-odo_raf1.pose.pose.position.y,2) + pow(bbox_isolator.midpoints[i].z-odo_raf1.pose.pose.position.z,2)));
	}

	// Trajectory msg for jurong
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg_jur;
	trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg_jur;
	geometry_msgs::Transform transform_msg_jur;
	geometry_msgs::Twist accel_msg_jur, vel_msg_jur;
	trajectory_msg_jur.header.stamp = ros::Time::now();

	// Trajectory msg for raffles
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg_raf;
	trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg_raf;
	geometry_msgs::Transform transform_msg_raf;
	geometry_msgs::Twist accel_msg_raf, vel_msg_raf;
	trajectory_msg_raf.header.stamp = ros::Time::now();

	for (int i = 0 ; i < bbox_isolator.midpoints.size() ; i++){

		auto it = std::min_element(std::begin(distance_jur), std::end(distance_jur));
		int index_jur = std::distance(std::begin(distance_jur), it);
		it = std::min_element(std::begin(distance_raf), std::end(distance_raf));
		int index_raf = std::distance(std::begin(distance_raf), it);

		if ( index_jur == index_raf ){
			distance_jur.erase(distance_jur.begin()+index_jur);
			it = std::min_element(std::begin(distance_jur), std::end(distance_jur));
			int index_jur = std::distance(std::begin(distance_jur), it);
		}

		// Message setting for jurong
		geometry_msgs::Vector3 point_jur;
		point_jur.x = bbox_isolator.midpoints[index_jur].x;
		point_jur.y = bbox_isolator.midpoints[index_jur].y;
		point_jur.z = bbox_isolator.midpoints[index_jur].z;
		transform_msg_jur.translation = point_jur;
		visited[index_jur] = true;
		distance_jur.erase(distance_jur.begin()+index_jur);

		transform_msg_jur.rotation.x = 0;
		transform_msg_jur.rotation.y = 0;
		transform_msg_jur.rotation.z = sinf(target_yaw_jur*0.5);
		transform_msg_jur.rotation.w = cosf(target_yaw_jur*0.5);

		trajpt_msg_jur.transforms.push_back(transform_msg_jur);

		vel_msg_jur.linear.x = 0;
		vel_msg_jur.linear.y = 0;
		vel_msg_jur.linear.z = 0;

		accel_msg_jur.linear.x = 0;
		accel_msg_jur.linear.y = 0;
		accel_msg_jur.linear.z = 0;

		trajpt_msg_jur.velocities.push_back(vel_msg_jur);
		trajpt_msg_jur.accelerations.push_back(accel_msg_jur);

		trajectory_msg_jur.points.push_back(trajpt_msg_jur);

		// Message setting for raffles
		geometry_msgs::Vector3 point_raf;
		point_raf.x = bbox_isolator.midpoints[index_raf].x;
		point_raf.y = bbox_isolator.midpoints[index_raf].y;
		point_raf.z = bbox_isolator.midpoints[index_raf].z;
		transform_msg_jur.translation = point_raf;	
		visited[index_raf] = true;
		distance_raf.erase(distance_raf.begin()+index_raf);

		transform_msg_raf.rotation.x = 0;
		transform_msg_raf.rotation.y = 0;
		transform_msg_raf.rotation.z = sinf(target_yaw_raf*0.5);
		transform_msg_raf.rotation.w = cosf(target_yaw_raf*0.5);

		trajpt_msg_raf.transforms.push_back(transform_msg_raf);
		
		vel_msg_raf.linear.x = 0;
		vel_msg_raf.linear.y = 0;
		vel_msg_raf.linear.z = 0;

		accel_msg_raf.linear.x = 0;
		accel_msg_raf.linear.y = 0;
		accel_msg_raf.linear.z = 0;

		trajpt_msg_raf.velocities.push_back(vel_msg_raf);
		trajpt_msg_raf.accelerations.push_back(accel_msg_raf);

		trajectory_msg_raf.points.push_back(trajpt_msg_raf);
	}

	while(ros::ok()){
		pub.publish(bbox_isolator);
		drone_cmd_jur.publish(trajectory_msg_jur);
		drone_cmd_raf.publish(trajectory_msg_raf);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
