/*
 This Programs helps to isolate the bounding box from the set of bounding boxe coordinate
 It publishes the 
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud.h>
#include <vixion_solution/bbox.h>
#include <bits/stdc++.h>

int** get_faces(int** bbox, int n){
	int** faces = new int*[n];
	for(int i=0; i<n; i++){
		faces[i] = new int[4];		// Dynamic allocation
		faces[i][0] = bbox[i][0];
		faces[i][1] = bbox[i][1];
		faces[i][2] = bbox[i][2];
		faces[i][3] = bbox[i][3];
	}
	return faces;
}

int** get_midpoints(int** bbox, int n){
	int** faces = new int*[n];
	for(int i=0; i<n; i++){
		faces[i] = new int[4];		// Dynamic allocation
		faces[i][0] = bbox[i][0];
		faces[i][1] = bbox[i][1];
		faces[i][2] = bbox[i][2];
		faces[i][3] = bbox[i][3];
	}
	return faces;
}

void bboxCallback(const sensor_msgs::PointCloud::ConstPtr& msg){

	vision_solution::msg::bbox bbox_isolator;

	for ( int i = 0; i < (msg->points.size()/12); i++){
		bbox_isolator->midpoints.x.push_back((msg->points[i].x + msg->points[i+1].x + msg->points[i+2].x + msg->points[i+3].x)/4);
		bbox_isolator->midpoints.y.push_back((msg->points[i].y + msg->points[i+1].y + msg->points[i+2].y + msg->points[i+3].y)/4);
		bbox_isolator->midpoints.z.push_back((msg->points[i].z + msg->points[i+1].z + msg->points[i+2].z + msg->points[i+3].z)/4);
	}
}

int main(int argc, char **argv){
	ros::init(argc,argv, "bbox_isolator");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("bbox_isolator", 1000);
	ros::Rate loop_rate(10);

	ros::Subscriber sub = nh.subscribe("bbox", 1000, bboxCallback);

	while(ros::ok()){
		// std_msgs::Float32 msg;
		// msg.data = 0.0;
		// pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
