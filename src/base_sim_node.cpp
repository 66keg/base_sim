/*!
 * @file base_sim_node.cpp
 * @brief 差動二輪ロボットの2D運動学シミュレーション
 * @author Keiji Muro
 * @data 2018.04.23: ver0.0.1 新規作成
 */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

static const double dt = 0.01;	// 制御周期 [sec]
double cmd_v_, cmd_w_ = 0.0;	// 制御コマンド
double odom_x_, odom_y_, odom_th_, odom_v_, odom_w_;	// オドメトリ

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	cmd_v_ = msg->linear.x;
	cmd_w_ = msg->angular.z;
}

bool reset(std_srvs::Empty::Request  &req,
           std_srvs::Empty::Response &res)
{
	odom_x_ = odom_y_ = odom_th_ = 0.0;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_sim_node");
	ros::NodeHandle n;

	ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>("odom", 1);
	ros::Subscriber sub_cmd = n.subscribe("cmd_vel", 1, cmdCallback);
	ros::ServiceServer service = n.advertiseService("reset_odom", reset);
	tf::TransformBroadcaster tf_odom2base;

	ros::Rate loop_rate(1.0/dt);

	odom_x_ = odom_y_ = odom_th_ = odom_v_ = odom_w_ = 0.0;

	while (ros::ok())
	{
		odom_v_ = cmd_v_;
		odom_w_ = cmd_w_;
		odom_x_ += odom_v_ * cos(odom_th_) * dt;
		odom_y_ += odom_v_ * sin(odom_th_) * dt;
		odom_th_ += odom_w_ * dt;

		geometry_msgs::Quaternion link_quat2D;
		link_quat2D = tf::createQuaternionMsgFromYaw(odom_th_);

		ros::Time current_time = ros::Time::now();

		nav_msgs::Odometry odomMsg;
		odomMsg.header.stamp 			= current_time;
		odomMsg.header.frame_id 	= "odom";
		odomMsg.child_frame_id 		= "base_link";
		odomMsg.pose.pose.position.x  	= odom_x_;
		odomMsg.pose.pose.position.y 	  = odom_y_;
		odomMsg.pose.pose.position.z  	= 0.0;
		odomMsg.pose.pose.orientation 	= link_quat2D;
		odomMsg.twist.twist.linear.x  	= odom_v_;
		odomMsg.twist.twist.linear.y  	= 0.0;
		odomMsg.twist.twist.linear.z  	= 0.0;
		odomMsg.twist.twist.angular.x 	= 0.0;
		odomMsg.twist.twist.angular.y 	= 0.0;
		odomMsg.twist.twist.angular.z 	= odom_w_;
		pub_odom.publish(odomMsg);

		geometry_msgs::TransformStamped trans_link;
		trans_link.header.stamp 			= current_time;
		trans_link.header.frame_id 		= "odom";
		trans_link.child_frame_id 		= "base_link";
		trans_link.transform.translation.x 	= odom_x_;
		trans_link.transform.translation.y 	= odom_y_;
		trans_link.transform.translation.z 	= 0.0;
		trans_link.transform.rotation 		= link_quat2D;
		tf_odom2base.sendTransform(trans_link);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
