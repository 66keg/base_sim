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

static const double CONTROL_PERIOD = 0.01;	// 制御周期 [sec]
double g_cmd_v, g_cmd_w;	// 制御コマンド
double g_odom_x, g_odom_y, g_odom_th, g_odom_v, g_odom_w;	// オドメトリ

void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmdMsg)
{
	g_cmd_v = cmdMsg->linear.x;
	g_cmd_w = cmdMsg->angular.z;
}

bool resetOdom(std_srvs::Empty::Request  &emptyReq,
               std_srvs::Empty::Response &emptyRes)
{
	g_odom_x = g_odom_y = g_odom_th = 0.0;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_sim_node");
	ros::NodeHandle n;

	ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>("odom", 1);
	ros::Subscriber sub_cmd = n.subscribe("cmd_vel", 1, cmdCallback);
	ros::ServiceServer srv_reset = n.advertiseService("reset_odom", resetOdom);
	tf::TransformBroadcaster tf_odom2base;

	ros::Rate loop_rate(1.0/CONTROL_PERIOD);

	g_cmd_v = g_cmd_w = 0.0;
	g_odom_x = g_odom_y = g_odom_th = g_odom_v = g_odom_w = 0.0;

	while (ros::ok())
	{
		g_odom_v = g_cmd_v;
		g_odom_w = g_cmd_w;
		g_odom_x += g_odom_v * cos(g_odom_th) * CONTROL_PERIOD;
		g_odom_y += g_odom_v * sin(g_odom_th) * CONTROL_PERIOD;
		g_odom_th += g_odom_w * CONTROL_PERIOD;

		geometry_msgs::Quaternion link_quat2D;
		link_quat2D = tf::createQuaternionMsgFromYaw(g_odom_th);

		ros::Time current_time = ros::Time::now();

		nav_msgs::Odometry odom_msg;
		odom_msg.header.stamp 			= current_time;
		odom_msg.header.frame_id 	= "odom";
		odom_msg.child_frame_id 		= "base_link";
		odom_msg.pose.pose.position.x  	= g_odom_x;
		odom_msg.pose.pose.position.y 	  = g_odom_y;
		odom_msg.pose.pose.position.z  	= 0.0;
		odom_msg.pose.pose.orientation 	= link_quat2D;
		odom_msg.twist.twist.linear.x  	= g_odom_v;
		odom_msg.twist.twist.linear.y  	= 0.0;
		odom_msg.twist.twist.linear.z  	= 0.0;
		odom_msg.twist.twist.angular.x 	= 0.0;
		odom_msg.twist.twist.angular.y 	= 0.0;
		odom_msg.twist.twist.angular.z 	= g_odom_w;
		pub_odom.publish(odom_msg);

		geometry_msgs::TransformStamped trans_link;
		trans_link.header.stamp 			= current_time;
		trans_link.header.frame_id 		= "odom";
		trans_link.child_frame_id 		= "base_link";
		trans_link.transform.translation.x 	= g_odom_x;
		trans_link.transform.translation.y 	= g_odom_y;
		trans_link.transform.translation.z 	= 0.0;
		trans_link.transform.rotation 		= link_quat2D;
		tf_odom2base.sendTransform(trans_link);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
