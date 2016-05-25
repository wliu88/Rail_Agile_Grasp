#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <rail_agile_grasp_msgs/RailAgileGraspAction.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "grasp");
	ros::NodeHandle nh;
	actionlib::SimpleActionClient<rail_agile_grasp_msgs::RailAgileGraspAction> client("/rail_agile_grasp_server", true);
	client.waitForServer();
	ROS_INFO("Find rail_agile_grasp server");
	rail_agile_grasp_msgs::RailAgileGraspGoal goal;
	goal.do_segment = true;
  geometry_msgs::PointStamped store_pose;
  store_pose.header.frame_id = "table_base_link";
  store_pose.pose.position.x = -0.511;
  store_pose.pose.position.y = 0.309;
  store_pose.pose.position.z = 0.373;
  store_pose.pose.orientation.x = 0.284;
  store_pose.pose.orientation.y = 0.649;
  store_pose.pose.orientation.z = 0.658;
  store_pose.pose.orientation.w = 0.256;
  goal.store_pose = store_pose;
	client.sendGoal(goal);
	while (!client.getState().isDone())
  	{
  	}

  	if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED || !client.getResult()->success)
  	{
  		ROS_INFO("rail_agile_grasp failed.");
  	}
  	else
  	{
  		ROS_INFO("rail_agile_grasp succeeded!");
  	}
}