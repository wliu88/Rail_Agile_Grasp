#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <rail_agile_grasp_msgs/RailAgileGraspAction.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "grasp");
	ros::NodeHandle nh;
	actionlib::SimpleActionClient<rail_agile_grasp_msgs::RailAgileGraspAction> client("rail_agile_grasp/rail_agile_grasp_server", true);
	client.waitForServer();
	ROS_INFO("Find rail_agile_grasp server");
	rail_agile_grasp_msgs::RailAgileGraspGoal goal;
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