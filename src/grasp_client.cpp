#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <tf/transform_listener.h>

void objects_callback(const rail_manipulation_msgs::SegmentedObjectList &objectList);

int main (int argc, char **argv)
{
	ros::init(argc, argv, "grasp");
 	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/object_recognition_listener/recognized_objects", 100, objects_callback);
    
	/***
  	// create the action client
  	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction> pickupClient("tablebot_moveit/common_actions/pickup", true);


  	ROS_INFO("Waiting for action server to start.");
  	// wait for the action server to start
  	pickupClient.waitForServer(); //will wait for infinite time


  	ROS_INFO("Action server started, sending goal.");
  	// send a goal to the action
  	rail_manipulation_msgs::PickupGoal pickupGoal;
  	pickupGoal.lift = true;
  	pickupGoal.verify = false;

  	//pickupGoal.pose = action.manipulation_pose;
  	pickupClient.sendGoal(pickupGoal);


  	//wait for the action to return
  	while (!pickupClient.getState().isDone())
  	{
  	}

  	if (pickupClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED || !pickupClient.getResult()->success)
  	{
  		ROS_INFO("Pickup failed.");
  	}
  	else
  	{
  		ROS_INFO("Pickup succeeded!");
  	}
  	**/
  	

  	tf::TransformListener listener;
	tf::StampedTransform transform;
	const ros::Time time = ros::Time(0);
	const std::string target_frame = "kinect2_rgb_optical_frame";
	const std::string reference_frame = "table_base_link";

	ros::Rate rate(10.0);

	while(nh.ok())
	{
		try
		{
			listener.waitForTransform(target_frame, reference_frame, time, ros::Duration(10.0));
			listener.lookupTransform(target_frame, reference_frame, time, transform);
			break;
		}
		catch(tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
		}
	}
	ROS_INFO("find transform");

  	return 0;
}

void objects_callback(const rail_manipulation_msgs::SegmentedObjectList &objectList)
{   
	//ROS_INFO("the frame_id is %s", objectList.header.frame_id);
	/**
	for (int i = 0; i < objectList.objects.size(); i++)
	{	
		rail_manipulation_msgs::SegmentedObject object = objectList.objects[i];
		ROS_INFO("This is object No.%d\n", i);
		ROS_INFO("This object is recognized? %d\n", object.recognized);
		ROS_INFO("The center of the object is\n\tx: %f\n\ty: %f\n\tz: %f\n", 
				 object.center.x, object.center.y, object.center.z);
		ROS_INFO("The dimension of the object is\n\twidth: %f\n\tdepth: %f\n\theight: %f\n\n", object.width, object.depth, object.height);
	}
	**/

	// the problem is to get the 
}
