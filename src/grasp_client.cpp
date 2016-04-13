#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <rail_agile_grasp_msgs/Workspace.h>
#include <rail_agile_grasp_msgs/FindGraspsAction.h>

// message callback after segmentation service is called
void objects_callback(const rail_manipulation_msgs::SegmentedObjectList &objectList);
// a list of workspaces of cluttered objects
std::list<rail_agile_grasp_msgs::Workspace> workspaceList;

int main (int argc, char **argv)
{
	ros::init(argc, argv, "grasp");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/object_recognition_listener/recognized_objects", 100, objects_callback);
	// initialize a service client for segmentation
	ros::ServiceClient segmentClient = nh.serviceClient<std_srvs::Empty>("/rail_segmentation/segment");
	// initialize a action client for finding grasps
	actionlib::SimpleActionClient<rail_agile_grasp_msgs::FindGraspsAction> findGraspsClient("/rail_agile_grasp/find_grasps", true);
	findGraspsClient.waitForServer();
	ROS_INFO("Find find_grasps server");


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
  	***/

	ros::Rate rate(10.0);

	bool calledSegmentation = false;
	bool calledOnce = false;

	while(nh.ok())
	{
		ros::spinOnce();
		if(!calledSegmentation) {
			std_srvs::Empty segmentSrv;
			if (!segmentClient.call(segmentSrv))
			{
				ROS_INFO("Couldn't call segmentation service.");
			}
			else 
			{
				ROS_INFO("Called segmentation service.");
				calledSegmentation = true;
			}
		}

		if (!workspaceList.empty() && !calledOnce)
		{
			rail_agile_grasp_msgs::FindGraspsGoal findGraspsGoal;
			findGraspsGoal.workspace = workspaceList.front();
			workspaceList.pop_front();
			findGraspsClient.sendGoal(findGraspsGoal);
			ROS_INFO("send one goal");
			// wait for the action to complete
			findGraspsClient.waitForResult(ros::Duration(10));
			calledOnce = true;
		}
		rate.sleep();
	}

	return 0;
}


void objects_callback(const rail_manipulation_msgs::SegmentedObjectList &objectList)
{   
	//ROS_INFO("the frame_id is %s", objectList.header.frame_id);
	
	for (int i = 0; i < objectList.objects.size(); i++)
	{	
		rail_manipulation_msgs::SegmentedObject object = objectList.objects[i];
		
		//ROS_INFO("This is object No.%d\n", i);
		//ROS_INFO("This object is recognized? %d\n", object.recognized);
		//ROS_INFO("The center of the object is\n\tx: %f\n\ty: %f\n\tz: %f\n", object.center.x, object.center.y, object.center.z);
		
		tf::TransformListener listener;
		const ros::Time time = ros::Time(0);
		const std::string target_frame = "kinect2_rgb_optical_frame";
		const std::string reference_frame = "table_base_link";
		geometry_msgs::PointStamped pt;
		geometry_msgs::PointStamped pt_transformed;
		pt.point = object.center;
		pt.header.frame_id = reference_frame;
		
		ros::Rate rate(100.0);
		while(true)
		{
			try
			{
				listener.waitForTransform(target_frame, reference_frame, time, ros::Duration(10.0));
				listener.transformPoint(target_frame, pt, pt_transformed);
				break;
			}
			catch(tf::TransformException ex)
			{
				ROS_ERROR("%s", ex.what());
			}
			rate.sleep();
		}
		ROS_INFO("Find transform");

		ROS_INFO("The center of the object after transform is\n\tx: %f\n\ty: %f\n\tz: %f\n", 
			     pt_transformed.point.x, pt_transformed.point.y, pt_transformed.point.z);
		rail_agile_grasp_msgs::Workspace wp;
		wp.x_min = pt_transformed.point.x - 0.1;
		wp.x_max = pt_transformed.point.x + 0.1;
		wp.y_min = pt_transformed.point.y - 0.1;
		wp.y_max = pt_transformed.point.y + 0.1;
		wp.z_min = -10;
		wp.z_max = 10;
		workspaceList.push_back(wp);

		//ROS_INFO("The dimension of the object is\n\twidth: %f\n\tdepth: %f\n\theight: %f\n\n", object.width, object.depth, object.height);
		
	}
}