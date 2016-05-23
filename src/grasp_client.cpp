#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <rail_agile_grasp_msgs/Workspace.h>
#include <rail_agile_grasp_msgs/FindGraspsAction.h>
#include <agile_grasp/Grasps.h>
// rail_action_queue dependencies
// service
#include <rail_action_queue_msgs/AddAction.h>
#include <rail_action_queue_msgs/GetActionList.h>
#include <rail_action_queue_msgs/InsertAction.h>
#include <rail_action_queue_msgs/RemoveAction.h>
#include <rail_action_queue_msgs/ExecuteAction.h>
// msg
#include <rail_action_queue_msgs/GeneralAction.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>


// message callback after segmentation service is called
void objects_callback(const rail_manipulation_msgs::SegmentedObjectList &objectList);
// message callback for receiving grasps from agile_grasp
void grasps_callback(const agile_grasp::Grasps &graspsList);
// a list of workspaces of cluttered objects
std::list<rail_agile_grasp_msgs::Workspace> workspaceList;
std::vector<agile_grasp::Grasp> graspsSet;

int main (int argc, char **argv)
{
	ros::init(argc, argv, "grasp");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/object_recognition_listener/recognized_objects", 100, objects_callback);
	ros::Subscriber sub_grasps = nh.subscribe("/find_grasps/grasps", 100, grasps_callback);
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/rail_agile_grasp/pose_st", 1000);
	// initialize a service client for segmentation
	ros::ServiceClient segmentClient = nh.serviceClient<std_srvs::Empty>("/rail_segmentation/segment");
	// initialize a action client for finding grasps
	actionlib::SimpleActionClient<rail_agile_grasp_msgs::FindGraspsAction> findGraspsClient("/rail_agile_grasp/find_grasps", true);
	findGraspsClient.waitForServer();
	ROS_INFO("Find find_grasps server");
    actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction> pickupClient("tablebot_moveit/common_actions/pickup", true);
	pickupClient.waitForServer();
    ROS_INFO("Find pickup server");
    /**
    // clients for using rail_action_queue
    ros::ServiceClient add_action_client = n.serviceClient<rail_action_queue_msgs::AddAction>("add_action");
    ros::serviceClient clear_action_list_client = n.serviceClient<rail_action_queue_msgs::ExecuteAction>("clear_action_list");
    ros::serviceClient get_action_list_client = n.serviceClient<rail_action_queue_msgs::GetActionList>("get_action_list");
    ros::serviceClient insert_action_client = n.serviceClient<std_srvs::Empty>("insert_aciton");
    ros::serviceClient remove_action_client = n.serviceClient<rail_action_queue_msgs::RemoveAction>("remove_action");
    **/

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
  	//bool calledOnce = false;

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

  		if (!workspaceList.empty())
  		{
  			rail_agile_grasp_msgs::FindGraspsGoal findGraspsGoal;
  			findGraspsGoal.workspace = workspaceList.front();
  			workspaceList.pop_front();
  			findGraspsClient.sendGoal(findGraspsGoal);
  			ROS_INFO("send one goal");
			// wait for the action to complete
  			findGraspsClient.waitForResult(ros::Duration(15));
  			findGraspsClient.cancelGoal();
  		}
  		else if (!graspsSet.empty())
  		{
  			ROS_INFO("Trying to pickup");
			agile_grasp::Grasp attemptGrasp = graspsSet.back();
			graspsSet.pop_back();

  			rail_manipulation_msgs::PickupGoal pickupGoal;
  			pickupGoal.lift = true;
  			pickupGoal.verify = false;

			// Generate pose
			Eigen::Vector3d center_; ///< the grasp position
			Eigen::Vector3d surface_center_; ///< the grasp position projected back onto the surface of the object
			Eigen::Vector3d axis_; ///< the hand axis
			Eigen::Vector3d approach_; ///< the grasp approach direction
			Eigen::Vector3d binormal_; ///< the vector orthogonal to the hand axis and the grasp approach direction
			tf::vectorMsgToEigen(attemptGrasp.axis, axis_);
			tf::vectorMsgToEigen(attemptGrasp.approach, approach_);
			tf::vectorMsgToEigen(attemptGrasp.center, center_);
			tf::vectorMsgToEigen(attemptGrasp.surface_center, surface_center_);
			
			Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
  			R.col(0) = approach_;
  			R.col(1) = axis_;
  			R.col(2) << R.col(0).cross(R.col(1));

  			tf::Matrix3x3 TF;
			tf::matrixEigenToTF(R, TF);
			tf::Quaternion quat;
			TF.getRotation(quat);
			quat.normalize();
            
            // this offset is needed for rail lab jaco arm
            surface_center_[0] = surface_center_[0] - 0.08 * approach_[0];
            surface_center_[1] = surface_center_[1] - 0.08 * approach_[1];
            surface_center_[2] = surface_center_[2] - 0.08 * approach_[2];

			Eigen::Vector3d position = surface_center_;
			geometry_msgs::PoseStamped pose_st;
			pose_st.header.stamp = ros::Time(0);
			pose_st.header.frame_id = "kinect2_rgb_optical_frame";
			tf::pointEigenToMsg(position, pose_st.pose.position);
			tf::quaternionTFToMsg(quat, pose_st.pose.orientation);

			pickupGoal.pose = pose_st;
			pose_pub.publish(pose_st);

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
  		}

  		rate.sleep();
  	}

  	return 0;
  }

  void grasps_callback(const agile_grasp::Grasps &graspsList)
  {
  	int size = graspsList.grasps.size();
  	int count = 0;
  	for (int i = 0; i < size; i++) 
  	{
  		if (fabs(graspsList.grasps[i].center.x) < 0.3 && fabs(graspsList.grasps[i].center.y < 0.3))
  		{
  			graspsSet.push_back(graspsList.grasps[i]);
  		}
		count++;
  	}
  	ROS_INFO("Received %d new unique grasps", count);
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

  		if (true)//(!object.recognized)
  		{
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
  }