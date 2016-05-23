#include <rail_agile_grasp/grasp_server.h>

GraspServer::GraspServer() :
	find_grasp_client("/rail_agile_grasp/find_grasps", true),
	pickup_client("tablebot_moveit/common_actions/pickup", true),
  rail_agile_grasp_server(nh, "rail_agile_grasp_server", boost::bind(&GraspServer::excute_grasp, this, _1), false),
	nh("~") 
{
	recognized_object_sub = nh.subscribe("/object_recognition_listener/recognized_objects", 100, &GraspServer::object_callback, this);
	grasp_sub = nh.subscribe("/find_grasps/grasps", 100, &GraspServer::grasp_callback, this);

	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/rail_agile_grasp/pose_st", 1000);

	segment_client = nh.serviceClient<std_srvs::Empty>("/rail_segmentation/segment");

  /**
	add_action_client = n.serviceClient<rail_action_queue_msgs::AddAction>("add_action");
	clear_action_list_client = n.serviceClient<rail_action_queue_msgs::ExecuteAction>("clear_action_list");
	get_action_list_client = n.serviceClient<rail_action_queue_msgs::GetActionList>("get_action_list");
	insert_action_client = n.serviceClient<std_srvs::Empty>("insert_aciton");
	remove_action_client = n.serviceClient<rail_action_queue_msgs::RemoveAction>("remove_action");
  **/

  rail_agile_grasp_server.start();

	find_grasp_client.waitForServer();
	ROS_INFO("Find find_grasps server");
	pickup_client.waitForServer();
  ROS_INFO("Find pickup server");
}


void GraspServer::excute_grasp(const rail_agile_grasp_msgs::RailAgileGraspGoalConstPtr &goal)
{

  ros::Rate rate(10.0);
  bool called_segmentation = false;
  bool success = false;

	while(!rail_agile_grasp_server.isPreemptRequested() && ros::ok())
	{
		if(!called_segmentation) {
  			std_srvs::Empty segment_srv;
  			if (!segment_client.call(segment_srv))
  			{
  				ROS_INFO("Couldn't call segmentation service.");
  			}
  			else 
  			{
  				ROS_INFO("Called segmentation service.");
  				called_segmentation = true;
  			}
  		}

  		if (!workspace_list.empty())
  		{
  			rail_agile_grasp_msgs::FindGraspsGoal find_grasp_goal;
  			find_grasp_goal.workspace = workspace_list.front();
  			workspace_list.pop_front();
  			find_grasp_client.sendGoal(find_grasp_goal);
  			ROS_INFO("send one goal");
			// wait for the action to complete
  			find_grasp_client.waitForResult(ros::Duration(15));
  			find_grasp_client.cancelGoal();
  		}
  		else if (!grasp_set.empty())
  		{ 
  			ROS_INFO("Trying to pickup");
				agile_grasp::Grasp attempt_grasp = grasp_set.back();
				grasp_set.pop_back();

				rail_manipulation_msgs::PickupGoal pickup_goal;
				pickup_goal.lift = true;
  			pickup_goal.verify = false;
				pickup_goal.pose = calculateGraspPose(attempt_grasp);

  			pickup_client.sendGoal(pickup_goal);
  			//wait for the action to return
  			while (!pickup_client.getState().isDone())
  			{
  			}
  			if (pickup_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED || !pickup_client.getResult()->success)
  			{
  				ROS_INFO("Pickup failed.");
  			}
  			else
  			{
  				ROS_INFO("Pickup succeeded!");
          success = true;
          break;
  			}
  		}
  	}

    if (success)
    {
      result.success = true;
      rail_agile_grasp_server.setSucceeded(result);
    }

}



void GraspServer::grasp_callback(const agile_grasp::Grasps &grasp_list)
{
	int size = grasp_list.grasps.size();
  int count = 0;
  for (int i = 0; i < size; i++) 
  {
  	if (fabs(grasp_list.grasps[i].center.x) < 0.3 && fabs(grasp_list.grasps[i].center.y < 0.3))
  	{
  		grasp_set.push_back(grasp_list.grasps[i]);
  	}
		count++;
  }
  ROS_INFO("Received %d new unique grasps", count);
}

void GraspServer::object_callback(const rail_manipulation_msgs::SegmentedObjectList &object_list)
{
	//ROS_INFO("the frame_id is %s", objectList.header.frame_id);
	for (int i = 0; i < object_list.objects.size(); i++)
  {	
  	rail_manipulation_msgs::SegmentedObject object = object_list.objects[i];
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
  		workspace_list.push_back(wp);
			//ROS_INFO("The dimension of the object is\n\twidth: %f\n\tdepth: %f\n\theight: %f\n\n", object.width, object.depth, object.height);
  	}
  }
}

geometry_msgs::PoseStamped GraspServer::calculateGraspPose(agile_grasp::Grasp attempt_grasp)
{
  // Generate pose
  Eigen::Vector3d center_; ///< the grasp position
  Eigen::Vector3d surface_center_; ///< the grasp position projected back onto the surface of the object
  Eigen::Vector3d axis_; ///< the hand axis
  Eigen::Vector3d approach_; ///< the grasp approach direction
  tf::vectorMsgToEigen(attempt_grasp.axis, axis_);
  tf::vectorMsgToEigen(attempt_grasp.approach, approach_);
  tf::vectorMsgToEigen(attempt_grasp.center, center_);
  tf::vectorMsgToEigen(attempt_grasp.surface_center, surface_center_);

  Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
  R.col(0) = approach_;
  R.col(1) = axis_;
  R.col(2) << R.col(0).cross(R.col(1));

  tf::Matrix3x3 TF;
  tf::matrixEigenToTF(R, TF);
  tf::Quaternion quat;
  TF.getRotation(quat);
  quat.normalize();

  // create an offset for jaco arm
  surface_center_[0] = surface_center_[0] - 0.08 * approach_[0];
  surface_center_[1] = surface_center_[1] - 0.08 * approach_[1];
  surface_center_[2] = surface_center_[2] - 0.08 * approach_[2];

  Eigen::Vector3d position = surface_center_;
  geometry_msgs::PoseStamped pose_st;
  pose_st.header.stamp = ros::Time(0);
  pose_st.header.frame_id = "kinect2_rgb_optical_frame";
  tf::pointEigenToMsg(position, pose_st.pose.position);
  tf::quaternionTFToMsg(quat, pose_st.pose.orientation);
 
  pose_pub.publish(pose_st);
  return pose_st;
}


int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rail_agile_grasp");
  GraspServer gs;
  ros::spin();
  ROS_INFO("Hello World!");
  return EXIT_SUCCESS;
}