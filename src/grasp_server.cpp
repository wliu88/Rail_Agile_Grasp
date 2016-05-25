#include <rail_agile_grasp/grasp_server.h>

GraspServer::GraspServer(ros::NodeHandle& nh_server) :
find_grasp_client("/rail_agile_grasp/find_grasps", true),
pickup_client("tablebot_moveit/common_actions/pickup", true),
store_client("tablebot_moveit/common_actions/store", true),
rail_agile_grasp_server(nh_server, "rail_agile_grasp_server", boost::bind(&GraspServer::excute_grasp, this, _1), false),
nh("~") 
{
	recognized_object_sub = nh.subscribe("/object_recognition_listener/recognized_objects", 100, &GraspServer::object_callback, this);
	grasp_sub = nh.subscribe("/find_grasps/grasps", 100, &GraspServer::grasp_callback, this);
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/rail_agile_grasp/pose_st", 1000);
	segment_client = nh.serviceClient<std_srvs::Empty>("/rail_segmentation/segment");

	//add_action_client = n.serviceClient<rail_action_queue_msgs::AddAction>("add_action");
	//clear_action_list_client = n.serviceClient<rail_action_queue_msgs::ExecuteAction>("clear_action_list");
	//get_action_list_client = n.serviceClient<rail_action_queue_msgs::GetActionList>("get_action_list");
	//insert_action_client = n.serviceClient<std_srvs::Empty>("insert_aciton");
	//remove_action_client = n.serviceClient<rail_action_queue_msgs::RemoveAction>("remove_action");

  rail_agile_grasp_server.start();
  ROS_INFO("Started rail_agile_grasp server");
  find_grasp_client.waitForServer();  
  ROS_INFO("Found find_grasps server");
  pickup_client.waitForServer();
  ROS_INFO("Found pickup server");
}


void GraspServer::excute_grasp(const rail_agile_grasp_msgs::RailAgileGraspGoalConstPtr &goal)
{
  ros::Rate rate(10.0);
  bool called_segmentation = false;
  int object_index = 0;
  bool object_in_progress = false;
  int grasp_index = 0;

  //ss.str("");
  //ss << "Moving to approach angle failed for this grasp.";
  //feedback.current_action = ss.str();
  //pickupServer.publishFeedback(feedback);

  // if scene is already segmented in the caller program
  if(!goal->do_segment)
  {
    called_segmentation = true;
    convert_to_workspace(goal->segmented_object_list);
  }

  // main loop
  while(!rail_agile_grasp_server.isPreemptRequested() && ros::ok())
  { 
    // 1. call segmentation
    if(!called_segmentation)
    {
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
      // spinOnce to get objects message from segmentation
      int i = 0;
      ros::Rate r(10);
      while(!rail_agile_grasp_server.isPreemptRequested() && ros::ok() && i < 300 && workspace_list.empty())
      {
        ros::spinOnce();
        i++;
        r.sleep();
      }
      if (workspace_list.empty())
      {
        ROS_INFO("Could not segment any object");
      }

    }

    // 2. call agile grasp
    // if there are object not processed and no object is being processed
    if (object_index < workspace_list.size() && !object_in_progress)
    {
      object_in_progress = true;
      rail_agile_grasp_msgs::FindGraspsGoal find_grasp_goal;
      find_grasp_goal.workspace = workspace_list.at(object_index);
      find_grasp_client.sendGoal(find_grasp_goal);
      ROS_INFO("Send one object workspace to agile");
      //find_grasp_client.waitForResult(ros::Duration(30)); // block until finish
      int i = 0;
      ros::Rate r(10);
      while(!rail_agile_grasp_server.isPreemptRequested() && ros::ok() && i < 300)
      {
        ros::spinOnce();
        i++;
        r.sleep();
      }
      ROS_INFO("Finished waiting for agile");
      find_grasp_client.cancelGoal();
      if (grasp_set.empty()) 
      {
        object_in_progress = false;
        object_index++;
      }
    }
   
    // 3. call pickup
    if (grasp_index < grasp_set.size() && object_in_progress)
    {
      ROS_INFO("Send one pickup goal");
      agile_grasp::Grasp attempt_grasp = grasp_set.at(grasp_index);

      rail_manipulation_msgs::PickupGoal pickup_goal;
      pickup_goal.lift = true;
      pickup_goal.verify = false;
      pickup_goal.pose = calculateGraspPose(attempt_grasp);

      pickup_client.sendGoal(pickup_goal);
  	  //wait for the action to return
      pickup_client.waitForResult(ros::Duration(500));
      if (pickup_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED || !pickup_client.getResult()->success)
      {
        ROS_INFO("Pickup No.%d failed.", grasp_index);
      }
      else
      {
        ROS_INFO("Pickup No.%d succeeded!", grasp_index);
        // TODO: add a store client.
        rail_manipulation_msgs::StoreGoal store_goal;
        store_goal.store_pose = goal.store_pose;
        store_client.sendGoal(store_goal);
        store_client.waitForResult(ros::Duration(500));
        if (pickup_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED || !pickup_client.getResult()->success)
        {
          ROS_INFO("store object No.%d failed.", object_index);
        }
        else
        {
          ROS_INFO("store object No.%d succeeded!", object_index);
        }
        // increment grasp_index to terminal state
        grasp_index = grasp_set.size();
      }

      grasp_index++;

      // pickup successfully or all grasps have failed
      if (grasp_index >= grasp_set.size())
      {
        grasp_index = 0;
        grasp_set.clear();
        object_in_progress = false;
        object_index++;
      }

      // all objects have been processed, exit main loop
      if (object_index >= workspace_list.size())
      {
        break;
      }
    }
    rate.sleep();
  }

  result.success = true;
  rail_agile_grasp_server.setSucceeded(result);
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
      count++;
  	}
  }
  ROS_INFO("Received %d new valid grasps", count);
  //int s = grasp_set.size();
  //ROS_INFO("grasp_set size is %d", s);
}


void GraspServer::object_callback(const rail_manipulation_msgs::SegmentedObjectList &object_list)
{
  // convert to workspace and add to workspace_list
  convert_to_workspace(object_list);
}


geometry_msgs::PoseStamped GraspServer::calculateGraspPose(agile_grasp::Grasp attempt_grasp)
{
  // Generate pose from two vector representation of orientation
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


void GraspServer::convert_to_workspace(const rail_manipulation_msgs::SegmentedObjectList &object_list)
{
  for (int i = 0; i < object_list.objects.size(); i++)
  { 
    rail_manipulation_msgs::SegmentedObject object = object_list.objects[i];
    if (true)//(!object.recognized)
    {
      // 1. transform the center of object to target reference frame
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
      //ROS_INFO("Find transform");

      // 2. compute the bounding box using the position of the center
      // TODO: use the width, depth, height of the segmented object
      //ROS_INFO("The dimension of the object is\n\twidth: %f\n\tdepth: %f\n\theight: %f\n\n", object.width, object.depth, object.height);
      rail_agile_grasp_msgs::Workspace wp;
      wp.x_min = pt_transformed.point.x - 0.1;
      wp.x_max = pt_transformed.point.x + 0.1;
      wp.y_min = pt_transformed.point.y - 0.1;
      wp.y_max = pt_transformed.point.y + 0.1;
      wp.z_min = -10;
      wp.z_max = 10;
      ROS_INFO("The center of the object after transform is\n\tx: %f\n\ty: %f\n\tz: %f\n", pt_transformed.point.x, pt_transformed.point.y, pt_transformed.point.z);
      workspace_list.push_back(wp);
    }
  }
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rail_agile_grasp");
  ros::NodeHandle nh_server;
  ros::CallbackQueue queue_server;
  nh_server.setCallbackQueue(&queue_server);
  GraspServer gs(nh_server);
  // TODO add a inidividual callback queue for receiving client calls
  while (ros::ok()) 
  {
    queue_server.callAvailable(ros::WallDuration());
  }
  return EXIT_SUCCESS;
}