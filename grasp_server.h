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

class GraspServer
{
  public:
    GraspServer();
  private:
    ros::NodeHandle nh;

    ros::Subscriber sub_recognized_objects;
    ros::Subscriber sub_grasps;

    ros::Publisher pose_pub;

    ros::ServiceClient segment_client;

    actionlib::SimpleActionClient<rail_agile_grasp_msgs::FindGraspsAction> find_grasps_client;
    actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction> pickup_client;

    ros::ServiceClient add_action_client;
    ros::serviceClient clear_action_list_client;
    ros::serviceClient get_action_list_client;
    ros::serviceClient insert_action_client;
    ros::serviceClient remove_action_client;


    std::list<rail_agile_grasp_msgs::Workspace> workspace_list;
    std::vector<agile_grasp::Grasp> grasps_set;

    /**
     * callback for getting grasps from agile
     * @param graspsList 
     */
    void grasps_callback(const agile_grasp::Grasps &graspsList);

    /**
     * callback for getting segmented objects 
     * @param objectList
     */
    void objects_callback(const rail_manipulation_msgs::SegmentedObjectList &objectList);

    /**
     * helper function for calculating goal pose (quaternion) from grasp (approach vector and orientation vector)
     * @param  attempt_grasp attemp agile grasp
     * @return  pickup pose
     */
    geometry_msgs::PoseStamped GraspServer::calculateGraspGoal(agile_grasp::Grasp attempt_grasp);

};