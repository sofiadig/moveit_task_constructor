#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <moveit_msgs/CollisionObject.h>
// MoveIt
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/collision_detection/collision_common.h>
// MTC
#include <moveit/task_constructor/task.h>


namespace moveit_task_constructor_demo {
	using namespace moveit::task_constructor;


class ObjectTracker
{
public:
    ObjectTracker();
    ~ObjectTracker() = default;
    ros::Publisher* g_marker_array_publisher;
    visualization_msgs::MarkerArray g_collision_points;


    void allowCollisions(const std::string& link_name, const std::string& object_id);
    void initObject(const geometry_msgs::PoseStamped& gripper_pose, const geometry_msgs::PoseStamped& gripper_2_pose,
                    moveit_msgs::CollisionObject& collision_object,
                    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
    void updateObject(const geometry_msgs::PoseStamped& gripper_pose, const geometry_msgs::PoseStamped& gripper_2_pose,
                    moveit_msgs::CollisionObject& collision_object,
                    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
    geometry_msgs::Pose isometryToPose(const Eigen::Isometry3d& transform);
    geometry_msgs::PoseStamped isometryToPoseStamped(const Eigen::Isometry3d& transform, const std::string& frame_id);
    bool updatePlanningScene(planning_scene::PlanningScene& planning_scene, ros::NodeHandle& nh);
    void publishMarkers(visualization_msgs::MarkerArray& markers);
    void computeCollisionContactPoints(planning_scene::PlanningScenePtr& planning_scene, moveit_msgs::CollisionObject& object,
                                       robot_state::RobotStatePtr& robot);
    void updateTransform(const geometry_msgs::Pose& pose, const std::string& parent_frame_id, const std::string& child_frame_id,
                        geometry_msgs::TransformStamped& transformStamped);
    void attachObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                  const std::string& object_id, const std::string& link_name);
    void detachObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                  const std::string& object_id);


};
}