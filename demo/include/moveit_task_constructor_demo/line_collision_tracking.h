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


class ObjectCollisionTracker
{
public:
    ObjectCollisionTracker();//const std::string& robot_description = "robot_description");
    ~ObjectCollisionTracker() = default;
    // /** access RobotModel */
    // moveit::core::RobotModelPtr& robotModel() {
    //     return robot_model_;
    // }
    // /** access RobotState */
    // moveit::core::RobotStatePtr& robotState() {
    //     return robot_state_;
    // }
    // class RobotLoadException : std::exception
    // {
    // };
    
    void initObject(const geometry_msgs::PoseStamped& gripper_pose,
                    const geometry_msgs::PoseStamped& gripper_2_pose,
                    moveit_msgs::CollisionObject& collision_object,
                    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
    void updateObject(const geometry_msgs::PoseStamped& gripper_pose,
                    const geometry_msgs::PoseStamped& gripper_2_pose,
                    moveit_msgs::CollisionObject& collision_object,
                    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
    void updateObject2(const geometry_msgs::PoseStamped& gripper_pose,
                        const geometry_msgs::PoseStamped& gripper_2_pose,
                        moveit_msgs::CollisionObject& collision_object,
                        moveit::planning_interface::PlanningSceneInterface& psi );
    void updateObjectShape(const Eigen::Isometry3d& gripper_pose,
                           const Eigen::Isometry3d& gripper_2_pose,
                           std::string& object_id,
                           planning_scene::PlanningScenePtr planning_scene_ptr);
    void updateObjectShape2(const geometry_msgs::PoseStamped& gripper_pose,
                            const geometry_msgs::PoseStamped& gripper_2_pose,
                            std::string& object_id,
                            planning_scene::PlanningScenePtr planning_scene_ptr);
    void determinePose(const geometry_msgs::PoseStamped& gripper_tip_pose,
                        const geometry_msgs::PoseStamped& gripper_tip_2_pose,
                        geometry_msgs::PoseStamped& result_pose_msgs,
                        Eigen::Isometry3d& result_pose_iso,
                        double& length);
    geometry_msgs::PoseStamped isometryToPoseStamped(const Eigen::Isometry3d& transform, const std::string& frame_id);
    void publishMarkers(visualization_msgs::MarkerArray& markers);
    void computeCollisionContactPoints(planning_scene::PlanningScenePtr planning_scene_ptr,
                                        std::vector<std::string> object_group1,
                                        std::vector<std::string> object_group2);
    moveit_msgs::CollisionObject createSimpleObst();
    void createPillarShape(planning_scene::PlanningScenePtr planning_scene_ptr);
    //planning_scene::PlanningScene* g_planning_scene;

    int contactPointCount;
    ros::Publisher* g_marker_array_publisher;
    visualization_msgs::MarkerArray g_collision_points;

private:    
    /* marker publishers */
    ros::NodeHandle nh_;
    // ros::Publisher robot_state_publisher_;
    // ros::Publisher world_state_publisher_;

    // /* robot info */
    // robot_model_loader::RobotModelLoader rm_loader_;
    // moveit::core::RobotModelPtr robot_model_;
    // moveit::core::RobotStatePtr robot_state_;

};
}