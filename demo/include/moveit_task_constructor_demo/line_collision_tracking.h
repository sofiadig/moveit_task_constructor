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
    
    void initObject(const geometry_msgs::PoseStamped& steady_point,
                    const geometry_msgs::PoseStamped& moving_point,
                    moveit_msgs::CollisionObject& collision_object,
                    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
    void updateObject2(const geometry_msgs::PoseStamped& gripper_pose,
                        const geometry_msgs::PoseStamped& gripper_2_pose,
                        moveit_msgs::CollisionObject& collision_object,
                        moveit::planning_interface::PlanningSceneInterface& psi );
    void updateObjectShape2(const geometry_msgs::PoseStamped& steady_point,
                            const geometry_msgs::PoseStamped& moving_point,
                            std::string& object_id,
                            planning_scene::PlanningScenePtr planning_scene_ptr);
    void initObjectShape(const geometry_msgs::PoseStamped&  steady_point,
                                const geometry_msgs::PoseStamped&  moving_point,
                                std::string& object_id,
                                planning_scene::PlanningScenePtr planning_scene_ptr);
    void determinePose(const geometry_msgs::PoseStamped& steady_point,
                        const geometry_msgs::PoseStamped& moving_point,
                        geometry_msgs::PoseStamped& result_pose_msgs,
                        Eigen::Isometry3d& result_pose_iso,
                        double& length);
    void updateDLO(const geometry_msgs::PoseStamped&  start_pose,
                    const geometry_msgs::PoseStamped&  end_pose,
                    moveit_msgs::CollisionObject& collision_object,
                    planning_scene::PlanningScenePtr planning_scene_ptr,
                    moveit::planning_interface::PlanningSceneInterface& psi,
                    const std::vector<collision_detection::Contact>& adjusted_contacts,
                    bool& hasNewContact,
                    int& num_segments);
    geometry_msgs::PoseStamped vectorToPoseStamped(const Eigen::Vector3d& position);
    geometry_msgs::PoseStamped isometryToPoseStamped(const Eigen::Isometry3d& transform, const std::string& frame_id);
    void publishMarkers(visualization_msgs::MarkerArray& markers);
    void computeCollisionContactPoints(planning_scene::PlanningScenePtr planning_scene_ptr,
                                        std::vector<std::string> object_group1,
                                        std::vector<std::string> object_group2,
                                        collision_detection::CollisionResult& c_res,
                                        std::vector<collision_detection::Contact>& stored_contacts,
                                        std::vector<collision_detection::Contact>& adjusted_contacts,
                                        bool& isNewContact);
    moveit_msgs::CollisionObject createSimpleObst();
    //collision_detection::Contact adjustContactPoint(const collision_detection::Contact& contact) ;
    Eigen::Vector3d determineNearestCornerPoint(const collision_detection::Contact& contact_point,
                                                const std::map<std::string,Eigen::Vector3d>& cornerPoints);
    std::map<std::string, Eigen::Vector3d> getCornerPoints(const moveit_msgs::CollisionObject& pillar);
    collision_detection::Contact adjustContactPoint(const collision_detection::Contact& contact_point);
    void createPillarShape(planning_scene::PlanningScenePtr planning_scene_ptr);
    //planning_scene::PlanningScene* g_planning_scene;

    int contactPointCount;
    ros::Publisher* g_marker_array_publisher;
    visualization_msgs::MarkerArray g_collision_points;
    std::map<std::string, Eigen::Vector3d> corner_points;

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