#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

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
    ObjectCollisionTracker();
    ~ObjectCollisionTracker() = default;

    // Initializing
    void initObject(const geometry_msgs::PoseStamped& steady_point,
                    const geometry_msgs::PoseStamped& moving_point,
                    moveit_msgs::CollisionObject& collision_object,
                    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
    moveit_msgs::CollisionObject createSimpleObst(const ros::NodeHandle& nh);


    // Updating at each iteration
    void updateDLO(const geometry_msgs::PoseStamped&  start_pose,
                    const geometry_msgs::PoseStamped&  end_pose,
                    moveit_msgs::CollisionObject& collision_object,
                    planning_scene::PlanningScenePtr& planning_scene_ptr,
                    moveit::planning_interface::PlanningSceneInterface& psi,
                    const std::vector<collision_detection::Contact>& adjusted_contacts,
                    bool& hasNewContact,
                    int& num_segments);
    void updateObject(const geometry_msgs::PoseStamped&  steady_point,
                        const geometry_msgs::PoseStamped&  moving_point,
                        moveit_msgs::CollisionObject& collision_object,
                        planning_scene::PlanningScenePtr& planning_scene_ptr,
                        moveit::planning_interface::PlanningSceneInterface& psi) ;
    


    // Pose computations
    void determinePose(const geometry_msgs::PoseStamped& steady_point,
                        const geometry_msgs::PoseStamped& moving_point,
                        geometry_msgs::PoseStamped& result_pose_msgs,
                        Eigen::Isometry3d& result_pose_iso,
                        double& length);
    geometry_msgs::PoseStamped isometryToPoseStamped(const Eigen::Isometry3d& transform, const std::string& frame_id);
    /** \brief Convert an Eigen::Vector3d into a geometry_msgs::PoseStamped. */
    geometry_msgs::PoseStamped vectorToPoseStamped(const Eigen::Vector3d& position);


    
    // Computing collisions
    void computeCollisionContactPoints(planning_scene::PlanningScenePtr planning_scene_ptr,
                                        std::vector<std::string> object_group1,
                                        std::vector<std::string> object_group2,
                                        collision_detection::CollisionResult& c_res,
                                        std::vector<collision_detection::Contact>& stored_contacts,
                                        std::vector<collision_detection::Contact>& adjusted_contacts,
                                        bool& isNewContact);
    
    // Handling & publishing contact points
    void publishMarkers(visualization_msgs::MarkerArray& markers);
    std::map<std::string, Eigen::Vector3d> getCornerPoints(const moveit_msgs::CollisionObject& pillar);
    Eigen::Vector3d determineNearestCornerPoint(const collision_detection::Contact& contact_point,
                                                const std::map<std::string,Eigen::Vector3d>& cornerPoints);
    collision_detection::Contact adjustContactPoint(const collision_detection::Contact& contact_point);
    




    // Publisher and MArkerArray for visualization of contact points
    ros::Publisher* g_marker_array_publisher;
    visualization_msgs::MarkerArray g_collision_points;

    std::map<std::string, Eigen::Vector3d> corner_points; // The corner points of the obstacle

private:
    ros::NodeHandle nh_;
};
}