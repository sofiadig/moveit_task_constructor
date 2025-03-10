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
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
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

    /** \brief Initialize the DLO object for visualization.
     * This function establishes the shape and pose of the object and adds it to the planning scene.
     */
    void initObject(const geometry_msgs::PoseStamped& start_pose, // static end
                    const geometry_msgs::PoseStamped& end_pose,   // moving end
                    moveit_msgs::CollisionObject& collision_object,
                    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface );

    /** \brief Initialize the object visualization. */
    moveit_msgs::CollisionObject createSimpleObstacle();


    /** \brief Update the DLO objects (collision checking and visualization).
     * This function determines the starting and end points for the DLO segments
     * and calls initObject() and updateObject() to implement those segments.
     */
    void updateDLO(const geometry_msgs::PoseStamped&  start_pose, // static end
                    const geometry_msgs::PoseStamped&  end_pose,  // moving end
                    moveit_msgs::CollisionObject& collision_object,
                    planning_scene::PlanningScenePtr& planning_scene_ptr,
                    moveit::planning_interface::PlanningSceneInterface& psi,
                    const std::vector<collision_detection::Contact>& adjusted_contacts,
                    const bool& hasNewContact,
                    int& num_segments );

    /** \brief Update the DLO objects (collision checking and visualization).
     * This function takes the given starting and end points and updates the
     * DLO object(s) to be a straight line between the two.
     * It call determinePose() to compute the pose and dimensions for the DLO object.
     * To visualize the object, it either applies it to the sceene as a collision
     * object or it publishes a line marker by calling updateLineMarker().
    */
    void updateObject(const geometry_msgs::PoseStamped&  start_pose, // static end
                        const geometry_msgs::PoseStamped&  end_pose, // moving end
                        moveit_msgs::CollisionObject& collision_object,
                        planning_scene::PlanningScenePtr& planning_scene_ptr,
                        moveit::planning_interface::PlanningSceneInterface& psi,
                        int& num_segments ) ;
    /** \brief Update the DLO visualization using a line marker. */
    void updateLineMarker(const geometry_msgs::Point& start,
                          const geometry_msgs::Point& end, 
                          int marker_id);
    


    /** \brief Compute the pose and dimensions that the DLO object needs to have
     * to be a straight line between the two given points.
     */
    void determinePose(const geometry_msgs::PoseStamped& start_pose, // static end
                        const geometry_msgs::PoseStamped& end_pose,  // moving end
                        geometry_msgs::PoseStamped& result_pose_msgs,
                        Eigen::Isometry3d& result_pose_iso,
                        double& length);

    /** \brief Convert an Eigen::Isometry3d into a geometry_msgs::PoseStamped. */
    geometry_msgs::PoseStamped isometryToPoseStamped(const Eigen::Isometry3d& transform, const std::string& frame_id);

    /** \brief Convert an Eigen::Vector3d into a geometry_msgs::PoseStamped. */
    geometry_msgs::PoseStamped vectorToPoseStamped(const Eigen::Vector3d& position);


    
    /** \brief Collision checking between object groups.
     * This function takes objects that are divided in two groups and checks for collisions
     * between the objects in those groups (in this case one group contains the DLO and the
     * other contains the obstacle).
     * It calls planning_scene_ptr->getCollisionEnv()->checkCollisionBetweenObjectGroups to
     * compute the collision contact points and calls publishMarkers() to publish their
     * visualization markers in Rviz.
    */
    void computeCollisionContactPoints(planning_scene::PlanningScenePtr planning_scene_ptr,
                                        std::vector<std::string> object_group1,
                                        std::vector<std::string> object_group2,
                                        collision_detection::CollisionResult& c_res,
                                        std::vector<collision_detection::Contact>& stored_contacts,
                                        std::vector<collision_detection::Contact>& adjusted_contacts,
                                        bool& isNewContact);
    

    /** \brief Publish the contact point markers given by computeCollisionContactPoints(). */
    void publishMarkers(visualization_msgs::MarkerArray& markers);

    /** \brief Compute the 2D (x,y) corner points of the horizontal cross section of the obstacle pillar.
     * It takes the pose and dimensions of the vertical standing obstacle and determines the position of
     * the corners (in 2D)/ vertical edges.
     */
    std::map<std::string, Eigen::Vector3d> getCornerPoints(const moveit_msgs::CollisionObject& pillar);

    /** \brief Adjustes the position of the computed contact points to be on the surface/ edge of the
     * obstacle.
     * Since the collision points are inside the object bodies, the points need to be projected to
     * the surface of the obstacle to reflect the actual physical behavior of the DLO.
     * This function calls determineNearestCornerPoint() to find the corner point to which to 
     * project the contact point.
     */
    collision_detection::Contact adjustContactPoint(const collision_detection::Contact& contact_point);

    /** \brief Determine the corner point that is nearest to the computed collision point between
     * the DLO and the obstacle.
     */
    Eigen::Vector3d determineNearestCornerPoint(const collision_detection::Contact& contact_point,
                                                const std::map<std::string,Eigen::Vector3d>& cornerPoints);
    
    

    // Publisher and MArkerArray for visualization of contact points
    ros::Publisher* g_marker_array_publisher; // publishes the collision point markers
    visualization_msgs::MarkerArray g_collision_points; // contains the collision point markers

    rviz_visual_tools::RvizVisualToolsPtr visual_tools; // publish the line marker
    bool isObjectDLO; // true if visualization is supposed to be a geometry_msgs::CollisionObject
                      // false if it's supposed to be a line marker

    std::map<std::string, Eigen::Vector3d> corner_points; // The corner points of the obstacle


};
}