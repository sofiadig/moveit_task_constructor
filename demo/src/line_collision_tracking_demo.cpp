// ROS
#include <ros/ros.h>
#include </home/sdg/ws_moveit/src/moveit_task_constructor/demo/include/moveit_task_constructor_demo/line_collision_tracking.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_object_tracker");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    using Contacts = std::vector<collision_detection::Contact>;

    // Initialize some necessary objects
    moveit::planning_interface::MoveGroupInterface move_group_interface("dual_arm"); // To keep track of the robot state
    moveit_task_constructor_demo::ObjectCollisionTracker* objectCollisionTracker = new moveit_task_constructor_demo::ObjectCollisionTracker(); // To use its functions
    objectCollisionTracker->g_marker_array_publisher = new ros::Publisher(nh.advertise<visualization_msgs::MarkerArray>("interactive_robot_marray", 100)); // To publish the contact point markers


    // ============================================= Planning Scene =============================================

    // Create a planning scene interface for adding the obstacle and object visualization to the planning scene
    moveit::planning_interface::PlanningSceneInterface psi;

    // Create a planning scene monitor for accessing the planning scene and updating the object for collision checking
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor(); // Include sensor updates
    planning_scene_monitor->startStateMonitor();


    // ======================================== Initialize scene objects ========================================

    // Create & initialize pillar then add i to the planning scene
    moveit_msgs::CollisionObject pillar = objectCollisionTracker->createSimpleObst(nh);
    psi.applyCollisionObject(pillar); // Add pillar using planning scene interface

    // Create a new DLO object
    moveit_msgs::CollisionObject dynamic_object;
    dynamic_object.id = "dynamic_object";
    dynamic_object.header.frame_id = "world";

    // Initialize object groups for collision detection
    std::vector<std::string> object_group1;
    std::vector<std::string> object_group2;
    object_group1.push_back(dynamic_object.id);
    object_group2.push_back(pillar.id);

    ros::Duration(1.0).sleep(); // Wait for planning scene to be updated
    planning_scene_monitor->requestPlanningSceneState();
    planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor->getPlanningScene();


    // ============================================= Set object pose ============================================
    
    robot_state::RobotStatePtr current_state = move_group_interface.getCurrentState(); // Get the current state of the robot
    Eigen::Isometry3d tip_pose_in_hand_frame = Eigen::Isometry3d::Identity(); // Use tip of hand instead of hand base
    tip_pose_in_hand_frame.translation().z() = 0.116; // Franka TCP configuration. To be in between the "fingers": 0.1034, but this causes collision before we can change the allowCollisionsMatrix
    
    Eigen::Isometry3d moving_hand_tip_iso = current_state->getGlobalLinkTransform("panda_1_hand") * tip_pose_in_hand_frame; // Gripper tip pose of panda_1
    geometry_msgs::PoseStamped moving_hand_tip_pose = objectCollisionTracker->isometryToPoseStamped(moving_hand_tip_iso, "world");
    
    Eigen::Isometry3d steady_hand_tip_iso = current_state->getGlobalLinkTransform("panda_2_hand") * tip_pose_in_hand_frame; // Gripper tip pose of panda_2
    geometry_msgs::PoseStamped steady_hand_tip_pose = objectCollisionTracker->isometryToPoseStamped(steady_hand_tip_iso, "world");


    // ==================== Add DLO object as moveit_msgs::CollisionObject and World::Object ====================

    objectCollisionTracker->initObject(steady_hand_tip_pose, moving_hand_tip_pose, dynamic_object, psi); // For visualization in Rviz. initObject must come before updateObject!
    objectCollisionTracker->updateObject(steady_hand_tip_pose, moving_hand_tip_pose, dynamic_object, planning_scene, psi); // For collision checking
    
    ros::Duration(1.0).sleep(); // Wait period to make sure they are added


    // ===================== Check if pillar and DLO are both successfully added in scene  ======================

    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor);
    if (scene->getWorld()->hasObject("pillar")) {
        if(scene->getWorld()->hasObject("dynamic_object")) { 
            ROS_INFO_STREAM("YES - Objects 'pillar' and 'dynamic_object' are visible in PlanningSceneMonitor!"); }
        else { 
            ROS_INFO_STREAM("NO/YES - Only object 'pillar' is visible in PlanningSceneMonitor!"); }
    }
    else { ROS_INFO_STREAM("NO - None of the objects are visible in PlanningSceneMonitor!"); }


    // ====================== Declare variables for collision checking and contact points =======================

    objectCollisionTracker->corner_points = objectCollisionTracker->getCornerPoints(pillar);
    collision_detection::CollisionResult c_res;
    Contacts original_contacts;
    Contacts adjusted_contacts;
    bool isNewContact = false;
    int num_segments = 1;


    // ======================= Loop for updating the object pose and checking collisions ========================

    while (ros::ok()) {
        c_res.clear(); // Clear previous collision results

        // Get the current state of the robot & current planning scene
        planning_scene = planning_scene_monitor->getPlanningScene();
        robot_state::RobotStatePtr current_state = move_group_interface.getCurrentState();

        // Derive the gripper positions from the current robot state
        moving_hand_tip_iso  = current_state->getGlobalLinkTransform("panda_1_hand") * tip_pose_in_hand_frame;
        moving_hand_tip_pose = objectCollisionTracker->isometryToPoseStamped(moving_hand_tip_iso, "world");

        // Depending on if there are contact points: define either gripper or contact point as the start of the updated DLO section
        if(isNewContact) {
            if (adjusted_contacts.size() >= 2) { // If there is a new collision point in this iteration and we already have one before this new one
                                                 // then the starting point should be the previous contact point
                steady_hand_tip_pose = objectCollisionTracker->vectorToPoseStamped(adjusted_contacts[adjusted_contacts.size()-2].pos);
            }
            else { // If we just got the first contact point, keep the starting point as the gripper tip
                steady_hand_tip_iso  = current_state->getGlobalLinkTransform("panda_2_hand") * tip_pose_in_hand_frame;
                steady_hand_tip_pose = objectCollisionTracker->isometryToPoseStamped(steady_hand_tip_iso, "world");
            }
        }
        else {
            if (!adjusted_contacts.empty()) { // If we don't have a new contact, but have had some before, use the most current point as starting point
                steady_hand_tip_pose = objectCollisionTracker->vectorToPoseStamped(adjusted_contacts[adjusted_contacts.size()-1].pos);
            }
            else { // If we haven't had any contact points at all, use the gripper tip as starting point
                steady_hand_tip_iso  = current_state->getGlobalLinkTransform("panda_2_hand") * tip_pose_in_hand_frame;
                steady_hand_tip_pose = objectCollisionTracker->isometryToPoseStamped(steady_hand_tip_iso, "world");
            }
        }

        // Update the DLO according to the new starting pose, end pose and adjusted contact points
        objectCollisionTracker->updateDLO(steady_hand_tip_pose, moving_hand_tip_pose, dynamic_object, planning_scene,
                                            psi, adjusted_contacts, isNewContact, num_segments);

        // Check for collisions between the DLO and the obstacle
        objectCollisionTracker->computeCollisionContactPoints(planning_scene, object_group1, object_group2, c_res, original_contacts, adjusted_contacts, isNewContact);
        
        ros::Duration(0.1).sleep(); // 0.1 sec gap between iterations
    }
    return 0;
}