#include <string>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/task_constructor/stage.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/task_constructor/marker_tools.h>

#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>

using namespace moveit::task_constructor;

Eigen::Isometry3d poseToIsometry(const geometry_msgs::Pose& pose_msg) {
    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
    
    // Translation
    isometry.translation() << pose_msg.position.x, pose_msg.position.y, pose_msg.position.z;
    
    // Rotation
    Eigen::Quaterniond quat(pose_msg.orientation.w, pose_msg.orientation.x,
                            pose_msg.orientation.y, pose_msg.orientation.z);
    isometry.linear() = quat.toRotationMatrix();
    
    return isometry;
}

void setPose(geometry_msgs::Pose& pose, double px, double py, double pz, double ox, double oy, double oz, double ow) {
    pose.position.x = px;
    pose.position.y = py;
    pose.position.z = pz;
    pose.orientation.x = ox;
    pose.orientation.y = oy;
    pose.orientation.z = oz;
    pose.orientation.w = ow;
}


// void computeCollisionContactPoints(InteractiveRobot& robot)
// {
//   Eigen::Isometry3d world_cube_pose;
//   double world_cube_size;
//   robot.getWorldGeometry(world_cube_pose, world_cube_size);
//   g_planning_scene->getWorldNonConst()->moveShapeInObject("world_cube", g_world_cube_shape, world_cube_pose);

//   std::vector<std::string> object_group1;
//   std::vector<std::string> object_group2;
//   object_group1.push_back("world_cube");
//   object_group2.push_back("cylinder");

//   // Collision Requests
//   collision_detection::CollisionRequest c_req;
//   collision_detection::CollisionResult c_res_robot;
//   collision_detection::CollisionResult c_res;
//   c_req.group_name = robot.getGroupName();
//   c_req.contacts = true;
//   c_req.max_contacts = 100;
//   c_req.max_contacts_per_pair = 5;
//   c_req.verbose = false;

//   // Checking for Collisions
//   g_planning_scene->checkCollision(c_req, c_res_robot, *robot.robotState());
//   c_res = g_planning_scene->getCollisionEnv()->checkCollisionBetweenObjectGroups(object_group1, object_group2);

//   if (c_res.collision)
//   {
//     ROS_INFO_STREAM("COLLIDING contact_point_count: " << c_res.contact_count);
//     if (c_res.contact_count > 0)
//     {
//       std_msgs::ColorRGBA color;
//       color.r = 1.0;
//       color.g = 0.0;
//       color.b = 1.0;
//       color.a = 0.5;
//       visualization_msgs::MarkerArray markers;

//       /* Get the contact points and display them as markers */
//       collision_detection::getCollisionMarkersFromContacts(markers, "panda_link0", c_res.contacts, color,
//                                                            ros::Duration(),  // remain until deleted
//                                                            0.01);            // radius
//       publishMarkers(markers);
//     }
//   }

//   else
//   {
//     ROS_INFO("Not colliding");

//     // delete the old collision point markers
//     visualization_msgs::MarkerArray empty_marker_array;
//     publishMarkers(empty_marker_array);
//   }
// }

int main(int argc, char** argv) {

    ros::init(argc, argv, "line_collision_tracker");
	ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface psi;
    moveit::planning_interface::MoveGroupInterface move_group_interface("dual_arm");

    // initialize rviz tool
    rviz_visual_tools::RvizVisualToolsPtr visual_tools(new rviz_visual_tools::RvizVisualTools("world", "/interactive_robot_marray"));
    visual_tools->loadMarkerPub();

    // Create a publisher for the line marker
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    // ros::Subscriber gripper_tip_sub = nh.subscribe("gripper_tip_position", 10, gripperTipPositionCallback);

    // Initialize marker properties
    // Eigen::Vector3d line_marker_start;
    // Eigen::Vector3d line_marker_end;
    std::vector<Eigen::Vector3d> line_starts;
    std::vector<Eigen::Vector3d> line_ends; 
    Eigen::Vector3d gripper_tip_position;
    rviz_visual_tools::colors line_marker_color = rviz_visual_tools::colors::RED;
    rviz_visual_tools::scales line_marker_scale = rviz_visual_tools::scales::MEDIUM;

    //std::vector<std::string> bend_points = {"bend1", "bend2"};

    geometry_msgs::Pose bend1; // On the base, between the robots
    geometry_msgs::Pose bend2; // And edge of the obstacle

    setPose(bend1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    setPose(bend2, 0.315, 0.2385, 1.485, 0.0, 0.0, 0.0, 1.0);
    
    //std::map<std::string, geometry_msgs::Pose> bend_poses = { {"bend1", bend1}, {"bend2", bend2} };
    Eigen::Isometry3d line_start_pose = poseToIsometry(bend1);
    line_starts.push_back(line_start_pose.translation());
    Eigen::Isometry3d line_ends_pose = poseToIsometry(bend2);
    line_ends.push_back(line_ends_pose.translation());

    line_start_pose = poseToIsometry(bend2);
    line_starts.push_back(line_start_pose.translation());
    // place holder
    line_ends.push_back(line_start_pose.translation());


    // Get the global transform of a specific link with respect to the world frame
    Eigen::Isometry3d tip_pose_in_hand = Eigen::Isometry3d::Identity();
    tip_pose_in_hand.translation().z() = 0.1034; // Franka TCP configuration
    // -------------------------------------------------------------
    
    // Main loop to continuously update the line marker
    while (ros::ok()) {
        // Get the current state of the robot
        robot_state::RobotStatePtr current_state = move_group_interface.getCurrentState();

        const Eigen::Isometry3d& gripper_1_tip_pose = current_state->getGlobalLinkTransform("panda_1_hand") * tip_pose_in_hand;
        // line_marker_end = gripper_tip_pose.translation();
        line_ends[0] = gripper_1_tip_pose.translation();

        const Eigen::Isometry3d& gripper_2_tip_pose = current_state->getGlobalLinkTransform("panda_2_hand") * tip_pose_in_hand;
        // // line_marker_end = gripper_tip_pose.translation();
        line_starts[0] = gripper_2_tip_pose.translation();
        //line_ends[0] = gripper_2_tip_pose.translation();


        // Extract the position components
        // gripper_tip_position = gripper_tip_pose.translation();
        // ROS_INFO_STREAM("Gripper tip position: " << gripper_tip_position.x() << ", "
        //                                             << gripper_tip_position.y() << ", "
        //                                             << gripper_tip_position.z());

        // bool publishing = visual_tools->publishLine(line_marker_start, line_marker_end, line_marker_color, line_marker_scale, 1);
        for (size_t i = 0; i < line_starts.size(); ++i) {
            visual_tools->publishLine(line_starts[i], line_ends[i], line_marker_color, line_marker_scale);//, i); // Pass 'i' as the ID
        }
        visual_tools->trigger();

        // Optionally, add a delay to control the update frequency
        ros::Duration(0.1).sleep();
    }

    return 0;
}