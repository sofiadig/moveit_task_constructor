//###########################################################################################################################
//################################################## Initialization #########################################################
//###########################################################################################################################

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <string>
#include <map>

enum GripperState {GRIPPER_OPEN, GRIPPER_CLOSE};
enum ObjectAction { ATTACH, DETACH };
// The hand over position w.r.t frame1. w.r.t frame2: x1=x2 and z1=z2, but for y we know the distance of 2 robots is 0.9 so
// y2 = y1 - 0.9 (negative: the ho position is between the robots, the movement of robot2 in y is opposite the one for robot1)
float ho_x = -0.15, ho_y = 0.5, ho_z = 0.5;
float obj_x = 0.65, obj_y = 0.0, obj_z = 0.5; // The position of the object.
float place_x = 0.55, place_y = 0.0, place_z = 0.5; // The destination position.
const double tau = 2 * M_PI; // tau = 2*pi. One tau is one rotation in radians. So one tau is one complete tyrn or rotation.

//###########################################################################################################################
//################################################# Gripper Operations ######################################################
//###########################################################################################################################

void gripperState(moveit::planning_interface::MoveGroupInterface& move_group, const std::string& robot_prefix, GripperState state) {
    double position = (state == GRIPPER_OPEN) ? 0.04 : 0.011; // Open or close position
    std::map<std::string, double> target;
    target[robot_prefix + "_finger_joint1"] = position;
    target[robot_prefix + "_finger_joint2"] = position;
    move_group.setJointValueTarget(target);
    move_group.move();
}

//###########################################################################################################################
//####################################################### move arm ##########################################################
//###########################################################################################################################

bool moveArm(const std::string& move_group_name,
             const std::string& reference_frame,
             const float& pos_x, const float& pos_y, const float& pos_z,
             const double& ori_roll, const double& ori_pitch, const double& ori_yaw) {
    // Initialize the move group
    moveit::planning_interface::MoveGroupInterface move_group(move_group_name);

    // Set the planning time and reference frame for the move group
    move_group.setPlanningTime(10.0);
    move_group.setPoseReferenceFrame(reference_frame);

    // Define the target pose
    geometry_msgs::Pose target_pose;
    target_pose.position.x = pos_x;
    target_pose.position.y = pos_y;
    target_pose.position.z = pos_z;

    // Convert RPY to Quaternion for the target pose orientation
    tf2::Quaternion orientation;
    orientation.setRPY(ori_roll, ori_pitch, ori_yaw);
    target_pose.orientation = tf2::toMsg(orientation);

    // Set the pose target and move the robot arm
    move_group.setPoseTarget(target_pose);
    moveit::core::MoveItErrorCode result = move_group.move(); // Updated line

    // Check if the movement was successful
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_INFO_STREAM("Arm " << move_group_name << " successfully moved to target.");
        return true;
    } else {
        ROS_ERROR_STREAM("Failed to move arm " << move_group_name << " with error code: " << result.val);
        return false;
    }
}

//###########################################################################################################################
//#################################################### attach and detach ####################################################
//###########################################################################################################################

void handleObject(moveit::planning_interface::MoveGroupInterface& move_group,
                  const std::string& object_id,
                  const std::string& link_name,
                  const std::vector<std::string>& touch_links,
                  ObjectAction action) {
    if (action == ATTACH) {
        move_group.attachObject(object_id, link_name, touch_links);
        ROS_INFO_STREAM("Object attached to " << link_name);
    } else if (action == DETACH) {
        move_group.detachObject(object_id);
        ROS_INFO_STREAM("Object detached from " << link_name);
    }
}

//###########################################################################################################################
//#################################################### Collision objects ####################################################
//###########################################################################################################################

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

  // // Table1, cylinder shape:
  //   collision_objects[0].id = "table1";
  //   collision_objects[0].header.frame_id = "panda_1_link0";
  //   // Shape and size
  //   collision_objects[0].primitives.resize(1);
  //   collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
  //   collision_objects[0].primitives[0].dimensions.resize(2);
  //   collision_objects[0].primitives[0].dimensions[0] = 0.4;  // Height
  //   collision_objects[0].primitives[0].dimensions[1] = 0.15;  // Radius
  //   // Position and orientation
  //   collision_objects[0].primitive_poses.resize(1);
  //   collision_objects[0].primitive_poses[0].position.x = 0.7;
  //   collision_objects[0].primitive_poses[0].position.y = 0;
  //   collision_objects[0].primitive_poses[0].position.z = 0.2;
  //   collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  //   collision_objects[0].operation = collision_objects[0].ADD;

  // // Table2, cubic shape:
  //   collision_objects[1].id = "table2";
  //   collision_objects[1].header.frame_id = "panda_2_link0";
  //   // Shape and size
  //   collision_objects[1].primitives.resize(1);
  //   collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  //   collision_objects[1].primitives[0].dimensions.resize(3);
  //   collision_objects[1].primitives[0].dimensions[0] = 0.2;
  //   collision_objects[1].primitives[0].dimensions[1] = 0.2;
  //   collision_objects[1].primitives[0].dimensions[2] = 0.4;
  //   // Position and orientation
  //   collision_objects[1].primitive_poses.resize(1);
  //   collision_objects[1].primitive_poses[0].position.x = 0.6;
  //   collision_objects[1].primitive_poses[0].position.y = 0.0;
  //   collision_objects[1].primitive_poses[0].position.z = 0.2;
  //   collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  //   collision_objects[1].operation = collision_objects[1].ADD;

  // Object, cubic box
    collision_objects[2].header.frame_id = "panda_1_link0";
    collision_objects[2].id = "object";
    // Shape and size
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.02;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;
    collision_objects[2].primitives[0].dimensions[2] = 0.2;
    // Position and orientation
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = obj_x;
    collision_objects[2].primitive_poses[0].position.y = obj_y;
    collision_objects[2].primitive_poses[0].position.z = obj_z;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

//###########################################################################################################################
//########################################################## main ###########################################################
//###########################################################################################################################

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_arm_to_pose_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;
  
  std::vector<std::string> touch_links_1 = {"panda_1_finger_link1", "panda_1_finger_link2"};
  std::vector<std::string> touch_links_2 = {"panda_2_finger_link1", "panda_2_finger_link2"};

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  addCollisionObjects(planning_scene_interface);

  moveit::planning_interface::MoveGroupInterface move_group_1("hand_1");
  moveit::planning_interface::MoveGroupInterface move_group_2("hand_2");

  // moveArm("panda_1", "panda_1_link0", obj_x - 0.2, obj_y, obj_z + 0.05, 0, M_PI / 2, 0); // pre_pick1

  gripperState(move_group_1, "panda_1", GRIPPER_OPEN);
  ros::Duration(1.5).sleep(); // Sleep to ensure the gripper has time to open
  moveArm("panda_1", "panda_1_link0", obj_x-0.1, obj_y, obj_z+0.05, 0, M_PI / 2, 0); // pick1
  gripperState(move_group_1, "panda_1", GRIPPER_CLOSE);
  handleObject(move_group_1, "object", "panda_1_link8", touch_links_1, ATTACH);
  ros::Duration(1.5).sleep(); // Sleep to ensure the gripper has time to close

  // moveArm("panda_1", "panda_1_link0", obj_x-0.1, obj_y, obj_z+0.1, 0, M_PI / 2, 0); // post_pick1
  moveArm("panda_1", "panda_1_link0", ho_x, ho_y-0.1, ho_z, -M_PI / 2, 0, 0); // pre_place1
  moveArm("panda_1", "panda_1_link0", ho_x, ho_y, ho_z, -M_PI / 2, 0, 0); // place1
  moveArm("panda_2", "panda_2_link0", ho_x+0.05, ho_y-0.7, ho_z, M_PI/2, -M_PI, 0); // pre_pick2

  gripperState(move_group_2, "panda_2", GRIPPER_OPEN);
  ros::Duration(1.5).sleep(); // Sleep to ensure the gripper has time to open
  moveArm("panda_2", "panda_2_link0", ho_x+0.05, ho_y-0.8, ho_z, M_PI/2, -M_PI, 0); // pick2
  handleObject(move_group_1, "object", "", {}, DETACH);
  gripperState(move_group_2, "panda_2", GRIPPER_CLOSE);
  handleObject(move_group_2, "object", "panda_2_link8", touch_links_2, ATTACH);
  ros::Duration(1.5).sleep(); // Sleep to ensure the gripper has time to close

  gripperState(move_group_1, "panda_1", GRIPPER_OPEN);
  ros::Duration(1.5).sleep(); // Sleep to ensure the gripper has time to open
  moveArm("panda_1", "panda_1_link0", ho_x, ho_y-0.1, ho_z, -M_PI / 2, 0, 0); // post_place1

  // moveArm("panda_2", "panda_2_link0", ho_x+0.1, ho_y-0.7, ho_z, M_PI/2, -M_PI, 0); // post_pick2
  // moveArm("panda_2", "panda_2_link0", place_x, place_y, place_z+0.3, 0, M_PI/2, 0); // pre_place2
  moveArm("panda_2", "panda_2_link0", place_x, place_y, place_z, 0, M_PI/2, 0); // place2

  gripperState(move_group_2, "panda_2", GRIPPER_OPEN);
  ros::Duration(1.5).sleep(); // Sleep to ensure the gripper has time to open
  handleObject(move_group_2, "object", "", {}, DETACH);
  moveArm("panda_2", "panda_2_link0", place_x, place_y, place_z+0.1, 0, M_PI/2, 0); // post_place2
  
  ros::waitForShutdown();
  return 0;
}