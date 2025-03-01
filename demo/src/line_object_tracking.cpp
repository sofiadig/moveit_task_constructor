#include </home/sdg/ws_moveit/src/moveit_task_constructor/demo/include/moveit_task_constructor_demo/line_object_tracking.h>
// Path: /home/sdg/ws_moveit/src/moveit_task_constructor/demo/include/moveit_task_constructor_demo/line_object_tracking.h
// Relative Path: src/moveit_task_constructor/demo/include/moveit_task_constructor_demo/line_object_tracking.h
//  /home/kifabrik/ws_moveit/src/moveit_task_constructor/demo/include/moveit_task_constructor_demo/line_object_tracking.h


namespace moveit_task_constructor_demo {

//g_marker_array_publisher = nullptr;
// visualization_msgs::MarkerArray g_collision_points;

ObjectTracker::ObjectTracker() {
        // Constructor implementation
}

void ObjectTracker::initObject(const geometry_msgs::PoseStamped& gripper_pose,
                                const geometry_msgs::PoseStamped& gripper_2_pose,
                                moveit_msgs::CollisionObject& collision_object,
                                moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    // Define the start point (e.g., robot base)
    geometry_msgs::Point start_point = gripper_2_pose.pose.position;
    // start_point.x = 0.0;
    // start_point.y = 0.0;
    // start_point.z = 1.1;

    // Define the end point as the gripper position
    geometry_msgs::Point end_point = gripper_pose.pose.position;

    // Compute the length of the object
    double length = sqrt(pow(end_point.x - start_point.x, 2) +
                         pow(end_point.y - start_point.y, 2) +
                         pow(end_point.z - start_point.z, 2));

    // Define the primitive and its dimensions (cylinder)
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = length; // height (length of the object)
    primitive.dimensions[1] = 0.005; // radius

    // Define the pose of the cylinder (midpoint between start and end points)
    geometry_msgs::Pose cylinder_pose;
    cylinder_pose.position.x = (start_point.x + end_point.x) / 2.0;
    cylinder_pose.position.y = (start_point.y + end_point.y) / 2.0;
    cylinder_pose.position.z = (start_point.z + end_point.z) / 2.0;

    // Compute the orientation to align the cylinder along the line
    Eigen::Vector3d p1(start_point.x, start_point.y, start_point.z);
    Eigen::Vector3d p2(end_point.x, end_point.y, end_point.z);
    Eigen::Vector3d axis = (p2 - p1).normalized();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), axis);
    cylinder_pose.orientation.w = orientation.w();
    cylinder_pose.orientation.x = orientation.x();
    cylinder_pose.orientation.y = orientation.y();
    cylinder_pose.orientation.z = orientation.z();

    // Add the primitive and pose to the collision object
    //collision_object.pose = cylinder_pose; // Change this when updating to multiple primitives! like = primitive_poses.peek() or omething
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = moveit_msgs::CollisionObject::ADD;

    // Apply the collision object to the planning scene
    planning_scene_interface.applyCollisionObject(collision_object);
}

void ObjectTracker::updateObject(const geometry_msgs::PoseStamped& gripper_pose,
                                    const geometry_msgs::PoseStamped& gripper_2_pose,
                                    moveit_msgs::CollisionObject& collision_object,
                                    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    // Define the start point (e.g., robot base)
    geometry_msgs::Point start_point = gripper_2_pose.pose.position;

    // Define the end point as the gripper position
    geometry_msgs::Point end_point = gripper_pose.pose.position;

    // Compute the length of the object
    double length = sqrt(pow(end_point.x - start_point.x, 2) +
                         pow(end_point.y - start_point.y, 2) +
                         pow(end_point.z - start_point.z, 2));

    collision_object.primitives[0].dimensions[0] = length;
    // Define the primitive and its dimensions (cylinder)
    // shape_msgs::SolidPrimitive primitive;
    // primitive.type = primitive.CYLINDER;
    // primitive.dimensions.resize(2);
    // primitive.dimensions[0] = length; // height (length of the object)
    // primitive.dimensions[1] = 0.005; // radius

    // Define the pose of the cylinder (midpoint between start and end points)
    //geometry_msgs::Pose cylinder_pose;
    collision_object.primitive_poses[0].position.x = (start_point.x + end_point.x) / 2.0;
    collision_object.primitive_poses[0].position.y = (start_point.y + end_point.y) / 2.0;
    collision_object.primitive_poses[0].position.z = (start_point.z + end_point.z) / 2.0;

    // Compute the orientation to align the cylinder along the line
    Eigen::Vector3d p1(start_point.x, start_point.y, start_point.z);
    Eigen::Vector3d p2(end_point.x, end_point.y, end_point.z);
    Eigen::Vector3d axis = (p2 - p1).normalized();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), axis);
    collision_object.primitive_poses[0].orientation.w = orientation.w();
    collision_object.primitive_poses[0].orientation.x = orientation.x();
    collision_object.primitive_poses[0].orientation.y = orientation.y();
    collision_object.primitive_poses[0].orientation.z = orientation.z();

    //collision_object.pose = collision_object.primitive_poses[0];

    // Add the primitive and pose to the collision object
    //collision_object.primitives.push_back(primitive);
    //collision_object.primitive_poses.push_back(cylinder_pose);
    //collision_object.operation = moveit_msgs::CollisionObject::ADD;

    //allowCollisions("panda_1_hand", collision_object.id);

    // Apply the collision object to the planning scene
    planning_scene_interface.applyCollisionObject(collision_object);
}

void ObjectTracker::createPillarObject(moveit_msgs::CollisionObject& object,
                                      moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
	  ROS_INFO_STREAM("We are in the createPillarObject function.");
    geometry_msgs::Pose pose;
    //moveit_msgs::CollisionObject object;
    object.id = "pillar";
    object.header.frame_id = "world";
    ROS_INFO_STREAM("Now let's edit the primitive.");
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    object.primitives[0].dimensions.resize(3);
    object.primitives[0].dimensions[0] = 0.1;
    object.primitives[0].dimensions[1] = 0.1;
    object.primitives[0].dimensions[2] = 0.6;

    // shape_msgs::SolidPrimitive primitive;
    // primitive.type = primitive.BOX;
    // primitive.dimensions.resize(2);
    // primitive.dimensions[0] = length; // height (length of the object)
    // primitive.dimensions[1] = 0.005; // radius
    ROS_INFO_STREAM("Now let's edit the pose.");
    pose.position.x = 0.5; pose.position.y = 0.0; pose.position.z = 1.2;
    pose.orientation.x = 0.0; pose.orientation.y = 0.0; pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    pose.position.z += 0.5 * object.primitives[0].dimensions[0];
    ROS_INFO_STREAM("Finishing touches.");
    object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::CollisionObject::ADD;

    ROS_INFO_STREAM("Now let's apply it to the planning scene via the psi.");
    planning_scene_interface.applyCollisionObject(object);
    ROS_INFO_STREAM("DONE.");
}



// Function to convert Eigen::Isometry3d to geometry_msgs::Pose
geometry_msgs::Pose ObjectTracker::isometryToPose(const Eigen::Isometry3d& transform) {
    geometry_msgs::Pose pose;
    
    // Set the position
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();

    // Set the orientation
    Eigen::Quaterniond quat(transform.rotation());
    pose.orientation.w = quat.w();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    
    return pose;
}

// Function to convert Eigen::Isometry3d to geometry_msgs::PoseStamped
geometry_msgs::PoseStamped ObjectTracker::isometryToPoseStamped(const Eigen::Isometry3d& transform, const std::string& frame_id) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose = isometryToPose(transform);
    
    return pose_stamped;
}

void ObjectTracker::publishMarkers(visualization_msgs::MarkerArray& markers)
{
  // delete old markers
  if (!g_collision_points.markers.empty())
  {
    for (auto& marker : g_collision_points.markers)
      marker.action = visualization_msgs::Marker::DELETE;

    g_marker_array_publisher->publish(g_collision_points);
  }

  // move new markers into g_collision_points
  std::swap(g_collision_points.markers, markers.markers);

  // draw new markers (if there are any)
  if (!g_collision_points.markers.empty())
    g_marker_array_publisher->publish(g_collision_points);
}

void ObjectTracker::computeCollisionContactPoints(planning_scene::PlanningScenePtr& planning_scene,
                                                std::vector<std::string> object_group1,
                                                std::vector<std::string> object_group2,
                                                //moveit_msgs::CollisionObject& object,
                                                //const geometry_msgs::TransformStamped& transformStamped,
                                                robot_state::RobotStatePtr& robot) {

  collision_detection::CollisionRequest c_req;
  collision_detection::CollisionResult c_res;
  //collision_detection::CollisionResult c_res_robot;
  //c_req.group_name = "dual_arm";
  c_req.contacts = true;
  c_req.max_contacts = 100;
  c_req.max_contacts_per_pair = 5;
  c_req.verbose = false;

  // ------------------ Checking for Collisions ------------------
  //planning_scene->checkCollision(c_req, c_res, *robot);//, object, transformStamped);//, *robot);
  
  
  
  c_res = planning_scene->getCollisionEnv()->checkCollisionBetweenObjectGroups(object_group1, object_group2);
  
  
  
  // ------------ Displaying Collision Contact Points ------------
  // If there are collisions, we get the contact points and display them as markers. **getCollisionMarkersFromContacts()** is a helper function that adds the
  // collision contact points into a MarkerArray message. If you want to use the contact points for something other than displaying them you can
  // iterate through **c_res.contacts** which is a std::map of contact points. Look at the implementation of getCollisionMarkersFromContacts() in `collision_tools.cpp
  // <https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_core/collision_detection/src/collision_tools.cpp>`_ for how.
  if (c_res.collision)
  {
    ROS_INFO_STREAM("COLLIDING contact_point_count: " << c_res.contact_count);
    if (c_res.contact_count > 0)
    {
      std_msgs::ColorRGBA color;
      color.r = 1.0;
      color.g = 0.0;
      color.b = 1.0;
      color.a = 0.5;
      visualization_msgs::MarkerArray markers;

      /* Get the contact points and display them as markers */
      collision_detection::getCollisionMarkersFromContacts(markers, "panda_link0", c_res.contacts, color,
                                                           ros::Duration(),  // remain until deleted
                                                           0.01);            // radius
      publishMarkers(markers);
    }
  }
  // END_SUB_TUTORIAL
  else
  {
    ROS_INFO("Not colliding");// <<<<<<<<<<<<<<<<<---------------------------------UNCOMMENT HERE------------------------------------------------------------------------

    // delete the old collision point markers
    visualization_msgs::MarkerArray empty_marker_array;
    publishMarkers(empty_marker_array);
  }
}


void ObjectTracker::updateTransform(const geometry_msgs::Pose& pose, const std::string& parent_frame_id, const std::string& child_frame_id,
                                    geometry_msgs::TransformStamped& transformStamped) {
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = parent_frame_id;
    transformStamped.child_frame_id = child_frame_id;

    // Set translation from the pose
    transformStamped.transform.translation.x = pose.position.x;
    transformStamped.transform.translation.y = pose.position.y;
    transformStamped.transform.translation.z = pose.position.z;

    // Set tf2 quaternion from given pose
    tf2::Quaternion quat(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    );
    //quaternion.normalize();
    // Apply quaternion to transform rotation
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    // Broadcast the transform
    //br.sendTransform(transformStamped);
}

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_object_tracker");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Set up MoveIt interfaces
    moveit::planning_interface::PlanningSceneInterface psi;
    moveit::planning_interface::MoveGroupInterface move_group_interface("dual_arm");
    moveit_task_constructor_demo::ObjectTracker* objectTracker = new moveit_task_constructor_demo::ObjectTracker();
    geometry_msgs::TransformStamped transformStamped;

    // tf2_ros::TransformBroadcaster br;
    // tf2_ros::Buffer tf_buffer_;
    // tf2_ros::TransformListener tf_listener_(tf_buffer_);

// ##################################### Initialize object & its position ############################################
    // Create a new object
    moveit_msgs::CollisionObject dynamic_object;
    dynamic_object.id = "dynamic_object";
    dynamic_object.header.frame_id = "world";

    moveit_msgs::CollisionObject pillar;
    pillar.id = "pillar";
    pillar.header.frame_id = "world";
    //dynamic_object.type =  shape_msgs::SolidPrimitive::CYLINDER; // object.type is different from primitive_type!
    //std::string dynamic_object_frame_id = "dynamic_object_frame";

    // Get the current state of the robot
    robot_state::RobotStatePtr current_state = move_group_interface.getCurrentState();

    // Use tip of hand instead of hand base
    Eigen::Isometry3d tip_pose_in_hand_frame = Eigen::Isometry3d::Identity();
    tip_pose_in_hand_frame.translation().z() = 0.116; // Franka TCP configuration // original: 0.1034
    const Eigen::Isometry3d& gripper_tip_iso = current_state->getGlobalLinkTransform("panda_1_hand") * tip_pose_in_hand_frame;
    geometry_msgs::PoseStamped gripper_tip_pose = objectTracker->isometryToPoseStamped(gripper_tip_iso, "world");

    const Eigen::Isometry3d& gripper_tip_2_iso = current_state->getGlobalLinkTransform("panda_2_hand") * tip_pose_in_hand_frame;
    geometry_msgs::PoseStamped gripper_tip_2_pose = objectTracker->isometryToPoseStamped(gripper_tip_2_iso, "world");

    
    objectTracker->initObject(gripper_tip_pose, gripper_tip_2_pose, dynamic_object, psi);
    objectTracker->createPillarObject(pillar, psi);
    // ----------------------------objectTracker->updateTransform(dynamic_object.primitive_poses[0], "world", "dynamic_object_frame", transformStamped);
// #####################################################################################################################

    std::string dynamic_object_id = "dynamic_object";
    std::string pillar_id = "pillar";

    // Initialize object groups for collision detection
    std::vector<std::string> object_group1;
    std::vector<std::string> object_group2;
    object_group1.push_back(dynamic_object_id);
    object_group2.push_back(pillar_id);


// ===================================== Planning Scene ======================================================

    // Create a planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
        std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor();
    planning_scene_monitor->startStateMonitor();

    // Create a PlanningScene
    planning_scene_monitor->requestPlanningSceneState();
    ros::Duration(1.0).sleep();

    planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor->getPlanningScene();
    // Update Planning Scene and check for collisons
    //if (moveit_task_constructor_demo::updatePlanningScene(planning_scene, nh)) {
        // Now you can use the planning_scene object as needed

// ===========================================================================================================
    ros::Duration(1.0).sleep();
    // First collision check
    objectTracker->computeCollisionContactPoints(planning_scene, object_group1, object_group2, current_state);


    while (ros::ok()) {
        //collision_result.clear();  // Clear previous results

        // Get the current state of the robot & current planning scene
        robot_state::RobotStatePtr current_state = move_group_interface.getCurrentState();
        planning_scene = planning_scene_monitor->getPlanningScene();

        const Eigen::Isometry3d& gripper_tip_iso = current_state->getGlobalLinkTransform("panda_1_hand") * tip_pose_in_hand_frame;
        geometry_msgs::PoseStamped gripper_tip_pose = objectTracker->isometryToPoseStamped(gripper_tip_iso, "world");
        const Eigen::Isometry3d& gripper_tip_2_iso = current_state->getGlobalLinkTransform("panda_2_hand") * tip_pose_in_hand_frame;
        geometry_msgs::PoseStamped gripper_tip_2_pose = objectTracker->isometryToPoseStamped(gripper_tip_2_iso, "world");

        // Update the dynamic object
        objectTracker->updateObject(gripper_tip_pose, gripper_tip_2_pose, dynamic_object, psi);
        // Broadcast the transform of object frame to world frame
        //objectTracker->updateTransform(dynamic_object.primitive_poses[0], "world", "dynamic_object_frame", transformStamped);
        // ROS_INFO_STREAM("Transform: Translation (" << transformStamped.transform.translation.x << ", "
        //                                             << transformStamped.transform.translation.y << ", "
        //                                             << transformStamped.transform.translation.z << ")");
        // ROS_INFO_STREAM("Transform: Rotation (" << transformStamped.transform.rotation.x << ", "
        //                                             << transformStamped.transform.rotation.y << ", "
        //                                             << transformStamped.transform.rotation.z << ", "
        //                                             << transformStamped.transform.rotation.w << ")");
        // br.sendTransform(transformStamped);

        
        // ====================================== Check for collisions =================================================
        
        //if (moveit_task_constructor_demo::updatePlanningScene(planning_scene, nh)) {

        // Check collision for attached object
        objectTracker->computeCollisionContactPoints(planning_scene, object_group1, object_group2, current_state);

        // Now you can use the planning_scene object as needed
        // collision_detection::CollisionRequest collision_request;
        // collision_detection::CollisionResult collision_result;
        // planning_scene->checkCollision(collision_request, collision_result, dynamic_object, transformStamped);

        // if (collision_result.collision){ ROS_INFO("Collision detected!"); }
        // else { ROS_INFO("No collision detected."); }
        //}
        //else { ROS_ERROR("Failed to call service get_planning_scene");}

        // =============================================================================================================

        ros::Duration(0.1).sleep();

    }

    return 0;
}

