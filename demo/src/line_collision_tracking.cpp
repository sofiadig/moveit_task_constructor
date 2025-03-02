#include </home/sdg/ws_moveit/src/moveit_task_constructor/demo/include/moveit_task_constructor_demo/line_collision_tracking.h>

namespace moveit_task_constructor_demo {

ObjectCollisionTracker::ObjectCollisionTracker() {
    g_marker_array_publisher = nullptr;
    contactPointCount = 0;
}

void ObjectCollisionTracker::publishMarkers(visualization_msgs::MarkerArray& markers)
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
  if (!g_collision_points.markers.empty()) {
    g_marker_array_publisher->publish(g_collision_points);
  }
}

void ObjectCollisionTracker::initObject(const geometry_msgs::PoseStamped& gripper_pose,
                                const geometry_msgs::PoseStamped& gripper_2_pose,
                                moveit_msgs::CollisionObject& collision_object,
                                moveit::planning_interface::PlanningSceneInterface& planning_scene_interface ) {
    // Define the start point (e.g., robot base)
    geometry_msgs::Point start_point = gripper_2_pose.pose.position;

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

void ObjectCollisionTracker::updateObjectShape(const Eigen::Isometry3d& gripper_pose,
                                const Eigen::Isometry3d& gripper_2_pose,
                                std::string& object_id,
                                planning_scene::PlanningScenePtr planning_scene_ptr) {
    // Compute the size of the cylinder
    Eigen::Vector3d start_point = gripper_pose.translation();
    Eigen::Vector3d end_point = gripper_2_pose.translation();
    // Compute the length of the object
    double cylinder_length = sqrt(pow(end_point.x() - start_point.x(), 2) +
                                  pow(end_point.y() - start_point.y(), 2) +
                                  pow(end_point.z() - start_point.z(), 2));
    double cylinder_radius = 0.005;

    // Define the pose of the cylinder (midpoint between start and end points)
    Eigen::Isometry3d cylinder_pose;
    cylinder_pose.translation().x() = (start_point.x() + end_point.x()) / 2.0;
    cylinder_pose.translation().y() = (start_point.y() + end_point.y()) / 2.0;
    cylinder_pose.translation().z() = (start_point.z() + end_point.z()) / 2.0;

    // Compute the orientation to align the cylinder along the line
    //Eigen::Vector3d axis = (end_point - start_point).normalized();
    Eigen::Vector3d p1(start_point.x(), start_point.y(), start_point.z());
    Eigen::Vector3d p2(end_point.x(), end_point.y(), end_point.z());
    Eigen::Vector3d axis = (p2 - p1).normalized();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), axis);
    cylinder_pose.rotate(orientation);
    
    // Define the shape using shapes::Cylinder and add the object to the planning scene
    shapes::ShapePtr cylinder_shape(new shapes::Cylinder(cylinder_radius, cylinder_length));
    planning_scene_ptr->getWorldNonConst()->addToObject(object_id, cylinder_shape, cylinder_pose);
}

void ObjectCollisionTracker::updateObject(const geometry_msgs::PoseStamped& gripper_pose,
                                    const geometry_msgs::PoseStamped& gripper_2_pose,
                                    moveit_msgs::CollisionObject& collision_object,
                                    moveit::planning_interface::PlanningSceneInterface& psi ) {
    // Define the start point
    geometry_msgs::Point start_point = gripper_2_pose.pose.position;

    // Define the end point as the gripper position
    geometry_msgs::Point end_point = gripper_pose.pose.position;

    // Compute the length of the object
    double length = sqrt(pow(end_point.x - start_point.x, 2) + pow(end_point.y - start_point.y, 2) + pow(end_point.z - start_point.z, 2));

    // Define the primitive and its dimensions (cylinder)
    collision_object.primitives[0].dimensions[0] = length;

    // Define the pose of the cylinder (midpoint between start and end points)
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

    // Apply the collision object to the planning scene
    // Create a PlanningScene message
    // moveit_msgs::PlanningScene planning_scene_msg;
    // planning_scene_msg.is_diff = true;  // Set as a "diff" to modify only necessary parts
    // planning_scene_msg.world.collision_objects.push_back(collision_object);

    // Apply the updated scene
    psi.applyCollisionObject(collision_object);
}

void ObjectCollisionTracker::updateObjectShape2(const geometry_msgs::PoseStamped&  gripper_pose,
                                const geometry_msgs::PoseStamped&  gripper_2_pose,
                                std::string& object_id,
                                planning_scene::PlanningScenePtr planning_scene_ptr) {
    
    geometry_msgs::PoseStamped result_pose_msgs;
    Eigen::Isometry3d cylinder_pose;
    double cylinder_length;
    double cylinder_radius = 0.005;
    determinePose(gripper_pose, gripper_2_pose, result_pose_msgs, cylinder_pose, cylinder_length);

    shapes::ShapePtr cylinder_shape(new shapes::Cylinder(cylinder_radius, cylinder_length));
    planning_scene_ptr->getWorldNonConst()->addToObject(object_id, cylinder_shape, cylinder_pose);
}

void ObjectCollisionTracker::updateObject2(const geometry_msgs::PoseStamped& gripper_pose,
                                    const geometry_msgs::PoseStamped& gripper_2_pose,
                                    moveit_msgs::CollisionObject& collision_object,
                                    moveit::planning_interface::PlanningSceneInterface& psi ) {
    geometry_msgs::PoseStamped result_pose_msgs;
    Eigen::Isometry3d result_pose_iso;
    double length;
    determinePose(gripper_pose, gripper_2_pose, result_pose_msgs, result_pose_iso, length);
    collision_object.primitives[0].dimensions[0] = length;
    collision_object.primitives[0].dimensions[1] = 0.005;
    collision_object.primitive_poses[0] = result_pose_msgs.pose;

    // Apply the updated scene
    psi.applyCollisionObject(collision_object);
}

void ObjectCollisionTracker::determinePose(const geometry_msgs::PoseStamped& gripper_tip_pose,
                                            const geometry_msgs::PoseStamped& gripper_tip_2_pose,
                                            geometry_msgs::PoseStamped& result_pose_msgs,
                                            Eigen::Isometry3d& result_pose_iso,
                                            double& length) {
    // Define the start & end point
    geometry_msgs::Point start_point = gripper_tip_2_pose.pose.position;
    geometry_msgs::Point end_point = gripper_tip_pose.pose.position;

    length = sqrt(pow(end_point.x - start_point.x, 2) +
                pow(end_point.y - start_point.y, 2) +
                pow(end_point.z - start_point.z, 2));

    // Compute the orientation to align the cylinder along the line
    result_pose_iso = Eigen::Isometry3d::Identity();
    Eigen::Vector3d p1(start_point.x, start_point.y, start_point.z);
    Eigen::Vector3d p2(end_point.x, end_point.y, end_point.z);
    Eigen::Vector3d axis = (p2 - p1).normalized();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), axis);
    result_pose_iso.rotate(orientation);


    // Define the pose of the cylinder (midpoint between start and end points)
    result_pose_iso.translation().x() = (start_point.x + end_point.x) / 2.0;
    result_pose_iso.translation().y() = (start_point.y + end_point.y) / 2.0;
    result_pose_iso.translation().z() = (start_point.z + end_point.z) / 2.0;


    result_pose_msgs = isometryToPoseStamped(result_pose_iso, "world");
}



// Function to convert Eigen::Isometry3d to geometry_msgs::PoseStamped
geometry_msgs::PoseStamped ObjectCollisionTracker::isometryToPoseStamped(const Eigen::Isometry3d& transform, const std::string& frame_id) {
    geometry_msgs::PoseStamped pose_stamped;
    // Set the header
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = ros::Time::now();
    
    // Set the position
    pose_stamped.pose.position.x = transform.translation().x();
    pose_stamped.pose.position.y = transform.translation().y();
    pose_stamped.pose.position.z = transform.translation().z();

    // Set the orientation
    Eigen::Quaterniond quat(transform.rotation());
    pose_stamped.pose.orientation.w = quat.w();
    pose_stamped.pose.orientation.x = quat.x();
    pose_stamped.pose.orientation.y = quat.y();
    pose_stamped.pose.orientation.z = quat.z();
    
    return pose_stamped;
}

void ObjectCollisionTracker::computeCollisionContactPoints(planning_scene::PlanningScenePtr planning_scene_ptr,
                                                            std::vector<std::string> object_group1,
                                                            std::vector<std::string> object_group2,
                                                            collision_detection::CollisionResult& c_res,
                                                            std::vector<collision_detection::Contact>& stored_contacts) {
  // ----------------------------------------- Checking for Collisions ------------------------------------------
  c_res = planning_scene_ptr->getCollisionEnv()->checkCollisionBetweenObjectGroups(object_group1, object_group2);
  std::pair<std::string, std::string> object_pair = {"dynamic_object", "pillar"};
  
  auto contact_points = c_res.contacts.find(object_pair);
  if (contact_points != c_res.contacts.end()) {
    // Print the points to console
    // std::cout << "These are the stored contacts: ";
    // for (auto c : contact_points->second) {
    //     std::cout << c.pos;
    // }
    for (auto& c : contact_points->second) {
        auto it = std::find_if(stored_contacts.begin(), stored_contacts.end(), [&c](const collision_detection::Contact& existing) { return existing == c; });
        //stored_contacts.find(c);
        if (it == stored_contacts.end()) {
            stored_contacts.push_back(c);
            std::cout << "New contact stored at: [" << c.pos.x() << ", " << c.pos.y() << ", " << c.pos.z() << std::endl;
        }
    }
    std::cout << "We have stored " << stored_contacts.size() << " contacts." << std::endl;
  }
  

  if (c_res.collision)
  {
    ROS_INFO_STREAM("COLLIDING contact_point_count: " << c_res.contact_count);
    if (c_res.contact_count > 0)
    {
      //ROS_INFO_STREAM("c_res.contact_count: " << static_cast<int>(c_res.contact_count) << "; contactPointCount: " << contactPointCount);
      //if (static_cast<int>(c_res.contact_count) != contactPointCount) {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 0.5;
        visualization_msgs::MarkerArray markers;

        /* Get the contact points and display them as markers */
        collision_detection::getCollisionMarkersFromContacts(markers, "world", c_res.contacts, color,
                                                            ros::Duration(),  // remain until deleted
                                                            0.01);            // radius
        publishMarkers(markers);
        //contactPointCount++;
      //}
    }
  }
  else
  {
    ROS_INFO("Not colliding");
    // delete the old collision point markers
    visualization_msgs::MarkerArray empty_marker_array;
    publishMarkers(empty_marker_array);
  }
}

moveit_msgs::CollisionObject ObjectCollisionTracker::createSimpleObst() {
	geometry_msgs::Pose pose;
	moveit_msgs::CollisionObject object;
	object.id = "pillar";
	object.header.frame_id = "world";
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    object.primitives[0].dimensions.resize(3);
	object.primitives[0].dimensions[0] = 0.1;
    object.primitives[0].dimensions[1] = 0.1;
    object.primitives[0].dimensions[2] = 0.5;
    pose.position.x = 0.5; pose.position.y = 0.0; pose.position.z = 1.2;
    pose.orientation.x = 0.0; pose.orientation.y = 0.0; pose.orientation.z = 0.0;
	pose.position.z += 0.5 * object.primitives[0].dimensions[0];
	object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::CollisionObject::ADD;
	return object;
}

// Not necessary
void ObjectCollisionTracker::createPillarShape(planning_scene::PlanningScenePtr planning_scene_ptr) {
	Eigen::Isometry3d pillar_pose = Eigen::Isometry3d::Identity();
    pillar_pose.translation() = Eigen::Vector3d(0.5, 0.0, 1.2);
    
    // Convert Quaternion to Rotation Matrix
    Eigen::Quaterniond quaternion(1.0, 0.0, 0.0, 0.0);
    pillar_pose.linear() = quaternion.toRotationMatrix();

	// Define the shape using shapes::Box and add the object to the planning scene
    shapes::ShapePtr pillar_shape(new shapes::Box(0.1, 0.1, 1.0));
    //pillar_pose.translation().z() += 0.5 * 0.1;
    planning_scene_ptr->getWorldNonConst()->addToObject("pillar", pillar_shape, pillar_pose);
}

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_object_tracker");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    using Contacts = std::vector<collision_detection::Contact>;

    moveit::planning_interface::PlanningSceneInterface psi;
    moveit::planning_interface::MoveGroupInterface move_group_interface("dual_arm");
    moveit_task_constructor_demo::ObjectCollisionTracker* objectCollisionTracker = new moveit_task_constructor_demo::ObjectCollisionTracker();
    // Create a marker array publisher for publishing contact points
    objectCollisionTracker->g_marker_array_publisher = new ros::Publisher(nh.advertise<visualization_msgs::MarkerArray>("interactive_robot_marray", 100));

    // ============================================= Planning Scene =============================================
    // ==========================================================================================================
    // Create a planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor(); // Include sensor updates
    planning_scene_monitor->startStateMonitor();

    // =========================================== Initialize pillar ============================================
    // ==========================================================================================================
    // Create pillar
    moveit_msgs::CollisionObject pillar = objectCollisionTracker->createSimpleObst();
    // Add pillar using planning scene interface
    psi.applyCollisionObject(pillar);

    ros::Duration(1.0).sleep(); // Wait for planning scene to be updated
    planning_scene_monitor->requestPlanningSceneState();
    planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor->getPlanningScene();

    // ========================= Initialize DLO & Object groups for collision checking ==========================
    // ==========================================================================================================
    // Create a new DLO object
    moveit_msgs::CollisionObject dynamic_object;
    dynamic_object.id = "dynamic_object";
    dynamic_object.header.frame_id = "world";

    // Initialize object groups for collision detection
    std::vector<std::string> object_group1;
    std::vector<std::string> object_group2;
    object_group1.push_back(dynamic_object.id);
    object_group2.push_back(pillar.id);

    // =========================================== Set object position ==========================================
    // ==========================================================================================================
    // Get the current state of the robot
    robot_state::RobotStatePtr current_state = move_group_interface.getCurrentState();
    
    // Use tip of hand instead of hand base
    Eigen::Isometry3d tip_pose_in_hand_frame = Eigen::Isometry3d::Identity();
    tip_pose_in_hand_frame.translation().z() = 0.116; // Franka TCP configuration // original: 0.1034
    // Gripper tip pose of panda_1
    Eigen::Isometry3d gripper_tip_iso = current_state->getGlobalLinkTransform("panda_1_hand") * tip_pose_in_hand_frame;
    geometry_msgs::PoseStamped gripper_tip_pose = objectCollisionTracker->isometryToPoseStamped(gripper_tip_iso, "world");
    // Gripper tip pose of panda_2
    Eigen::Isometry3d gripper_tip_2_iso = current_state->getGlobalLinkTransform("panda_2_hand") * tip_pose_in_hand_frame;
    geometry_msgs::PoseStamped gripper_tip_2_pose = objectCollisionTracker->isometryToPoseStamped(gripper_tip_2_iso, "world");

    // ==================== Add DLO object as World::Object and moveit_msgs::CollisionObject ====================
    // ==========================================================================================================
    objectCollisionTracker->updateObjectShape2(gripper_tip_pose, gripper_tip_2_pose, dynamic_object.id, planning_scene);
    //objectCollisionTracker->updateObjectShape(gripper_tip_iso, gripper_tip_2_iso, dynamic_object.id, planning_scene);
    objectCollisionTracker->initObject(gripper_tip_pose, gripper_tip_2_pose, dynamic_object, psi);
    ros::Duration(1.0).sleep(); // Wait for planning scene to be updated

    // ================= Test to check if pillar and DLO are both successfully added in scene  ==================
    // ==========================================================================================================
    //planning_scene_monitor->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor);
    if (scene->getWorld()->hasObject("pillar")) {
        if(scene->getWorld()->hasObject("dynamic_object")) { ROS_INFO_STREAM("YES - Objects 'pillar' and 'dynamic_object' are visible in PlanningSceneMonitor!"); }
        else { ROS_INFO_STREAM("NO/YES - Only object 'pillar' is visible in PlanningSceneMonitor!"); }
    }
    else { ROS_INFO_STREAM("NO - None of the objects are visible in PlanningSceneMonitor!"); }

    // ========================================== First collision check =========================================
    // ==========================================================================================================

    // First collision check
    collision_detection::CollisionResult c_res;
    Contacts contacts;
    //objectCollisionTracker->computeCollisionContactPoints(planning_scene, object_group1, object_group2, c_res);

    // ====================== Loop for updating the object position and checking collisions =====================
    // ==========================================================================================================
    while (ros::ok()) {
        c_res.clear();
        // Get the current state of the robot & current planning scene
        planning_scene = planning_scene_monitor->getPlanningScene();
        robot_state::RobotStatePtr current_state = move_group_interface.getCurrentState();

        // Derive the gripper positions from the current robot state
        gripper_tip_iso = current_state->getGlobalLinkTransform("panda_1_hand") * tip_pose_in_hand_frame;
        gripper_tip_2_iso = current_state->getGlobalLinkTransform("panda_2_hand") * tip_pose_in_hand_frame;
        gripper_tip_pose = objectCollisionTracker->isometryToPoseStamped(gripper_tip_iso, "world");
        gripper_tip_2_pose = objectCollisionTracker->isometryToPoseStamped(gripper_tip_2_iso, "world");

        // Update the dynamic object
        objectCollisionTracker->updateObjectShape2(gripper_tip_pose, gripper_tip_2_pose, dynamic_object.id, planning_scene);
        // objectCollisionTracker->updateObjectShape(gripper_tip_iso, gripper_tip_2_iso, dynamic_object.id, planning_scene);
        objectCollisionTracker->updateObject2(gripper_tip_pose, gripper_tip_2_pose, dynamic_object, psi);

        // Check for collisions between the object and the environment
        objectCollisionTracker->computeCollisionContactPoints(planning_scene, object_group1, object_group2, c_res, contacts);
        ros::Duration(0.1).sleep();
    }
    return 0;
}

