#include </home/kifabrik/ws_moveit/src/moveit_task_constructor/demo/include/moveit_task_constructor_demo/line_object_tracking.h>
//  /home/kifabrik/ws_moveit/src/moveit_task_constructor/demo/include/moveit_task_constructor_demo/line_object_tracking.h


namespace moveit_task_constructor_demo {

//g_marker_array_publisher = nullptr;
// visualization_msgs::MarkerArray g_collision_points;

ObjectTracker::ObjectTracker() {
        // Constructor implementation
}

void ObjectTracker::allowCollisions(const std::string& link_name, const std::string& object_id) {
    // Initialize the PlanningSceneMonitor
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(
        new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    // Wait for the PlanningScene to be updated
    planning_scene_monitor->requestPlanningSceneState("/get_planning_scene");
    ros::Duration(0.1).sleep();

    // Lock the scene for reading
    planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor);

    // Get the current planning scene
    planning_scene::PlanningScenePtr scene = planning_scene->diff();

    // Access the ACM
    collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();

    // Allow collision between the specified link and object
    acm.setEntry(link_name, object_id, true);

    // Create a PlanningScene message
    moveit_msgs::PlanningScene planning_scene_msg;
    scene->getPlanningSceneDiffMsg(planning_scene_msg);

    // Publish the modified planning scene
    planning_scene_monitor->getPlanningScene()->setPlanningSceneDiffMsg(planning_scene_msg);
    planning_scene_monitor->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
}

void ObjectTracker::initObject(const geometry_msgs::PoseStamped& gripper_pose, moveit_msgs::CollisionObject& collision_object, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    // Define the start point (e.g., robot base)
    geometry_msgs::Point start_point;
    start_point.x = 0.0;
    start_point.y = 0.0;
    start_point.z = 1.1;

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
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = moveit_msgs::CollisionObject::ADD;

    //allowCollisions("panda_1_hand", collision_object.id);

    // Apply the collision object to the planning scene
    planning_scene_interface.applyCollisionObject(collision_object);
}

void ObjectTracker::updateObject(const geometry_msgs::PoseStamped& gripper_pose, moveit_msgs::CollisionObject& collision_object, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    // Define the start point (e.g., robot base)
    geometry_msgs::Point start_point;
    start_point.x = 0.0;
    start_point.y = 0.0;
    start_point.z = 1.1;

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

    // Add the primitive and pose to the collision object
    //collision_object.primitives.push_back(primitive);
    //collision_object.primitive_poses.push_back(cylinder_pose);
    //collision_object.operation = moveit_msgs::CollisionObject::ADD;

    //allowCollisions("panda_1_hand", collision_object.id);

    // Apply the collision object to the planning scene
    planning_scene_interface.applyCollisionObject(collision_object);
}



// Function to convert Eigen::Isometry3d to geometry_msgs::Pose
geometry_msgs::Pose ObjectTracker::isometryToPose(const Eigen::Isometry3d& transform) {
    geometry_msgs::Pose pose;
    // From other way round 
    //isometry.translation() << pose_msg.position.x, pose_msg.position.y, pose_msg.position.z;
    
    //pose.position << transform.translation().x(), transform.translation().y(), transform.translation().z();
    
    // Set the position
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();

    // Set the orientation
    Eigen::Quaterniond quat(transform.rotation());
    //pose.orientation << quat.x(), quat.y(), quat.z(), quat.w();
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

bool ObjectTracker::updatePlanningScene(planning_scene::PlanningScene& planning_scene, ros::NodeHandle& nh) {
    // Create a service client to get the planning scene
    ros::ServiceClient planning_scene_service_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
    planning_scene_service_client.waitForExistence();

    // Create a request for the complete planning scene
    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS | moveit_msgs::PlanningSceneComponents::ROBOT_STATE | moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
                                        moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY | moveit_msgs::PlanningSceneComponents::OCTOMAP |
                                        moveit_msgs::PlanningSceneComponents::TRANSFORMS | moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX | moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING |
                                        moveit_msgs::PlanningSceneComponents::TRANSFORMS | moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX | moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING |
                                        moveit_msgs::PlanningSceneComponents::TRANSFORMS | moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX | moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING |
                                        moveit_msgs::PlanningSceneComponents::OBJECT_COLORS;

    // Call the service to get the planning scene
    if (planning_scene_service_client.call(srv)) {
        // Update the PlanningScene object with the retrieved planning scene message
        planning_scene.usePlanningSceneMsg(srv.response.scene);
        return true;
    }
    else {
        ROS_ERROR("Failed to call service get_planning_scene");
        return false;
    }
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
                                                moveit_msgs::CollisionObject& object,
                                                const geometry_msgs::TransformStamped& transformStamped) {

  collision_detection::CollisionRequest c_req;
  collision_detection::CollisionResult c_res;
  //c_req.group_name = "dual_arm";
  c_req.contacts = true;
  c_req.max_contacts = 100;
  c_req.max_contacts_per_pair = 5;
  c_req.verbose = false;

  // ------------------ Checking for Collisions ------------------
  planning_scene->checkCollision(c_req, c_res, object, transformStamped);//, *robot);

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
    ROS_INFO("Not colliding");

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
    //std::string dynamic_object_frame_id = "dynamic_object_frame";

    // Get the current state of the robot
    robot_state::RobotStatePtr current_state = move_group_interface.getCurrentState();

    // Use tip of hand instead of hand base
    Eigen::Isometry3d tip_pose_in_hand_frame = Eigen::Isometry3d::Identity();
    tip_pose_in_hand_frame.translation().z() = 0.1034; // Franka TCP configuration
    const Eigen::Isometry3d& gripper_tip_iso = current_state->getGlobalLinkTransform("panda_1_hand") * tip_pose_in_hand_frame;
    geometry_msgs::PoseStamped gripper_tip_pose = objectTracker->isometryToPoseStamped(gripper_tip_iso, "world");

    objectTracker->initObject(gripper_tip_pose, dynamic_object, psi);
    objectTracker->updateTransform(dynamic_object.pose, "world", "dynamic_object_frame", transformStamped);
// #####################################################################################################################



// ===================================== Planning Scene ======================================================
    // // Load the robot model
    //robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    //robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    // Create a planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
        std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor();
    planning_scene_monitor->startStateMonitor();

    // Create a PlanningScene
    planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor->getPlanningScene();
    // Update Planning Scene and check for collisons
    //if (moveit_task_constructor_demo::updatePlanningScene(planning_scene, nh)) {
        // Now you can use the planning_scene object as needed

    // ============================================================

    // // Pre check for collision
    // collision_detection::CollisionRequest collision_request;
    // collision_detection::CollisionResult collision_result;
    // planning_scene->checkCollision(collision_request, collision_result, dynamic_object);

    // if (collision_result.collision){ ROS_INFO("Collision detected!"); }
    // else { ROS_INFO("No collision detected."); }

// ===========================================================================================================

    // First collision check
    objectTracker->computeCollisionContactPoints(planning_scene, dynamic_object, transformStamped);


    while (ros::ok()) {
        //collision_result.clear();  // Clear previous results

        // Get the current state of the robot & current planning scene
        robot_state::RobotStatePtr current_state = move_group_interface.getCurrentState();
        planning_scene = planning_scene_monitor->getPlanningScene();

        const Eigen::Isometry3d& gripper_tip_iso = current_state->getGlobalLinkTransform("panda_1_hand") * tip_pose_in_hand_frame;
        geometry_msgs::PoseStamped gripper_tip_pose = objectTracker->isometryToPoseStamped(gripper_tip_iso, "world");

        // Update the dynamic object
        objectTracker->updateObject(gripper_tip_pose, dynamic_object, psi);
        // Broadcast the transform of object frame to world frame
        objectTracker->updateTransform(dynamic_object.pose, "world", "dynamic_object_frame", transformStamped);

        
        // ====================================== Check for collisions =================================================
        
        //if (moveit_task_constructor_demo::updatePlanningScene(planning_scene, nh)) {

        objectTracker->computeCollisionContactPoints(planning_scene, dynamic_object, transformStamped);

            // Now you can use the planning_scene object as needed
            //collision_detection::CollisionRequest collision_request;
            //collision_detection::CollisionResult collision_result;
            //planning_scene->checkCollision(collision_request, collision_result, dynamic_object);

            // if (collision_result.collision){ ROS_INFO("Collision detected!"); }
            // else { ROS_INFO("No collision detected."); }
        //}
        //else { ROS_ERROR("Failed to call service get_planning_scene");}

        // =============================================================================================================

        ros::Duration(0.1).sleep();
    }

    return 0;
}

