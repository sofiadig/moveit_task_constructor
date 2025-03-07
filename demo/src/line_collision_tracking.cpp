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

void ObjectCollisionTracker::initObject(const geometry_msgs::PoseStamped& steady_point,
                                const geometry_msgs::PoseStamped& moving_point,
                                moveit_msgs::CollisionObject& collision_object,
                                moveit::planning_interface::PlanningSceneInterface& planning_scene_interface ) {
    geometry_msgs::Point start_point = steady_point.pose.position;
    geometry_msgs::Point end_point = moving_point.pose.position;

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



void ObjectCollisionTracker::updateDLO(const geometry_msgs::PoseStamped&  start_pose,
                                    const geometry_msgs::PoseStamped&  end_pose,
                                    moveit_msgs::CollisionObject& collision_object,
                                    planning_scene::PlanningScenePtr planning_scene_ptr,
                                    moveit::planning_interface::PlanningSceneInterface& psi,
                                    const std::vector<collision_detection::Contact>& adjusted_contacts,
                                    bool& hasNewContact,
                                    int& num_segments) {
    geometry_msgs::PoseStamped contactPos;
    if (!adjusted_contacts.empty()) {
        contactPos = vectorToPoseStamped(adjusted_contacts[adjusted_contacts.size()-1].pos);
    }
    else {
        contactPos = start_pose;
    }
    
    if (hasNewContact) {
        // For moveit_msgs part: Create two lines: 
        // 1. from the previous start point to the current contact point
        // First section should be new object called [previous_id]_1 , 2, 3, etc.
        moveit_msgs::CollisionObject first_segment;
        std::string segment_name = "segment_" + std::to_string(num_segments);
        first_segment.id = segment_name;
        first_segment.header.frame_id = "world";
        initObject(start_pose, contactPos, first_segment, psi);
        num_segments++;
    }
    // For shapes & Eigen part: only do step 2
    // 2. Set the start pose of the second line as the contact point, end pose stays the same
    updateObjectShape2(contactPos, end_pose, collision_object.id, planning_scene_ptr);
    updateObject2(contactPos, end_pose, collision_object, psi);
    
    
    // else if (hasNewContact && adjusted_contacts.empty()) {
    //     ROS_WARN_STREAM("Problem: adjusted_contacts should have elements but is empty instead.");
    // }
    // else {
    //     updateObjectShape2(start_pose, end_pose, collision_object.id, planning_scene_ptr);
    //     updateObject2(start_pose, end_pose, collision_object, psi);
    // }
}

void ObjectCollisionTracker::initObjectShape(const geometry_msgs::PoseStamped&  steady_point,
                                const geometry_msgs::PoseStamped&  moving_point,
                                std::string& object_id,
                                planning_scene::PlanningScenePtr planning_scene_ptr) {
    geometry_msgs::PoseStamped result_pose_msgs;
    Eigen::Isometry3d cylinder_pose;
    double cylinder_length;
    double cylinder_radius = 0.005;
    determinePose(steady_point, moving_point, result_pose_msgs, cylinder_pose, cylinder_length);

    shapes::ShapePtr cylinder_shape(new shapes::Cylinder(cylinder_radius, cylinder_length));
    planning_scene_ptr->getWorldNonConst()->addToObject(object_id, cylinder_shape, cylinder_pose);
}

void ObjectCollisionTracker::updateObjectShape2(const geometry_msgs::PoseStamped&  steady_point,
                                const geometry_msgs::PoseStamped&  moving_point,
                                std::string& object_id,
                                planning_scene::PlanningScenePtr planning_scene_ptr) {
    
    geometry_msgs::PoseStamped result_pose_msgs;
    Eigen::Isometry3d cylinder_pose;
    double cylinder_length;
    double cylinder_radius = 0.005;
    //std::vector<shapes::ShapeConstPtr> old_shapes = planning_scene_ptr->getWorldNonConst()->getObject(object_id)->shapes_;

    determinePose(steady_point, moving_point, result_pose_msgs, cylinder_pose, cylinder_length);
    shapes::ShapePtr cylinder_shape(new shapes::Cylinder(cylinder_radius, cylinder_length));

    //planning_scene_ptr->getWorldNonConst()->removeObject(object_id);
    planning_scene_ptr->getWorldNonConst()->addToObject(object_id, cylinder_shape, cylinder_pose);

    //planning_scene_ptr->getWorldNonConst()->removeShapeFromObject(object_id, old_shapes.first);
    // if (!old_shapes.empty()) {
    //     for (auto it : old_shapes) {
    //         planning_scene_ptr->getWorldNonConst()->removeShapeFromObject(object_id, it);
    //     }
    // }
    
}

void ObjectCollisionTracker::updateObject2(const geometry_msgs::PoseStamped& steady_point,
                                    const geometry_msgs::PoseStamped& moving_point,
                                    moveit_msgs::CollisionObject& collision_object,
                                    moveit::planning_interface::PlanningSceneInterface& psi) {
    geometry_msgs::PoseStamped result_pose_msgs;
    Eigen::Isometry3d result_pose_iso;
    double length;

    determinePose(steady_point, moving_point, result_pose_msgs, result_pose_iso, length);
    collision_object.primitives[0].dimensions[0] = length;
    collision_object.primitives[0].dimensions[1] = 0.005;
    collision_object.primitive_poses[0] = result_pose_msgs.pose;

    // Apply the updated scene
    psi.applyCollisionObject(collision_object);
}

void ObjectCollisionTracker::determinePose(const geometry_msgs::PoseStamped& steady_point,
                                            const geometry_msgs::PoseStamped& moving_point,
                                            geometry_msgs::PoseStamped& result_pose_msgs,
                                            Eigen::Isometry3d& result_pose_iso,
                                            double& length) {
    // Define the start & end point
    geometry_msgs::Point start_point = steady_point.pose.position;
    geometry_msgs::Point end_point = moving_point.pose.position;

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

geometry_msgs::PoseStamped ObjectCollisionTracker::vectorToPoseStamped(const Eigen::Vector3d& position) {
    geometry_msgs::PoseStamped pose_stamped;
    // Set the header
    pose_stamped.header.frame_id = "world";
    pose_stamped.header.stamp = ros::Time::now();
    
    // Set the position
    pose_stamped.pose.position.x = position.x();
    pose_stamped.pose.position.y = position.y();
    pose_stamped.pose.position.z = position.z();

    // Set the orientation
    Eigen::Quaterniond quat(Eigen::Isometry3d::Identity().rotation());
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
                                                            std::vector<collision_detection::Contact>& stored_contacts,
                                                            std::vector<collision_detection::Contact>& adjusted_contacts,
                                                            bool& isNewContact) {
  // ----------------------------------------- Checking for Collisions ------------------------------------------
  isNewContact = false;
  // Suppress std::cout
  std::ofstream null_stream("/dev/null");
  std::streambuf* cout_buffer = std::cout.rdbuf(nullptr);  // Save original buffer (to nullptr aka dont save)
  std::cout.rdbuf(null_stream.rdbuf());  // Redirect std::cout to null

  c_res = planning_scene_ptr->getCollisionEnv()->checkCollisionBetweenObjectGroups(object_group1, object_group2); // cout suppressed

  // Restore std::cout
  std::cout.rdbuf(cout_buffer);

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
        if (it == stored_contacts.end()) {
            stored_contacts.push_back(c);
            adjusted_contacts.push_back(adjustContactPoint(c));
            std::cout << " +++++++++++ New contact stored at: [" << c.pos.x() << ", " << c.pos.y() << ", " << c.pos.z() << "] +++++++++++" << std::endl;
            isNewContact = true;
        }
    }
    //std::cout << "We have stored " << stored_contacts.size() << " contacts." << std::endl;
  }

  std::map<std::pair<std::string, std::string>, std::vector<collision_detection::Contact>> adjusted_map = {{object_pair, adjusted_contacts}};

  if (c_res.collision)
  {
    ROS_INFO_STREAM("COLLIDING");// contact_point_count: " << c_res.contact_count);
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
        collision_detection::getCollisionMarkersFromContacts(markers, "world", adjusted_map, color, // originally: markers, "world", c_res.contacts, color,
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

// Function to move contact point outside of the obstacle, because the collisions only get detected inside
// collision_detection::Contact ObjectCollisionTracker::adjustContactPoint(const collision_detection::Contact& contact) {
//     Eigen::Vector3d direction = contact.normal;
//     double distance = contact.depth + 0.01;

//     collision_detection::Contact adjusted_contact = contact;
//     adjusted_contact.pos = contact.pos + distance * direction;
//     return adjusted_contact;
// }

// Get the corners of the rectangular cross section of the pillar in the x-y-plane
std::map<std::string, Eigen::Vector3d> ObjectCollisionTracker::getCornerPoints(const moveit_msgs::CollisionObject& pillar) {
    std::map<std::string, Eigen::Vector3d> corner_points;
    Eigen::Vector3d x_low_y_low;
    Eigen::Vector3d x_low_y_high;
    Eigen::Vector3d x_high_y_low;
    Eigen::Vector3d x_high_y_high;

    double x_center = pillar.primitive_poses[0].position.x;
    double y_center = pillar.primitive_poses[0].position.y;
    double x_low  = x_center - pillar.primitives[0].dimensions[0] / 2 - 0.01;
    double x_high = x_center + pillar.primitives[0].dimensions[0] / 2 + 0.01;
    double y_low  = y_center - pillar.primitives[0].dimensions[1] / 2 - 0.01;
    double y_high = y_center + pillar.primitives[0].dimensions[1] / 2 + 0.01;

    x_low_y_low.x() = x_low;
    x_low_y_low.y() = y_low;
    x_low_y_low.z() = 0;
    corner_points["x_low_y_low"] =  x_low_y_low;
    x_low_y_high.x() = x_low;
    x_low_y_high.y() = y_high;
    x_low_y_high.z() = 0;
    corner_points["x_low_y_high"] =  x_low_y_high;
    x_high_y_low.x() = x_high;
    x_high_y_low.y() = y_high;
    x_high_y_low.z() = 0;
    corner_points["x_high_y_low"] =  x_high_y_low;
    x_high_y_high.x() = x_high;
    x_high_y_high.y() = y_high;
    x_high_y_high.z() = 0;
    corner_points["x_high_y_high"] =  x_high_y_high;

    return corner_points;
}

Eigen::Vector3d ObjectCollisionTracker::determineNearestCornerPoint(const collision_detection::Contact& contact_point,
                                                                    const std::map<std::string,Eigen::Vector3d>& cornerPoints) {
    Eigen::Vector3d nearest_corner;
    double shortest_distance = 100.0;
    //std::string nearest_corner;
    for (auto pair : cornerPoints) {
        double distance = (contact_point.pos - pair.second).norm();
        if (distance < shortest_distance) {
            shortest_distance = distance;
            nearest_corner = pair.second;
        }
    }
     //= cornerPoints[nearest_corner];
    return nearest_corner;
}

collision_detection::Contact ObjectCollisionTracker::adjustContactPoint(const collision_detection::Contact& contact_point) {
    collision_detection::Contact adjusted_contact = contact_point;
    adjusted_contact.pos = determineNearestCornerPoint(contact_point, corner_points);
    adjusted_contact.pos.z() = contact_point.pos.z();
    return adjusted_contact;
}

// collision_detection::Contact ObjectCollisionTracker::adjustContactPoint(const collision_detection::Contact& contact, const moveit_msgs::CollisionObject& pillar) {
//     collision_detection::Contact adjusted_contact = contact;
//     double x_O = pillar.primitive_poses[0].position.x;
//     double y_O = pillar.primitive_poses[0].position.y;

//     double x_Olow  = x_O - pillar.primitives[0].dimensions[0] / 2;
//     double x_Ohigh = x_O + pillar.primitives[0].dimensions[0] / 2;
//     double y_Olow  = y_O - pillar.primitives[0].dimensions[1] / 2;
//     double y_Ohigh = y_O + pillar.primitives[0].dimensions[1] / 2;

//     double c_x = contact.pos.x();
//     double c_y = contact.pos.y();

//     if (x_Olow <= c_x && c_x <= x_Ohigh) {
//         if ( (x_Ohigh-c_x) >= (c_x-x_Olow) ) {
//             adjusted_contact.pos.x() = x_Olow - 0.01;
//         }
//         else {
//             adjusted_contact.pos.x() = x_Ohigh + 0.01;
//         }
//     }
// }

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
    object.primitives[0].dimensions[2] = 0.35;
    pose.position.x = 0.45; pose.position.y = 0.2; pose.position.z = 1.0;
    pose.orientation.x = 0.0; pose.orientation.y = 0.0; pose.orientation.z = 0.0;
	pose.position.z += 0.5 * object.primitives[0].dimensions[2];
	object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::CollisionObject::ADD;
	return object;
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
    Eigen::Isometry3d moving_hand_tip_iso = current_state->getGlobalLinkTransform("panda_1_hand") * tip_pose_in_hand_frame;
    geometry_msgs::PoseStamped moving_hand_tip_pose = objectCollisionTracker->isometryToPoseStamped(moving_hand_tip_iso, "world");
    // Gripper tip pose of panda_2
    Eigen::Isometry3d steady_hand_tip_iso = current_state->getGlobalLinkTransform("panda_2_hand") * tip_pose_in_hand_frame;
    geometry_msgs::PoseStamped steady_hand_tip_pose = objectCollisionTracker->isometryToPoseStamped(steady_hand_tip_iso, "world");

    // ==================== Add DLO object as World::Object and moveit_msgs::CollisionObject ====================
    // ==========================================================================================================
    objectCollisionTracker->initObjectShape(steady_hand_tip_pose, moving_hand_tip_pose, dynamic_object.id, planning_scene);
    //objectCollisionTracker->updateObjectShape(moving_hand_tip_iso, steady_hand_tip_iso, dynamic_object.id, planning_scene);
    objectCollisionTracker->initObject(steady_hand_tip_pose, moving_hand_tip_pose, dynamic_object, psi);
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
    objectCollisionTracker->corner_points = objectCollisionTracker->getCornerPoints(pillar);
    collision_detection::CollisionResult c_res;
    Contacts original_contacts;
    Contacts adjusted_contacts;
    bool isNewContact = false;
    int num_segments = 1;
    //objectCollisionTracker->computeCollisionContactPoints(planning_scene, object_group1, object_group2, c_res, contacts, isNewContact);

    // ====================== Loop for updating the object position and checking collisions =====================
    // ==========================================================================================================
    while (ros::ok()) {
        c_res.clear();
        // Get the current state of the robot & current planning scene
        planning_scene = planning_scene_monitor->getPlanningScene();
        robot_state::RobotStatePtr current_state = move_group_interface.getCurrentState();

        // Derive the gripper positions from the current robot state
        moving_hand_tip_iso  = current_state->getGlobalLinkTransform("panda_1_hand") * tip_pose_in_hand_frame;
        moving_hand_tip_pose = objectCollisionTracker->isometryToPoseStamped(moving_hand_tip_iso, "world");

        if(isNewContact) {
            if (adjusted_contacts.size() >= 2) {
                steady_hand_tip_pose = objectCollisionTracker->vectorToPoseStamped(adjusted_contacts[adjusted_contacts.size()-2].pos);
            }
            else {
                steady_hand_tip_iso  = current_state->getGlobalLinkTransform("panda_2_hand") * tip_pose_in_hand_frame;
                steady_hand_tip_pose = objectCollisionTracker->isometryToPoseStamped(steady_hand_tip_iso, "world");
            }
        }
        else {
            if (!adjusted_contacts.empty()) {
                steady_hand_tip_pose = objectCollisionTracker->vectorToPoseStamped(adjusted_contacts[adjusted_contacts.size()-1].pos);
            }
            else {
                steady_hand_tip_iso  = current_state->getGlobalLinkTransform("panda_2_hand") * tip_pose_in_hand_frame;
                steady_hand_tip_pose = objectCollisionTracker->isometryToPoseStamped(steady_hand_tip_iso, "world");
            }
        }
        
        
        
        // if (num_segments == 1) {
        //     steady_hand_tip_iso  = current_state->getGlobalLinkTransform("panda_2_hand") * tip_pose_in_hand_frame;
        //     steady_hand_tip_pose = objectCollisionTracker->isometryToPoseStamped(steady_hand_tip_iso, "world");
        // }
        // else {
        //     if (isNewContact && !adjusted_contacts.empty()) {
        //         steady_hand_tip_pose = objectCollisionTracker->vectorToPoseStamped(adjusted_contacts[adjusted_contacts.size()-1].pos);
        //     }
        //     else if (!adjusted_contacts.empty()){
        //         steady_hand_tip_pose = objectCollisionTracker->vectorToPoseStamped(adjusted_contacts[adjusted_contacts.size()-1].pos);
        //     }
        //     else {
        //         ROS_WARN_STREAM("Problem: Adjusted_contacts is empty, but the number of segments is not 1");
        //     }
        // }
        
        // Update the dynamic object
        // objectCollisionTracker->updateObjectShape2(moving_hand_tip_pose, steady_hand_tip_pose, dynamic_object.id, planning_scene);
        // objectCollisionTracker->updateObject2(moving_hand_tip_pose, steady_hand_tip_pose, dynamic_object, psi);
        objectCollisionTracker->updateDLO(steady_hand_tip_pose, moving_hand_tip_pose, dynamic_object, planning_scene,
                                            psi, adjusted_contacts, isNewContact, num_segments);

        // Check for collisions between the object and the environment
        objectCollisionTracker->computeCollisionContactPoints(planning_scene, object_group1, object_group2, c_res, original_contacts, adjusted_contacts, isNewContact);
        //std::cout << isNewContact << std::endl;
        ros::Duration(0.1).sleep();
    }
    return 0;
}

