#include </home/sdg/ws_moveit/src/moveit_task_constructor/demo/include/moveit_task_constructor_demo/line_collision_tracking.h>

namespace moveit_task_constructor_demo {

ObjectCollisionTracker::ObjectCollisionTracker() {
    visual_tools.reset(new rviz_visual_tools::RvizVisualTools("world", "/interactive_robot_marray"));
    visual_tools->loadMarkerPub();
    g_marker_array_publisher = nullptr;
    isObjectDLO = false; // false: line marker, true: object
}

// ==================================================================================================================================================
// ========================== 1. Initializing =======================================================================================================
// ==================================================================================================================================================

void ObjectCollisionTracker::initObject(const geometry_msgs::PoseStamped& start_pose,
                                const geometry_msgs::PoseStamped& end_pose,
                                moveit_msgs::CollisionObject& collision_object,
                                moveit::planning_interface::PlanningSceneInterface& planning_scene_interface ) {
    geometry_msgs::PoseStamped result_pose_msgs;
    Eigen::Isometry3d result_pose_iso;
    double length;
    int num_segments = 1;
    determinePose(start_pose, end_pose, result_pose_msgs, result_pose_iso, length);
    
    if (isObjectDLO) {
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = length;
        primitive.dimensions[1] = 0.005;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(result_pose_msgs.pose);
        collision_object.operation = moveit_msgs::CollisionObject::ADD;

        planning_scene_interface.applyCollisionObject(collision_object);
    }
    else {
        updateLineMarker(start_pose.pose.position, end_pose.pose.position, num_segments);
    }
}

moveit_msgs::CollisionObject ObjectCollisionTracker::createSimpleObstacle() {
	moveit_msgs::CollisionObject object;
    object.id = "pillar";
    object.header.frame_id = "world";
    geometry_msgs::Pose pose;
	
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    object.primitives[0].dimensions.resize(3);
	object.primitives[0].dimensions[0] = 0.1;
    object.primitives[0].dimensions[1] = 0.1;
    object.primitives[0].dimensions[2] = 0.4;
    pose.position.x = 0.45;
    pose.position.y = 0.37;
    pose.position.z = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
	pose.position.z += 0.5 * object.primitives[0].dimensions[2];

	object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::CollisionObject::ADD;
	return object;
}

// ==================================================================================================================================================
// ========================== 2. Updates at each iteration ==========================================================================================
// ==================================================================================================================================================

void ObjectCollisionTracker::updateDLO(const geometry_msgs::PoseStamped&  start_pose,
                                    const geometry_msgs::PoseStamped&  end_pose,
                                    moveit_msgs::CollisionObject& collision_object,
                                    planning_scene::PlanningScenePtr& planning_scene_ptr,
                                    moveit::planning_interface::PlanningSceneInterface& psi,
                                    const std::vector<collision_detection::Contact>& adjusted_contacts,
                                    const bool& hasNewContact,
                                    int& num_segments) {
    geometry_msgs::PoseStamped contactPos;
    if (!adjusted_contacts.empty()) {
        contactPos = vectorToPoseStamped(adjusted_contacts[adjusted_contacts.size()-1].pos);
    }
    else {
        contactPos = start_pose;
    }
    
    if (hasNewContact) {
        // 1. Create segment from the previous start point to the current contact point
        if (isObjectDLO) {
            moveit_msgs::CollisionObject first_segment;
            std::string segment_name = "segment_" + std::to_string(num_segments);
            first_segment.id = segment_name;
            first_segment.header.frame_id = "world";
            segments.push_back(first_segment);
            initObject(start_pose, contactPos, segments[segments.size()-1], psi);
            ROS_INFO_STREAM("Added a segment object called: " + segments.back().id);
            ROS_INFO_STREAM("The following segments exist now: ");
            for (auto it : segments) {
                std::cout << it.id << ", ";
            }
            
            // num_segments++;
        }
        else {
            updateLineMarker(start_pose.pose.position, contactPos.pose.position, num_segments);
            // num_segments++;
        }
        num_segments++;
    }
    // 2. Set the start pose of the second line as the contact point, end pose stays the same
    updateObject(contactPos, end_pose, collision_object, planning_scene_ptr, psi, num_segments);
}

void ObjectCollisionTracker::updateObject(const geometry_msgs::PoseStamped&  start_pose,
                                const geometry_msgs::PoseStamped&  end_pose,
                                moveit_msgs::CollisionObject& collision_object,
                                planning_scene::PlanningScenePtr& planning_scene_ptr,
                                moveit::planning_interface::PlanningSceneInterface& psi,
                                int& num_segments ) {
    geometry_msgs::PoseStamped result_pose_msgs;
    Eigen::Isometry3d cylinder_pose;
    double cylinder_length;
    double cylinder_radius = 0.005;
    determinePose(start_pose, end_pose, result_pose_msgs, cylinder_pose, cylinder_length);
    
    shapes::ShapePtr cylinder_shape(new shapes::Cylinder(cylinder_radius, cylinder_length));
    planning_scene_ptr->getWorldNonConst()->addToObject(collision_object.id, cylinder_shape, cylinder_pose);

    if (isObjectDLO) {
        collision_object.primitives[0].dimensions[0] = cylinder_length;
        collision_object.primitives[0].dimensions[1] = cylinder_radius;
        collision_object.primitive_poses[0] = result_pose_msgs.pose;
        psi.applyCollisionObject(collision_object);
    }
    else {
        updateLineMarker(start_pose.pose.position, end_pose.pose.position, num_segments);
    }
}

void ObjectCollisionTracker::updateLineMarker(const geometry_msgs::Point& start,
                                              const geometry_msgs::Point& end,
                                              int marker_id) {
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = "world";
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "line_markers";
    line_marker.id = marker_id;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.scale.x = 0.01; // Line width
    line_marker.scale.y = 0.01;
    line_marker.color.r = 0.0;
    line_marker.color.g = 0.0;
    line_marker.color.b = 1.0;
    line_marker.color.a = 1.0;

    line_marker.points.push_back(start);
    line_marker.points.push_back(end);

    // Delete the old line marker
    visualization_msgs::Marker delete_marker = line_marker;
    delete_marker.action = visualization_msgs::Marker::DELETE;
    visual_tools->publishMarker(delete_marker);

    // Publish the new line marker
    visual_tools->publishMarker(line_marker);
    visual_tools->trigger();
}

// ==================================================================================================================================================
// ========================== 3. Pose computations ==================================================================================================
// ==================================================================================================================================================

void ObjectCollisionTracker::determinePose(const geometry_msgs::PoseStamped& start_pose,
                                            const geometry_msgs::PoseStamped& end_pose,
                                            geometry_msgs::PoseStamped& result_pose_msgs,
                                            Eigen::Isometry3d& result_pose_iso,
                                            double& length) {
    // Define the start & end point
    geometry_msgs::Point start_point = start_pose.pose.position;
    geometry_msgs::Point end_point = end_pose.pose.position;

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
    pose_stamped.header.frame_id = "world";
    pose_stamped.header.stamp = ros::Time::now();
    
    pose_stamped.pose.position.x = position.x();
    pose_stamped.pose.position.y = position.y();
    pose_stamped.pose.position.z = position.z();

    Eigen::Quaterniond quat(Eigen::Isometry3d::Identity().rotation());
    pose_stamped.pose.orientation.w = quat.w();
    pose_stamped.pose.orientation.x = quat.x();
    pose_stamped.pose.orientation.y = quat.y();
    pose_stamped.pose.orientation.z = quat.z();
    
    return pose_stamped;
}



// ==================================================================================================================================================
// ========================== 4. Computing collisions ===============================================================================================
// ==================================================================================================================================================

void ObjectCollisionTracker::computeCollisionContactPoints(planning_scene::PlanningScenePtr planning_scene_ptr,
                                                            std::vector<std::string> object_group1,
                                                            std::vector<std::string> object_group2,
                                                            collision_detection::CollisionResult& c_res,
                                                            std::vector<collision_detection::Contact>& stored_contacts,
                                                            std::vector<collision_detection::Contact>& adjusted_contacts,
                                                            bool& hasNewContact) {
    hasNewContact = false;
    // Suppress std::cout (because each collision check prints quite a few lines which clogs up the console)
    std::ofstream null_stream("/dev/null");
    std::streambuf* cout_buffer = std::cout.rdbuf(nullptr);  // Save original buffer (to nullptr aka dont save)
    std::cout.rdbuf(null_stream.rdbuf());  // Redirect std::cout to null
    
    // Check collisions between DLO and obstacle
    c_res = planning_scene_ptr->getCollisionEnv()->checkCollisionBetweenObjectGroups(object_group1, object_group2); // cout suppressed

    // Restore std::cout
    std::cout.rdbuf(cout_buffer);

    std::pair<std::string, std::string> object_pair = {"dynamic_object", "pillar"};
    auto contact_points = c_res.contacts.find(object_pair);
        if (contact_points != c_res.contacts.end()) {
        for (auto& c : contact_points->second) {
            auto it = std::find_if(stored_contacts.begin(), stored_contacts.end(), [&c](const collision_detection::Contact& existing) { return existing == c; });
            if (it == stored_contacts.end()) {
                stored_contacts.push_back(c);
                adjusted_contacts.push_back(adjustContactPoint(c, adjusted_contacts));
                std::cout << " +++++++++++ New contact stored at: [" << c.pos.x() << ", " << c.pos.y() << ", " << c.pos.z() << "] +++++++++++" << std::endl;
                std::cout << " +++++++++++ And adjusted to      : [" << adjusted_contacts[adjusted_contacts.size()-1].pos.x() << ", " << adjusted_contacts[adjusted_contacts.size()-1].pos.y() << ", " << adjusted_contacts[adjusted_contacts.size()-1].pos.z() << "] +++++++++++" << std::endl;
                hasNewContact = true;
            }
        }
    }

    // Create a map, because the getCollisionMarkersFromContacts function takes that as a parameter
    std::map<std::pair<std::string, std::string>, std::vector<collision_detection::Contact>> adjusted_map = {{object_pair, adjusted_contacts}};

    if (c_res.collision) {
        ROS_INFO_STREAM("COLLIDING");// Optional: "contact_point_count: " << c_res.contact_count);
        if (c_res.contact_count > 0)
        {
            std_msgs::ColorRGBA color;
            color.r = 1.0;
            color.g = 0.0;
            color.b = 1.0;
            color.a = 0.5;
            visualization_msgs::MarkerArray markers;

            /* Get the contact points and display them as markers */
            collision_detection::getCollisionMarkersFromContacts(markers, "world", adjusted_map, color,
                                                                ros::Duration(),  // remain until deleted
                                                                0.01);            // radius
            publishMarkers(markers);
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

// ==================================================================================================================================================
// ========================== 5. Handling & publishing contact points ===============================================================================
// ==================================================================================================================================================

void ObjectCollisionTracker::publishMarkers(visualization_msgs::MarkerArray& markers)
{
    // delete old markers
    if (!g_collision_points.markers.empty()) {
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
    x_high_y_low.y() = y_low;
    x_high_y_low.z() = 0;
    corner_points["x_high_y_low"] =  x_high_y_low;
    x_high_y_high.x() = x_high;
    x_high_y_high.y() = y_high;
    x_high_y_high.z() = 0;
    corner_points["x_high_y_high"] =  x_high_y_high;

    return corner_points;
}

collision_detection::Contact ObjectCollisionTracker::adjustContactPoint(const collision_detection::Contact& contact_point,
                                                                        const std::vector<collision_detection::Contact>& adjusted_contact_list) {
    collision_detection::Contact adjusted_contact = contact_point;
    adjusted_contact.pos = determineNearestCornerPoint(contact_point, corner_points, adjusted_contact_list);
    adjusted_contact.pos.z() = contact_point.pos.z();
    return adjusted_contact;
}

bool ObjectCollisionTracker::arePoints2DEqual(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double epsilon) {
    return (std::abs(p1.x() - p2.x()) < epsilon) && (std::abs(p1.y() - p2.y()) < epsilon);
}

Eigen::Vector3d ObjectCollisionTracker::determineNearestCornerPoint(const collision_detection::Contact& contact_point,
                                                                    const std::map<std::string,Eigen::Vector3d>& cornerPoints,
                                                                    const std::vector<collision_detection::Contact>& adjusted_contact_list) {
    // Determine which corners are even possible from the current position of the DLO
    std::map<std::string,Eigen::Vector3d> possible_corners;
    collision_detection::Contact last_corner;

    if (!adjusted_contact_list.empty()) {
        last_corner = adjusted_contact_list[adjusted_contact_list.size()-1];

        // if ( last_corner.pos.isApprox(cornerPoints.at("x_low_y_low"), epsilon) || last_corner.pos.isApprox(cornerPoints.at("x_high_y_high"), epsilon)) {
        if ( arePoints2DEqual(last_corner.pos, cornerPoints.at("x_low_y_low")) || arePoints2DEqual(last_corner.pos, cornerPoints.at("x_high_y_high"))) {
            ROS_INFO_STREAM("We are in the case x_low_y_low OR x_high_y_high.");
            possible_corners.insert({"x_low_y_high",cornerPoints.at("x_low_y_high")});
            possible_corners.insert({"x_high_y_low",cornerPoints.at("x_high_y_low")});
        }
        // else if (last_corner.pos.isApprox(cornerPoints.at("x_low_y_high"), epsilon) || last_corner.pos.isApprox(cornerPoints.at("x_high_y_low"), epsilon)) { // & y_high
        else if ( arePoints2DEqual(last_corner.pos, cornerPoints.at("x_low_y_high"))  || arePoints2DEqual(last_corner.pos, cornerPoints.at("x_high_y_low"))) {
            ROS_INFO_STREAM("We are in the case x_low_y_high OR x_high_y_low.");
            possible_corners.insert({"x_low_y_low",cornerPoints.at("x_low_y_low")});
            possible_corners.insert({"x_high_y_high",cornerPoints.at("x_high_y_high")});
        }
        else {
            ROS_WARN_STREAM("Point is not within epsilon from any corner.");
            possible_corners = cornerPoints;
        }
    }
    else {
        ROS_INFO_STREAM("We don't have any previous contact points.");
        possible_corners = cornerPoints;
    }

    Eigen::Vector3d nearest_corner;
    double shortest_distance = 100.0;
    for (auto pair : possible_corners) {
        double distance = (contact_point.pos - pair.second).norm();
        if (distance < shortest_distance) {
            shortest_distance = distance;
            nearest_corner = pair.second;
        }
    }
    return nearest_corner;
}



// ==================================================================================================================================================
// ==================================================================================================================================================

}

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
    planning_scene_monitor->startWorldGeometryMonitor();
    planning_scene_monitor->startStateMonitor();


    // ======================================== Initialize scene objects ========================================

    // Create & initialize pillar then add i to the planning scene
    moveit_msgs::CollisionObject pillar = objectCollisionTracker->createSimpleObstacle();
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

    //ros::Duration(1.0).sleep(); // Wait for planning scene to be updated
    planning_scene_monitor->requestPlanningSceneState();
    planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor->getPlanningScene();


    // ====================== Declare variables for collision checking and contact points =======================

    objectCollisionTracker->corner_points = objectCollisionTracker->getCornerPoints(pillar); // Determine the corner points of the obstacle
    collision_detection::CollisionResult c_res;
    Contacts original_contacts; // Stores the contact points in the location where they are detected
    Contacts adjusted_contacts; // Stores the contact points after projecting them to the edges of the obstacle
    bool hasNewContact = false; // Becomes true when a new contact is added in an iteration
    int num_segments = 1; // Count of the segments between contact points / robot grippers


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
    objectCollisionTracker->updateObject(steady_hand_tip_pose, moving_hand_tip_pose, dynamic_object, planning_scene, psi, num_segments); // For collision checking
    
    ros::Duration(0.5).sleep(); // Wait period to make sure the objects are added


    // ===================== Check if pillar and DLO are both successfully added in scene  ======================

    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor);
    if (scene->getWorld()->hasObject("pillar")) {
        if(scene->getWorld()->hasObject("dynamic_object")) { 
            ROS_INFO_STREAM("Objects 'pillar' and 'dynamic_object' are visible in PlanningSceneMonitor!"); }
        else { 
            ROS_WARN_STREAM("Only object 'pillar' is visible in PlanningSceneMonitor. DLO isn't visible!"); }
    }
    else { ROS_WARN_STREAM("The object(s) are not visible in PlanningSceneMonitor!"); }


    // ======================= Loop for updating the object pose and checking collisions ========================

    while (ros::ok()) {
        c_res.clear(); // Clear previous collision results

        // Get the current state of the robot & current planning scene
        planning_scene = planning_scene_monitor->getPlanningScene();
        robot_state::RobotStatePtr current_state = move_group_interface.getCurrentState();

        // Derive the gripper positions from the current robot state
        moving_hand_tip_iso  = current_state->getGlobalLinkTransform("panda_1_hand") * tip_pose_in_hand_frame;
        moving_hand_tip_pose = objectCollisionTracker->isometryToPoseStamped(moving_hand_tip_iso, "world");

        // Depending on if there are (new) contact points: define either gripper or contact point as the start of the updated DLO section
        if(hasNewContact) {
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
                                            psi, adjusted_contacts, hasNewContact, num_segments);

        // Check for collisions between the DLO and the obstacle
        objectCollisionTracker->computeCollisionContactPoints(planning_scene, object_group1, object_group2, c_res, original_contacts, adjusted_contacts, hasNewContact);
        
        ros::Duration(0.1).sleep(); // 0.1 sec gap between iterations
    }
    return 0;
}

