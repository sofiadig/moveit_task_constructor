#include <moveit_task_constructor_demo/dlo_collision_handling.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace moveit_task_constructor_demo {

constexpr char LOGNAME[] = "moveit_task_constructor_demo";
constexpr char Dlo_Collision_Handling::LOGNAME[];

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
	if (!psi.applyCollisionObject(object))
		throw std::runtime_error("Failed to spawn object: " + object.id);
}

moveit_msgs::CollisionObject createTable(const ros::NodeHandle& pnh, const bool& createTable_1) {
	std::string table_name, table_reference_frame;
	std::vector<double> table_dimensions;
	geometry_msgs::Pose pose;
	std::size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_reference_frame", table_reference_frame);
	if(createTable_1) {
		errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_1_name", table_name);
		errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_1_dimensions", table_dimensions);
		errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_1_pose", pose);
	}
	else {
		errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_2_name", table_name);
		errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_2_dimensions", table_dimensions);
		errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_2_pose", pose);
	}
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

	moveit_msgs::CollisionObject object;
	object.id = table_name;
	object.header.frame_id = table_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	object.primitives[0].dimensions = table_dimensions;
	pose.position.z += 0.5 * table_dimensions[2];  // align surface with world
	object.primitive_poses.push_back(pose);
	return object;
}

moveit_msgs::CollisionObject createObject(const ros::NodeHandle& pnh) {
	std::string object_name, object_reference_frame;
	std::vector<double> object_dimensions;
	geometry_msgs::Pose pose;
	std::size_t error = 0;
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_dimensions", object_dimensions);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_pose", pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, error);

	moveit_msgs::CollisionObject object;
	object.id = object_name;
	object.header.frame_id = object_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	object.primitives[0].dimensions = object_dimensions;
	pose.position.z += 0.5 * object_dimensions[0];
	object.primitive_poses.push_back(pose);
	return object;
}

moveit_msgs::CollisionObject createSimpleObst(const ros::NodeHandle& pnh) {
	std::string simple_obst_name, simple_obst_reference_frame;
	std::vector<double> simple_obst_dimensions;
	geometry_msgs::Pose pose;
	std::size_t error = 0;
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "simple_obst_name", simple_obst_name);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "simple_obst_reference_frame", simple_obst_reference_frame);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "simple_obst_dimensions", simple_obst_dimensions);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "simple_obst_pose", pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, error);

	moveit_msgs::CollisionObject object;
	object.id = simple_obst_name;
	object.header.frame_id = simple_obst_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	object.primitives[0].dimensions = simple_obst_dimensions;
	pose.position.z += 0.5 * simple_obst_dimensions[0];
	object.primitive_poses.push_back(pose);
	return object;
}

moveit_msgs::CollisionObject createObstacle(const ros::NodeHandle& pnh) {
	std::string obstacle_name, obstacle_reference_frame;
	std::vector<double> obst_dim0, obst_dim1, obst_dim2, obst_dim3, obst_dim4;
	geometry_msgs::Pose pose0, pose1, pose2, pose3, pose4;
	std::size_t error = 0;
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "obstacle_name", obstacle_name);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "obstacle_reference_frame", obstacle_reference_frame);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "obstacle_dimensions_0", obst_dim0);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "obstacle_dimensions_1", obst_dim1);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "obstacle_dimensions_2", obst_dim2);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "obstacle_dimensions_3", obst_dim3);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "obstacle_dimensions_4", obst_dim4);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "obstacle_pose_0", pose0);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "obstacle_pose_1", pose1);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "obstacle_pose_2", pose2);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "obstacle_pose_3", pose3);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "obstacle_pose_4", pose4);
	rosparam_shortcuts::shutdownIfError(LOGNAME, error);

	// Vectors for iteration in for-loop
	std::vector<std::vector<double>> obstacle_dimensions = {obst_dim0, obst_dim1, obst_dim2, obst_dim3, obst_dim4};
	std::vector<geometry_msgs::Pose> poses = {pose0, pose1, pose2, pose3, pose4};

	// Create the obstacle: name, frame, number of primitives
	moveit_msgs::CollisionObject obstacle;
	obstacle.id = obstacle_name;
	obstacle.header.frame_id = obstacle_reference_frame;
	std::size_t num_primitives = obstacle_dimensions.size();
	obstacle.primitives.resize(num_primitives);

	// Add the primitives to the obstacle
	for (std::size_t i = 0; i < num_primitives; i++) {
		obstacle.primitives[i].type = shape_msgs::SolidPrimitive::BOX;
		obstacle.primitives[i].dimensions = obstacle_dimensions[i];
		obstacle.primitive_poses.push_back(poses[i]);
	}
	return obstacle;
}

void setupDemoScene(ros::NodeHandle& pnh) {
	// Add table and object to planning scene
	ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	if (pnh.param("spawn_table", true)) {
		spawnObject(psi, createTable(pnh, true));
		spawnObject(psi, createTable(pnh, false));
	}
	
	//spawnObject(psi, createSimpleObst(pnh));
	// spawnObject(psi, createObject(pnh));
	// spawnObject(psi, createObstacle(pnh));
}

Dlo_Collision_Handling::Dlo_Collision_Handling(const std::string& task_name, const ros::NodeHandle& pnh)
  : pnh_(pnh), task_name_(task_name) {
	loadParameters();
}

void Dlo_Collision_Handling::loadParameters() {
	/****************************************************
	 *                                                  *
	 *               Load Parameters                    *
	 *                                                  *
	 ***************************************************/
	ROS_INFO_NAMED(LOGNAME, "Loading task parameters");

	// Planning group_1 properties: robot 1
	size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_1_group_name", arm_1_group_name_);//arm_1_group_name_=panda_1
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_1_group_name", hand_1_group_name_);//hand_1_group_name_: "hand_1"
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "eef_1_name", eef_1_name_);//eef_1_name_: "hand_1"
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_1_frame", hand_1_frame_);//hand_1_frame_: "panda_1_link8"

    // Planning group_1 properties: robot 2
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_2_group_name", arm_2_group_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_2_group_name", hand_2_group_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "eef_2_name", eef_2_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_2_frame", hand_2_frame_);

	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "world_frame", world_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "grasp_frame_transform", grasp_frame_transform_);

	// Predefined pose targets
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_1_open_pose", hand_1_open_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_1_close_pose", hand_1_close_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_1_home_pose", arm_1_home_pose_);

	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_2_open_pose", hand_2_open_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_2_close_pose", hand_2_close_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_2_home_pose", arm_2_home_pose_);

	// Target object
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object_name", object_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object_dimensions", object_dimensions_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object_reference_frame", object_reference_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "surface_1_link", surface_1_link_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "surface_2_link", surface_2_link_);
	support_surfaces_ = { surface_1_link_, surface_2_link_ };
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "dlo_name", dlo_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "dlo_reference_frame", dlo_reference_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "simple_obst_name", simple_obst_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "simple_obst_reference_frame", simple_obst_reference_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "simple_obst_dimensions", simple_obst_dimensions_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "simple_obst_pose", simple_obst_pose_);

	// Pick/Place metrics
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_min_dist", approach_object_min_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_max_dist", approach_object_max_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_min_dist", lift_object_min_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_max_dist", lift_object_max_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_surface_offset", place_surface_offset_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_pose", place_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "up_pose", up_pose_);

	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "center_pose_1", center_pose_1_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "center_pose_2", center_pose_2_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "min_distance", min_distance_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "max_distance", max_distance_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "clockwise", clockwise_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "cc_pose", cc_pose_);
	

	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

bool Dlo_Collision_Handling::init() {
	ROS_INFO_NAMED(LOGNAME, "Sofia's Version: Initializing task pipeline");
	const std::string dlo = dlo_name_;

	// Reset ROS introspection before constructing the new object
	// TODO(v4hn): global storage for Introspection services to enable one-liner
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());

	Task& t = *task_;
	t.stages()->setName(task_name_);
	t.loadRobotModel();

	// Sampling planner
	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

	// Cartesian planner
	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScalingFactor(0.1);
	cartesian_planner->setMaxAccelerationScalingFactor(0.1);
	cartesian_planner->setStepSize(.01);

	// Set task properties
	// on left side in "": property names (dont change!) and on right side: value you want to give that property
	t.setProperty("group", arm_1_group_name_);
	t.setProperty("eef", eef_1_name_);
	t.setProperty("hand", hand_1_group_name_);
	t.setProperty("hand_grasping_frame", hand_1_frame_);
	t.setProperty("ik_frame", hand_1_frame_);

	// Central middle/ grasp pose
	geometry_msgs::PoseStamped center_pose;
	// Position
	center_pose.header.frame_id = object_reference_frame_;
	center_pose.pose.position.x = 0.5;
	center_pose.pose.position.y = 0.0;
	center_pose.pose.position.z = 1.5;
	// Orientation
	tf2::Quaternion orientation;
	orientation.setRPY(M_PI/2, 0, 0);
	center_pose.pose.orientation = tf2::toMsg(orientation);


	// Central middle/ grasp pose
	geometry_msgs::PoseStamped up_pose;
	// Position
	up_pose.header.frame_id = object_reference_frame_;
	up_pose.pose = up_pose_;
	// up_pose.point.x = 0.0;
	// up_pose.point.y = 0.5;
	// up_pose.point.z = 1.8;


	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	{
		auto current_state = std::make_unique<stages::CurrentState>("current state");

		// Verify that dlo is not attached
		auto applicability_filter =
		    std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
		applicability_filter->setPredicate([dlo](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(dlo)) {
				comment = "dlo with id '" + dlo + "' is already attached and cannot be picked";
				return false;
			}
			return true;
		});
		t.add(std::move(applicability_filter));
	}
	
	/****************************************************
	 *                                                  *
	 *               Open Hand                          *
	 *                                                  *
	 ***************************************************/
	//Stage* initial_state_ptr = nullptr;
	{  // Open Hand
		auto stage = std::make_unique<stages::MoveTo>("open hand_1", sampling_planner);
		stage->setGroup(hand_1_group_name_);
		stage->setGoal(hand_1_open_pose_);
		//initial_state_ptr = stage.get();  // remember start state for monitoring grasp pose generator
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Allow Collision (robots,dlo)       *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collisions (robots <-> dlo)");
		stage->allowCollisions({dlo}, t.getRobotModel()->getJointModelGroup(hand_1_group_name_)->getLinkModelNamesWithCollisionGeometry(),true);
		stage->allowCollisions({dlo}, t.getRobotModel()->getJointModelGroup(arm_1_group_name_)->getLinkModelNamesWithCollisionGeometry(),true);
		stage->allowCollisions({dlo}, t.getRobotModel()->getJointModelGroup(hand_2_group_name_)->getLinkModelNamesWithCollisionGeometry(),true);
		stage->allowCollisions({dlo}, t.getRobotModel()->getJointModelGroup(arm_2_group_name_)->getLinkModelNamesWithCollisionGeometry(),true);
		t.add(std::move(stage));
	}

	// /****************************************************
	//  *                                                  *
	//  *               Allow Collision (hand_2,dlo)       *
	//  *                                                  *
	//  ***************************************************/
	// {
	// 	auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand_2,dlo)");
	// 	stage->allowCollisions({dlo}, t.getRobotModel()->getJointModelGroup(hand_2_group_name_)->getLinkModelNamesWithCollisionGeometry(),true);
	// 	stage->allowCollisions({dlo}, t.getRobotModel()->getJointModelGroup(arm_2_group_name_)->getLinkModelNamesWithCollisionGeometry(),true);
	// 	t.add(std::move(stage));
	// }

	/****************************************************
	 *                                                  *
	 *              Close hand                          *
	 *                                                  *
	 ***************************************************/
	Stage* initial_state_ptr = nullptr;
	{
		auto stage = std::make_unique<stages::MoveTo>("close hand_1", sampling_planner);
		stage->setGroup(hand_1_group_name_);
		stage->setGoal(hand_1_close_pose_);
		// initial_state_ptr = stage.get();
		t.add(std::move(stage));
	}


	/****************************************************
	 *                                                  *
	 *              Close hand 2                        *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("close hand_2", sampling_planner);
		stage->setGroup(hand_2_group_name_);
		stage->setGoal(hand_2_close_pose_);
		initial_state_ptr = stage.get();
		t.add(std::move(stage));
	}


	/****************************************************
	 *                                                  *
	 *        Connect Hand_1 movement to Hand_2         *
	 *                                                  *
	 ***************************************************/
	{  // Move-to pre-grasp
		auto stage = std::make_unique<stages::Connect>(
		    "connect hand_1 and hand_2", stages::Connect::GroupPlannerVector{ { arm_2_group_name_, sampling_planner }});
		stage->setTimeout(5.0);
		//stage->properties().configureInitFrom(Stage::PARENT);
		stage->setProperty("group", arm_2_group_name_);
		stage->setProperty("eef", eef_2_name_);
		stage->setProperty("hand", hand_2_group_name_);
		stage->setProperty("ik_frame", hand_2_frame_);
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *          Hand_2     Move inward                  *
	 *                                                  *
	 ***************************************************/
	Stage* hand_2_prep_ptr = nullptr;
	{
		auto prep = std::make_unique<SerialContainer>("prep hand_2");
		//t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
		//grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });
		prep->setProperty("group", arm_2_group_name_);
		prep->setProperty("eef", eef_2_name_);
		prep->setProperty("hand", hand_2_group_name_);
		prep->setProperty("ik_frame", hand_2_frame_);

		

		/****************************************************
  ---- *               Approach Object                      *
		 ***************************************************/
		{
			// Create grasp pose
			geometry_msgs::PoseStamped hand_center_pose;
			// Position
			hand_center_pose.header.frame_id = object_reference_frame_;
			
			// hand_center_pose.pose.position = center_pose_1_.position;
			hand_center_pose.pose.position.x = center_pose_1_[0];
			hand_center_pose.pose.position.y = center_pose_1_[1];
			hand_center_pose.pose.position.z = center_pose_1_[2];
			
			// Orientation
			tf2::Quaternion orientation;
			orientation.setRPY(M_PI, 0.0, -M_PI/4);
			hand_center_pose.pose.orientation = tf2::toMsg(orientation);
			
			// Add fixed pose as stage
			auto stage = std::make_unique<stages::FixedCartesianPoses>("hand 2 center pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			//stage->setGroup(arm_2_group_name_);
			stage->properties().set("marker_ns", "hand_center_pose");
			stage->addPose(hand_center_pose);
			stage->setMonitoredStage(initial_state_ptr);
			
			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("center pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(hand_2_frame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			prep->insert(std::move(wrapper));
		}
		hand_2_prep_ptr = prep.get();  // remember for monitoring hold pose generator

		// Add grasp container to task
		t.add(std::move(prep));
	}



	/****************************************************
	 *                                                  *
	 *      Connect Hand_1 movement to Hand_2           *
	 *                                                  *
	 ***************************************************/
	{  // Move-to pre-grasp
		auto stage = std::make_unique<stages::Connect>(
		    "connect hand_2 and hand_1", stages::Connect::GroupPlannerVector{ { arm_1_group_name_, sampling_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}


	/****************************************************
	 *                                                  *
	 *          Hand_2     Move inward                  *
	 *                                                  *
	 ***************************************************/
	//Stage* hand_2_prep_ptr = nullptr;
	{
		auto prep = std::make_unique<SerialContainer>("hand_1 sideways");
		//t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
		//grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });
		prep->setProperty("group", arm_1_group_name_);
		prep->setProperty("eef", eef_1_name_);
		prep->setProperty("hand", hand_1_group_name_);
		prep->setProperty("ik_frame", hand_1_frame_);

		

		/****************************************************
  ---- *               Approach Object                      *
		 ***************************************************/
		{
			// Create grasp pose
			geometry_msgs::PoseStamped hand_center_pose;
			// Position
			hand_center_pose.header.frame_id = object_reference_frame_;
			// hand_center_pose.pose.position = center_pose_1_.position;
			if (clockwise_)	{ // hand_center_pose.pose.position = center_pose_1_.position;
				hand_center_pose.pose.position.x = center_pose_2_[0];
				hand_center_pose.pose.position.y = center_pose_2_[1];
				hand_center_pose.pose.position.z = center_pose_2_[2];
			}
			else {
				hand_center_pose.pose.position.x = cc_pose_[0];
				hand_center_pose.pose.position.y = cc_pose_[1];
				hand_center_pose.pose.position.z = cc_pose_[2];
			}
				// hand_center_pose.pose.position.x = center_pose_2_[0];
				// hand_center_pose.pose.position.y = center_pose_2_[1];
				// hand_center_pose.pose.position.z = center_pose_2_[2];

			// Orientation
			tf2::Quaternion orientation;
			orientation.setRPY(M_PI, 0.0, -M_PI/4);
			hand_center_pose.pose.orientation = tf2::toMsg(orientation);
			
			// Add fixed pose as stage
			auto stage = std::make_unique<stages::FixedCartesianPoses>("hand 1 center pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			//stage->setGroup(arm_2_group_name_);
			stage->properties().set("marker_ns", "hand_center_pose");
			stage->addPose(hand_center_pose);
			stage->setMonitoredStage(hand_2_prep_ptr);
			
			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("center pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(hand_1_frame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			prep->insert(std::move(wrapper));
		}
		hand_2_prep_ptr = prep.get();  // remember for monitoring hold pose generator

		// Add grasp container to task
		t.add(std::move(prep));
	}

	/****************************************************
	 *                                                  *
	 *          Hand_1     Move forward                 *
	 *                                                  *
	 ***************************************************/
	//Stage* move_forward_stage_ptr = nullptr;
	{
		auto stage = std::make_unique<stages::MoveRelative>("move forward", cartesian_planner);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		//stage->setGroup(arm_1_group_name_);
		stage->setMinMaxDistance(0.3, 0.4);
		stage->setIKFrame(hand_1_frame_);
		stage->properties().set("marker_ns", "retreat");
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = world_frame_;
		if (clockwise_) {
			vec.vector.x = 1.0;
			vec.vector.y = 0.0;
		}
		else {
			vec.vector.x = 1.0;
			vec.vector.y = 0.0;
		}
		
		vec.vector.z = 0.1;
		stage->setDirection(vec);
		//hand_2_prep_ptr = stage.get();
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *          Hand_1     Move sideways                *
	 *                                                  *
	 ***************************************************/
	//Stage* move_forward_stage_ptr = nullptr;
	{
		auto stage = std::make_unique<stages::MoveRelative>("move sideways", cartesian_planner);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		//stage->setGroup(arm_1_group_name_);
		stage->setMinMaxDistance(0.3, 0.4);
		stage->setIKFrame(hand_1_frame_);
		stage->properties().set("marker_ns", "retreat");
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = world_frame_;
		if (clockwise_) {
			vec.vector.x = 0.0;
			vec.vector.y = -1.0;
		}
		else {
			vec.vector.x = 0.0;
			vec.vector.y = 1.0;
		}
		
		vec.vector.z = 0.1;
		stage->setDirection(vec);
		//hand_2_prep_ptr = stage.get();
		t.add(std::move(stage));
	}

	if (!clockwise_) {
		/****************************************************
		 *                                                  *
		 *          Hand_1     Move backwards               *
		 *                                                  *
		 ***************************************************/
		//Stage* move_forward_stage_ptr = nullptr;
		{
			auto stage = std::make_unique<stages::MoveRelative>("move backward", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			//stage->setGroup(arm_1_group_name_);
			stage->setMinMaxDistance(0.3, 0.4);
			stage->setIKFrame(hand_1_frame_);
			stage->properties().set("marker_ns", "retreat");
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = world_frame_;
			
			vec.vector.x = -1.0;
			vec.vector.y = 0.0;
			
			vec.vector.z = 0.1;
			stage->setDirection(vec);
			t.add(std::move(stage));
		}

	}





	// /******************************************************
	//  *                                                    *
	//  *          Move hand_2 to Home                       *
	//  *                                                    *
	//  *****************************************************/
	// {
	// 	auto stage = std::make_unique<stages::MoveTo>("move hand_2 home", sampling_planner);
	// 	stage->setGroup(arm_2_group_name_);
	// 	stage->setGoal(arm_2_home_pose_);
	// 	stage->restrictDirection(stages::MoveTo::FORWARD);
	// 	t.add(std::move(stage));
	// }


	// prepare Task structure for planning
	try {
		t.init();
	} catch (InitStageException& e) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
		return false;
	}

	return true;
}

bool Dlo_Collision_Handling::plan() {
	ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
	int max_solutions = pnh_.param<int>("max_solutions", 10);
	ROS_INFO_NAMED(LOGNAME, "Max_solutions for planning are set.");

	return static_cast<bool>(task_->plan(max_solutions));
}

bool Dlo_Collision_Handling::execute() {
	ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory");
	moveit_msgs::MoveItErrorCodes execute_result;

	execute_result = task_->execute(*task_->solutions().front());
	// // If you want to inspect the goal message, use this instead:
	// actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>
	// execute("execute_task_solution", true); execute.waitForServer();
	// moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
	// task_->solutions().front()->toMsg(execute_goal.solution);
	// execute.sendGoalAndWait(execute_goal);
	// execute_result = execute.getResult()->error_code;

	if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_result.val);
		return false;
	}

	return true;
}
}  // namespace moveit_task_constructor_demo
