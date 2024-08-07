#include <moveit_task_constructor_demo/dual_pickplace.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace moveit_task_constructor_demo {

constexpr char LOGNAME[] = "moveit_task_constructor_demo";
constexpr char Dual_Pickplace::LOGNAME[];

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
	spawnObject(psi, createObject(pnh));
	//spawnObject(psi, createObstacle(pnh));
}

Dual_Pickplace::Dual_Pickplace(const std::string& task_name, const ros::NodeHandle& pnh)
  : pnh_(pnh), task_name_(task_name) {
	loadParameters();
}

void Dual_Pickplace::loadParameters() {
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

	// Pick/Place metrics
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_min_dist", approach_object_min_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_max_dist", approach_object_max_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_min_dist", lift_object_min_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_max_dist", lift_object_max_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_surface_offset", place_surface_offset_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_pose", place_pose_);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

bool Dual_Pickplace::init() {
	ROS_INFO_NAMED(LOGNAME, "Sofia's Version: Initializing task pipeline");
	const std::string object = object_name_;

	// Set up line marker publisher
	// ros::Publisher marker_pub = pnh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

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
	cartesian_planner->setMaxVelocityScalingFactor(1.0);
	cartesian_planner->setMaxAccelerationScalingFactor(1.0);
	cartesian_planner->setStepSize(.01);

	// Set task properties
	// on left side in "": property names (dont change!) and on right side: value you want to give that property
	t.setProperty("group", arm_1_group_name_);
	t.setProperty("eef", eef_1_name_);
	t.setProperty("hand", hand_1_group_name_);
	t.setProperty("hand_grasping_frame", hand_1_frame_);
	t.setProperty("ik_frame", hand_1_frame_);

	// Central hold/ grasp pose
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

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	{
		auto current_state = std::make_unique<stages::CurrentState>("current state");

		// Verify that object is not attached
		auto applicability_filter =
		    std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
		applicability_filter->setPredicate([object](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				comment = "object with id '" + object + "' is already attached and cannot be picked";
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
	Stage* initial_state_ptr = nullptr;
	{  // Open Hand
		auto stage = std::make_unique<stages::MoveTo>("open hand_1", sampling_planner);
		stage->setGroup(hand_1_group_name_);
		stage->setGoal(hand_1_open_pose_);
		initial_state_ptr = stage.get();  // remember start state for monitoring grasp pose generator
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Move Hand_1 to Pick                *
	 *                                                  *
	 ***************************************************/
	{  // Move-to pre-grasp
		auto stage = std::make_unique<stages::Connect>(
		    "move to pick", stages::Connect::GroupPlannerVector{ { arm_1_group_name_, sampling_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Pick Object                        *
	 *                                                  *
	 ***************************************************/
	Stage* pick_stage_ptr = nullptr;
	{
		auto grasp = std::make_unique<SerialContainer>("pick object");
		t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

		/****************************************************
  ---- *               Approach Object                      *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
			stage->properties().set("marker_ns", "approach_object");
			stage->properties().set("link", hand_1_frame_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);

			// Set hand_1 forward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = hand_1_frame_;
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Fixed Grasp Pose       			     *
		 ***************************************************/
		{
			// Create grasp pose
			geometry_msgs::PoseStamped grasp_pose;
			// Position
			grasp_pose.header.frame_id = object_reference_frame_;
			grasp_pose.pose.position.x = 0.6;
			grasp_pose.pose.position.y = 0.5;
			grasp_pose.pose.position.z = 1.1;
			grasp_pose.pose.position.z += 0.2 * object_dimensions_[0] + place_surface_offset_;

			// Orientation
			tf2::Quaternion orientation;
			orientation.setRPY(0, 0, 0);
			grasp_pose.pose.orientation = tf2::toMsg(orientation);
			
			// Add fixed pose as stage
			auto stage = std::make_unique<stages::FixedCartesianPoses>("fixed grasp pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->properties().set("marker_ns", "grasp_pose");
			stage->addPose(grasp_pose);
			stage->setMonitoredStage(initial_state_ptr);
			
			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(grasp_frame_transform_, hand_1_frame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			grasp->insert(std::move(wrapper));
		}

		/****************************************************
  ---- *               Allow Collision (hand_1 object)        *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand_1,object)");
			stage->allowCollisions(
			    object, t.getRobotModel()->getJointModelGroup(hand_1_group_name_)->getLinkModelNamesWithCollisionGeometry(),
			    true);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Close Hand                           *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand_1", sampling_planner);
			stage->setGroup(hand_1_group_name_);
			stage->setGoal(hand_1_close_pose_);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Attach Object                        *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
			stage->attachObject(object, hand_1_frame_);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Allow collision (object support)     *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
			stage->allowCollisions({ object }, support_surfaces_, true);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Lift object                          *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			//stage->setGroup("");
			stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
			stage->setIKFrame(hand_1_frame_);
			stage->properties().set("marker_ns", "lift_object");

			// Set upward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = world_frame_;
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *         Forbid collision (object support)          *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,support)");
			stage->allowCollisions({ object }, support_surfaces_, false);
			grasp->insert(std::move(stage));
		}

		pick_stage_ptr = grasp.get();  // remember for monitoring hold pose generator

		// Add grasp container to task
		t.add(std::move(grasp));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Hold                              *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to hold", stages::Connect::GroupPlannerVector{ { arm_1_group_name_, sampling_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Hold Object in air                        *
	 *                                                    *
	 *****************************************************/
	Stage* hold_stage_ptr = nullptr;
	{
		auto hold = std::make_unique<SerialContainer>("hold object");
		t.properties().exposeTo(hold->properties(), { "eef", "hand", "group" });
		hold->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

		/****************************************************
  ---- *               Fixed Hold Pose       			     *
		 ***************************************************/
		{
			// Create hold pose
			geometry_msgs::PoseStamped hold_pose;
			hold_pose = center_pose;
			hold_pose.pose.position.y += 0.3 * object_dimensions_[0];
			
			// Add fixed pose as stage
			auto stage = std::make_unique<stages::FixedCartesianPoses>("fixed hold pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->properties().set("marker_ns", "hold_pose");
			stage->addPose(hold_pose);
			stage->setMonitoredStage(pick_stage_ptr);
			
			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("hold pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(grasp_frame_transform_, hand_1_frame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			hold->insert(std::move(wrapper));
		}

		hold_stage_ptr = hold.get();
		// Add hold container to task
		t.add(std::move(hold));
	}

	/******************************************************
	 *                                                    *
	 *          Connect the two stages                    *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>("connect to hand_2 stage", stages::Connect::GroupPlannerVector{
			{ arm_1_group_name_, sampling_planner }, { arm_2_group_name_, sampling_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Grab Object hand_2                        *
	 *                                                    *
	 *****************************************************/
	Stage* pick_2_stage_ptr = nullptr;
	{
		auto takeover = std::make_unique<SerialContainer>("hand_2 take over object");
		//t.properties().exposeTo(takeover->properties(), { "eef", "hand", "group", "ik_frame" });
		takeover->properties().set("group", arm_2_group_name_);
		takeover->properties().set("eef", eef_2_name_);
		takeover->properties().set("hand", hand_2_group_name_);
		takeover->properties().set("hand_grasping_frame", hand_2_frame_);
		takeover->properties().set("ik_frame", hand_2_frame_);

		/****************************************************
  ---- *               Approach Object                      *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
			stage->properties().set("marker_ns", "approach_object");
			stage->properties().set("link", hand_2_frame_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);

			// Set hand_1 forward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = hand_2_frame_;
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			takeover->insert(std::move(stage));
		}

		/****************************************************
  ---- *            Fixed Grasp Pose hand_2                 *
		 ***************************************************/
		{
			// Create grasp pose
			geometry_msgs::PoseStamped grasp_pose;
			grasp_pose = center_pose;
			grasp_pose.pose.position.y -= 0.4 * object_dimensions_[0];
			
			// Add fixed pose as stage
			auto stage = std::make_unique<stages::FixedCartesianPoses>("fixed grasp 2 pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->properties().set("marker_ns", "grasp_pose");
			stage->addPose(grasp_pose);
			stage->setMonitoredStage(hold_stage_ptr);
			
			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("grasp 2 pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(grasp_frame_transform_, hand_2_frame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			takeover->insert(std::move(wrapper));
		}
		/****************************************************
  ---- *           Allow Collision (hand_2 object)          *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand_2, object)");
			stage->allowCollisions(
			    object, t.getRobotModel()->getJointModelGroup(hand_2_group_name_)->getLinkModelNamesWithCollisionGeometry(),
			    true);
			takeover->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Close Hand_2                         *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand_2", sampling_planner);
			stage->setGroup(hand_2_group_name_);
			stage->setGoal(hand_2_close_pose_);
			takeover->insert(std::move(stage));
		}

		/******************************************************
  ---- *                 Open Hand 1                          *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("open hand_1", sampling_planner);
			stage->setGroup(hand_1_group_name_);
			stage->setGoal(hand_1_open_pose_);
			takeover->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Forbid collision (hand_1, object)           *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand_1,object)");
			stage->allowCollisions(object_name_, *t.getRobotModel()->getJointModelGroup(hand_1_group_name_), false);
			takeover->insert(std::move(stage));
		}
		
		/******************************************************
  ---- *          Detach Object from hand_1                   *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object from hand_1");
			stage->detachObject(object_name_, hand_1_frame_);
			takeover->insert(std::move(stage));
		}

		/****************************************************
  .... *           Attach Object to hand_2                  *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object to hand_2");
			stage->attachObject(object, hand_2_frame_);
			takeover->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Retreat Motion                            *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("retreat after takeover", cartesian_planner);
			//stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setGroup(arm_1_group_name_);
			stage->setMinMaxDistance(.04, .25);
			stage->setIKFrame(hand_1_frame_);
			stage->properties().set("marker_ns", "retreat");
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = hand_1_frame_;
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			takeover->insert(std::move(stage));
		}

		pick_2_stage_ptr = takeover.get();
		t.add(std::move(takeover));
	}

	// =================doesnt seem to work:=========Reset task properties for second part===================================
	// on left side in "": property names (dont change!) and on right side: value you want to give that property
	// t.setProperty("group", arm_2_group_name_);
	// t.setProperty("eef", eef_2_name_);
	// t.setProperty("hand", hand_2_group_name_);
	// t.setProperty("hand_grasping_frame", hand_2_frame_);
	// t.setProperty("ik_frame", hand_2_frame_);

	/******************************************************
	 *                                                    *
	 *          Move Hand_2 to Place                      *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>("move hand_2 to place", stages::Connect::GroupPlannerVector{ 
			{ arm_1_group_name_, sampling_planner }, { arm_2_group_name_, sampling_planner } });
		stage->setTimeout(5.0);
		//stage->properties().configureInitFrom(Stage::PARENT);
		stage->properties().set("group", arm_2_group_name_);
		stage->properties().set("eef", eef_2_name_);
		stage->properties().set("hand", hand_2_group_name_);
		stage->properties().set("hand_grasping_frame", hand_2_frame_);
		stage->properties().set("ik_frame", hand_2_frame_);
		//------------
		t.add(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Place Object                              *
	 *                                                    *
	 *****************************************************/
	{
		auto place = std::make_unique<SerialContainer>("place object");
		place->properties().set("group", arm_2_group_name_);
		place->properties().set("eef", eef_2_name_);
		place->properties().set("hand", hand_2_group_name_);
		place->properties().set("hand_grasping_frame", hand_2_frame_);
		place->properties().set("ik_frame", hand_2_frame_);

		/******************************************************
  ---- *          Lower Object                                *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
			stage->properties().set("marker_ns", "lower_object");
			stage->properties().set("link", hand_2_frame_);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(.03, .13);

			// Set downward direction
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = world_frame_;
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Fixed Place Pose       			    *
		 ***************************************************/
		{
			// Create place pose
			geometry_msgs::PoseStamped place_pose;
			// Position
			place_pose.header.frame_id = object_reference_frame_;
			place_pose.pose.position.x = 0.6;
			place_pose.pose.position.y = -0.5;
			place_pose.pose.position.z = 1.1;
			place_pose.pose.position.z += 0.9 * object_dimensions_[0] + place_surface_offset_;

			// Orientation
			tf2::Quaternion orientation;
			orientation.setRPY(0, 0, 0);
			place_pose.pose.orientation = tf2::toMsg(orientation);
			
			// Add fixed pose as stage
			auto stage = std::make_unique<stages::FixedCartesianPoses>("fixed place pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->properties().set("marker_ns", "place_pose");
			stage->addPose(place_pose);
			stage->setMonitoredStage(pick_2_stage_ptr);
			
			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(grasp_frame_transform_, hand_2_frame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			place->insert(std::move(wrapper));
		}

		/******************************************************
  ---- *                Open Hand                             *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("open hand_2", sampling_planner);
			stage->setGroup(hand_2_group_name_);
			stage->setGoal(hand_2_open_pose_);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Forbid collision (hand, object)             *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand_2,object)");
			stage->allowCollisions(object_name_, *t.getRobotModel()->getJointModelGroup(hand_2_group_name_), false);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Detach Object                               *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
			stage->detachObject(object_name_, hand_2_frame_);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Retreat Motion                            *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(.12, .25);
			stage->setIKFrame(hand_2_frame_);
			stage->properties().set("marker_ns", "retreat");
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = hand_2_frame_;
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}

		// Add place container to task
		t.add(std::move(place));
	}

	/******************************************************
	 *                                                    *
	 *          Move hand_1 to Home                       *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("move hand_1 home", sampling_planner);
		//stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->setGroup(arm_1_group_name_);
		stage->setGoal(arm_1_home_pose_);
		stage->restrictDirection(stages::MoveTo::FORWARD);
		t.add(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Move hand_2 to Home                       *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("move hand_2 home", sampling_planner);
		stage->setGroup(arm_2_group_name_);
		stage->setGoal(arm_2_home_pose_);
		stage->restrictDirection(stages::MoveTo::FORWARD);
		t.add(std::move(stage));
	}


	// prepare Task structure for planning
	try {
		t.init();
	} catch (InitStageException& e) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
		return false;
	}

	return true;
}

bool Dual_Pickplace::plan() {
	ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
	int max_solutions = pnh_.param<int>("max_solutions", 10);
	ROS_INFO_NAMED(LOGNAME, "Max_solutions for planning are set.");

	return static_cast<bool>(task_->plan(max_solutions));
}

bool Dual_Pickplace::execute() {
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
