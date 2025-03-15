#include <moveit_task_constructor_demo/dlo_collision_handling.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace moveit_task_constructor_demo {

constexpr char LOGNAME[] = "moveit_task_constructor_demo";
constexpr char Dlo_Collision_Handling::LOGNAME[];

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
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "dlo_name", dlo_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "dlo_reference_frame", dlo_reference_frame_);

	// Poses for DLO wrapping motion - clockwise and counterclockwise
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "center_pose_1", center_pose_1_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "center_pose_2", center_pose_2_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "clockwise", clockwise_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "counterclockwise_pose", counterclockwise_pose_);

	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

bool Dlo_Collision_Handling::init() {
	ROS_INFO_NAMED(LOGNAME, "Sofia's Version: Initializing task pipeline");
	const std::string dlo = dlo_name_;
	
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


	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	{
		auto current_state = std::make_unique<stages::CurrentState>("current state");

		// Verify that dlo is not attached
		auto applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
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
	 *         Allow Collision (robots,dlo)             *
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


	/****************************************************
	 *                                                  *
	 *              Close hand_1                        *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("close hand_1", sampling_planner);
		stage->setGroup(hand_1_group_name_);
		stage->setGoal(hand_1_close_pose_);
		t.add(std::move(stage));
	}


	/****************************************************
	 *                                                  *
	 *              Close hand_2                        *
	 *                                                  *
	 ***************************************************/
	Stage* initial_state_ptr = nullptr;
	{
		auto stage = std::make_unique<stages::MoveTo>("close hand_2", sampling_planner);
		stage->setGroup(hand_2_group_name_);
		stage->setGoal(hand_2_close_pose_);
		initial_state_ptr = stage.get();
		t.add(std::move(stage));
	}


	/****************************************************
	 *                                                  *
	 *             Move hand_2 to fixed pose            *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
			"move hand_2 to fixed pose", stages::Connect::GroupPlannerVector{ { arm_2_group_name_, sampling_planner }});
		stage->setTimeout(5.0);
		stage->setProperty("group", arm_2_group_name_);
		stage->setProperty("eef", eef_2_name_);
		stage->setProperty("hand", hand_2_group_name_);
		stage->setProperty("ik_frame", hand_2_frame_);
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *          Hand_2  Fixed center pose               *
	 *                                                  *
	 ***************************************************/
	Stage* hand_2_prep_ptr = nullptr;
	{
		auto prep = std::make_unique<SerialContainer>("hand_2 fixed pose");
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
			hand_center_pose.header.frame_id = object_reference_frame_;
			hand_center_pose.pose.position.x = center_pose_1_[0];
			hand_center_pose.pose.position.y = center_pose_1_[1];
			hand_center_pose.pose.position.z = center_pose_1_[2];
			
			tf2::Quaternion orientation;
			orientation.setRPY(M_PI, 0.0, -M_PI/4);
			hand_center_pose.pose.orientation = tf2::toMsg(orientation);
			
			// Add fixed pose as stage
			auto stage = std::make_unique<stages::FixedCartesianPoses>("hand 2 center pose");
			stage->properties().configureInitFrom(Stage::PARENT);
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
	 *               Move Hand_2 to pose                *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move hand_2 to pose", stages::Connect::GroupPlannerVector{ { arm_1_group_name_, sampling_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}


	/****************************************************
	 *                                                  *
	 *          Hand_1     Move to center               *
	 *                                                  *
	 ***************************************************/
	{
		auto prep = std::make_unique<SerialContainer>("hand_1 center");
		prep->setProperty("group", arm_1_group_name_);
		prep->setProperty("eef", eef_1_name_);
		prep->setProperty("hand", hand_1_group_name_);
		prep->setProperty("ik_frame", hand_1_frame_);

		/****************************************************
  ---- *               Center pose + IK                      *
		 ***************************************************/
		{
			// Create grasp pose
			geometry_msgs::PoseStamped hand_center_pose;
			hand_center_pose.header.frame_id = object_reference_frame_;
			if (clockwise_)	{
				hand_center_pose.pose.position.x = center_pose_2_[0];
				hand_center_pose.pose.position.y = center_pose_2_[1];
				hand_center_pose.pose.position.z = center_pose_2_[2];
			}
			else {
				hand_center_pose.pose.position.x = counterclockwise_pose_[0];
				hand_center_pose.pose.position.y = counterclockwise_pose_[1];
				hand_center_pose.pose.position.z = counterclockwise_pose_[2];
			}

			tf2::Quaternion orientation;
			orientation.setRPY(M_PI, 0.0, -M_PI/4);
			hand_center_pose.pose.orientation = tf2::toMsg(orientation);
			
			// Add fixed pose as stage
			auto stage = std::make_unique<stages::FixedCartesianPoses>("hand_1 center pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->properties().set("marker_ns", "hand_center_pose");
			stage->addPose(hand_center_pose);
			stage->setMonitoredStage(hand_2_prep_ptr);
			
			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("center pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(10);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(hand_1_frame_);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });

			prep->insert(std::move(wrapper));
		}

		// Add grasp container to task
		t.add(std::move(prep));
	}

	if (!clockwise_) {
		/****************************************************
		 *                                                  *
		 *          Hand_1     Move to center               *
		 *                                                  *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("move to center", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(0.19, 0.25);
			stage->setIKFrame(hand_1_frame_);
			stage->properties().set("marker_ns", "retreat");
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = world_frame_;
			
			vec.vector.x = 0.0;
			vec.vector.y = -1.0;
			
			stage->setDirection(vec);
			t.add(std::move(stage));
		}
	

		/****************************************************
		 *                                                  *
		 *          Hand_1     Move forward                 *
		 *                                                  *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("move forward", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			
			stage->setIKFrame(hand_1_frame_);
			stage->properties().set("marker_ns", "retreat");
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = world_frame_;
			vec.vector.x = 1.0;
			vec.vector.y = 0.0;
			stage->setMinMaxDistance(0.3, 0.35);
			
			vec.vector.z = 0.1;
			stage->setDirection(vec);
			t.add(std::move(stage));
		}
	}

	/****************************************************
	 *                                                  *
	 *          Hand_1     Move sideways                *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("move sideways", cartesian_planner);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		
		stage->setIKFrame(hand_1_frame_);
		stage->properties().set("marker_ns", "retreat");
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = world_frame_;
		if (clockwise_) {
			vec.vector.x = 0.0;
			vec.vector.y = -1.0;
			stage->setMinMaxDistance(0.2, 0.3);
		}
		else {
			vec.vector.x = 0.0;
			vec.vector.y = 1.0;
			stage->setMinMaxDistance(0.3, 0.45);
		}
		
		vec.vector.z = 0.1;
		stage->setDirection(vec);
		t.add(std::move(stage));
	}

	if (!clockwise_) {
		/****************************************************
		 *                                                  *
		 *          Hand_1     Move backwards               *
		 *                                                  *
		 ***************************************************/
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

		/****************************************************
		 *                                                  *
		 *          Hand_1     Move sideways                *
		 *                                                  *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("move sideways", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			//stage->setGroup(arm_1_group_name_);
			stage->setMinMaxDistance(0.3, 0.4);
			stage->setIKFrame(hand_1_frame_);
			stage->properties().set("marker_ns", "retreat");
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = world_frame_;
			
			vec.vector.x = 0.0;
			vec.vector.y = -1.0;
			
			vec.vector.z = 0.1;
			stage->setDirection(vec);
			t.add(std::move(stage));
		}

		/****************************************************
		 *                                                  *
		 *          Hand_1     Move forward                 *
		 *                                                  *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("move forward", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			//stage->setGroup(arm_1_group_name_);
			stage->setMinMaxDistance(0.3, 0.4);
			stage->setIKFrame(hand_1_frame_);
			stage->properties().set("marker_ns", "retreat");
			geometry_msgs::Vector3Stamped vec;
			vec.header.frame_id = world_frame_;
			
			vec.vector.x = 1.0;
			vec.vector.y = 0.0;
			
			vec.vector.z = 0.1;
			stage->setDirection(vec);
			t.add(std::move(stage));
		}


	}
	else {
		/****************************************************
		 *                                                  *
		 *          Hand_1     Move backwards               *
		 *                                                  *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("move backwards", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
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

	if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_result.val);
		return false;
	}

	return true;
}
}  // namespace moveit_task_constructor_demo
