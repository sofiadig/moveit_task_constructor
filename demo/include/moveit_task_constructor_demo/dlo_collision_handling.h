// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Constraints.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/fixed_cartesian_poses.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>


// Marker
#include <visualization_msgs/Marker.h>
// #include <geometry_msgs/Point.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <map>


namespace moveit_task_constructor_demo {
	using namespace moveit::task_constructor;

// prepare a demo environment from ROS parameters under pnh
// void setupDemoScene(ros::NodeHandle& pnh);
// void setupLineMarker(ros::NodeHandle& pnh, ros::Publisher marker_pub);
class Dlo_Collision_Handling
{
	public:
		Dlo_Collision_Handling(const std::string& task_name, const ros::NodeHandle& pnh);
		~Dlo_Collision_Handling() = default;

		bool init();

		bool plan();

		bool execute();

	private:
		void loadParameters();

		static constexpr char LOGNAME[]{ "dlo_collision_handling" };

		ros::NodeHandle pnh_;

		std::string task_name_;
		moveit::task_constructor::TaskPtr task_;

		// planning group properties
		std::string arm_1_group_name_;
		std::string eef_1_name_;
		std::string hand_1_group_name_;
		std::string hand_1_frame_;
		// planning group properties
		std::string arm_2_group_name_;
		std::string eef_2_name_;
		std::string hand_2_group_name_;
		std::string hand_2_frame_;

		// object + surface
		std::vector<std::string> support_surfaces_;
		std::string object_reference_frame_;
		std::string surface_1_link_;
		std::string surface_2_link_;
		std::string object_name_;
		std::string world_frame_;
		std::vector<double> object_dimensions_;

		// DLO
		std::string dlo_name_;
		std::string dlo_reference_frame_;

		// Simple obstacle
		std::string simple_obst_name_;
		std::string simple_obst_reference_frame_;
		std::string simple_obst_dimensions_;
		std::string simple_obst_pose_;

		// Predefined pose targets
		std::string hand_1_open_pose_;
		std::string hand_1_close_pose_;
		std::string arm_1_home_pose_;
		std::string hand_2_open_pose_;
		std::string hand_2_close_pose_;
		std::string arm_2_home_pose_;

		// Pick metrics
		Eigen::Isometry3d grasp_frame_transform_;
		double approach_object_min_dist_;
		double approach_object_max_dist_;
		double lift_object_min_dist_;
		double lift_object_max_dist_;

		// Place metrics
		geometry_msgs::Pose place_pose_;
		double place_surface_offset_;
		geometry_msgs::Pose up_pose_;

		std::vector<double> center_pose_2_;
		std::vector<double> center_pose_1_;
		std::vector<double> counterclockwise_pose_;
		bool clockwise_;
		double min_distance_;
		double max_distance_;
};
}