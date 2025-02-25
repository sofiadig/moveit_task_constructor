// ROS
#include <ros/ros.h>

// MTC pick/place demo implementation
#include <moveit_task_constructor_demo/dlo_collision_handling.h>

constexpr char LOGNAME[] = "moveit_task_constructor_demo";


int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_dlo_collision_handling");
	ros::NodeHandle nh, pnh("~");

	// Handle Task introspection requests from RViz & feedback during execution
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit_task_constructor_demo::setupDemoScene(pnh);

	// Construct and run pick/place task
	moveit_task_constructor_demo::Dlo_Collision_Handling dlo_collision_handling("dlo_collision_handling", pnh);
	
	if (!dlo_collision_handling.init()) {
		ROS_INFO_NAMED(LOGNAME, "Initialization failed");
		return 1;
	}
	ROS_INFO_NAMED(LOGNAME, "Initialization successful. Starting Planning.");

	if (dlo_collision_handling.plan()) {
		ROS_INFO_NAMED(LOGNAME, "Planning succeded");
		if (pnh.param("execute", false)) {
			dlo_collision_handling.execute();
			ROS_INFO_NAMED(LOGNAME, "Execution complete");
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
	}

	// Keep introspection alive
	ros::waitForShutdown();
	return 0;
}
