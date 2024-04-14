cmake_minimum_required(VERSION 3.1.3)
project(moveit_task_constructor_demo)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	moveit_core
	moveit_task_constructor_core
	moveit_ros_planning_interface
	rosparam_shortcuts
	urdf
)

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )

moveit_build_options()

catkin_package(
	CATKIN_DEPENDS roscpp # urdf moveit_core moveit_ros_planning_interface
)


# added by me
add_library(${PROJECT_NAME}_pick_place_task_sofia src/pick_place_task_sofia.cpp)
target_link_libraries(${PROJECT_NAME}_pick_place_task_sofia ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_pick_place_task_sofia PUBLIC include)
target_include_directories(${PROJECT_NAME}_pick_place_task_sofia SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
add_dependencies(${PROJECT_NAME}_pick_place_task_sofia ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
install(TARGETS ${PROJECT_NAME}_pick_place_task_sofia
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# until here

add_library(${PROJECT_NAME}_pick_place_task src/pick_place_task.cpp)
target_link_libraries(${PROJECT_NAME}_pick_place_task ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_pick_place_task PUBLIC include)
target_include_directories(${PROJECT_NAME}_pick_place_task SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
add_dependencies(${PROJECT_NAME}_pick_place_task ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
install(TARGETS ${PROJECT_NAME}_pick_place_task
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# declare a demo consisting of a single cpp file
function(demo name)
	add_executable(${PROJECT_NAME}_${name} src/${name}.cpp)
	add_dependencies(${PROJECT_NAME}_${name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	target_link_libraries(${PROJECT_NAME}_${name} ${catkin_LIBRARIES})
	target_include_directories(${PROJECT_NAME}_${name} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
	set_target_properties(${PROJECT_NAME}_${name} PROPERTIES OUTPUT_NAME ${name} PREFIX "")
	install(TARGETS ${PROJECT_NAME}_${name}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
endfunction()

demo(cartesian)
demo(modular)
demo(alternative_path_costs)
demo(ik_clearance_cost)
demo(fallbacks_move_to)

demo(pick_place_demo)
target_link_libraries(${PROJECT_NAME}_pick_place_demo ${PROJECT_NAME}_pick_place_task)

add_executable(dual_demo src/dual_with_main.cpp)
target_link_libraries(dual_demo ${catkin_LIBRARIES})
target_include_directories(dual_demo SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

demo(pick_place_demo_sofia)
target_link_libraries(${PROJECT_NAME}_pick_place_demo_sofia ${PROJECT_NAME}_pick_place_task_sofia)


install(DIRECTORY launch config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_subdirectory(test)
