/* Author: Carlos Quintero Pena */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

using namespace robowflex;

static const std::string GROUP = "manipulator";
static const std::string OBJECT = "Cube3";
static const std::string END_EFFECTOR = "ee_link";
static const std::string SCENE_FILE = "package://robowflex_library/yaml/blocks_table.yml";
static const std::vector<double> START = {{0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0}};

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default UR5 robot.
    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();
    ur5->setGroupState(GROUP, START);  // Stow

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by
    // default.
    IO::RVIZHelper rviz(ur5);
    IO::RobotBroadcaster bc(ur5);
    bc.start();

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    // Load a scene from a YAML file.
    auto scene = std::make_shared<Scene>(ur5);
    scene->fromYAMLFile(SCENE_FILE);
    scene->getCurrentState() = *ur5->getScratchState();
    
    // Move the robot down.
    auto robot_offset = TF::createPoseQ(Eigen::Vector3d{0.0, 0.0, 0.15}, Eigen::Quaterniond{1, 0, 0, 0});
    scene->moveAllObjectsGlobal(robot_offset);

    // Visualize the scene in RViz.
    rviz.updateScene(scene);

    // Define goal 
    auto object_pose = scene->getObjectPose(OBJECT);
    auto grasp_offset = TF::createPoseQ(Eigen::Vector3d{0.0, 0.0, 0.2}, Eigen::Quaterniond{0.5, -0.5, 0.5, 0.5});
    RobotPose pose;
    pose = TF::identity();
    pose.translate(object_pose.translation());
    pose.translate(grasp_offset.translation());
    pose.rotate(grasp_offset.linear());
    
    rviz.addTransformMarker("goal", "map", pose);
    rviz.updateMarkers();
    
    // Create IK query.
    const auto query = Robot::IKQuery(GROUP, {pose}, {END_EFFECTOR}, constants::ik_tolerance,
                                          constants::ik_vec_tolerance, scene);
    
    // Set goal configuration from IK query.
    if (not ur5->setFromIK(query))
    {
        RBX_ERROR("IK solution not found");
        return 0;
    }
    
    // Create the default planner for the UR5.
    auto planner = std::make_shared<OMPL::UR5OMPLPipelinePlanner>(ur5, "default");
    planner->initialize();
    
    // Create a motion planning request.
    MotionRequestBuilder request(planner, GROUP);    
    request.useSceneStateAsStart(scene);
    request.setGoalConfiguration(ur5->getScratchState());

    RBX_INFO("Scene and Goal displayed! Press enter to plan...");
    std::cin.get();

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz.
    rviz.updateTrajectory(res);

    // Create a trajectory object for better manipulation.
    auto trajectory = std::make_shared<Trajectory>(res.trajectory_);

    // Output path to a file for visualization.
    trajectory->toYAMLFile("ur5_block.yml");

    RBX_INFO("Press enter to remove goal and scene.");
    std::cin.get();

    // Clean up RViz.
    rviz.removeMarker("goal");
    rviz.updateMarkers();
    rviz.removeScene();

    RBX_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
