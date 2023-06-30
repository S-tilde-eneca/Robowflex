/* Author: Carlos Quintero Pena */

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";
static const std::string OBJECT = "Cube3";
static const std::string END_EFFECTOR = "wrist_roll_link";
static const std::string SCENE_FILE = "package://robowflex_library/yaml/blocks_table.yml";
static const std::vector<double> STOWED = {0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0};


int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();
    fetch->setGroupState(GROUP, STOWED);  // Stow

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by
    // default.
    IO::RVIZHelper rviz(fetch);
    IO::RobotBroadcaster bc(fetch);
    bc.start();

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();
    
    // Load a scene from a YAML file.
    auto scene = std::make_shared<Scene>(fetch);
    scene->fromYAMLFile(SCENE_FILE);
    scene->getCurrentState() = *fetch->getScratchState();
    
    // Move the robot down.
    auto robot_offset = TF::createPoseQ(Eigen::Vector3d{0, 0.45, 0}, Eigen::Quaterniond{0.707, 0, 0, -0.707});//-90 z
    scene->moveAllObjectsGlobal(robot_offset);

    // Visualize the scene in RViz.
    rviz.updateScene(scene);

    // Define goal 
    auto object_pose = scene->getObjectPose(OBJECT);
    auto grasp_offset = TF::createPoseQ(Eigen::Vector3d{0.0, 0.0, 0.2}, Eigen::Quaterniond{0.707, 0, 0.707, 0});
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
    if (not fetch->setFromIK(query))
    {
        RBX_ERROR("IK solution not found");
        return 0;
    }
    
    RBX_INFO("Scene and Goal displayed! Press enter to plan...");
    std::cin.get();
    
    // Create the default planner for the Fetch.
    auto planner = std::make_shared<OMPL::FetchOMPLPipelinePlanner>(fetch, "default");
    planner->initialize();
    
    // Create a motion planning request.
    MotionRequestBuilder request(planner, GROUP);    
    request.useSceneStateAsStart(scene);
    request.setGoalConfiguration(fetch->getScratchState());

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz.
    rviz.updateTrajectory(res);

    // Create a trajectory object for better manipulation.
    auto trajectory = std::make_shared<Trajectory>(res.trajectory_);

    // Output path to a file for visualization.
    trajectory->toYAMLFile("fetch_block.yml");

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
