/* Author: Carlos Quintero Pena */

#include <robowflex_library/log.h>
#include <robowflex_library/detail/cob4.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

using namespace robowflex;

static const std::string RIGHT_ARM = "arm_right";
static const std::string LEFT_ARM = "arm_left";
static const std::string OBJECT = "Cube1";
static const std::string END_EFFECTOR = "arm_left_7_link";
static const std::string SCENE_FILE = "package://robowflex_library/yaml/blocks_table_pick_and_place.yml";
static const std::vector<double> START_RIGHT = {2.69, 1.70, -0.91, 1.50, -2.14, -2.35, 1.06};
static const std::vector<double> START_LEFT = {-1.14, -1.50, 0.34, -1.50, 0.43, -1.56, -1.20};
static const std::vector< std::string > touch_links = {"arm_left_7_link"};

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Care-O-Bot4 robot.
    auto cob4 = std::make_shared<Cob4Robot>();
    cob4->initialize();
    cob4->loadKinematics(LEFT_ARM);
    cob4->setGroupState(RIGHT_ARM, START_RIGHT);
    cob4->setGroupState(LEFT_ARM, START_LEFT);

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by default.
    IO::RVIZHelper rviz(cob4);
    IO::RobotBroadcaster bc(cob4);
    bc.start();

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();
    
    // Load a scene from a YAML file.
    auto scene = std::make_shared<Scene>(cob4);
    scene->fromYAMLFile(SCENE_FILE);
    scene->getCurrentState() = *cob4->getScratchState();
    
    // Move the robot down.
    auto robot_offset = TF::createPoseQ(Eigen::Vector3d{0.15, 0.3, 0}, Eigen::Quaterniond{0.707, 0, 0, -0.707});//-90 z
    scene->moveAllObjectsGlobal(robot_offset);
    
    // Visualize the scene in RViz.
    rviz.updateScene(scene);
    
    // Define goal 
    auto object_pose = scene->getObjectPose(OBJECT);
    auto grasp_offset = TF::createPoseQ(Eigen::Vector3d{-0.268, -0.01615, -0.005}, Eigen::Quaterniond{ 0.707, 0.0,  0.707, 0.0});
    RobotPose pose;
    pose = TF::identity();
    pose.translate(object_pose.translation());
    pose.translate(grasp_offset.translation());
    pose.rotate(grasp_offset.linear());
    
    rviz.addTransformMarker("goal", "map", pose);
    rviz.updateMarkers();
    
    RBX_INFO("Scene and Goal displayed! Press enter to plan...");
    std::cin.get();
    
    // Create the default planner for the COB4.
    auto planner = std::make_shared<OMPL::Cob4OMPLPipelinePlanner>(cob4);
    planner->initialize();

    // Create a motion planning request with a joint position goal for the right arm.
    MotionRequestBuilder request(planner, LEFT_ARM);
    request.useSceneStateAsStart(scene);
    request.setStartConfiguration(cob4->getScratchState());
    request.setGoalPose(END_EFFECTOR, "base_link", pose);

    // Do motion planning!
    request.setConfig("RRTConnect");
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz
    rviz.updateTrajectory(res);

    // Create a trajectory object for better manipulation.
    auto trajectory = std::make_shared<Trajectory>(res.trajectory_);
    moveit_msgs::RobotTrajectory trajectory_msg = trajectory->getMessage();
    std::size_t tam = trajectory->getNumWaypoints();
    // Output path to a file for visualization.
    trajectory->toYAMLFile("cob4_block.yml");
    
    // Visualize the scene in RViz.
    

    cob4->setGroupState(LEFT_ARM, {trajectory_msg.joint_trajectory.points[tam-1].positions[0], trajectory_msg.joint_trajectory.points[tam-1].positions[1], trajectory_msg.joint_trajectory.points[tam-1].positions[2], trajectory_msg.joint_trajectory.points[tam-1].positions[3], trajectory_msg.joint_trajectory.points[tam-1].positions[4], trajectory_msg.joint_trajectory.points[tam-1].positions[5], trajectory_msg.joint_trajectory.points[tam-1].positions[6],trajectory_msg.joint_trajectory.points[tam-1].positions[7]});
    request.setStartConfiguration(cob4->getScratchState());
    scene->getCurrentState() = *cob4->getScratchState();
    RobotPose current_pose;
    current_pose = cob4->getLinkTF(END_EFFECTOR);

    request.setGoalPose(END_EFFECTOR, "base_link", current_pose);

    rviz.updateScene(scene);
    // Define goal 
    auto object_pose_2 = scene->getObjectPose(OBJECT);
    auto grasp_offset_2 = TF::createPoseQ(Eigen::Vector3d{-0.268, -0.33, -0.005}, Eigen::Quaterniond{ 0.707, 0.0,  0.707, 0.0});
    RobotPose pose_2;
    pose_2 = TF::identity();
    pose_2.translate(object_pose_2.translation());
    pose_2.translate(grasp_offset_2.translation());
    pose_2.rotate(grasp_offset_2.linear());
    
    rviz.addTransformMarker("goal", "map", pose_2);
    rviz.updateMarkers();
    scene->getCurrentState() = *cob4->getScratchState();    
    scene->attachObject(*cob4->getScratchState(), OBJECT, END_EFFECTOR, touch_links);
    // Create a motion planning request with a joint position goal for the right arm.
    rviz.updateScene(scene);
    MotionRequestBuilder request_2(planner, LEFT_ARM);
    request_2.useSceneStateAsStart(scene);
    request_2.setStartConfiguration(cob4->getScratchState());
    request_2.setGoalPose(END_EFFECTOR, "base_link", pose_2);
    // Do motion planning!
    request_2.setConfig("RRTConnect");
    planning_interface::MotionPlanResponse res_2 = planner->plan(scene, request_2.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz
    rviz.updateTrajectory(res_2);
    

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
