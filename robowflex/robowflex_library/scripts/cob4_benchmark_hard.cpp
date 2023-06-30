/* Author: Zachary Kingston */
/* Modified by: Juan D. Hernandez */

#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <robowflex_library/log.h>
#include <robowflex_library/detail/cob4.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/io/gnuplot.h>

using namespace robowflex;

/* \file cob4_benchmark.cpp
 * A basic script that demonstrates benchmarking with the Cob4 robot.
 * Benchmarking output is saved in the OMPL format. See
 * https://ompl.kavrakilab.org/benchmark.html for more information on the
 * benchmark data format and how to use. http://plannerarena.org/ can be used to
 * visualize results.
 * Note: This script requires GNUPlot for live visualization of timing data.
 */

static const std::string RIGHT_ARM = "arm_right";
static const std::string LEFT_ARM = "arm_left";
static const std::string OBJECT = "Cube1";
static const std::string END_EFFECTOR = "arm_left_7_link";
static const std::string SCENE_FILE = "package://robowflex_library/yaml/blocks_table_hard.yml";
static const std::vector<double> START_RIGHT = {2.69, 1.70, -0.91, 1.50, -2.14, -2.35, 1.06};
static const std::vector<double> START_LEFT = {-1.14, -1.50, 0.34, -1.50, 0.43, -1.56, -1.20};

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

    // Load a scene from a YAML file.
    auto scene = std::make_shared<Scene>(cob4);
    scene->fromYAMLFile(SCENE_FILE);
    scene->getCurrentState() = *cob4->getScratchState();

    // Create the default planner for the COB4.
    auto planner = std::make_shared<OMPL::Cob4OMPLPipelinePlanner>(cob4);
    planner->initialize();

    // Setup a benchmarking request for the joint and pose motion plan requests.
    Profiler::Options options;
    options.metrics = Profiler::WAYPOINTS | Profiler::CORRECT | Profiler::LENGTH | Profiler::SMOOTHNESS;
    Experiment experiment("unfurl",  // Name of experiment
                          options,   // Options for internal profiler
                          10.0,       // Timeout allowed for ALL queries
                          100);      // Number of trials


    // Define goal 
    auto object_pose = scene->getObjectPose(OBJECT);
    auto grasp_offset = TF::createPoseQ(Eigen::Vector3d{-0.268, -0.01615, -0.005}, Eigen::Quaterniond{ 0.707, 0.0,  0.707, 0.0});
    RobotPose pose;
    pose = TF::identity();
    pose.translate(object_pose.translation());
    pose.translate(grasp_offset.translation());
    pose.rotate(grasp_offset.linear());

    // Create a motion planning request with a joint position goal for the right arm.
    auto request = std::make_shared<MotionRequestBuilder>(planner, LEFT_ARM);
    request->useSceneStateAsStart(scene);
    request->setStartConfiguration(cob4->getScratchState());
    request->setGoalPose(END_EFFECTOR, "base_link", pose);

    request->setConfig("PRM");
    experiment.addQuery("prm", scene, planner, request->getRequest());

    request->setConfig("EST");
    experiment.addQuery("est", scene, planner, request->getRequest());

    // Use the post-query callback to visualize the data live.
    IO::GNUPlotPlanDataSetOutputter plot("time");
    experiment.setPostQueryCallback(
        [&](PlanDataSetPtr dataset, const PlanningQuery &) { plot.dump(*dataset); });

    auto dataset = experiment.benchmark(1);

    OMPLPlanDataSetOutputter output("robowflex_cob4_hard");
    output.dump(*dataset);

    return 0;
}
