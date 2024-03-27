#include "State/AccueilMotivation/AccueilMotiv_StateManager.h"

#include <ros/ros.h>

#include <hbba_lite/core/DesireSet.h>
#include <hbba_lite/core/RosFilterPool.h>
#include <hbba_lite/core/GecodeSolver.h>
#include <hbba_lite/core/HbbaLite.h>
#include <hbba_lite/core/RosStrategyStateLogger.h>
#include <hbba_lite/core/Strategy.h>

//#include "hbba_core/HDStrategies.h"

#include <typeindex>
#include <memory>

using namespace std;
constexpr bool WAIT_FOR_SERVICE = true;


void startNode(ros::NodeHandle& nodeHandle)
{
    auto desireSet = make_shared<DesireSet>();
    auto filterPool = make_shared<RosFilterPool>(nodeHandle, WAIT_FOR_SERVICE);

    vector<unique_ptr<BaseStrategy>> strategies;
    auto solver = make_unique<GecodeSolver>();
    auto strategyStateLogger = make_unique<RosTopicStrategyStateLogger>(nodeHandle);
    HbbaLite hbba(desireSet, move(strategies), {/*ressource*/}, move(solver), move(strategyStateLogger));
    ROS_INFO("Allo HBBA lite");
    AccueilStateManager stateManager(desireSet, nodeHandle, move(strategies), move(filterPool));

    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hbba_lite_main_node");
    ros::NodeHandle nodeHandle;
    ros::NodeHandle privateNodeHandle("~");
    ROS_INFO("Allo node");


    startNode(nodeHandle);
    return 0;
}