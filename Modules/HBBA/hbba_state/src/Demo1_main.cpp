#include "State/StateManager.h"
#include "State/commons/GoToTableState.h"
#include "State/AccueilMotivation/GreetingState.h"
#include "State/AccueilMotivation/GoToAccueilState.h"
#include <ros/ros.h>

#include <hbba_lite/core/DesireSet.h>
#include <hbba_lite/core/RosFilterPool.h>
#include <hbba_lite/core/GecodeSolver.h>
#include <hbba_lite/core/HbbaLite.h>
#include <hbba_lite/core/RosStrategyStateLogger.h>
#include <hbba_lite/core/Strategy.h>
#include <homodeus_hbba_lite/HDMotivations.h>

#include <homodeus_hbba_lite/HDStrategies.h>

#include <typeindex>
#include <memory>

using namespace std;
constexpr bool WAIT_FOR_SERVICE = true;

void startNode(ros::NodeHandle& nodeHandle)
{
    shared_ptr<DesireSet> desireSet = make_shared<DesireSet>();
    shared_ptr<RosFilterPool> filterPool = make_shared<RosFilterPool>(nodeHandle, WAIT_FOR_SERVICE);

    vector<unique_ptr<BaseStrategy>> strategies;
    //setup strategy needed for state
    //strategies.emplace_back(createSpeechToTextStrategy(filterPool)); // does not exist
    strategies.emplace_back(createExploreStrategy(filterPool, desireSet, nodeHandle));
    strategies.emplace_back(createTalkStrategy(filterPool, desireSet, nodeHandle));
    strategies.emplace_back(createGoToStrategy(filterPool, desireSet, nodeHandle));

    unique_ptr<GecodeSolver> solver = make_unique<GecodeSolver>();
    unique_ptr<RosTopicStrategyStateLogger> strategyStateLogger = make_unique<RosTopicStrategyStateLogger>(nodeHandle);
    HbbaLite hbba(desireSet, move(strategies), {/*ressource*/}, move(solver), move(strategyStateLogger));
    ROS_INFO("Allo HBBA lite");
    StateManager stateManager;

    // type_index gotoTableStateType(typeid(GoToTableState));
    type_index greetingStateType = std::type_index(typeid(GreetingState));

    stateManager.addState(
        make_unique<GoToAccueilState>(stateManager, desireSet, nodeHandle)
    );
    stateManager.addState(
        make_unique<GreetingState>(stateManager, desireSet, nodeHandle)
    );
    stateManager.addState(
        make_unique<GoToTableState>(stateManager, desireSet, nodeHandle, greetingStateType)
    );
    ROS_INFO("state gotoTableState fait");

    vector<unique_ptr<HDMotivation>> motivations;
    //setup motivations
    motivations.emplace_back(createAcceuillirMotivation(nodeHandle, desireSet));

    stateManager.switchTo<GoToAccueilState>();

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