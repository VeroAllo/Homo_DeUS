#include "State/StateManager.h"
#include "State/commons/GoToTableState.h"
#include "State/AccueilMotivation/GreetingState.h"
#include "State/AccueilMotivation/GoToAccueilState.h"
// #include "State/DiscussionState.cpp"
// #include "State/TakeState.cpp"
// #include "State/DropState.cpp"
// #include "State/GoToKitchenState.h"

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
    //setup strategy needed for state
    // strategies.emplace_back(createSpeechToTextStrategy(filterPool));
    // strategies.emplace_back(createExploreStrategy(filterPool));
    // strategies.emplace_back(createTalkStrategy(filterPool, desireSet, nodeHandle));
    // strategies.emplace_back(createGoToStrategy(filterPool));

    auto solver = make_unique<GecodeSolver>();
    auto strategyStateLogger = make_unique<RosTopicStrategyStateLogger>(nodeHandle);
    HbbaLite hbba(desireSet, move(strategies), {/*ressource*/}, move(solver), move(strategyStateLogger));
    ROS_INFO("Allo HBBA lite");
    StateManager stateManager;


    // type_index gotoTableStateType(typeid(GoToTableState));
    type_index greetingStateType = type_index(typeid(GreetingState));
    // type_index discussStateType = type_index(typeid(DiscussionState));
    // type_index takeStateType = type_index(typeid(TakeState));
    // type_index kitchenStateType = type_index(typeid(GoToKitchenState));
    // type_index dropStatetype = type_index(typeid(DropState));

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
    // stateManager.addState(
    //     make_unique<DiscussionState>(stateManager, desireSet, nodeHandle, kitchenStateType)
    // );
    // stateManager.addState(
    //     make_unique<TakeState>(stateManager, desireSet, nodeHandle, kitchenStateType)
    // );
    // stateManager.addState(
    //     make_unique<GoToKitchenState>(stateManager, desireSet, nodeHandle, greetingStateType)
    // );
    // stateManager.addState(
    //     make_unique<DropState>(stateManager, desireSet, nodeHandle, greetingStateType)
    // );
    ROS_INFO("state DropState fait");

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