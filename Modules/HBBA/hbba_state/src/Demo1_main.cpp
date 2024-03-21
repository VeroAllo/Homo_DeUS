#include "State/StateManager.h"
#include "State/GoToTableState.h"
#include "State/GreetingState.h"

#include <ros/ros.h>

#include <hbba_lite/core/DesireSet.h>
#include <hbba_lite/core/RosFilterPool.h>
#include <hbba_lite/core/GecodeSolver.h>
#include <hbba_lite/core/HbbaLite.h>
#include <hbba_lite/core/RosStrategyStateLogger.h>

//#include "hbba_core/HDStrategies.h"

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

    StateManager stateManager;

    stateManager.addState(
        make_unique<GreetingState>(stateManager, desireSet, nodeHandle)
    );
    stateManager.addState(
        make_unique<GoToTableState>(stateManager, desireSet, nodeHandle)
    );




    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hbba_lite_main_node");
    ros::NodeHandle nodeHandle;
    ros::NodeHandle privateNodeHandle("~");


    startNode(nodeHandle);
    return 0;
}