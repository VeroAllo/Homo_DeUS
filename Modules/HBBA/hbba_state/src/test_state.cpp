#include "State/StateManager.h"
#include "State/commons/GoToTableState.h"
#include "State/AccueilMotivation/GreetingState.h"
#include "State/AccueilMotivation/GoToAccueilState.h"
#include "State/DiscussionState.h"
#include "State/TakeState.h"
#include "State/DropState.h"
#include "State/GoToKitchenState.h"
#include "State/commons/IdleState.h"

#include <ros/ros.h>

#include <hbba_lite/core/DesireSet.h>
#include <hbba_lite/core/RosFilterPool.h>
#include <hbba_lite/core/GecodeSolver.h>
#include <hbba_lite/core/HbbaLite.h>
#include <hbba_lite/core/RosStrategyStateLogger.h>
#include <hbba_lite/core/Strategy.h>
#include <homodeus_hbba_lite/HDMotivations.h>
#include <hbba_lite/core/Motivation.h>

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
    strategies.emplace_back(createTalkStrategy(filterPool, desireSet, nodeHandle));
    strategies.emplace_back(createGoToStrategy(filterPool, desireSet, nodeHandle));
    strategies.emplace_back(createDiscussStrategy(filterPool, desireSet, nodeHandle));
    strategies.emplace_back(createTakeStrategy(filterPool, desireSet, nodeHandle));
    strategies.emplace_back(createDropStrategy(filterPool, desireSet, nodeHandle));
    // strategies.emplace_back(createExploreStrategy(filterPool, desireSet, nodeHandle));


    unique_ptr<GecodeSolver> solver = make_unique<GecodeSolver>();
    unique_ptr<RosTopicStrategyStateLogger> strategyStateLogger = make_unique<RosTopicStrategyStateLogger>(nodeHandle);
    HbbaLite hbba(desireSet, move(strategies), {/*ressource*/}, move(solver), move(strategyStateLogger));
    ROS_INFO("Allo HBBA lite");
    StateManager stateManager;

    type_index greetingStateType = type_index(typeid(GreetingState));
    type_index discussStateType = type_index(typeid(DiscussionState));
    type_index takeStateType = type_index(typeid(TakeState));
    type_index kitchenStateType = type_index(typeid(GoToKitchenState));
    type_index dropStatetype = type_index(typeid(DropState));
    type_index idleStateType = type_index(typeid(IdleState));
    type_index gotoAccueilStateType = type_index(typeid(GoToAccueilState));
    type_index gotoTableStateType = type_index(typeid(GoToTableState));
    


    stateManager.addListStates(0,
        make_unique<IdleState>(stateManager, desireSet, nodeHandle, gotoAccueilStateType, 0)
    );
    stateManager.addState(0, 
        make_unique<GoToAccueilState>(stateManager, desireSet, nodeHandle, greetingStateType, 1)
    );
    stateManager.addState(0,
        make_unique<GreetingState>(stateManager, desireSet, nodeHandle, gotoTableStateType, 2)
    );
    stateManager.addState(0,
        make_unique<GoToTableState>(stateManager, desireSet, nodeHandle, idleStateType, 3)
    );
    stateManager.addListStates(1,
        make_unique<IdleState>(stateManager, desireSet, nodeHandle, gotoTableStateType, 0)
    );
    stateManager.addState(1,
        make_unique<GoToTableState>(stateManager, desireSet, nodeHandle, discussStateType, 1)
    );
    stateManager.addState(1,
        make_unique<DiscussionState>(stateManager, desireSet, nodeHandle, idleStateType, 2, "serveur")
    );
    stateManager.addListStates(2,
        make_unique<IdleState>(stateManager, desireSet, nodeHandle, kitchenStateType, 0)
    );
    stateManager.addState(2,
        make_unique<GoToKitchenState>(stateManager, desireSet, nodeHandle, takeStateType, 1)
    );
    stateManager.addState(2,
        make_unique<TakeState>(stateManager, desireSet, nodeHandle, idleStateType, 2)
    );

    //stateManager.switchTo<IdleState>(0);
    //stateManager.switchTo<IdleState>(1); /*Help vero*/
    stateManager.switchTo<IdleState>(1);

    vector<unique_ptr<Motivation>> motivations;

    motivations.emplace_back(createAccueillirMotivation(nodeHandle,desireSet,&stateManager));
    motivations.emplace_back(createPrendreCommande(nodeHandle,desireSet,&stateManager));
    motivations.emplace_back(createChercherCommande(nodeHandle, desireSet, &stateManager));


    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hbba_state_node");
    ros::NodeHandle nodeHandle;
    ros::NodeHandle privateNodeHandle("~");

    startNode(nodeHandle);
    return 0;
}