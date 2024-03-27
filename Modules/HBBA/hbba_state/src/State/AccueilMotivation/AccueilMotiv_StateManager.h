#ifndef HD_ACCUEIL_MOTIVATION_STATE_MANAGER_H
#define HD_ACCUEIL_MOTIVATION_STATE_MANAGER_H

#include "../StateManager.h"

#include <memory>
#include <hbba_lite/core/Strategy.h>

class AccueilStateManager: StateManager
{
    protected:
    std::shared_ptr<DesireSet> m_desireSet;
    ros::NodeHandle& m_nodeHandle;
    
    public:
    AccueilStateManager(
        std::shared_ptr<DesireSet> desireSet,
        ros::NodeHandle& nodeHandle,
        std::vector<std::unique_ptr<BaseStrategy>> strategies,
        std::shared_ptr<FilterPool> filterPool);
    ~AccueilStateManager();
private:
    void setup(std::vector<std::unique_ptr<BaseStrategy>> strategies,std::shared_ptr<FilterPool> filterPool);
};
#endif
