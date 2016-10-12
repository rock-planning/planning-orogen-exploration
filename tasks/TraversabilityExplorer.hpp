#ifndef EXPLORATION_TRAVERSABILITYEXPLORER_TASK_HPP
#define EXPLORATION_TRAVERSABILITYEXPLORER_TASK_HPP

#include "exploration/TraversabilityExplorerBase.hpp"

namespace exploration
{
    class TraversabilityExplorer : public TraversabilityExplorerBase
    {
	friend class TraversabilityExplorerBase;
    protected:

    public:
        TraversabilityExplorer(std::string const& name = "exploration::TraversabilityExplorer");
        TraversabilityExplorer(std::string const& name, RTT::ExecutionEngine* engine);
        ~TraversabilityExplorer();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif

