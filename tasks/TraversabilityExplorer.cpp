#include "TraversabilityExplorer.hpp"

using namespace exploration;

TraversabilityExplorer::TraversabilityExplorer(std::string const& name)
    : TraversabilityExplorerBase(name)
{
}

TraversabilityExplorer::TraversabilityExplorer(std::string const& name, RTT::ExecutionEngine* engine)
    : TraversabilityExplorerBase(name, engine)
{
}

TraversabilityExplorer::~TraversabilityExplorer()
{
}

bool TraversabilityExplorer::configureHook()
{
    if (! TraversabilityExplorerBase::configureHook())
        return false;
    return true;
}

bool TraversabilityExplorer::startHook()
{
    if (! TraversabilityExplorerBase::startHook())
        return false;
    return true;
}

void TraversabilityExplorer::updateHook()
{
    TraversabilityExplorerBase::updateHook();
}

void TraversabilityExplorer::errorHook()
{
    TraversabilityExplorerBase::errorHook();
}

void TraversabilityExplorer::stopHook()
{
    TraversabilityExplorerBase::stopHook();
}

void TraversabilityExplorer::cleanupHook()
{
    TraversabilityExplorerBase::cleanupHook();
}

