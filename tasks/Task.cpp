/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
using namespace exploration;

    Task::Task(std::string const& name)
: TaskBase(name),
    mEnv(NULL)
{
}

    Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
: TaskBase(name, engine),
    mEnv(NULL)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    mEnv = new envire::Environment();
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    planner = Planner();
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    // Receive map.
    RTT::FlowStatus ret = receiveEnvireData();
    if (ret == RTT::NoData || !traversability) {
        return;
    }


    // convert TraversabilityMap to GridMap
    size_t xi = traversability->getCellSizeX();
    size_t yi = traversability->getCellSizeY();
    double x = 0.0, y = 0.0;
    size_t gridX,gridY;
    traversability->fromGrid(xi,yi,x,y);
    struct GridPoint point;
    GridMap* map = new GridMap((int)x, (int)y);
    if(x<=0 || y<=0)
        return;

    for(int i = 0; i < xi; i++){
        point.x = i;
        for (int j = 0; j < yi; j++){
            point.y = j;
            traversability->toGrid(x,y,gridX,gridY);
            map->setData(point,traversability->getFromRaster(traversability->getBands().front(),x,y));
        }
    }

    PointList frontiers;
    GridPoint start;
    //_start_position_in.read(start_vec);
    //start.x = start_vec.position.x();
    //start.y = start_vec.position.y();
    PointList goals_tmp = planner.getFrontierCells(map, start, false);

    for(PointList::iterator it = goals_tmp.begin(); it != goals_tmp.end(); ++it) {
            goals.push_back(base::Vector3d(it->x, it->y, 0));
    }

    _goals_out.write(goals);

}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}


RTT::FlowStatus Task::receiveEnvireData()
{
    envire::OrocosEmitter::Ptr binary_event;
    RTT::FlowStatus ret = mTraversabilityMapStatus;

    while(_envire_environment_in.read(binary_event) == RTT::NewData)
    {
        ret = RTT::NewData;
        mEnv->applyEvents(*binary_event);
    }

    if ((ret == RTT::NoData) || (ret == RTT::OldData))
    {
        return ret;
    }

    // Extracts data and adds it to the planner. 
    if(!extractTraversability()) {
        return mTraversabilityMapStatus;
    }

    // Set from NoData to OldData, variable should only be used locally.
    mTraversabilityMapStatus = RTT::OldData;
    return RTT::NewData;
}

bool Task::extractTraversability() {
    std::vector<envire::TraversabilityGrid*> maps = mEnv->getItems<envire::TraversabilityGrid>();

    // Lists all received traversability maps.
    std::stringstream ss;
    if(maps.size()) {
        ss << "Received traversability map(s): " << std::endl;

        std::string trav_map_id;
        std::vector<envire::TraversabilityGrid*>::iterator it = maps.begin();
        for(int i=0; it != maps.end(); ++it, ++i)
        {
            ss << i << ": " << (*it)->getUniqueId() << std::endl;
        }
        RTT::log(RTT::Info) << ss.str() << RTT::endlog(); 
    } else {
        RTT::log(RTT::Warning) << "Environment does not contain any traversability grids" << RTT::endlog();
        return false;
    }

    // Extract traversability map from evironment.
     traversability =
        mEnv->getItem< envire::TraversabilityGrid >(_traversability_map_id.get()).get();
    if (!traversability)
    {
        RTT::log(RTT::Info) << "No traversability map with id" << _traversability_map_id.get() << RTT::endlog();
        if(maps.size() > 1) {
            RTT::log(RTT::Warning) << "The environment contains more than one traversability map, please specify the map ID" << RTT::endlog();
            return false;
        } else {
            RTT::log(RTT::Info) << "The only given traversability map will be used" << RTT::endlog();
            std::vector<envire::TraversabilityGrid*>::iterator it = maps.begin();
            traversability = mEnv->getItem< envire::TraversabilityGrid >((*it)->getUniqueId()).get();
            if (!traversability)
            {
                RTT::log(RTT::Warning) << "Traversability map '" << (*it)->getUniqueId() << 
                    "' could not be extracted" << RTT::endlog();
                return false;
            } 
        }
    } 

    RTT::log(RTT::Info) << "Traversability map " << traversability->getUniqueId() << " extracted" << RTT::endlog();

    return true;
}
