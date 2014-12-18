/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <envire/operators/SimpleTraversability.hpp>
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

    mTraversabilityMapStatus = RTT::NoData;

    planner = Planner();
    
    //Add fake camera looking forward
    Polygon poly;
    FloatPoint p;
    p.x =   -2; p.y =  2; poly.push_back(p);
    p.x =   -2; p.y = -2; poly.push_back(p);
    p.x =  80; p.y = -40; poly.push_back(p);
    p.x =  80; p.y =  40; poly.push_back(p);
    planner.addSensor(poly); 
    
    initialized = false;
    
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
    // Receive map.

    RTT::FlowStatus ret = receiveEnvireData(); //hier gibt es keine traversGrids
    if (ret == RTT::NoData || ret == RTT::OldData || !traversability) {
        //RTT::log(RTT::Warning) << "no data available" << RTT::endlog();
        return;
    } 

    envire::TraversabilityGrid::ArrayType& trav_array = traversability->getGridData();
    
    size_t xi = traversability->getCellSizeX();
    size_t yi = traversability->getCellSizeY();
    //std::cout << "getCellSizeX: " << xi << "   getCellSizeY: " << yi << std::endl;
    struct GridPoint point;
    
    if(!initialized)
    {
        GridMap map((int)xi, (int)yi);
        int cnt = 0;
        for(size_t y = 0; y < yi; y++){
            point.y = y;
            for (size_t x = 0; x < xi; x++){
                point.x = x;
                char value = 0;
                switch(trav_array[y][x])
                {
                    case envire::SimpleTraversability::CLASS_UNKNOWN:
                        value = 0;
                        break;
                    case envire::SimpleTraversability::CLASS_OBSTACLE:
                        cnt++;
                        value = 1;
                        break;
                    default:
                        break;
                }
                map.setData(point, value);
            }
        }
        
        planner.initCoverageMap(&map);
        std::cout << "Obstacle cnt " << cnt << std::endl;
        initialized = true;
        std::cout << "Planner init complete" << std::endl;
    }
    //{
        if(ret == RTT::NewData)
        {
            //update obstacles
            PointList obstacles;
        
            for(size_t y = 0; y < yi; y++){
                point.y = y;
                for (size_t x = 0; x < xi; x++){
                    point.x = x;
                    switch(trav_array[y][x])
                    {
                        case envire::SimpleTraversability::CLASS_OBSTACLE:
                            obstacles.push_back(point);
                            break;
                        default:
                            break;
                    }
                }
            }
            planner.setCoverageMap(obstacles, 1);
            std::cout << "Updated obstacles" << std::endl;
        }
    //}

    ret = RTT::NoData;
    base::samples::RigidBodyState robotPose;
    while(_pose_samples.read(robotPose, true) == RTT::NewData)
    {
        ret = RTT::NewData;
        exploration::Pose pose;
        pose.theta = robotPose.getYaw();
        size_t x, y;

        if(traversability->toGrid(robotPose.position, x, y, traversability->getFrameNode()))
        {
            std::cout << "Adding Re4ading" << std::endl;
            pose.x = x;
            pose.y = y;
            planner.addReading(pose);
        }
    }
    
    if(ret == RTT::NoData)
        return;

    exploration::Pose pose;
    pose.x = robotPose.position.x();
    pose.y = robotPose.position.y();
    pose.theta = robotPose.getYaw();

    
    exploration::Pose goalPose = planner.getCoverageTarget(pose); 
    
    PointList frontiers;
    FrontierList goals_tmp = planner.getCoverageFrontiers(pose);

    for(FrontierList::const_iterator frIt = goals_tmp.begin(); frIt != goals_tmp.end(); ++frIt) {
        for(PointList::const_iterator pointIt = frIt->begin(); pointIt != frIt->end(); ++pointIt) {
            double x_tr;
            double y_tr;
            traversability->fromGrid(pointIt->x, pointIt->y, x_tr, y_tr);
            goals.push_back(base::Vector3d(x_tr, y_tr, 0));
        }
    }

    if(_explore_map.connected())
    {
        std::cout << "Outputting new map" << std::endl;
        envire::Environment tr;
        envire::TraversabilityGrid *exploreMap = new envire::TraversabilityGrid(traversability->getWidth(), traversability->getHeight(), traversability->getScaleX(), traversability->getScaleY(), traversability->getOffsetX(), traversability->getOffsetY());
        
        envire::FrameNode *fr = new envire::FrameNode(traversability->getFrameNode()->getTransform());
        
        envire::TraversabilityGrid::ArrayType& exp_array = exploreMap->getGridData();
        
        const GridMap &map(planner.getCoverageMap());
        
        size_t xi = traversability->getCellSizeX();
        size_t yi = traversability->getCellSizeY();
        std::cout << "Output map size " << xi << " " << yi << std::endl;
        struct GridPoint point;
        
        for(size_t y = 0; y < yi; y++){
            point.y = y;
            for (size_t x = 0; x < xi; x++){
                point.x = x;
                char value = map.getData(point);
                switch(value)
                {
                    case -1:
                        exp_array[y][x] = envire::SimpleTraversability::CLASS_UNKNOWN;
                        break;
                    case 0:
                        exp_array[y][x] = envire::SimpleTraversability::CUSTOM_CLASSES + 5;
                        break;
                    case 1:
                        exp_array[y][x] = envire::SimpleTraversability::CLASS_OBSTACLE;
                        break;
                    default:
                        break;
                }
            }
        }

        tr.getRootNode()->addChild(fr);
        tr.attachItem(exploreMap, fr);
        
        exploreMap->setFrameNode(fr);
        
        envire::OrocosEmitter emitter(&tr, _explore_map);
        emitter.setTime(base::Time::now());
        emitter.flush();        
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
    //RTT::FlowStatus ret = RTT::NoData;
    RTT::FlowStatus ret = mTraversabilityMapStatus;
    while(_envire_environment_in.read(binary_event) == RTT::NewData)
    {
        ret = RTT::NewData;
        //RTT::log(RTT::Warning) << "NewData in receiveEnvireData: " << ret << RTT::endlog();
        mEnv->applyEvents(*binary_event);
    }

    if ((ret == RTT::NoData) /*|| (ret == RTT::OldData)*/)
    {
        //RTT::log(RTT::Warning) << "NoData in receiveEnvireData: " << ret << RTT::endlog();
        return ret;
    }

    if (/*(ret == RTT::NoData) || */(ret == RTT::OldData))
    {
        //RTT::log(RTT::Warning) << "OldData in receiveEnvireData: " << ret << RTT::endlog();
        return ret;
    }

    // Extracts data and adds it to the planner. 
    if(!extractTraversability()) {
        RTT::log(RTT::Warning) << "Extracting traversability failed" << RTT::endlog();
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
        //ss used for output strings
        std::cout << "Received traversability map(s): " << std::endl;
/////
        std::string trav_map_id;
        std::vector<envire::TraversabilityGrid*>::iterator it = maps.begin();
        for(int i=0; it != maps.end(); ++it, ++i)
        {
            std::cout << i << ": " << (*it)->getUniqueId() << std::endl;
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
