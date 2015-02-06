/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "../../../../install/include/envire/maps/TraversabilityGrid.hpp"
#include <envire/operators/SimpleTraversability.hpp>
#define OBSTACLE_DRIVABILITY 0.1

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
    _goals_out.keepLastWrittenValue(false);
    return true;
}
bool Task::startHook()
{
    mTraversabilityMapStatus = RTT::NoData;
    if (! TaskBase::startHook())
        return false;

    mTraversabilityMapStatus = RTT::NoData;

    planner = Planner();
    
    //Add fake camera looking forward
    Polygon poly;
    FloatPoint p;
    p.x =  20; p.y = 20; poly.push_back(p);
    p.x =  -20; p.y = 20; poly.push_back(p);
    p.x =  -20; p.y = -20; poly.push_back(p);
    p.x =  20; p.y =  -20; poly.push_back(p);
    p.x =  20; p.y =  20; poly.push_back(p);
    planner.addSensor(poly); 
    
    initialized = false;
    
    return true;
}

void Task::updateHook()
{   TaskBase::updateHook();
    
    // Receive map.
    if(initialized)
    {
        updateMap();
        flushMap();
    }

    RTT::FlowStatus ret = receiveEnvireData();
    if (ret == RTT::NoData || ret == RTT::OldData || !traversability) {
        return;
    } 

    envire::TraversabilityGrid::ArrayType& trav_array = traversability->getGridData();
    
    size_t xi = traversability->getCellSizeX();
    size_t yi = traversability->getCellSizeY();
    
    if(xi <= 1 || yi <= 1) {return;}
    
    std::cout << "received travMap with cellSize x/y :    " << xi << "  " << yi << ",    initialized???    " << initialized << std::endl;

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
                //std::cout << "INITIALIZING: value at point in ORIGINAL travData" << point.x << "/" << point.y << " is     " << (int)trav_array[y][x] << std::endl;
                for(int i = 0; i < traversability->getTraversabilityClasses().size(); i++)
                {
                    if(i == trav_array[y][x] && traversability->getTraversabilityClass(i).getDrivability() <= OBSTACLE_DRIVABILITY)  
                    { 
                        value = 1;
                        cnt++;
                        break;
                    }
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
            const GridMap &map = planner.getCoverageMap();
        
            for(size_t y = 0; y < yi; y++){
                point.y = y;
                for (size_t x = 0; x < xi; x++){
                    point.x = x;
                    for(int i = 0; i < traversability->getTraversabilityClasses().size(); i++)
                    {
                        if(i == trav_array[y][x] && traversability->getTraversabilityClass(i).getDrivability() <= OBSTACLE_DRIVABILITY)  
                        {obstacles.push_back(point);}
                    }
                }
            }
            planner.setCoverageMap(obstacles, 1);
            std::cout << "Updated obstacles" << std::endl;
        }
    //}
    
    if(updateMap() == RTT::NoData)
        return;
    
    this->goals.erase(this->goals.begin(), this->goals.begin() + this->goals.size());

    PointList frontiers;
    //std::cout << "Pose fuer getCoverageFrontiers: " << pose.x << "/" << pose.y << std::endl;
    FrontierList goals_tmp = planner.getCoverageFrontiers(pose);
    
        exploration::Pose goalPose = planner.getCoverageTarget(pose); 
        //std::cout << "Goal Pose:   " << goalPose.x << "/" << goalPose.y << std::endl;
    

    for(FrontierList::const_iterator frIt = goals_tmp.begin(); frIt != goals_tmp.end(); ++frIt) {
      for(PointList::const_iterator pointIt = frIt->begin(); pointIt != frIt->end(); ++pointIt) {
            double x_tr;
            double y_tr;
            //goals_tmp;
            traversability->fromGrid(goalPose.x, goalPose.y, x_tr, y_tr);
            //std::cout << "Frontier Count:  " <<  planner.getFrontierCellCount() << std::endl;
            goals.push_back(base::Vector3d(x_tr, y_tr, 0));
        }
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

    if ((ret == RTT::NoData) || (ret == RTT::OldData))
    {
        //RTT::log(RTT::Warning) << "NoData in receiveEnvireData: " << ret << RTT::endlog();
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

RTT::FlowStatus Task::updateMap()
{
    RTT::FlowStatus ret = RTT::NoData;
    base::samples::RigidBodyState robotPose;
    while(_pose_samples.read(robotPose, true) == RTT::NewData)
    {
        ret = RTT::NewData;
        this->pose.theta = robotPose.getYaw();
        size_t x, y;

        if(this->traversability->toGrid(robotPose.position, x, y, this->traversability->getFrameNode()))
        {
            std::cout << "Adding Re4ading" << std::endl;
            this->pose.x = x;
            this->pose.y = y;
            std::cout << "transformed position: " << x << "/" << y <<std::endl;
            this->planner.addReading(pose);
        }
    }
    return ret;
}

bool Task::flushMap()
{
        std::cout << "Outputting new map" << std::endl;
        envire::Environment tr;
        envire::TraversabilityGrid *exploreMap = new envire::TraversabilityGrid(traversability->getWidth(), traversability->getHeight(), traversability->getScaleX(), traversability->getScaleY(), traversability->getOffsetX(), traversability->getOffsetY());

        envire::TraversabilityClass obstacle(0.0);
        envire::TraversabilityClass explored(1.0);
        envire::TraversabilityClass unknown(0.5);
        exploreMap->setTraversabilityClass(5, obstacle);
        exploreMap->setTraversabilityClass(6, explored);
        exploreMap->setTraversabilityClass(7, unknown);
         
        envire::FrameNode *fr = new envire::FrameNode(traversability->getFrameNode()->getTransform());
        
        envire::TraversabilityGrid::ArrayType& exp_array = exploreMap->getGridData();
        
        const GridMap &map(planner.getCoverageMap());
        
        size_t xiExplo = traversability->getCellSizeX();
        size_t yiExplo = traversability->getCellSizeY();
        std::cout << "Output map size " << xiExplo << " " << yiExplo << std::endl;
        struct GridPoint pointExplo;
        
        for(size_t y = 0; y < yiExplo; y++){
            pointExplo.y = y;
            for (size_t x = 0; x < xiExplo; x++){
                pointExplo.x = x;
                int value = map.getData(pointExplo);
                exploreMap->setProbability(1.0, x, y);
                //std::cout << "value at OUTPUTMAP point " << pointExplo.x << "/" << pointExplo.y << " is     " << value << std::endl;
                switch(value)
                {
                    case -1:
                        exp_array[y][x] = 7;
                        break;
                    case 0:
                        exp_array[y][x] = 6;
                        //std::cout << "explored, set status to " << (int)envire::SimpleTraversability::CUSTOM_CLASSES + 5 << std::endl;
                        break;
                    case 1:
                        exp_array[y][x] = 5;
                        //std::cout << "OBSTACLE, set status to " << (int)envire::SimpleTraversability::CLASS_OBSTACLE << std::endl;
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
        return true;
}
