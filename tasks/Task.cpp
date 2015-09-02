/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "../../../../install/include/envire/maps/TraversabilityGrid.hpp"
#include <envire/operators/SimpleTraversability.hpp>
#define OBSTACLE_DRIVABILITY 0.0

using namespace exploration;

Task::Task(std::string const& name) : TaskBase(name), mEnv(NULL)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine) : 
        TaskBase(name, engine), mEnv(NULL)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    mEnv = new envire::Environment();
    triggered = false;
    _goals_out.keepLastWrittenValue(false);
    return true;
}

bool Task::startHook()
{
    mTraversabilityMapStatus = RTT::NoData;
    if (!TaskBase::startHook()) {
        return false;
    }

    mTraversabilityMapStatus = RTT::NoData;

    planner = Planner();
    
    // Adds fake camera polygons.
    std::vector<ConfPolygon> polys = _polygons;
    std::vector<ConfPolygon>::iterator polygon; std::vector<FloatPoint>::iterator currPoint;
    for(polygon = polys.begin(); polygon < polys.end(); polygon++ )
    {
        Polygon polyToRender;
        for(currPoint = polygon->points.begin(); currPoint < polygon->points.end(); currPoint++)
        {
            polyToRender.push_back(*currPoint);
        }
        planner.addSensor(polyToRender);
    }
    
    // Goal within this distance to the robot are discarded.
    planner.setMinGoalDistance(_min_goal_distance);
    
    initialized = false;
    
    return true;
}

void Task::calculateGoals()
{
    triggered = true;
}

void Task::updateHook()
{   TaskBase::updateHook();
    
    // After a map has been received.
    if(initialized)
    {
        updateMap();
        flushMap();
        generateGoals();
    }

    // Go on if a new map has been received.
    envire::TraversabilityGrid* oldTraversabilityGrid = planner.mTraversability;
    RTT::FlowStatus ret = receiveEnvireData();
    if (ret == RTT::NoData || ret == RTT::OldData || !planner.mTraversability) {
        return;
    } 
    envire::TraversabilityGrid::ArrayType& trav_array = planner.mTraversability->getGridData();
    size_t cell_size_x = planner.mTraversability->getCellSizeX();
    size_t cell_size_y = planner.mTraversability->getCellSizeY();
    
    if(cell_size_x <= 1 || cell_size_y <= 1) {
        LOG_WARN("Map with wrong cellsize (<=1) has been received.");
        return;
    }
    
    struct GridPoint point;
    if(!initialized)
    {
        GridMap map((int)cell_size_x, (int)cell_size_y);
        int obstacle_cnt = 0;
        for(size_t y = 0; y < cell_size_y; y++){
            point.y = y;
            for (size_t x = 0; x < cell_size_x; x++){
                point.x = x;
                char value = -1;
                //std::cout << "INITIALIZING: value at point in ORIGINAL travData" << point.x << "/" << point.y << " is     " << (int)trav_array[y][x] << std::endl;
                
                // planner.mTraversability->getTraversabilityClass(trav_array[y][x]).getDrivability() == 0 -> obstacle
                
                for(unsigned int i = 0; i < planner.mTraversability->getTraversabilityClasses().size(); i++)
                {
                    if(i == trav_array[y][x] && planner.mTraversability->getTraversabilityClass(i).getDrivability() <= OBSTACLE_DRIVABILITY 
                        && planner.mTraversability->getProbability(x,y) >= 0.99)  
                    { 
                        value = 1;
                        obstacle_cnt++;
                        break;
                    }
                }
                map.setData(point, value);
            }
        }
        //initializing internal coverageMap
        planner.initCoverageMap(&map);
 
        lastTraversabilityFrameNode = new envire::FrameNode(planner.mTraversability->getFrameNode()->getTransform());
        lastOffsetX = planner.mTraversability->getOffsetX();
        lastOffsetY = planner.mTraversability->getOffsetY();
       
        LOG_INFO("Planner initialization complete, obstacle count %d", obstacle_cnt);
        initialized = true;
        
    }

    // Resize map if a bigger travGrid has been received.
    else if(initialized && (cell_size_x > planner.getCoverageMap().getWidth() || 
                            cell_size_y > planner.getCoverageMap().getHeight()))
    {
        //transfer current coverage map into a travgrid so it can be transformed with ease
        //envire::TraversabilityGrid* currentMap = planner.coverageMapToTravGrid(planner.getCoverageMap(), *oldTraversabilityGrid);
        //create frameNode with the transformation of the old frameNode
        //envire::FrameNode* currentTraversabilityFrameNode = new envire::FrameNode(planner.mTraversability->getFrameNode()->getTransform());
        //envire::FrameNode* lastTraversabilityFrameNode = new envire::FrameNode(oldTraversabilityGrid->getFrameNode()->getTransform());
        //envire::Environment tr;
        
        envire::Transform old_to_new = lastTraversabilityFrameNode->getTransform().inverse() * 
                planner.mTraversability->getFrameNode()->getTransform();
        
        int deltaX = (cell_size_x - planner.getCoverageMap().getWidth())/2;
        int deltaY = (cell_size_y - planner.getCoverageMap().getHeight())/2;
        GridMap map((int)cell_size_x, (int)cell_size_y);
        for(size_t y = 0; y < cell_size_y; y++){
            point.y = y;
            for (size_t x = 0; x < cell_size_x; x++){
                Eigen::Vector3d p_old((x + lastOffsetX) * planner.mTraversability->getScaleX(), 
                                      (y + lastOffsetY) * planner.mTraversability->getScaleY(), 0);
                Eigen::Vector3d p_new = old_to_new * p_old;
                p_new = Eigen::Vector3d(p_new.x() / planner.mTraversability->getScaleX() - planner.mTraversability->getOffsetX(), 
                                        p_new.y() / planner.mTraversability->getScaleY() - planner.mTraversability->getOffsetY(), 0);
                point.x = x;
                //return -1 if point is outOfBounds
                // TODO At the moment the old point is tested which cannot be 
                int value = planner.getCoverageMap().getData(point);
                /*
                 * explored points need to be set with an offset since those values originate from a smaller array
                 * NOT TESTED YET!!
                 */
                if(value == 0)
                {
                     exploration::GridPoint offsetPoint;
                     offsetPoint.x = p_new.x();
                     offsetPoint.y = p_new.y();
                     map.setData(offsetPoint, value);
                } 
                else {
                    map.setData(point, value);
                 }
            }
        }
        planner.initCoverageMap(&map);
        lastTraversabilityFrameNode = new envire::FrameNode(planner.mTraversability->getFrameNode()->getTransform());
        lastOffsetX = planner.mTraversability->getOffsetX();
        lastOffsetY = planner.mTraversability->getOffsetY();
    }
    
    if(ret == RTT::NewData)
    {
        //update obstacles
        PointList obstacles;
        PointList obstacleToUnknown;
        bool isObstacle;
        const GridMap &map = planner.getCoverageMap();
    
        for(size_t y = 0; y < cell_size_y; y++){
            point.y = y;
            for (size_t x = 0; x < cell_size_x; x++){
                point.x = x;
                isObstacle = false;
                for(unsigned int i = 0; i < planner.mTraversability->getTraversabilityClasses().size(); i++)
                {
                    if((i == trav_array[y][x] && planner.mTraversability->getTraversabilityClass(i).getDrivability() <= OBSTACLE_DRIVABILITY) ||
                        (y == 0 || y == cell_size_y-1 || x == 0 || x == cell_size_x-1))  
                    {
                        obstacles.push_back(point); isObstacle = true; break;
                    }
                }
                if(!isObstacle && map.getData(point) == 1)
                {
                    obstacleToUnknown.push_back(point);
                }
            }
        }
        planner.setCoverageMap(obstacles, 1);
        planner.setCoverageMap(obstacleToUnknown, -1);
        std::cout << "Updated obstacles" << std::endl;
    }
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
        mEnv->applyEvents(*binary_event);
    }

    if ((ret == RTT::NoData) || (ret == RTT::OldData))
    {
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
        std::cout << "Received traversability map(s): " << std::endl;
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
    planner.mTraversability =
        mEnv->getItem< envire::TraversabilityGrid >(_traversability_map_id.get()).get();
    if (!planner.mTraversability)
    {
        RTT::log(RTT::Info) << "No traversability map with id" << _traversability_map_id.get() << RTT::endlog();
        if(maps.size() > 1) {
            RTT::log(RTT::Warning) << "The environment contains more    than one traversability map, please specify the map ID" << RTT::endlog();
            return false;
        } else {
            RTT::log(RTT::Info) << "The only given traversability map will be used" << RTT::endlog();
            std::vector<envire::TraversabilityGrid*>::iterator it = maps.begin();
            planner.mTraversability = mEnv->getItem< envire::TraversabilityGrid >((*it)->getUniqueId()).get();
            if (!planner.mTraversability)
            {
                RTT::log(RTT::Warning) << "Traversability map '" << (*it)->getUniqueId() << 
                    "' could not be extracted" << RTT::endlog();
                return false;
            } 
        }
    } 
    
    RTT::log(RTT::Info) << "Traversability map " << planner.mTraversability->getUniqueId() << " extracted" << RTT::endlog();
    return true;
}

RTT::FlowStatus Task::updateMap()
{
    RTT::FlowStatus ret = RTT::NoData;
    while(_pose_samples.read(robotPose, true) == RTT::NewData)
    {
        ret = RTT::NewData;
        this->pose.theta = robotPose.getYaw();
        
        size_t x, y;
        if(planner.mTraversability->toGrid(robotPose.position, x, y, planner.mTraversability->getFrameNode()))
        {
            //std::cout << "Adding Reading" << std::endl;
            this->pose.x = x;
            this->pose.y = y;
            this->planner.addReading(pose);
        }
    }
    return ret; 
}

bool Task::flushMap()
{
    //std::cout << "Outputting new map" << std::endl;
    envire::Environment tr;
    
    //generating exploreMap that is going to be dumped
    envire::TraversabilityGrid* exploreMap = planner.coverageMapToTravGrid(planner.getCoverageMap(), *planner.mTraversability);
    
    envire::FrameNode *fr = new envire::FrameNode(planner.mTraversability->getFrameNode()->getTransform());

    tr.getRootNode()->addChild(fr);
    tr.attachItem(exploreMap, fr);
    
    exploreMap->setFrameNode(fr);
    
    envire::OrocosEmitter emitter(&tr, _explore_map);
    emitter.setTime(base::Time::now());
    emitter.flush(); 
    return true;
}

void Task::generateGoals()
{
    goals.clear();

    if(!triggered)
    {
        return;
    }
    
    
    std::cout << "triggered goalPose-calculation!!" << std::endl;
    

    PointList frontiers;
    //std::cout << "Pose fuer getCoverageFrontiers: " << pose.x << "/" << pose.y << std::endl;
    //exploration::Pose goalPose = planner.getCoverageTarget(pose); 
    FrontierList goals_tmp = planner.getCoverageFrontiers(pose);
    //std::cout << "Number of Frontiers: " << goals_tmp.size() << std::endl;
    int front = 1;

    for(FrontierList::const_iterator frIt = goals_tmp.begin(); frIt != goals_tmp.end(); ++frIt) {
        //std::cout << "Frontier number " << front << " has " << frIt->size() << " points" << std::endl; 
      for(PointList::const_iterator pointIt = frIt->begin(); pointIt != frIt->end(); ++pointIt) {
            double x_tr, y_tr;
            base::Pose2D bla;
            
            planner.mTraversability->fromGrid(pointIt->x, pointIt->y, x_tr, y_tr);
            bla.position = base::Vector2d(x_tr, y_tr);
            //check if point is too close to obstacle. box is set in confirguration-file
            try
            {
                if(planner.mTraversability->getWorstTraversabilityClassInRectangle(bla , 1.0, 1.0).getDrivability() > OBSTACLE_DRIVABILITY)
                {
                    goals.push_back(base::Vector3d(x_tr, y_tr, 0));
                }
            } 
            catch (std::runtime_error &e) 
            {
//                 goals.push_back(base::Vector3d(x_tr, y_tr, 0));
                continue;
            }
        }
        front ++;
    }
    
    //copy of robotPose should not be necessary
    base::samples::RigidBodyState robotStateCopy = robotPose;
    std::vector<base::samples::RigidBodyState> finGoals = planner.getCheapest(goals, robotStateCopy);
    
    triggered = false;
    _goals_out.write(finGoals);
}


