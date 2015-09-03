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
    RTT::FlowStatus ret = receiveEnvireData();
    if (ret == RTT::NoData || ret == RTT::OldData || !planner.mTraversability) {
        return;
    } 
    envire::TraversabilityGrid::ArrayType& trav_array = planner.mTraversability->getGridData();
    size_t new_cell_size_x = planner.mTraversability->getCellSizeX();
    size_t new_cell_size_y = planner.mTraversability->getCellSizeY();
    double new_offset_x = planner.mTraversability->getOffsetX();
    double new_offset_y = planner.mTraversability->getOffsetY();
    double new_scale_x = planner.mTraversability->getScaleX();
    double new_scale_y = planner.mTraversability->getScaleY();
    
    if(new_cell_size_x <= 1 || new_cell_size_y <= 1) {
        LOG_WARN("Map with wrong cellsize (<=1) has been received.");
        return;
    }
    
    // Create first exploration map.
    struct GridPoint point;
    if(!initialized)
    {
        GridMap map((int)new_cell_size_x, (int)new_cell_size_y);
        int obstacle_cnt = 0;
        for(size_t y = 0; y < new_cell_size_y; y++){
            point.y = y;
            for (size_t x = 0; x < new_cell_size_x; x++){
                point.x = x;
                char value = UNKNOWN;
                
                if(planner.mTraversability->getTraversabilityClass(trav_array[y][x]).getDrivability() <= OBSTACLE_DRIVABILITY)  
                { 
                    value = OBSTACLE;
                    obstacle_cnt++;
                }
                map.setData(point, value);
            }
        }

        planner.initCoverageMap(&map);
    
        LOG_INFO("Planner initialization complete, obstacle count %d", obstacle_cnt);
        initialized = true;
        
    }
    // Resize map if a different traversability map has been received.
    else if(new_cell_size_x != planner.getCoverageMap().getWidth() || 
            new_cell_size_y != planner.getCoverageMap().getHeight() ||
            new_offset_x != lastOffsetX ||
            new_offset_y != lastOffsetY ||
            new_scale_x != lastScaleX ||
            new_scale_y != lastScaleY)
            
    {
        envire::Transform new_to_old = lastTraversabilityFrameNode->getTransform().inverse() * 
                planner.mTraversability->getFrameNode()->getTransform();

        GridMap map((int)new_cell_size_x, (int)new_cell_size_y);
        // Runs through new map and transforms each cell into the old map to get its value.
        for(size_t y = 0; y < new_cell_size_y; y++){
            for (size_t x = 0; x < new_cell_size_x; x++){
                Eigen::Vector3d p_new(0,0,0);
                // TODO fromGrid add shifts to the center of a cell (+0.5), problem?
                planner.mTraversability->fromGrid(x, y, p_new[0], p_new[1]);
                Eigen::Vector3d p_old = new_to_old * p_new;
                Eigen::Vector3d p_old_grid = Eigen::Vector3d((p_old.x() - lastOffsetX) / lastScaleX, 
                        (p_old.y() - lastOffsetY) / lastScaleY, 0);

                GridPoint old_grid_point(p_old_grid[0], p_old_grid[1], 0);
                GridPoint new_grid_point(x, y, 0);
                char value_old_map = 0;
                
                // The coverage map within the planner still contains the old data.
                if(planner.getCoverageMap().getData(old_grid_point, value_old_map)) {
                    // Transfer old value if the according cell lies within the new map.
                    map.setData(new_grid_point, value_old_map);       
                }
            }
        }
        planner.initCoverageMap(&map);
    }
    
    // Store new map informations.
    lastTraversabilityFrameNode = new envire::FrameNode(planner.mTraversability->getFrameNode()->getTransform());
    lastOffsetX = planner.mTraversability->getOffsetX();
    lastOffsetY = planner.mTraversability->getOffsetY();
    lastScaleX = planner.mTraversability->getScaleX();
    lastScaleY = planner.mTraversability->getScaleY();
    
    if(ret == RTT::NewData)
    {
        //update obstacles
        PointList obstacles;
        PointList obstacleToUnknown;
        bool isObstacle;
        const GridMap &map = planner.getCoverageMap();
        char value = 0;
    
        for(size_t y = 0; y < new_cell_size_y; y++){
            point.y = y;
            for (size_t x = 0; x < new_cell_size_x; x++){
                point.x = x;
                isObstacle = false;
          
                if(planner.mTraversability->getTraversabilityClass(trav_array[y][x]).getDrivability() <= OBSTACLE_DRIVABILITY ||
                    (y == 0 || y == new_cell_size_y-1 || x == 0 || x == new_cell_size_x-1))  
                {
                    obstacles.push_back(point); 
                    isObstacle = true; 
                }
                
                if(!isObstacle && map.getData(point, value) && value == OBSTACLE)
                {
                    obstacleToUnknown.push_back(point);
                }
            }
        }
        planner.setCoverageMap(obstacles, OBSTACLE);
        // TODO Why is the the cell set to UNKNOWN and not to EXPLORED?
        planner.setCoverageMap(obstacleToUnknown, UNKNOWN);
        LOG_INFO("Obstacles have been updated");
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
    envire::Environment tr;
    
    // Generating exploreMap that is going to be dumped.
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
            base::Pose2D pose_local;
            
            planner.mTraversability->fromGrid(pointIt->x, pointIt->y, x_tr, y_tr);
            pose_local.position = base::Vector2d(x_tr, y_tr);
            //check if point is too close to obstacle. box is set in configuration-file
            try
            {
                if(planner.mTraversability->getWorstTraversabilityClassInRectangle
                        (pose_local , _robot_length_x_m, _robot_width_y_m).getDrivability() > 
                        OBSTACLE_DRIVABILITY)
                {
                    goals.push_back(base::Vector3d(x_tr, y_tr, 0));
                }
            } 
            catch (std::runtime_error &e) 
            {
                continue;
            }
        }
        front ++;
    }
    
    // Copy of robotPose should not be necessary.
    base::samples::RigidBodyState robotStateCopy = robotPose;
    std::vector<base::samples::RigidBodyState> finGoals = planner.getCheapest(goals, robotStateCopy);
    triggered = false;
    _goals_out.write(finGoals);
}
