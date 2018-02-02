#include "TraversabilityExplorer.hpp"

#include <envire/Orocos.hpp>
#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/operators/SimpleTraversability.hpp>

#define CLASS_UNKNOWN 0
#define CLASS_OBSTACLE 1
#define CUSTOM_CLASSES 2

using namespace exploration;

void growObstacles(envire::TraversabilityGrid& map, double width)
{
	const double width_square = pow(width,2);
	const int 
	wx = width / map.getScaleX(), 
	wy = width / map.getScaleY();
	const double 
	sx = map.getScaleX(),
	sy = map.getScaleY();

	envire::TraversabilityGrid::ArrayType& orig_data = map.getGridData(envire::TraversabilityGrid::TRAVERSABILITY);
	envire::TraversabilityGrid::ArrayType data( orig_data );
	envire::TraversabilityGrid::ArrayType &probabilityArray(map.getGridData(envire::TraversabilityGrid::PROBABILITY));

	for (unsigned int y = 0; y < map.getHeight(); ++y)
	{
		for (unsigned int x = 0; x < map.getWidth(); ++x)
		{
			int value = orig_data[y][x];
			if (value == CLASS_OBSTACLE)
			{
				// make everything with radius width around the obstacle also
				// an obstacle
				for( int oy = -wy; oy <= wy; ++oy )
				{
					for( int ox = -wx; ox <= wx; ++ox )
					{
						const int tx = x+ox;
						const int ty = y+oy;
						if( (pow(ox*sx,2) + pow(oy*sy,2) < width_square )
							&& tx >= 0 && tx < (int)map.getWidth()
							&& ty >= 0 && ty < (int)map.getHeight() )
						{
							data[ty][tx] = CLASS_OBSTACLE;
							probabilityArray[y][x] = std::numeric_limits< uint8_t >::max();
						}
					}
				}	
			}
		}
	}
	std::swap( data, orig_data );
}

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
	
	// Read in the new map
	envire::OrocosEmitter::Ptr binary_event;
	while(_envire_map.read(binary_event) == RTT::NewData)
	{
		mEnvironment.applyEvents(*binary_event);   
	}
	std::vector<envire::TraversabilityGrid*> maps = mEnvironment.getItems<envire::TraversabilityGrid>();
	if(maps.size() == 0)
	{
		LOG_ERROR("Environment has no traversability map!");
		return;
	}
	if(maps.size() > 1)
	{
		LOG_WARN("The environment contains more than one traversability map, will use the first!");
	}
	
	// Inflate obstacles	
	envire::TraversabilityGrid* trav = maps[0];	
	growObstacles(*trav, _obstacle_clearance.get());
	
	size_t size_x = trav->getCellSizeX();
	size_t size_y = trav->getCellSizeY();
	GridMap gridMap(size_x, size_y);
	GridPoint point;
	for(size_t y = 0; y < size_y; y++)
	{
		point.y = y;
		for (size_t x = 0; x < size_x; x++)
		{
			point.x = x;
			if(trav->getProbability(x,y) < 0.1)
				gridMap.setData(point, UNKNOWN);
			else if(trav->getTraversability(x,y).getDrivability() <= 0.25)
				gridMap.setData(point, OBSTACLE);
			else
				gridMap.setData(point, VISIBLE);
		}
	}
	
	// Add boundary
	size_t grid_x, grid_y;
	GridPoint pnt;
	double x_min = _boundary_x_min.get();
	double x_max = _boundary_x_max.get();
	double y_min = _boundary_y_min.get();
	double y_max = _boundary_y_max.get();
	
	for(double map_x = x_min; map_x < x_max; map_x += trav->getScaleX() )
	{
		if(trav->toGrid(map_x, y_min, grid_x, grid_y))
			gridMap.setData(GridPoint(grid_x, grid_y, 0), OBSTACLE);
		if(trav->toGrid(map_x, y_max, grid_x, grid_y))
			gridMap.setData(GridPoint(grid_x, grid_y, 0), OBSTACLE);
	}
	
	for(double map_y = y_min; map_y < y_max; map_y += trav->getScaleY() )
	{
		if(trav->toGrid(x_min, map_y, grid_x, grid_y))
			gridMap.setData(GridPoint(grid_x, grid_y, 0), OBSTACLE);
		if(trav->toGrid(x_max, map_y, grid_x, grid_y))
			gridMap.setData(GridPoint(grid_x, grid_y, 0), OBSTACLE);
	}
	
	// Debug output map
	envire::Environment tr;
	envire::TraversabilityGrid* exploreMap = mPlanner.coverageMapToTravGrid(gridMap, *trav);

	envire::FrameNode *fr = new envire::FrameNode(trav->getFrameNode()->getTransform());
	tr.getRootNode()->addChild(fr);
	tr.attachItem(exploreMap, fr);
	
	exploreMap->setFrameNode(fr);
	envire::OrocosEmitter emitter(&tr, _explore_map);
	emitter.setTime(base::Time::now());
	emitter.flush(); 
	
	// Read in start point
	base::samples::RigidBodyState rbs;
	while(_robot_pose.read(rbs) == RTT::NewData)
	{
	}
	
	size_t gridx, gridy;
	if(trav->toGrid(rbs.position[0],rbs.position[1],gridx,gridy))
	{
		GridPoint start(gridx, gridy, 0);
		FrontierList frontiers = mPlanner.getFrontiers(&gridMap, start);
		if(frontiers.size() == 0)
		{
			if(mPlanner.getStatus() == SUCCESS)
			{
				LOG_INFO("%s", mPlanner.getStatusMessage());
				state(EXPLORATION_DONE);
				return;
			}else
			{
				LOG_ERROR("%s", mPlanner.getStatusMessage());
				return;
			}
		}
		state(RUNNING);
		
		GridPoint goal;
                double squared_distance = 0;

		FrontierList::iterator largestFrontier;
		size_t max_length = 0;
		for(FrontierList::iterator it = frontiers.begin(); it < frontiers.end(); it++)
		{
			if(it->size() > max_length)
			{
                                // default heuristic
		                goal = it->at(it->size() / 2);
                                double posex, posey;
                                trav->fromGrid(goal.x, goal.y, posex, posey);
                                squared_distance = pow(posex - rbs.position[0], 2) + pow(posey - rbs.position[1],2) ;
                                LOG_INFO_S << "Squared distance: " << squared_distance  << " min goal" << pow(_min_goal_distance.value(), 2);
                                if( squared_distance > pow(_min_goal_distance.value(),2) )
                                {
                                    max_length = it->size();
                                    largestFrontier = it;
                                } else {
                                    LOG_WARN_S << "Squared distance: " << squared_distance  << " min goal" << pow(_min_goal_distance.value(), 2);
                                    // search for a goal at valid distance
                                    for(int i = 0; i < it->size(); ++i)
                                    {
                                        goal = it->at(i);

                                        double posex, posey;
                                        trav->fromGrid(goal.x, goal.y, posex, posey);
                                        squared_distance = pow(posex - rbs.position[0], 2) + pow(posey - rbs.position[1],2) ;
                                        if( squared_distance > pow(_min_goal_distance.value(),2) )
                                        {
                                            LOG_WARN_S << "Linear search: squared distance: " << squared_distance  << " min goal" << pow(_min_goal_distance.value(), 2);
                                            max_length = it->size();
                                            largestFrontier = it;
                                        } else {
                                            LOG_INFO_S << "Linear search: squared distance: " << squared_distance  << " min goal" << pow(_min_goal_distance.value(), 2);
                                        }
                                    }
                                } 
			}
		}

                if(max_length == 0)
                {
		    LOG_INFO("No remaining frontier -- exploration is done");
                    state(EXPLORATION_DONE);
                } else {
		    LOG_INFO("Largest frontier has %d cells.", max_length);
                    double posex, posey;
                    trav->fromGrid(goal.x, goal.y, posex, posey);
                    _goal_distance.write(sqrt(squared_distance));
                    
                    rbs.position[0] = posex;
                    rbs.position[1] = posey;
                    rbs.time = base::Time::now();
                    _goal_pose.write(rbs);
                }
	}else
	{
		LOG_ERROR_S << "Start point x: " << rbs.position[0]
                    << " y: " << rbs.position[1] 
                    << " is not in map.";
                state(START_NOT_IN_MAP);
	}
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

