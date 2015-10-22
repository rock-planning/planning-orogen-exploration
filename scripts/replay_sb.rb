#! /usr/bin/env ruby
#library for displaying data
require 'vizkit'
require 'readline'
require 'eigen'
require 'rock/bundle'

if !ARGV[0]  then 
    puts "usage: replay.rb log_dir"
    exit
end


#load log file 
log = Orocos::Log::Replay.open(ARGV[0])
Orocos::CORBA::max_message_size = 100000000

#log.exploration_planner.track(false) 
#log.transformer_broadcaster.track(false) 
#log.transformer_broadcaster.rename('foo')
#log.name_service.deregister 'exploration_planner'
#log.name_service.deregister 'transformer_broadcaster'

Bundles.initialize
#Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_scripts.rb'))

Bundles.run 'exploration::Task' => 'exploration_planner2',  
        'motion_planning_libraries::Task' => 'global_planner2',
        :valgrind => false do |p|
        
    Orocos.log_all_ports()
    
    # Replay tasks
    global_planner = Bundles::get 'global_planner'
    exploration_planner = Bundles::get 'exploration'
    traversability = Bundles::get 'traversability'
    velodyne_slam = Bundles::get 'velodyne_slam'

    # New exploration task.
    global_planner2 = Bundles::get 'global_planner2'
    exploration_planner2 = Bundles::get 'exploration_planner2'

    # Feed new exploration task.
    traversability.traversability_map.connect_to(exploration_planner2.envire_environment_in)
    traversability.traversability_map.connect_to(global_planner2.traversability_map)
    velodyne_slam.pose_samples.connect_to(exploration_planner2.pose_samples)
    velodyne_slam.pose_samples.connect_to(global_planner2.start_pose_samples)
    exploration_planner2.goal_out_best.connect_to(global_planner2.goal_pose_samples)
    
    # Start new expl.
    exploration_planner2.apply_conf(['default', 'spacebot'])
    exploration_planner2.configure()
    exploration_planner2.start()
    
    global_planner2.apply_conf(['default'])
    global_planner2.configure
    global_planner2.start

    # Display replay data.
    Vizkit.display exploration_planner.explore_map
    Vizkit.display global_planner.start_pose_samples_debug, :widget => Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.display global_planner.goal_pose_samples_debug, :widget => Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.display global_planner.trajectory

    Vizkit.control log

    Vizkit.exec()
end

