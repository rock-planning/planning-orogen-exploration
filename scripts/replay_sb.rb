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
log.transformer_broadcaster.track(false) 
log.transformer_broadcaster.rename('foo')
#log.name_service.deregister 'exploration_planner'
log.name_service.deregister 'transformer_broadcaster'

Bundles.initialize
Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_scripts.rb'))

Bundles.run 'exploration::Task' => 'exploration_planner', :valgrind => false, :output => nil do |p|
    
    exploration_planner = Bundles::get 'exploration_planner'
    traversability = Bundles::get 'traversability'
    velodyne_slam = Bundles::get 'velodyne_slam'

    traversability.traversability_map.connect_to(exploration_planner.envire_environment_in)
    velodyne_slam.pose_samples.connect_to(exploration_planner.pose_samples)

    
    
    exploration_planner.apply_conf(['default'])
    
    exploration_planner.configure()
    exploration_planner.start()

    Vizkit.control log

    
    Vizkit.display exploration_planner.explore_map
    Vizkit.display velodyne_slam.pose_samples, :widget => Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.display traversability.traversability_map

    Vizkit.exec()
end

