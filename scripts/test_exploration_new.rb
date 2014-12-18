require 'orocos'
require 'vizkit'
require 'readline'
Orocos::CORBA.max_message_size = 12000000 # stucks if > than this

include Orocos
Orocos.initialize

# Content of these language variables must not be german, otherwise '.' and ','
# are mixed reading numbers and a bad_alloc error occurs loading the scenes.
ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'

Orocos.run  'exploration::Task' => 'exploration', #'motion_planning_libraries::Test' => 'traversability', #:gdb=>true, :valgrind=>true,  
        "wait" => 1000 do
    
    exploration = TaskContext::get 'exploration'
    traversability = TaskContext::get 'traversability'
    odometry = TaskContext::get 'odometry'

    odometry.odometry_samples.connect_to(exploration.pose_samples) 
    traversability.traversability_map.connect_to(exploration.envire_environment_in)

    #traversability.traversability_map_id = 'trav'
    #traversability.traversability_map_type = 'SMALL_OPENING'
    #traversability.traversability_map_width_m = 120
    #traversability.traversability_map_height_m = 10
    #traversability.traversability_map_scalex =  0.1   
    #traversability.traversability_map_scaley = 0.1
    #traversability.number_of_random_circles = 50
    #traversability.opening_length = 2.0

    exploration.configure()
    exploration.start()

    #traversability.configure()
    #traversability.start()


    #t1 = Thread.new do
    #    reader = traversability.traversability_map.reader 
    #    exploreader = exploration.explore_map.reader
    #    while true do
    #        if(!reader.read.nil? ||!exploreader.read.nil?)
    #            map = reader.read
    #            explomap = exploreader.read
    #            puts("output trav: #{map}")
    #            puts("output explo: #{explomap}")
    #            sleep 1.0
    #        end
    #    end
    #end

    # CONNECT PORTS

    ##transmitter.envire_events.connect_to(traversability.mls_map)

    #transmitter.envire_events.connect_to(exploration.envire_environment_in)

    ##traversability.traversability_map.connect_to(exploration.envire_environment_in) 

    # LOAD MAP
    #transmitter.loadEnvironment("#{ENV['AUTOPROJ_PROJECT_BASE']}/bundles/spacebot/data/traversability_maps/dlr.env")
    
    #while true
    #end
   
    Readline::readline("Press ENTER to exit ...")
end
