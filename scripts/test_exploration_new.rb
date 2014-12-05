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

Orocos.run 'exploration::Task' => 'exploration',
        #:gdb=>true, :valgrind=>true,  
        "wait" => 1000 do
    
    puts "#{TaskContext::get 'exploration'}"
    exploration = TaskContext::get 'exploration'
    traversability = TaskContext::get 'traversability'
    odometry = TaskContext::get 'odometry'

    traversability.traversability_map.connect_to(exploration.envire_environment_in)
    odometry.odometry_samples.connect_to(exploration.pose_samples) 

    exploration.configure()
    exploration.start()

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

