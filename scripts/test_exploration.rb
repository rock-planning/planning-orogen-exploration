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

Orocos.run 'envire::SynchronizationTransmitter' => 'transmitter', 
        'corridor_planner::Traversability' => 'traversability',
        'exploration::Task' => 'exploration',
        "wait" => 1000 do
        
    #Orocos.conf.load_dir("./config")
    
    # LOAD ENV AND CREATE TRAV
    # What is 'Orocos.name_service.get' instead of 'TaskContext::get'
    transmitter = TaskContext::get 'transmitter'
    transmitter.configure()
    transmitter.start()
    
    traversability = Orocos.name_service.get 'traversability'
    traversability.apply_conf(['default'])
    traversability.configure()
    traversability.start()
    
    exploration = TaskContext::get 'exploration'
    exploration.configure()
    exploration.start()
    # CONNECT PORTS
    transmitter.envire_events.connect_to(traversability.mls_map)
   # transmitter.envire_events.connect_to(exploration.envire_environment_in)
    traversability.traversability_map.connect_to(exploration.envire_environment_in) 
    # LOAD MAP
    transmitter.loadEnvironment("#{ENV['AUTOPROJ_PROJECT_BASE']}/bundles/spacebot/data/traversability_maps/dlr.env")
    
    #while true
        transmitter.loadEnvironment("#{ENV['AUTOPROJ_PROJECT_BASE']}/bundles/spacebot/data/traversability_maps/dlr.env")
    #end
   
    Readline::readline("Press ENTER to exit ...")
end

