require 'vizkit'
require 'orocos'

include Orocos
Orocos.initialize

Orocos.run "velodyne_lidar::LaserScanner" => "velodyne" do 
    #Orocos.log_all_ports
    
    ## setup velodyne driver
    velodyne = TaskContext.get 'velodyne'
    velodyne.configure
    velodyne.start

    Vizkit.display velodyne
    begin
        Vizkit.exec
    rescue Interrupt => e
        velodyne.stop
    end
end
