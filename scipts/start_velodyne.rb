require 'vizkit'
require 'orocos'

include Orocos
Orocos.initialize

# Orocos.run "velodyne_lidar::LaserScanner" => "velodyne", "velodyne_lidar::Positioning" => "positioning" do 
Orocos.run "velodyne_lidar::LaserScanner" => "velodyne" do 
#     Orocos.log_all_ports
    
    ## setup velodyne driver
    velodyne = TaskContext.get 'velodyne'
    velodyne.configure
    velodyne.start

    ## setup positioning driver
#     positioning = TaskContext.get 'positioning'
#     positioning.configure
#     positioning.start

    Vizkit.display velodyne
	Vizkit.display velodyne.laser_scans
#     Vizkit.display positioning

    begin
        Vizkit.exec
    rescue Interrupt => e
       velodyne.stop
#        positioning.stop
    end
end
