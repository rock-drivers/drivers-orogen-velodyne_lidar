/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "LaserScanner.hpp"

#include <rtt/extras/FileDescriptorActivity.hpp>
#include <aggregator/TimestampEstimator.hpp>

using namespace velodyne_lidar;


LaserScanner::LaserScanner(std::string const& name)
    : LaserScannerBase(name), timestamp_estimator(NULL)
{

}

LaserScanner::LaserScanner(std::string const& name, RTT::ExecutionEngine* engine)
    : LaserScannerBase(name, engine), timestamp_estimator(NULL)
{
}

LaserScanner::~LaserScanner()
{
}

bool LaserScanner::isScanComplete(const LaserScanner::LaserHeadVariables& laser_vars, const base::Angle& current_angle) const
{    
	//check if current angle crossed the last angle 
	return (laser_vars.horizontal_scan_count > 1 && laser_vars.dmap.horizontal_interval.back() > 0 && current_angle.getRad() <= 0);
}


void LaserScanner::handleHorizontalScan(const velodyne_fire_t& horizontal_scan, LaserScanner::LaserHeadVariables& laser_vars, const base::Time &shotTime)
{
    base::Angle scan_angle = base::Angle::fromDeg(360.0 - (static_cast<double>(horizontal_scan.rotational_pos) * 0.01));

    if(isScanComplete(laser_vars, scan_angle))
    {
        laser_vars.dmap.horizontal_size = laser_vars.horizontal_scan_count;
        
        laser_vars.dmap.time = laser_vars.last_sample_time;

        //handle data mapping for distances
        handleDataMapping(laser_vars.dmap.distances, laser_vars.buffer.distances, 
                                laser_vars.dmap.vertical_size, laser_vars.dmap.horizontal_size);

        if(_use_remissions)
                handleDataMapping(laser_vars.dmap.remissions, laser_vars.buffer.remissions, 
                                laser_vars.dmap.vertical_size, laser_vars.dmap.horizontal_size);

        //write sample to output port
        if(laser_vars.head_pos == UpperHead)
                _laser_scans.write(laser_vars.dmap);
        else
                _laser_scans_lower_head.write(laser_vars.dmap);

        resetSample(laser_vars);
    }

    
    laserdriver.collectColumn(horizontal_scan, laser_vars.buffer.distances, laser_vars.buffer.remissions, laser_vars.horizontal_scan_count, _use_remissions);
    laser_vars.dmap.horizontal_interval.push_back(scan_angle.getRad());
    laser_vars.dmap.timestamps.push_back(shotTime);
    laser_vars.horizontal_scan_count++;
    laser_vars.last_sample_time = shotTime;
}

void LaserScanner::resetSample(LaserScanner::LaserHeadVariables& laser_vars)
{
	//reset current sample
	laser_vars.horizontal_scan_count = 0;
	laser_vars.dmap.horizontal_size = 0;
	laser_vars.dmap.horizontal_interval.clear();
	laser_vars.dmap.timestamps.clear();
	laser_vars.dmap.distances.clear();
	
	if(_use_remissions)
		laser_vars.dmap.remissions.clear();
}


void LaserScanner::handleDataMapping(std::vector<base::samples::DepthMap::scalar> &target_container, Buffer::CollectorMatrix &source_container, uint32_t vertical_size, uint32_t horizontal_size)
{
	source_container.conservativeResize(vertical_size, horizontal_size);
	target_container.resize(horizontal_size * vertical_size);
	base::samples::DepthMap::DepthMatrixMap (target_container.data(), vertical_size, horizontal_size) 
		= Eigen::Map< Buffer::CollectorMatrix > (source_container.data(), vertical_size, horizontal_size);
}


bool LaserScanner::getFirstAngle(LaserHead head_pos, const velodyne_data_packet& new_scans, base::Angle& first_angle) const
{
    for(unsigned i = 0; i < VELODYNE_NUM_SHOTS; i++)
    {
        if((head_pos == UpperHead && new_scans.shots[i].lower_upper == VELODYNE_UPPER_HEADER_BYTES) || 
           (head_pos == LowerHead && new_scans.shots[i].lower_upper == VELODYNE_LOWER_HEADER_BYTES))
        {
            first_angle = base::Angle::fromDeg(360.0 - ((static_cast<double>(new_scans.shots[i].rotational_pos)) * 0.01));
            return true;
        }
    }
    return false;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.




bool LaserScanner::configureHook()
{
    if (! RTT::TaskContext::configureHook())
        return false;
    
    timestamp_estimator = new aggregator::TimestampEstimator(base::Time::fromSeconds(20), base::Time::fromMilliseconds(100));
        
    return true;
}



bool LaserScanner::startHook()
{
    
    if (! RTT::TaskContext::startHook())
        return false;
    
    // setup udp server
    laserdriver.openUDP("", VELODYNE_DATA_UDP_PORT);
    
    // trigger the update hook on fd activity
    RTT::extras::FileDescriptorActivity* fd_activity =
        getActivity<RTT::extras::FileDescriptorActivity>();
    if (fd_activity)
    {
        fd_activity->watch(laserdriver.getFileDescriptor());
    }
    else
    {
        RTT::log(RTT::Error) << TaskContext::getName() << ": "
                    << "Error: the task needs to be fd driven." << RTT::endlog();
        return false;
    }
    
    last_state = PRE_OPERATIONAL;
    last_packet_period = 1000000;
    last_gps_timestamp = 0;
    gps_timestamp_tolerance = 100;
    static const uint SCANS_PER_TURN = 2200; // upper guess
    
    integratedSensorTime = 0;
    lastEstimatedPacketTime = base::Time();
    timestamp_estimator->reset();
    
    // initiate head variables
    upper_head.horizontal_scan_count = 0;
    lower_head.horizontal_scan_count = 0;
    upper_head.head_pos = UpperHead;
    lower_head.head_pos = LowerHead;
	
	// initiate depthmap variables
	upper_head.dmap.horizontal_projection = base::samples::DepthMap::POLAR;
	lower_head.dmap.horizontal_projection = base::samples::DepthMap::POLAR;
	upper_head.dmap.vertical_projection = base::samples::DepthMap::POLAR;
	lower_head.dmap.vertical_projection = base::samples::DepthMap::POLAR;
	upper_head.dmap.vertical_interval.push_back(base::Angle::deg2Rad(VELODYNE_VERTICAL_START_ANGLE));
	lower_head.dmap.vertical_interval.push_back(base::Angle::deg2Rad(VELODYNE_VERTICAL_START_ANGLE));
	upper_head.dmap.vertical_interval.push_back(base::Angle::deg2Rad(VELODYNE_VERTICAL_END_ANGLE));
	lower_head.dmap.vertical_interval.push_back(base::Angle::deg2Rad(VELODYNE_VERTICAL_END_ANGLE));
	upper_head.dmap.vertical_size = VELODYNE_NUM_LASERS;
	lower_head.dmap.vertical_size = VELODYNE_NUM_LASERS;
	
	upper_head.dmap.distances.reserve(VELODYNE_NUM_LASERS * SCANS_PER_TURN);
	lower_head.dmap.distances.reserve(VELODYNE_NUM_LASERS * SCANS_PER_TURN);

    //init buffer
    upper_head.buffer.distances = Buffer::CollectorMatrix(VELODYNE_NUM_LASERS, SCANS_PER_TURN);
    lower_head.buffer.distances = Buffer::CollectorMatrix(VELODYNE_NUM_LASERS, SCANS_PER_TURN);
	
	if(_use_remissions)
	{
		upper_head.dmap.remissions.reserve(VELODYNE_NUM_LASERS * SCANS_PER_TURN);
		lower_head.dmap.remissions.reserve(VELODYNE_NUM_LASERS * SCANS_PER_TURN);
		upper_head.buffer.remissions = Buffer::CollectorMatrix(VELODYNE_NUM_LASERS, SCANS_PER_TURN);
		lower_head.buffer.remissions = Buffer::CollectorMatrix(VELODYNE_NUM_LASERS, SCANS_PER_TURN);
	}

    return true;
}



void LaserScanner::updateHook()
{
    RTT::TaskContext::updateHook();
    
    States actual_state = RUNNING;
    
    base::Time timeout = base::Time::fromMilliseconds(_timeout.get());
    int size = 0;
    try 
    {
        size = laserdriver.readPacket((uint8_t*)&buffer, VELODYNE_DATA_MSG_BUFFER_SIZE, timeout, timeout);
    }
    catch (std::runtime_error e)
    {
        RTT::log(RTT::Error) << TaskContext::getName() << ": " << e.what() << RTT::endlog();
        actual_state = IO_TIMEOUT;
    }
    
    if(size == (int)VELODYNE_DATA_MSG_BUFFER_SIZE)
    {
        if(last_gps_timestamp != buffer.gps_timestamp)
        {
            uint32_t currentPeriod = buffer.gps_timestamp - last_gps_timestamp;
            //check for wrap around
            if(last_gps_timestamp > buffer.gps_timestamp)
            {
                currentPeriod = (std::numeric_limits< uint32_t >:: max() - last_gps_timestamp) + buffer.gps_timestamp;
            }
            integratedSensorTime += currentPeriod;
            
            int packetCnt = integratedSensorTime / expectedPacketPeriod;
            estimatedPacketTime = timestamp_estimator->update(base::Time::now(), packetCnt);
        }
        
        base::Time timeBetweenShots = (estimatedPacketTime - lastEstimatedPacketTime) / VELODYNE_NUM_SHOTS;
        for(unsigned i = 0; i < VELODYNE_NUM_SHOTS; i++)
        {
            base::Time curEstimatedTime = lastEstimatedPacketTime + timeBetweenShots * 1;
            if(buffer.shots[i].lower_upper == VELODYNE_UPPER_HEADER_BYTES)
            {
                // handle upper laser
                handleHorizontalScan(buffer.shots[i], upper_head, curEstimatedTime);
            }
            else
            {
                // handle lower laser
                handleHorizontalScan(buffer.shots[i], lower_head, curEstimatedTime);
            }
        }

        if(last_gps_timestamp != buffer.gps_timestamp)
        {
            last_gps_timestamp = buffer.gps_timestamp;
            lastEstimatedPacketTime = estimatedPacketTime;
        }
    }
    else
    {
        RTT::log(RTT::Error) << TaskContext::getName() << ": "
            << "Received an unknown packet of size " << size 
            << ". Velodyne data message should be of size " 
            << VELODYNE_DATA_MSG_BUFFER_SIZE << RTT::endlog();
        actual_state = IO_ERROR;
    }
    
    // write state if it has changed
    if(last_state != actual_state)
    {
        last_state = actual_state;
        state(actual_state);
    }
    
}



void LaserScanner::errorHook()
{
    RTT::TaskContext::errorHook();
}



void LaserScanner::stopHook()
{
    RTT::TaskContext::stopHook();
    
    laserdriver.close();
}



void LaserScanner::cleanupHook()
{
    RTT::TaskContext::cleanupHook();

    delete timestamp_estimator;
}
