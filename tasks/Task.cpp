/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <rtt/extras/FileDescriptorActivity.hpp>
#include <aggregator/TimestampEstimator.hpp>

using namespace velodyne_lidar;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

bool Task::isScanComplete(const base::Angle& current_angle, const Task::LaserHeadVariables& laser_vars) const
{    
    if(laser_vars.horizontal_scan_count > 1)
    {
        if(current_angle == laser_vars.output_scan.horizontal_scans[0].horizontal_angle ||
           current_angle.isInRange(laser_vars.output_scan.horizontal_scans[0].horizontal_angle, 
                                   laser_vars.output_scan.horizontal_scans[laser_vars.horizontal_scan_count-1].horizontal_angle))
        {
            return true;
        }
    }
    return false;
}

void Task::handleHorizontalScan(const velodyne_fire_t& horizontal_scan, Task::LaserHeadVariables& laser_vars)
{
    base::Angle scan_angle = base::Angle::fromDeg(((double)horizontal_scan.rotational_pos) * 0.01);
    if(isScanComplete(scan_angle, laser_vars))
    {
        // resize the array of all scans to the actual count
        laser_vars.output_scan.horizontal_scans.resize(laser_vars.horizontal_scan_count);
        
        // interpolate time between the actual and the last sample for all vertical scans
        base::Time last_output_time = laser_vars.output_scan.time;
        laser_vars.output_scan.time = laser_vars.timestamp_estimator->update(laser_vars.last_sample_time);
        
        if(last_output_time.microseconds == 0 || last_output_time > laser_vars.output_scan.time)
            last_output_time = laser_vars.output_scan.time;
        
        double time_between_scans_in_microseconds = ((double)(laser_vars.output_scan.time - last_output_time).microseconds) / (double)laser_vars.horizontal_scan_count;
        for(unsigned i = 0; i < laser_vars.horizontal_scan_count; i++)
        {
            laser_vars.output_scan.horizontal_scans[(laser_vars.horizontal_scan_count-1) - i].time = laser_vars.output_scan.time - base::Time::fromMicroseconds((int64_t)(time_between_scans_in_microseconds * i));
        }

        // write sample to output port
        if(laser_vars.head_pos == UpperHead)
            _laser_scans.write(laser_vars.output_scan);
        else
            _laser_scans_lower_head.write(laser_vars.output_scan);
        laser_vars.horizontal_scan_count = 0;
    }

    // increase array size of horizontal scans if needed
    if(laser_vars.horizontal_scan_count >= laser_vars.output_scan.horizontal_scans.size())
        laser_vars.output_scan.horizontal_scans.resize(laser_vars.horizontal_scan_count + 1);
    
    // add the new vertical scan to the output data
    laserdriver.convertToVerticalMultilevelScan(horizontal_scan, laser_vars.output_scan.horizontal_scans[laser_vars.horizontal_scan_count]);
    laser_vars.horizontal_scan_count++;
    laser_vars.last_sample_time = base::Time::now();
}

void Task::addDummyData(const base::Angle& next_angle, Task::LaserHeadVariables& laser_vars)
{
    if(laser_vars.horizontal_scan_count > 1)
    {
        // get angular resolution of the first two scans
        base::Angle angular_resolution = laser_vars.output_scan.horizontal_scans[1].horizontal_angle - laser_vars.output_scan.horizontal_scans[0].horizontal_angle;
        if(angular_resolution.rad > 0.0)
        {
            base::Angle last_angle = laser_vars.output_scan.horizontal_scans[laser_vars.horizontal_scan_count-1].horizontal_angle;
            base::Angle new_angle = next_angle;
            // limit the new angle if the scan is almost complete 
            if(isScanComplete(next_angle, laser_vars))
                new_angle = laser_vars.output_scan.horizontal_scans[0].horizontal_angle;
            
            base::Angle angular_gap = new_angle - last_angle;
            if(angular_gap.rad > 0.0)
            {
                // fill the gap with dummy samples
                int missing_sample_count = ((int)(angular_gap.rad / angular_resolution.rad)) - 1;
                for(int i = 1; i <= missing_sample_count; i++)
                {
                    // add a dummy sample to the output scan
                    base::Angle missing_sample_angle = last_angle + i * angular_resolution;
                    velodyne_fire_t horizontal_scan;
                    createHorizontalDummy(missing_sample_angle, laser_vars.head_pos, horizontal_scan);
                    handleHorizontalScan(horizontal_scan, laser_vars);
                }
            }
        }
    }
}

bool Task::getFirstAngle(LaserHead head_pos, const velodyne_data_packet& new_scans, base::Angle& first_angle) const
{
    for(unsigned i = 0; i < VELODYNE_NUM_SHOTS; i++)
    {
        if((head_pos == UpperHead && new_scans.shots[i].lower_upper == VELODYNE_UPPER_HEADER_BYTES) || 
           (head_pos == LowerHead && new_scans.shots[i].lower_upper == VELODYNE_LOWER_HEADER_BYTES))
        {
            first_angle = base::Angle::fromDeg(((double)new_scans.shots[i].rotational_pos) * 0.01);
            return true;
        }
    }
    return false;
}

void Task::createHorizontalDummy(const base::Angle& angle, Task::LaserHead head_pos, velodyne_fire_t& horizontal_scan) const
{
    // set all 32 vertical scans to out of range
    for(unsigned i = 0; i < VELODYNE_NUM_LASERS; i++)
    {
        horizontal_scan.lasers[i].distance = 0;
        horizontal_scan.lasers[i].intensity = 0;
    }
    // set head position
    if(head_pos == UpperHead)
        horizontal_scan.lower_upper = VELODYNE_UPPER_HEADER_BYTES;
    else
        horizontal_scan.lower_upper = VELODYNE_LOWER_HEADER_BYTES;
    // set horizontal angle
    double angle_deg = angle.getDeg();
    if(angle_deg < 0.0)
        angle_deg += 360.0;
    horizontal_scan.rotational_pos = (uint16_t)(angle_deg * 100.0);
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.




bool Task::configureHook()
{
    if (! RTT::TaskContext::configureHook())
        return false;
    
    upper_head.timestamp_estimator = new aggregator::TimestampEstimator(base::Time::fromSeconds(20), base::Time::fromMilliseconds(100));
    lower_head.timestamp_estimator = new aggregator::TimestampEstimator(base::Time::fromSeconds(20), base::Time::fromMilliseconds(100));
        
    return true;
}



bool Task::startHook()
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
    unsigned scans_per_turn = 2000; // upper guess
    
    // initiate head variables
    upper_head.timestamp_estimator->reset();
    lower_head.timestamp_estimator->reset();
    upper_head.horizontal_scan_count = 0;
    lower_head.horizontal_scan_count = 0;
    upper_head.output_scan.horizontal_scans.reserve(scans_per_turn);
    lower_head.output_scan.horizontal_scans.reserve(scans_per_turn);
    upper_head.output_scan.time.microseconds = 0;
    lower_head.output_scan.time.microseconds = 0;
    upper_head.head_pos = UpperHead;
    lower_head.head_pos = LowerHead;
    
    return true;
}



void Task::updateHook()
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
        // check packet timestamp
        if(last_gps_timestamp > 0 && last_gps_timestamp < buffer.gps_timestamp)
        {
            if(last_packet_period + gps_timestamp_tolerance < buffer.gps_timestamp - last_gps_timestamp)
            {
                // handle packet lost
                RTT::log(RTT::Info) << TaskContext::getName() << ": "
                    << "Lost at least one data packet. Filling up the hole in the output scan with dummy data." << RTT::endlog();
                    
                base::Angle first_upper_angle, first_lower_angle;
                if(getFirstAngle(UpperHead, buffer, first_upper_angle))
                {
                    addDummyData(first_upper_angle, upper_head);
                }
                if(getFirstAngle(LowerHead, buffer, first_lower_angle))
                {
                    addDummyData(first_lower_angle, lower_head);
                }
            }
            last_packet_period = buffer.gps_timestamp - last_gps_timestamp;
        }
        last_gps_timestamp = buffer.gps_timestamp;
            
        for(unsigned i = 0; i < VELODYNE_NUM_SHOTS; i++)
        {
            if(buffer.shots[i].lower_upper == VELODYNE_UPPER_HEADER_BYTES)
            {
                // handle upper laser
                handleHorizontalScan(buffer.shots[i], upper_head);
            }
            else
            {
                // handle lower laser
                handleHorizontalScan(buffer.shots[i], lower_head);
            }
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



void Task::errorHook()
{
    RTT::TaskContext::errorHook();
}



void Task::stopHook()
{
    RTT::TaskContext::stopHook();
    
    laserdriver.close();
}



void Task::cleanupHook()
{
    RTT::TaskContext::cleanupHook();
    
    delete upper_head.timestamp_estimator;
    delete lower_head.timestamp_estimator;
}

