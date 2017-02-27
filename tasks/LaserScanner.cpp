/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "LaserScanner.hpp"

#include <rtt/extras/FileDescriptorActivity.hpp>
#include <rtt/base/ActionInterface.hpp>
#include <velodyne_lidar/VelodyneHDL32EDriver.hpp>
#include <velodyne_lidar/VelodyneVLP16Driver.hpp>
#include <sys/socket.h>
#include "VelodyneLidarTypes.hpp"

using namespace velodyne_lidar;


LaserScanner::LaserScanner(std::string const& name)
    : LaserScannerBase(name), laserdriver(NULL)
{
    _io_write_timeout.set(base::Time::fromSeconds(1.0));
    _io_read_timeout.set(base::Time::fromSeconds(1.0));
}

LaserScanner::LaserScanner(std::string const& name, RTT::ExecutionEngine* engine)
    : LaserScannerBase(name, engine), laserdriver(NULL)
{
    _io_write_timeout.set(base::Time::fromSeconds(1.0));
    _io_read_timeout.set(base::Time::fromSeconds(1.0));
}

LaserScanner::~LaserScanner()
{
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool LaserScanner::configureHook()
{
    if (! RTT::TaskContext::configureHook())
        return false;

    delete laserdriver;
    laserdriver = NULL;

    if(_sensor_type.value() == HDL64E)
    {
        RTT::log(RTT::Error) << "The HDL-64E is currently not supported!" << RTT::endlog();
        return false;
    }
    else if(_sensor_type.value() == HDL32E)
        laserdriver = new VelodyneHDL32EDriver();
    else if(_sensor_type.value() == VLP16)
        laserdriver = new VelodyneVLP16Driver();
    else
    {
        RTT::log(RTT::Error) << "No valid sensor type selected!" << RTT::endlog();
        return false;
    }

    // setup udp server
    laserdriver->openUDP("", VELODYNE_DATA_UDP_PORT);
    laserdriver->setReadTimeout(base::Time::fromSeconds(0.5/laserdriver->getBroadcastFrequency()));
    laserdriver->setWriteTimeout(_io_write_timeout.value());
    laserdriver->setScanSize(_full_scan_size.value());
    no_packet_timeout = base::Timeout(_io_read_timeout.value());

    // trigger the update hook on fd activity
    RTT::extras::FileDescriptorActivity* fd_activity =
        getActivity<RTT::extras::FileDescriptorActivity>();
    if (fd_activity)
    {
        fd_activity->watch(laserdriver->getFileDescriptor());
        fd_activity->setTimeout(_io_read_timeout.get().toMilliseconds());
    }

    RTT::base::ActivityInterface* activity = getActivity();
    if (activity && !fd_activity)
    {
        // check socket receive buffer size
        int expected_buffer_size = (double)(VELODYNE_DATA_MSG_BUFFER_SIZE + 52) * laserdriver->getBroadcastFrequency() * activity->getPeriod() * 1.5;
        if(setsockopt(laserdriver->getFileDescriptor(), SOL_SOCKET, SO_RCVBUF, &expected_buffer_size, sizeof(expected_buffer_size)) < 0)
        {
            RTT::log(RTT::Error) << "Failed to set the socket receive buffer size. " << strerror(errno) << RTT::endlog();
            return false;
        }

        int current_buffer_size = 0;
        socklen_t current_buffer_size_len = sizeof(current_buffer_size);
        if(getsockopt(laserdriver->getFileDescriptor(), SOL_SOCKET, SO_RCVBUF, &current_buffer_size, &current_buffer_size_len) < 0)
        {
            RTT::log(RTT::Error) << "Failed to get the socket receive buffer size. " << strerror(errno) << RTT::endlog();
            return false;
        }
        // Note that the kernel already doubles the buffer size to compensate for bookkeeping overhead
        if(current_buffer_size < expected_buffer_size * 2)
        {
            double min_period = current_buffer_size / ((double)(VELODYNE_DATA_MSG_BUFFER_SIZE + 52) * laserdriver->getBroadcastFrequency() * 3.0);
            RTT::log(RTT::Error) << "Failed to set the socket receive buffer size to " << expected_buffer_size << "\n" <<
                                    "Either increase the period in which this task is triggered to at least " << min_period << "\n" <<
                                    "or change the systems max buffer size in /proc/sys/net/core/rmem_max accordingly." << RTT::endlog();
            return false;
        }
    }

    return true;
}

bool LaserScanner::startHook()
{
    if (! RTT::TaskContext::startHook())
        return false;

    no_packet_timeout.restart();

    return true;
}

void LaserScanner::updateHook()
{
    RTT::TaskContext::updateHook();

    bool valid_sample = false;
    base::samples::DepthMap upper_sample;
    base::samples::DepthMap lower_sample;

    try
    {
        while(laserdriver->readNewPacket())
        {
            if(laserdriver->isScanComplete())
            {
                valid_sample = laserdriver->convertScanToSample(upper_sample, _use_remissions.value());

                // write out new samples
                if(!_only_write_newest_sample.value() && valid_sample)
                    _laser_scans.write(upper_sample);
            }
            no_packet_timeout.restart();
        }

        if(no_packet_timeout.elapsed())
            throw std::runtime_error("Read timeout! Received no new packets from the sensor.");

        // write latest samples after buffer was red out
        if(_only_write_newest_sample.value() && valid_sample)
            _laser_scans.write(upper_sample);

        // check if packets have been lost
        uint64_t current_lost_packet_count = laserdriver->getPacketLostCount();
        uint64_t recently_lost_packets = current_lost_packet_count - last_lost_packet_count;
        last_lost_packet_count = current_lost_packet_count;
        if(recently_lost_packets > 0)
        {
            RTT::log(RTT::Info) << recently_lost_packets << " packets have been lost recently!" << RTT::endlog();
            state(PACKET_LOSS);
        }
        else if(state() != RUNNING)
            state(RUNNING);

    }
    catch (const std::runtime_error & e)
    {
        RTT::log(RTT::Error) << TaskContext::getName() << ": " << e.what() << RTT::endlog();
        exception(IO_ERROR);
    }
}

void LaserScanner::errorHook()
{
    RTT::TaskContext::errorHook();
}

void LaserScanner::stopHook()
{
    RTT::TaskContext::stopHook();
}

void LaserScanner::cleanupHook()
{
    RTT::TaskContext::cleanupHook();

    RTT::extras::FileDescriptorActivity* fd_activity = getActivity<RTT::extras::FileDescriptorActivity>();
    if(fd_activity)
        fd_activity->clearAllWatches();
    laserdriver->close();

    delete laserdriver;
    laserdriver = NULL;
}
