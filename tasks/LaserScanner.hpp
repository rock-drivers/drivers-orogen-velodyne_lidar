/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef VELODYNE_LIDAR_LaserScanner_TASK_HPP
#define VELODYNE_LIDAR_LaserScanner_TASK_HPP

#include "velodyne_lidar/LaserScannerBase.hpp"
#include <velodyne_lidar/velodyneDataDriver.hpp>
#include <base/samples/DepthMap.hpp>


namespace aggregator
{
    class TimestampEstimator;
}

namespace velodyne_lidar {

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','velodyne_lidar::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class LaserScanner : public LaserScannerBase
    {
		
	friend class LaserScannerBase;
    
    enum LaserHead
    {
        LowerHead,
        UpperHead
    };
	
	struct Buffer
	{
		typedef Eigen::Matrix<base::samples::DepthMap::scalar, Eigen::Dynamic, Eigen::Dynamic> CollectorMatrix;
		CollectorMatrix distances, remissions;
	};
               
    struct LaserHeadVariables
    {
        LaserHead head_pos;
        unsigned int horizontal_scan_count;
        base::Time last_sample_time;
        base::samples::DepthMap dmap;
        Buffer buffer;
        LaserHeadVariables() : horizontal_scan_count(0) {};
    };
    
    protected:
        VelodyneDataDriver laserdriver;
        velodyne_data_packet_t buffer;
        States last_state;
        int last_packet_period; // in microseconds
        uint32_t last_gps_timestamp; // in microseconds
        uint32_t gps_timestamp_tolerance; // in microseconds
        
        uint32_t expectedPacketPeriod;
        uint64_t integratedSensorTime;
        base::Time estimatedPacketTime;
        base::Time lastEstimatedPacketTime;
        aggregator::TimestampEstimator* timestamp_estimator;
                
        /* The HDL-32E has only an upper head */
        LaserHeadVariables upper_head;
        LaserHeadVariables lower_head;
        
    protected:
        bool isScanComplete(const LaserScanner::LaserHeadVariables& laser_vars, const base::Angle& current_angle) const;
        void handleHorizontalScan(const velodyne_lidar::velodyne_fire_t& horizontal_scan, velodyne_lidar::LaserScanner::LaserHeadVariables& laser_vars, const base::Time& shotTime);
        void addDummyData(const base::Angle &next_angle, LaserHeadVariables& laser_vars);
        
    private:
        bool getFirstAngle(LaserHead head_pos, const velodyne_data_packet& new_scans, base::Angle& first_angle) const;
        void createHorizontalDummy(const base::Angle &angle, LaserHead head_pos, velodyne_fire_t& horizontal_scan) const;
		void handleDataMapping(std::vector<base::samples::DepthMap::scalar> &target_container, Buffer::CollectorMatrix &source_container, uint32_t vertical_size, uint32_t horizontal_size);
		void resetSample(LaserHeadVariables& laser_vars);
		
    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        LaserScanner(std::string const& name = "velodyne_lidar::LaserScanner");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        LaserScanner(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~LaserScanner();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

