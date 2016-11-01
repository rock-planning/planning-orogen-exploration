/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef EXPLORATION_TASK_TASK_HPP
#define EXPLORATION_TASK_TASK_HPP

#include "exploration/TaskBase.hpp"
#include "orocos/envire/Orocos.hpp"
#include "envire/Core.hpp"
#include "envire/maps/MLSGrid.hpp"
#include "envire/maps/TraversabilityGrid.hpp"
#include "exploration/Planner.hpp"
#include <base/Timeout.hpp>

namespace envire {
    class Environment;
}

namespace exploration {

    /** \class Task 
     * Calculates exploration goals. First a traversability map has to be received,
     * from which the exploration map is created. A new traversability map with a 
     * different size/offset/scale will create a new exploration map and the previous
     * content will be transferred. A trav map with the same configuration just updates
     * the obstacles.
     * \warning Exploration goals have to be triggered explicitely using the port 'calculateGoals'!
     */
    class Task : public TaskBase
    {
        friend class TaskBase;
    protected:

        virtual bool sendNextGoal();

        Planner planner;
        envire::Environment* mEnv; 
        RTT::FlowStatus mTraversabilityMapStatus;
        RTT::FlowStatus receiveEnvireData();
        envire::TraversabilityGrid* traversability;
        envire::FrameNode* lastTraversabilityFrameNode;
        double lastOffsetX;
        double lastOffsetY;
        double lastScaleX;
        double lastScaleY;
        
        
        base::samples::RigidBodyState start_vec;
        std::vector<base::Vector3d> goals;
        std::vector<base::samples::RigidBodyState> finGoals;
        std::vector<base::samples::RigidBodyState>::iterator nextGoal;
		
        bool extractTraversability();
        bool extractMLS();
        //reveales areas that have been seen now
        RTT::FlowStatus updateMap();
        //dumps the map on debugport
        bool flushMap();        
        // generates the goals
        void generateGoals();
        bool initialized;
        /** robotposition tranformed to the exploramap-grid **/
        exploration::Pose pose;
        /** robotpose in world-coordinates **/
        base::samples::RigidBodyState robotPose;
        bool triggered;
        base::Timeout* timeout;
        
        public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "exploration::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
        */
        ~Task();

        virtual void calculateGoals();
        
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
        
        inline void clearPlannerMap() {
            planner.clearCoverageMap();
        }
    };
}

#endif

