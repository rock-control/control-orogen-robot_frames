/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROBOT_FRAMES_CHAINPUBLISHER_TASK_HPP
#define ROBOT_FRAMES_CHAINPUBLISHER_TASK_HPP

#include "robot_frames/ChainPublisherBase.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base-logging/Logging.hpp>

#include <robot_frames/RobotFrames.hpp>

namespace robot_frames {


class ChainPublisher : public ChainPublisherBase
{
    friend class ChainPublisherBase;
protected:
     base::samples::Joints joint_state;
     std::vector<base::samples::RigidBodyState> bt_frames_;
     std::vector<RTT::OutputPort<base::samples::RigidBodyState>*> output_ports_;

     ChainTransformationCalculator* chain_transformer;

public:
    ChainPublisher(std::string const& name = "robot_frames::ChainPublisher");
    ChainPublisher(std::string const& name, RTT::ExecutionEngine* engine);

    ~ChainPublisher();

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

