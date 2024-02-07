/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ChainPublisher.hpp"

using namespace robot_frames;

ChainPublisher::ChainPublisher(std::string const& name)
    : ChainPublisherBase(name)
{
}

ChainPublisher::ChainPublisher(std::string const& name, RTT::ExecutionEngine* engine)
    : ChainPublisherBase(name, engine)
{
}

ChainPublisher::~ChainPublisher()
{
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ChainPublisher.hpp for more detailed
// documentation about them.

bool ChainPublisher::configureHook()
{
    if (! ChainPublisherBase::configureHook())
        return false;

    std::vector<robot_frames::Chain> chain_definitions = _chains.get();
    std::string urdf_file_path = _urdf_file.get();

    ChainTransformationCalculator transform_calculator(chain_definitions, urdf_file_path);
    // assign the created object to the chain_transformer pointer to access this object in update_hook 
    chain_transformer = &transform_calculator;

    //Create dynamic output per chain
    for(int i=0; i< chain_transformer->get_number_chains(); i++){
        std::string port_name = chain_transformer->get_chain_names()[i];
        LOG_INFO("Adding port %s to component", port_name.c_str());
        RTT::OutputPort<base::samples::RigidBodyState>* output_port =
                new RTT::OutputPort<base::samples::RigidBodyState>(port_name);
        ports()->addPort(port_name, *output_port);
        out_ports_[i] = output_port;
    }

    return true;
}
bool ChainPublisher::startHook()
{
    if (! ChainPublisherBase::startHook())
        return false;
    return true;
}
void ChainPublisher::updateHook()
{
    ChainPublisherBase::updateHook();

    while (_input.read(joint_state, false) == RTT::NewData){
        chain_transformer->update_transforms(joint_state, bt_frames_);
        for(int i=0; i< chain_transformer->get_number_chains(); i++){
            out_ports_[i]->write(bt_frames_[i]);
            }
        }
}
void ChainPublisher::errorHook()
{
    ChainPublisherBase::errorHook();
}
void ChainPublisher::stopHook()
{
    ChainPublisherBase::stopHook();
}
void ChainPublisher::cleanupHook()
{
    ChainPublisherBase::cleanupHook();
}
