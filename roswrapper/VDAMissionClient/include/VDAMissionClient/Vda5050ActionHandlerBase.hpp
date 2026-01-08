#pragma once

#include "vda5050_msgs/msg/action.hpp"
#include <string>
class VDAMissionClient;
class Vda5050ActionHandlerBase
{

public:
    virtual void Initialize(VDAMissionClient *client_node) = 0;
    // Execute the action. It should update the action state to RUNNING when the action is executed
    // and set the action state to FINISHED/FAILED when the action is done.
    virtual void Execute(const vda5050_msgs::msg::Action &vda5050_action) = 0;
    // Cancel the action. Default implementation is empty, mission client will wait for the
    // action to finish when canceling the order. If the action can be interrupted, the action
    // handler should implement this function to interrupt the action and set the action state to
    // FAILED
    virtual void Cancel(const std::string & /*action_id*/) {}
    // Pause the action. It should be implemented if the action can be paused(can_be_paused is true)
    // If the action is paused, the action handler should set the action state to PAUSED
    virtual void Pause(const std::string & /*action_id*/) {}
    // Resume the action. It should be implemented if the action can be paused.
    // If the action is resumed, the action handler should set the action state to RUNNING
    virtual void Resume(const std::string & /*action_id*/) {}
    virtual ~Vda5050ActionHandlerBase() {}

    bool can_be_paused{false};

protected:
    Vda5050ActionHandlerBase() {}
    VDAMissionClient *client_node_;
};
// namespace mission_client
