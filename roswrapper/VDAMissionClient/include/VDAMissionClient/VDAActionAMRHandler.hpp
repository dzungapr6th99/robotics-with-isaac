#include <string>
#include "VDAMissionClient/Vda5050ActionHandlerBase.hpp"
#include "VDAMissionClient/VDAMissionClient.hpp"
class VDAActionAMRHandler : public Vda5050ActionHandlerBase
{
public:
    void Initialize(VDAMissionClient *client_node) override;
    void Execute(const vda5050_msgs::msg::Action &vda5050_action) override;
    void Cancel(const std::string &action_id) override;
private:
    void HandlePick(vda5050_msgs::msg::Action action);
    void HandleDrop(vda5050_msgs::msg::Action action);
};