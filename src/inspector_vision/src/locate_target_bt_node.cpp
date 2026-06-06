#include <string>
#include <memory>
#include <cstdio>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "inspector_interfaces/action/locate_target.hpp"
#include "behaviortree_cpp/bt_factory.h"
// #include "pluginlib/class_list_macros.hpp"
// #include "nav2_behavior_tree/bt_plugin.hpp"

namespace inspector_bt_plugins
{

class LocateTargetAction : public nav2_behavior_tree::BtActionNode<inspector_interfaces::action::LocateTarget>
{
public:
    // Constructor
    LocateTargetAction(const std::string & xml_tag_name, const std::string & action_name, const BT::NodeConfig & conf)
    : BtActionNode<inspector_interfaces::action::LocateTarget>(xml_tag_name, action_name, conf)
    {
    }

    // Called when the Behavior Tree reaches this node
    void on_tick() override
    {
        // Pull the target_id from the BT XML and push it into the Action Goal
        std::string target_id;
        getInput("target_id", target_id);
        goal_.target_id = target_id;
        
        RCLCPP_INFO(node_->get_logger(), "BT Node Ticked: Requesting vision lock for %s", target_id.c_str());
    }

    // Called when the Action Server returns SUCCESS
    BT::NodeStatus on_success() override
    {
        RCLCPP_INFO(node_->get_logger(), "BT Node Succeeded: Vision locked onto target.");
        return BT::NodeStatus::SUCCESS;
    }

    // Define the required inputs from the XML file
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<std::string>("target_id", "The ID of the ArUco payload to locate")
        });
    }
};

}  // namespace inspector_bt_plugins

// ==========================================================
// Plugin Registration Macro (Allows Nav2 to load this at runtime)
// ==========================================================
BT_REGISTER_NODES(factory)
{
    // ===== DIAGNOSTIC: Remove after confirming registration works =====
    fprintf(stderr, "\n\n========================================\n");
    fprintf(stderr, ">>> BT_REGISTER_NODES: LocateTarget plugin LOADING <<<\n");
    fprintf(stderr, "========================================\n\n");

    // The explicit lambda builder bypasses dummy instantiation crashes
    BT::NodeBuilder builder =
        [](const std::string & name, const BT::NodeConfig & config)
    {
        return std::make_unique<inspector_bt_plugins::LocateTargetAction>(
            name, "locate_target", config);
    };

    factory.registerBuilder<inspector_bt_plugins::LocateTargetAction>(
        "LocateTarget", builder);

    fprintf(stderr, "\n>>> BT_REGISTER_NODES: LocateTarget REGISTERED SUCCESSFULLY <<<\n\n");
}
// PLUGINLIB_EXPORT_CLASS(inspector_bt_plugins::LocateTargetAction, nav2_behavior_tree::BtActionNode<inspector_interfaces::action::LocateTarget>)
// NAV2_DYNAMIC_PLUGIN(inspector_bt_plugins::LocateTargetAction)