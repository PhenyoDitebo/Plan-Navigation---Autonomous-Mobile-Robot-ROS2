#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <thread>

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std;

class SimpleLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
    public:
        explicit SimpleLifecycleNode (const std::string & node_name, bool intra_process_comms = false)
        : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) 
        {
            
        }

        // this will get called back when node goes from unconfigured -> inactive state.
        // typically used to set up resources that don't need to be active yet (allocate memory, create publishers/subscribers, load parameters, etc.)
        // must return a [CallBackReturn] value indicating success or failure. 
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
        {
            // ----- UNCONFIG -> INACTIVE FUNCTION -----
            sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleLifecycleNode::msgCallback, this, _1));
            RCLCPP_INFO(get_logger(), "Lifecycle Node on_configure() called");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        // ------- SHUT DOWN FUNCTION -------
        // takes the lifecycle node to its finalised state where it will be destroyed eventually.
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
        {
            sub_.reset();
            RCLCPP_INFO(get_logger(), "Lifecycle Node on_shutdown() called");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        // ------- CLEAN UP FUNCTION -------- 
        // takes node from inactive state to unconfigured.
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
        {
            sub_.reset();
            RCLCPP_INFO(get_logger(), "Lifecycle Node on_cleanup() called");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        // ------- ACTIVATE FUNCTION --------
        // takes inactive mode -> active mode
        // only mode in which nodes can proccess requests
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
        {
            LifecycleNode::on_activate(state);
            RCLCPP_INFO(get_logger(), "Lifecycle Node on_activate() called");
            std::this_thread::sleep_for(2s); // used to simulate hardware initialization e.g. some devices need time to warm up and stabalize after activation
            // allow dependent systems to catch up. 
            // testing and debugging - make the activation process observable so you can see the state transitions happening
            // Prevent race conditions - ensure that resources are truly ready before processes starts.
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        // ------- DEACTIVATE FUNCTION -------- 
        // takes node from active state to active.
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state)
        {
            LifecycleNode::on_deactivate(state);
            RCLCPP_INFO(get_logger(), "Lifecycle Node on_deactivate() called");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        // --------- CALL BACK FUNCTION -------
        void msgCallback(const std_msgs::msg::String & msg) {
            
            auto state = get_current_state();

            if (state.label() == "active") {
                RCLCPP_INFO_STREAM(get_logger(), "Lifecylce node heard: " <<msg.data.c_str());
            }
            else {
                cout << "The node may be unconfigured or inactive. Or an error has occured." << endl;
            }
        }
    

        private:
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

};

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv); // initialise ROS 2.
    rclcpp::executors::SingleThreadedExecutor ste; // this manages node call backs
    std::shared_ptr<SimpleLifecycleNode> simple_lifecycle_node = std::make_shared<SimpleLifecycleNode>("simple_lifecycle_node"); //makes an instance of the lifecycle node
    ste.add_node(simple_lifecycle_node->get_node_base_interface()); // registers node with tthe executor
    ste.spin(); // starts the executor loop, processes call backs indefinately
    rclcpp::shutdown(); // cleans up ROS 2 when done

    return 0; // exits the program.
}
