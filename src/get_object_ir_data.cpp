#include <iostream>
#include <memory>
#include <functional>
#include <chrono>
#include <rclcpp/rclcpp.hpp> 
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <motion_planning_abstractions/dual_arm_waypoint_programming.hpp>
#include <std_srvs/srv/trigger.hpp>


int main(int argc, char**argv){
    
    rclcpp::init(argc,argv);
    
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("get_object_ir_data");
    
    auto daci = std::make_shared<DualArmControlInterface>(node);
    
    auto server = [node,daci](
        std_srvs::srv::Trigger::Request::SharedPtr req, 
        std_srvs::srv::Trigger::Response::SharedPtr res
    ){
        daci->move_to_joint_positions(
            std::vector<double>{
                0.0,
                -1.26797,
                -2.32253,
                -2.78958,
                -1.62478,
                1.56858
            },
            daci->right_move_group_interface_
        );

        // auto curr_js = daci->get_current_joint_state("left");
        // for(int i=0; i<curr_js.size(); i++){
        //     std::cout<<curr_js[i]<<std::endl;
        // }

        // float H = 0.35;// scanning at 35cm height

        
        res->success = true;
        res->message = "muahahahaha!";
    };

    auto recb = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto service_server = node->create_service<std_srvs::srv::Trigger>(
        "~/call_server",
        server,
        rmw_qos_profile_services_default,
        recb
    );

    auto mex = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    
    mex->add_node(node);
    mex->spin();
    
    return 0;
}