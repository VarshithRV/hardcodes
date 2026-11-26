#include <iostream>
#include <memory>
#include <functional>
#include <chrono>
#include <rclcpp/rclcpp.hpp> 
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <motion_planning_abstractions/dual_arm_waypoint_programming.hpp>
#include <std_srvs/srv/trigger.hpp>


using namespace std::chrono_literals;

int main(int argc, char**argv){
    
    rclcpp::init(argc,argv);
    
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("get_object_ir_data");
    
    auto daci = std::make_shared<DualArmControlInterface>(node);
    
    auto print_vector = [](std::vector<double> vec){
        for(int i=0; i<vec.size(); i++){
            std::cout<<vec[i]<<std::endl;
        }
    };

    auto isometry2pose = [](Eigen::Isometry3d frame){
        geometry_msgs::msg::Pose pose;
        pose.position.x = frame.translation().x();
        pose.position.y = frame.translation().y();
        pose.position.z = frame.translation().z();
        auto rotation = Eigen::Quaterniond(frame.rotation());
        pose.orientation.w = rotation.w();
        pose.orientation.x = rotation.x();
        pose.orientation.y = rotation.y();
        pose.orientation.z = rotation.z();
        return pose;
    };

    auto server = [node,daci,print_vector,isometry2pose](
        std_srvs::srv::Trigger::Request::SharedPtr req, 
        std_srvs::srv::Trigger::Response::SharedPtr res
    ){
        daci->move_to_joint_positions(
            {
                0.0,
                -1.26797,
                -2.32253,
                -2.78958,
                -1.62478,
                1.56858 + M_PI
            },
            daci->left_move_group_interface_
        );
        // print_vector(daci->get_current_joint_state("left"));

        float H = 0.35,R = 0.35;// scanning at 35cm height
        float theta = M_PI/3; // 60 deg swing -M_PI/12 to M_PI/12
        
        auto current_pose = daci->get_current_ee_pose("left");
        
        Eigen::Vector3d current_position(
            current_pose->position.x,
            current_pose->position.y,
            current_pose->position.z
        );
        
        Eigen::Quaterniond current_orientation(
            current_pose->orientation.w,
            current_pose->orientation.x,
            current_pose->orientation.y,
            current_pose->orientation.z
        );
        
        std::vector<Eigen::Isometry3d> poses;
        poses.reserve(6);
        
        Eigen::Vector3d refpt(current_position - R*Eigen::Vector3d::UnitY());
        
        for(int i=0; i<7; i++){
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            
            auto current_angle = -theta/2 + (i*theta/6);

            pose.translate(refpt + (Eigen::AngleAxisd(
                    -current_angle,
                    Eigen::Vector3d::UnitZ()
                    ).toRotationMatrix()*
                    (R*Eigen::Vector3d::UnitY())
                )
            );
            
            pose.rotate(
                current_orientation.toRotationMatrix()*
                (Eigen::AngleAxisd(
                    current_angle,
                    Eigen::Vector3d::UnitX()
                ).toRotationMatrix())
            );

            poses.push_back(pose);
        }

        // go to each point synchronously using task space cubic trajectory
        for(int i=0; i<poses.size(); i++){
            auto target = isometry2pose(poses[i]);
            daci->execute_waypoints_cubic({target},{2},0.3,0.0,"left");
            std::this_thread::sleep_for(5s);
        }

        daci->move_to_joint_positions(
            {
                0.0,
                -1.26797,
                -2.32253,
                -2.78958,
                -1.62478,
                1.56858 + M_PI
            },
            daci->left_move_group_interface_
        );

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