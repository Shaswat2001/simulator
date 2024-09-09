#include <functional>
#include <memory>
#include <vector>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <rosgraph_msgs/msg/clock.hpp>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_diff_drive.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <sdf/sdf.hh>
#include<ignition/math6/ignition/math/Pose3.hh>
#include "std_msgs/msg/float32.hpp"
#include <std_msgs/msg/string.hpp>
#include "uam_msgs/srv/request_uav_vel.hpp"
#include "uam_msgs/srv/response_uav_pose.hpp"
#include "uam_msgs/srv/request_uav_pose.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace gazebo
{
    class ObsModelPush : public ModelPlugin
    {
    public:
        /// A pointer to the GazeboROS node.
        gazebo_ros::Node::SharedPtr ros_node_;

        /// Connection to event called at every world iteration.
        gazebo::event::ConnectionPtr update_connection_;

        rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clk_subscriber_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            this->model = _model;
            // Initialize ROS node
            this->ros_node_ = gazebo_ros::Node::Get(_sdf);
            // Get QoS profiles
            const gazebo_ros::QoS &qos = this->ros_node_->get_qos();
            RCLCPP_WARN(this->ros_node_->get_logger(), "*Loading OBS* (LOL)");

            clk_subscriber_ = this->ros_node_->create_subscription<rosgraph_msgs::msg::Clock>("clock", 10, std::bind(&ObsModelPush::clk_update, this, std::placeholders::_1));           
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ObsModelPush::OnUpdate, this));

        }

        void OnUpdate()
        {
            
            this->model->GetLink("link_obs")->SetLinearVel({ -0.1, 0, 0});

        }


        void clk_update(const rosgraph_msgs::msg::Clock::SharedPtr time) const
        {
            
        }

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
        physics::ModelPtr model;
        std::vector<std::string> substrings;
        std::string substring;
        std::vector<double> doubles;
        
        std::string token;
        std::vector<std::string> tokens;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ObsModelPush)
}