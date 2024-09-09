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
#include "uam_msgs/srv/uam.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace gazebo
{
    class ModelPush : public ModelPlugin
    {
    public:
        /// A pointer to the GazeboROS node.
        gazebo_ros::Node::SharedPtr ros_node_;

        /// Subscriber to command velocities
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

        /// Odometry publisher
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

        /// Connection to event called at every world iteration.
        gazebo::event::ConnectionPtr update_connection_;

        rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clk_subscriber_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    
        rclcpp::Service<uam_msgs::srv::Uam>::SharedPtr service;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twst_subscriber_;

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr manip_ee_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr manip_ee_diff_pub_;
        
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr joint_subscription_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr body_subscription_;


        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            this->model = _model;
            // Initialize ROS node
            this->ros_node_ = gazebo_ros::Node::Get(_sdf);
            // Get QoS profiles
            const gazebo_ros::QoS &qos = this->ros_node_->get_qos();
            RCLCPP_WARN(this->ros_node_->get_logger(), "*Loading Calmly* (LOL)");

            clk_subscriber_ = this->ros_node_->create_subscription<rosgraph_msgs::msg::Clock>("clock", 10, std::bind(&ModelPush::clk_update, this, std::placeholders::_1));

            manip_ee_pub_ = this->ros_node_->create_publisher<geometry_msgs::msg::Pose>("/manipulator_ee",10);
            manip_ee_diff_pub_ = this->ros_node_->create_publisher<geometry_msgs::msg::Pose>("/manipulator_ee_difference",10);

            service = this->ros_node_->create_service<uam_msgs::srv::Uam>("get_manip_pose", std::bind(&ModelPush::add, this, std::placeholders::_1,std::placeholders::_2));
            
            joint_subscription_ = this->ros_node_->create_subscription<std_msgs::msg::String>("/robot_joints_string", 10,std::bind(&ModelPush::robot_joint_stringCallback, this, std::placeholders::_1));
            body_subscription_ = this->ros_node_->create_subscription<std_msgs::msg::String>("/drone_body_string", 10,std::bind(&ModelPush::drone_body_stringCallback, this, std::placeholders::_1));
           
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));
            joint1_value_=0;
            joint2_value_=0;
            joint3_value_=0;
            joint4_value_=0;
            gripper_=0;

            x_value_=0;
            y_value_=0;
            z_value_=0.50;
            r1_value_=0;
            r2_value_=0;
            r3_value_=0.0;
            
        }

        void OnUpdate()
        {
            this->model->GetJoint("prop_1_joint")->SetVelocity(0, 100);
            this->model->GetJoint("prop_2_joint")->SetVelocity(0, -100);
            this->model->GetJoint("Prop_3_joint")->SetVelocity(0, 100);
            this->model->GetJoint("prop_4_joint")->SetVelocity(0, -100);
            this->model->GetJoint("prop_5_joint")->SetVelocity(0, 100);
            this->model->GetJoint("prop_6_joint")->SetVelocity(0, -100);

            this->model->GetJoint("arm_1_joint")->SetPosition(0,joint1_value_);
            this->model->GetJoint("arm_2_joint")->SetPosition(0,joint2_value_);
            this->model->GetJoint("arm_3_joint")->SetPosition(0,joint3_value_);
            this->model->GetJoint("arm_4_joint")->SetPosition(0,joint4_value_);
            this->model->GetJoint("gripper_left_joint")->SetPosition(0,gripper_);
            this->model->GetJoint("gripper_right_joint")->SetPosition(0,-gripper_);
            this->model->SetWorldPose(ignition::math::Pose3d( x_value_,  y_value_, z_value_,  r1_value_, r2_value_,  r3_value_));

            ignition::math::Pose3d pose = this->model->GetLink("gripper_left_link")->WorldPose();
            ignition::math::Pose3d pose_base = this->model->GetLink("base_link")->WorldPose();

            geometry_msgs::msg::Pose eePose;
            eePose.position.x = pose.Pos().X();
            eePose.position.y = pose.Pos().Y();
            eePose.position.z = pose.Pos().Z();

            eePose.orientation.x = pose.Rot().X();
            eePose.orientation.y = pose.Rot().Y();
            eePose.orientation.z = pose.Rot().Z();
            eePose.orientation.w = pose.Rot().W();

            geometry_msgs::msg::Pose PoseDifference;
            PoseDifference.position.x = pose_base.Pos().X() - pose.Pos().X();
            PoseDifference.position.y = pose_base.Pos().Y() - pose.Pos().Y();
            PoseDifference.position.z = pose_base.Pos().Z() - pose.Pos().Z();

            PoseDifference.orientation.x = pose_base.Rot().X() - pose.Rot().X();
            PoseDifference.orientation.y = pose_base.Rot().Y() - pose.Rot().Y();
            PoseDifference.orientation.z = pose_base.Rot().Z() - pose.Rot().Z();
            PoseDifference.orientation.w = pose_base.Rot().W() - pose.Rot().W();

            this->manip_ee_diff_pub_->publish(PoseDifference);
            this->manip_ee_pub_->publish(eePose);

        }


        void clk_update(const rosgraph_msgs::msg::Clock::SharedPtr time) const
        {
            
        }

        void pose_update(const geometry_msgs::msg::PoseStamped::SharedPtr pose) const
        { 
        
        }

 

        void robot_joint_stringCallback(std_msgs::msg::String::SharedPtr string_msg) 
        { 
            RCLCPP_INFO(this->ros_node_->get_logger(), "robot_joint message: %s", string_msg->data.c_str());
            
        
            std::cout << string_msg->data << std::endl;
            std::vector<std::string> stringVector;
            std::stringstream ss(string_msg->data);
            std::string token;

            while (std::getline(ss, token, ',')) 
            {
                stringVector.push_back(token);
            }

            // Print the strings from the vector (optional)
        
            double double_joint1_value_ = std::stod(stringVector[0]);
            double double_joint2_value_ = std::stod(stringVector[1]);
            double double_joint3_value_ = std::stod(stringVector[2]);
            double double_joint4_value_ = std::stod(stringVector[3]);
            double double_gripper_value_ = std::stod(stringVector[4]);

            joint1_value_=double_joint1_value_;
            joint2_value_=double_joint2_value_;
            joint3_value_=double_joint3_value_;
            joint4_value_=double_joint4_value_;
            gripper_=double_gripper_value_;
        }

        void drone_body_stringCallback(std_msgs::msg::String::SharedPtr string_msg) 
        { 
            RCLCPP_INFO(this->ros_node_->get_logger(), "drone body message: %s", string_msg->data.c_str());
          
            std::cout << string_msg->data << std::endl;
            std::vector<std::string> stringVector;
            std::stringstream ss(string_msg->data);
            std::string token;

            while (std::getline(ss, token, ',')) 
            {
                stringVector.push_back(token);
            }

            // Print the strings from the vector (optional)
        
            double double_x_value_ = std::stod(stringVector[0]);
            double double_y_value_ = std::stod(stringVector[1]);
            double double_z_value_ = std::stod(stringVector[2]);
            double double_r1_value_ = std::stod(stringVector[3]);
            double double_r2_value_ = std::stod(stringVector[4]);
            double double_r3_value_ = std::stod(stringVector[5]);

            x_value_=double_x_value_;
            y_value_=double_y_value_;
            z_value_=double_z_value_;
            r1_value_=double_r1_value_;
            r2_value_=double_r2_value_;
            r3_value_=double_r3_value_;
        }

        void robot_joint_string(std_msgs::msg::String string_msg) 
        { 
            RCLCPP_INFO(this->ros_node_->get_logger(), "robot_joint message: %s", string_msg.data.c_str());
            
        
            std::cout << string_msg.data << std::endl;
            std::vector<std::string> stringVector;
            std::stringstream ss(string_msg.data);
            std::string token;

            while (std::getline(ss, token, ',')) 
            {
                stringVector.push_back(token);
            }

            // Print the strings from the vector (optional)
        
            double double_joint1_value_ = std::stod(stringVector[0]);
            double double_joint2_value_ = std::stod(stringVector[1]);
            double double_joint3_value_ = std::stod(stringVector[2]);
            double double_joint4_value_ = std::stod(stringVector[3]);
            double double_gripper_value_ = std::stod(stringVector[4]);

            joint1_value_=double_joint1_value_;
            joint2_value_=double_joint2_value_;
            joint3_value_=double_joint3_value_;
            joint4_value_=double_joint4_value_;
            gripper_=double_gripper_value_;
        }

        void drone_body_string(std_msgs::msg::String string_msg) 
        { 
            RCLCPP_INFO(this->ros_node_->get_logger(), "drone body message: %s", string_msg.data.c_str());
          
            std::cout << string_msg.data << std::endl;
            std::vector<std::string> stringVector;
            std::stringstream ss(string_msg.data);
            std::string token;

            while (std::getline(ss, token, ',')) 
            {
                stringVector.push_back(token);
            }

            // Print the strings from the vector (optional)
        
            double double_x_value_ = std::stod(stringVector[0]);
            double double_y_value_ = std::stod(stringVector[1]);
            double double_z_value_ = std::stod(stringVector[2]);
            double double_r1_value_ = std::stod(stringVector[3]);
            double double_r2_value_ = std::stod(stringVector[4]);
            double double_r3_value_ = std::stod(stringVector[5]);

            x_value_=double_x_value_;
            y_value_=double_y_value_;
            z_value_=double_z_value_;
            r1_value_=double_r1_value_;
            r2_value_=double_r2_value_;
            r3_value_=double_r3_value_;
        }

        void add(std::shared_ptr<uam_msgs::srv::Uam::Request> request,
          std::shared_ptr<uam_msgs::srv::Uam::Response> response)
        {

            drone_body_string(request->uav_pose);
            robot_joint_string(request->manip_joint);

            ignition::math::Pose3d pose = this->model->GetLink("gripper_left_link")->WorldPose();
            ignition::math::Pose3d pose_base = this->model->GetLink("base_link")->WorldPose();

            geometry_msgs::msg::Pose eePose;
            eePose.position.x = pose.Pos().X();
            eePose.position.y = pose.Pos().Y();
            eePose.position.z = pose.Pos().Z();

            eePose.orientation.x = pose.Rot().X();
            eePose.orientation.y = pose.Rot().Y();
            eePose.orientation.z = pose.Rot().Z();
            eePose.orientation.w = pose.Rot().W();

            geometry_msgs::msg::Pose PoseDifference;
            PoseDifference.position.x = pose_base.Pos().X() - pose.Pos().X();
            PoseDifference.position.y = pose_base.Pos().Y() - pose.Pos().Y();
            PoseDifference.position.z = pose_base.Pos().Z() - pose.Pos().Z();

            PoseDifference.orientation.x = pose_base.Rot().X() - pose.Rot().X();
            PoseDifference.orientation.y = pose_base.Rot().Y() - pose.Rot().Y();
            PoseDifference.orientation.z = pose_base.Rot().Z() - pose.Rot().Z();
            PoseDifference.orientation.w = pose_base.Rot().W() - pose.Rot().W();

            response->pose_uam = eePose;
            response->manip_difference = PoseDifference;
        }



        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
        physics::ModelPtr model;
        double joint1_value_=0.;
        double joint2_value_=0.;
        double joint3_value_=0.;
        double joint4_value_=0.;
        double gripper_=0.;
        double x_value_=0.;
        double y_value_=0.;
        double z_value_=0.;
        double r1_value_=0.;
        double r2_value_=0.;
        double r3_value_=0.;
        std::vector<std::string> substrings;
        std::string substring;
        std::vector<double> doubles;
        
        std::string token;
        std::vector<std::string> tokens;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}