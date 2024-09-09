#ifndef _SDF_JOINT_CONTROLLER_HPP_
#define _SDF_JOINT_CONTROLLER_HPP_

#include <memory>
#include <thread>
#include <vector>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math.hh>
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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <sdf/sdf.hh>
#include <ignition/math6/ignition/math/Pose3.hh>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace gazebo
{
    class SDFJointController : public ModelPlugin
    {
    public:
        SDFJointController() {}

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
    

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twst_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr manip_ee_pub_;
        
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr joint_subscription_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr body_subscription_;

        physics::JointPtr arm1_joint;
        physics::JointPtr arm2_joint;
        physics::JointPtr arm3_joint;
        physics::JointPtr arm4_joint;

        virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {

            this->model = _model;
            // Initialize ROS node
            this->ros_node_ = gazebo_ros::Node::Get(_sdf);

            this->arm1_joint = this->model->GetJoint("arm_1_joint");
            this->arm2_joint = this->model->GetJoint("arm_2_joint");
            this->arm3_joint = this->model->GetJoint("arm_3_joint");
            this->arm4_joint = this->model->GetJoint("arm_4_joint");


            if (_model->GetJointCount() <= 0)
            {
                std::cerr << "Invalid joint count, ROS_SDF plugin not loaded\n";
                return;
            }
            else
                std::cout << "\nThe ROS_SDF plugin is attached to model[" << _model->GetName() << "]\n";

            m_joints = this->model->GetJoints();
            common::PID pid(0.4, 0.01, 0.01); //placeholder values, need to do an input parameter for this

            // for (const auto& joint: m_joints)
            this->model->GetJointController()->SetPositionPID(this->arm1_joint->GetScopedName(), pid);
            this->model->GetJointController()->SetPositionPID(this->arm2_joint->GetScopedName(), pid);
            this->model->GetJointController()->SetPositionPID(this->arm3_joint->GetScopedName(), pid);
            this->model->GetJointController()->SetPositionPID(this->arm4_joint->GetScopedName(), pid);

            this->model->GetJoint("arm_1_joint")->SetPosition(0,0.01);
            this->model->GetJoint("arm_2_joint")->SetPosition(0,0.01);
            this->model->GetJoint("arm_3_joint")->SetPosition(0,0.01);
            this->model->GetJoint("arm_4_joint")->SetPosition(0,0.01);

            int argc = 0;
            // Subscribe to /model_name/position_cmd topic (you publish to this to set positions)
            manip_ee_pub_ = this->ros_node_->create_publisher<geometry_msgs::msg::Pose>("/manipulator_ee",10);
            joint_subscription_ = this->ros_node_->create_subscription<std_msgs::msg::String>("/robot_joints_string", 10,std::bind(&SDFJointController::robot_joint_stringCallback, this, std::placeholders::_1));
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&SDFJointController::OnUpdate, this));
        }

        void OnUpdate()
        {
            // this->model->GetJoint("prop_1_joint")->SetVelocity(0, 100);
            // this->model->GetJoint("prop_2_joint")->SetVelocity(0, -100);
            // this->model->GetJoint("Prop_3_joint")->SetVelocity(0, 100);
            // this->model->GetJoint("prop_4_joint")->SetVelocity(0, -100);
            // this->model->GetJoint("prop_5_joint")->SetVelocity(0, 100);
            // this->model->GetJoint("prop_6_joint")->SetVelocity(0, -100);
            this->model->GetJoint("gripper_left_joint")->SetPosition(0,0.1);
            this->model->GetJoint("gripper_right_joint")->SetPosition(0,-0.1);
            // this->model->SetWorldPose(ignition::math::Pose3d( 0,  0, 0.4,  0, 0,  0));

            // std::cout<<"The joint1 values : "<<this->arm1_joint->Position(0)<<std::endl;
            // std::cout<<"The joint2 values : "<<this->arm2_joint->Position(0)<<std::endl;
            // std::cout<<"The joint3 values : "<<this->arm3_joint->Position(0)<<std::endl;
            // std::cout<<"The joint4 values : "<<this->arm4_joint->Position(0)<<std::endl;
            ignition::math::Pose3d pose = this->model->GetLink("gripper_left_link")->WorldPose();
            geometry_msgs::msg::Pose eePose;
            eePose.position.x = pose.Pos().X();
            eePose.position.y = pose.Pos().Y();
            eePose.position.z = pose.Pos().Z();

            eePose.orientation.x = pose.Rot().X();
            eePose.orientation.y = pose.Rot().Y();
            eePose.orientation.z = pose.Rot().Z();
            eePose.orientation.w = pose.Rot().W();

            this->manip_ee_pub_->publish(eePose);

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
            this->model->GetJointController()->SetPositionTarget(this->arm1_joint->GetScopedName(), std::stod(stringVector[0]));
            this->model->GetJointController()->SetPositionTarget(this->arm2_joint->GetScopedName(), std::stod(stringVector[1]));
            this->model->GetJointController()->SetPositionTarget(this->arm3_joint->GetScopedName(), std::stod(stringVector[2]));
            this->model->GetJointController()->SetPositionTarget(this->arm4_joint->GetScopedName(), std::stod(stringVector[3]));
        }

    private:
        event::ConnectionPtr updateConnection;
        physics::ModelPtr model;

        std::vector<std::string> substrings;
        std::string substring;
        std::vector<double> doubles;
        
        std::string token;
        std::vector<std::string> tokens;
        std::vector<physics::JointPtr> m_joints;
    };

    GZ_REGISTER_MODEL_PLUGIN(SDFJointController)
}

#endif // _SDF_JOINT_CONTROLLER_HPP_