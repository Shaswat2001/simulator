#include <functional>
#include <memory>
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
#include "std_msgs/msg/float32.hpp"
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include "Eigen/Dense"
#include "aerialsys_msgs/msg/states.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace gazebo
{   
    typedef Eigen::Matrix<double, 2, 1> Vector2;
    typedef Eigen::Matrix<double, 3, 1> Vector3;
    typedef Eigen::Matrix3d Matrix3;

    class UAMcontrolPush : public ModelPlugin
    {
    public:
        /// A pointer to the GazeboROS node.
        gazebo_ros::Node::SharedPtr ros_node_;

        /// Subscriber to command velocities
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Publisher<aerialsys_msgs::msg::States>::SharedPtr uam_states_pub_;
        aerialsys_msgs::msg::States uam_state;

        /// Connection to event called at every world iteration.
        gazebo::event::ConnectionPtr update_connection_;

        rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clk_subscriber_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            this->model = _model;
            this->link = this->model->GetLink((std::string) "base_link");
            // Initialize ROS node
            this->ros_node_ = gazebo_ros::Node::Get(_sdf);
            // Get QoS profiles
            const gazebo_ros::QoS &qos = this->ros_node_->get_qos();
            RCLCPP_WARN(this->ros_node_->get_logger(), "*Loading PID controller");

            clk_subscriber_ = this->ros_node_->create_subscription<rosgraph_msgs::msg::Clock>("clock", 10, std::bind(&UAMcontrolPush::clk_update, this, std::placeholders::_1));
            uam_states_pub_ = this->ros_node_->create_publisher<aerialsys_msgs::msg::States>("/states",10);
                       
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UAMcontrolPush::OnUpdate, this));
            joint1_value_=0;
            joint2_value_=0;
            joint3_value_=0;
            joint4_value_=0;
            gripper_=0;

            x_value_=0;
            y_value_=0;
            z_value_=0.40;
            r1_value_=0;
            r2_value_=0;
            r3_value_=0.0;

            physics::InertialPtr uav_inertia = this->link->GetInertial();
            m = uav_inertia->Mass();
            J(0, 0) = uav_inertia->IXX();
            J(0, 1) = uav_inertia->IXY();
            J(0, 2) = uav_inertia->IXZ();
            J(1, 0) = uav_inertia->IXY();
            J(1, 1) = uav_inertia->IYY();
            J(1, 2) = uav_inertia->IYZ();
            J(2, 0) = uav_inertia->IXZ();
            J(2, 1) = uav_inertia->IYZ();
            J(2, 2) = uav_inertia->IZZ();

            std::cout << "\nUAV properties:"
                << "\n m: " << m
                << "\n J: " << J
                << std::endl;
            
            // Set gain matrices.
            kP(0, 1) = kp;
            kP(1, 0) = -kp;
            
            kD(0, 1) = kd;
            kD(1, 0) = -kd;

            kX << kx, 0.0, 0.0,
            0.0, kx, 0.0,
            0.0, 0.0, kx;
            
            kV << kv, 0.0, 0.0,
            0.0, kv, 0.0,
            0.0, 0.0, kv;

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
            this->model->SetWorldPose(ignition::math::Pose3d(x_value_,  y_value_, z_value_,  r1_value_, r2_value_,  r3_value_));
            
        }

    public:
        void OnUpdate()
        {
            uam_state.header.stamp = this->ros_node_->get_clock()->now();
            uam_state.header.frame_id = (std::string) "uav_states";
            update_states();
            update_ros_message();

            // rclcpp::Time t = this->ros_node_->get_clock()->now();
            // rclcpp::Duration t_elapsed = t - t0;
            // if (t_elapsed.seconds() < 2.0) 
            // {
            //     return;
            // }

            xd << 1.0, 1.0, 1.0;
            vd = (x - xd) / 2.0;

            ex = x - xd;
            ev = v - vd;

            // Calculate desired attitude.
            Rd = kP * ex + kD * ev;
            Rd(0) = saturate(Rd(0), -M_PI/3, M_PI/3); 
            Rd(1) = saturate(Rd(1), -M_PI/3, M_PI/3); 
            Wd << 0.0, 0.0, 0.0;   

            eR = R - Rd;
            eW = W - Wd;

            // Simple PID control.
            Vector3 e3(0.0, 0.0, 1.0);
            Vector3 M = - kR * eR - kW * eW;
            Vector3 F = - kX * ex - kV * ev + m * g * e3;

            Matrix3 rot_mat;
            euler_to_R(R, rot_mat);

            Vector3 Re3 = rot_mat * e3;

            double f = Re3.transpose() * F;

            force << 0.0, 0.0, f;

            ignition::math::Vector3d force_out, moment_out;
            Vector3 force_world = rot_mat * force;
            eigen_to_ignition(force_world, force_out);
            this->link->SetForce(force_out);
        
            // Torque is applied in body frame. 
            eigen_to_ignition(M, moment_out);
            this->link->SetTorque(moment_out);
        }

        void update_states(void)
        {
            // "pose" include both position and the rotation.
            ignition::math::Pose3d pose = this->model->WorldPose();
            ignition::math::Quaterniond rot = pose.Rot();
            
            ignition_to_eigen(pose.Pos(), x);
            ignition_to_eigen(pose.Rot().Euler(), R);

            // Get other states of the model.
            ignition_to_eigen(this->model->WorldLinearVel(), v);
            ignition_to_eigen(this->model->WorldLinearAccel(), a);
            ignition_to_eigen(this->model->WorldAngularVel(), W);
        }

        void update_ros_message(void)
        {
            // Update the ROS message.
            vector_to_msg(x, uam_state.x);
            vector_to_msg(v, uam_state.v);
            vector_to_msg(a, uam_state.a);
            vector_to_msg(R, uam_state.rotation);
            vector_to_msg(W, uam_state.w);

            // Publish all the states of the model to the ros publisher.
            this->uam_states_pub_->publish(uam_state);
        }


        void ignition_to_eigen(const ignition::math::Vector3d input, Vector3 &output)
        {
            output(0) = input[0];
            output(1) = input[1];
            output(2) = input[2];
        } 

        void eigen_to_ignition(const Vector3 input, ignition::math::Vector3d &output)
        {
            output[0] = input(0);
            output[1] = input(1);
            output[2] = input(2);
        } 


        void euler_to_R(const Vector3 rpy, Matrix3 &R)
        {
            const double r = rpy(0), p = rpy(1), y = rpy(2);
            const double cr = cos(r), sr = sin(r);
            const double cp = cos(p), sp = sin(p);
            const double cy = cos(y), sy = sin(y);

            R(0, 0) = cy * cp;
            R(1, 0) = cy * sp * sr + sy * cr;
            R(2, 0) = - cy * sp * cr + sy * sr;
            R(0, 1) = - sy * cp;
            R(1, 1) = - sy * sp * sr + cy * cr;
            R(2, 1) = sy * sp * cr + cy * sr;
            R(0, 2) = sp;
            R(1, 2) = - cp * sr;
            R(2, 2) = cp * cr;
        }

        void vector_to_msg(Vector3 vec, geometry_msgs::msg::Vector3 &msg_vec)
        {
            msg_vec.x = vec(0);
            msg_vec.y = vec(1);
            msg_vec.z = vec(2);
        } 

        double saturate(const double a, const double min_a, const double max_a)
        {
            if (a < min_a) return min_a;
            if (a > max_a) return max_a;
            return a;
        }



        void clk_update(const rosgraph_msgs::msg::Clock::SharedPtr time) const
        {
            
        }

        void pose_update(const geometry_msgs::msg::PoseStamped::SharedPtr pose) const
        { 
        
        }

        // Pointer to the update event connection
    private:


        event::ConnectionPtr updateConnection;
        physics::ModelPtr model;
        physics::LinkPtr link;
        // rclcpp::Time t0 = this->ros_node_->get_clock()->now();
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


        Vector3 x = Vector3::Zero();
        Vector3 v = Vector3::Zero();
        Vector3 a = Vector3::Zero();
        Vector3 R = Vector3::Zero();
        Vector3 W = Vector3::Zero();
        
        Vector3 xd = Vector3::Zero();
        Vector3 vd = Vector3::Zero();
        Vector3 Rd = Vector3::Zero();
        Vector3 Wd = Vector3::Zero();

        Vector3 ex = Vector3::Zero();
        Vector3 ev = Vector3::Zero();
        Vector3 ei = Vector3::Zero();
        Vector3 eR = Vector3::Zero();
        Vector3 eW = Vector3::Zero();

        Vector3 force = Vector3::Zero();

        double m = 1.0;
        double g = 9.8;
        Matrix3 J = Matrix3::Identity();

        // PID gains
        const double kx = 100;
        const double kv = 100;
        const double kR = 5.0;
        const double kW = 2.5;
        const double kp = 0.95;
        const double kd = 1.8;

        Matrix3 kP = Matrix3::Zero();
        Matrix3 kD = Matrix3::Zero();
        Matrix3 kX = Matrix3::Zero();
        Matrix3 kV = Matrix3::Zero();

        std::vector<std::string> substrings;
        std::string substring;
        std::vector<double> doubles;
        
        std::string token;
        std::vector<std::string> tokens;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(UAMcontrolPush)
}