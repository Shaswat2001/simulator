#include <functional>
#include <memory>
#include <vector>
#include <numeric>
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
#include <std_msgs/msg/multi_array_dimension.h>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include "Eigen/Dense"
#include "Eigen/Core"
#include <unsupported/Eigen/MatrixFunctions>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen_algebra.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include "uam_msgs/msg/dynamics.hpp"
#include "libraries/C_matrix.cpp"

using namespace std::chrono_literals;
using namespace boost::numeric::odeint;
typedef boost::numeric::ublas::matrix< double > matrix_type;
using std::placeholders::_1;

namespace gazebo
{   

    class UAMcontrolPush : public ModelPlugin
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
        rclcpp::Subscription<uam_msgs::msg::Dynamics>::SharedPtr desired_state_subscriber_;

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
            RCLCPP_WARN(this->ros_node_->get_logger(), "*Loading UAM Dynamics");

            desired_state_subscriber_ = this->ros_node_->create_subscription<uam_msgs::msg::Dynamics>("/desired_uam_state", 10,std::bind(&UAMcontrolPush::uam_desired_stateCallback, this, std::placeholders::_1));
            clk_subscriber_ = this->ros_node_->create_subscription<rosgraph_msgs::msg::Clock>("clock", 10, std::bind(&UAMcontrolPush::clk_update, this, std::placeholders::_1));
                       
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UAMcontrolPush::OnUpdate, this));
            
            lmd = 2*Eigen::Matrix<double,10,10>::Identity();
            M_hat = Eigen::Matrix<double,10,10>::Identity();
            C_hat = Eigen::Matrix<double,10,10>::Identity();
            G_hat<<0,0,-10,0,0,0,0,0,0,0;
            K = 8*Eigen::Matrix<double,10,10>::Identity();
            A = 10*Eigen::Matrix<double,10,10>::Identity();
            K_A = 10*Eigen::Matrix<double,10,10>::Identity();

            m_b = 4.68848;
            I.diagonal() = {0.430475,0.430587,0.592651};
            m_l = {0.0466133,0.0881361,0.0841268,0.093795};
            length = {0.077,0.13,0.124,0.126};
            I_l.reserve(4);
            I_l[0].diagonal() = {1.2004e-05,7.48803e-06,1.11387e-05};
            I_l[1].diagonal() = {2.0389e-05,2.80387e-05,2.41553e-05};
            I_l[2].diagonal() = {1.54135e-05,2.30716e-05,2.3716e-05};
            I_l[3].diagonal() = {2.2174e-05,2.6687e-05,3.1188e-05};

            delta_hat<<0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01;
            pos<<1,1,0.5,0.01,0.01,0.01,0.01,0.01,0.01,0.01;
            vel<<0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01;

            // ode_func(x_init,dxdt,0);
            // std::cout<<dxdt;
            // std::cout <<std::endl << "final state vector: " << x_init <<std::endl;

            // Eigen::Matrix<double,10,1> qdotdot = solve(0,tau,q,qdot);

            // std::cout << "The matrix m is of size "<< qdotdot(0,0) << qdotdot(1,0) << std::endl;

        }

        void OnUpdate()
        {

            this->model->GetJoint("prop_1_joint")->SetVelocity(0, 100);
            this->model->GetJoint("prop_2_joint")->SetVelocity(0, -100);
            this->model->GetJoint("Prop_3_joint")->SetVelocity(0, 100);
            this->model->GetJoint("prop_4_joint")->SetVelocity(0, -100);
            this->model->GetJoint("prop_5_joint")->SetVelocity(0, 100);
            this->model->GetJoint("prop_6_joint")->SetVelocity(0, -100);

            this->model->GetJoint("arm_1_joint")->SetPosition(0,pos(6,0));
            this->model->GetJoint("arm_2_joint")->SetPosition(0,pos(7,0));
            this->model->GetJoint("arm_3_joint")->SetPosition(0,pos(8,0));
            this->model->GetJoint("arm_4_joint")->SetPosition(0,pos(9,0));
            this->model->GetJoint("gripper_left_joint")->SetPosition(0,0.0);
            this->model->GetJoint("gripper_right_joint")->SetPosition(0,0.0);
            // this->model->SetWorldPose(ignition::math::Pose3d(pos(0,0),  pos(1,0), pos(2,0),  pos(3,0), pos(4,0),  pos(5,0)));
        }

        Eigen::Matrix<double,10,1> solve(double time, const Eigen::Matrix<double,10,1> tau, const Eigen::Matrix<double,10,1> q, const Eigen::Matrix<double,10,1> qdot)
        {
            double x = q(0,0), y  = q(1,0), z = q(2,0);
            double phi = q(3,0), theta = q(4,0), psi = q(5,0);
            double n1 = q(6,0), n2 = q(7,0), n3 = q(8,0), n4 = q(9,0);

            double x_dot = qdot(0,0), y_dot = qdot(1,0), z_dot = qdot(2,0);
            double phi_dot = qdot(3,0), theta_dot = qdot(4,0), psi_dot = qdot(5,0);
            double n1_dot = qdot(6,0), n2_dot = qdot(7,0), n3_dot = qdot(8,0), n4_dot = qdot(9,0);

            double sin_psi = sin(psi), cos_psi = cos(psi);
            double sin_theta = sin(theta), cos_theta = cos(theta);
            double sin_phi = sin(phi), cos_phi = cos(phi);

            double sin_n1 = sin(n1), cos_n1 = cos(n1);
            double sin_n2 = sin(n2), cos_n2 = cos(n2);
            double sin_n3 = sin(n3), cos_n3 = cos(n3);
            double sin_n4 = sin(n4), cos_n4 = cos(n4);

            Eigen::Matrix<double,3,1> Mat31Zero = Eigen::Matrix<double,3,1>::Zero();
            Eigen::Matrix<double,3,3> Mat3Identity = Eigen::Matrix3d::Identity();
            Eigen::Matrix<double,3,3> Mat33Zero = Eigen::Matrix3d::Zero();

            Eigen::Matrix<double,3,1> p_b {x,y,z};
            Eigen::Matrix3d Tb;
            Tb<<0,-sin_psi,cos_psi*cos_theta,
                0,cos_psi, cos_theta*sin_psi,
                1, 0, -sin_theta;

            Eigen::Matrix3d Rb;
            Rb<<cos_psi*cos_theta, cos_psi*sin_phi*sin_theta - cos_phi*sin_psi, sin_phi*sin_psi+cos_phi*cos_psi*sin_theta,
                sin_psi*cos_theta, sin_psi*sin_phi*sin_theta + cos_phi*cos_psi, -sin_phi*cos_psi+cos_phi*sin_psi*sin_theta,
                -sin_theta, cos_theta*sin_phi, cos_phi*cos_theta;

            Eigen::Matrix3d Q = Rb.transpose() * Tb;

            Eigen::Matrix<double,3,1> h1 {0,0,1};
            Eigen::Matrix<double,3,1> h1_b {0,0,1};
            Eigen::Matrix<double,3,1> P10 {0,0,0};
            Eigen::Matrix<double,3,1> p1c {0,0,length[0]/2};

            Eigen::Matrix3d R01 = (hat(h1) * n1).exp();
            Eigen::Matrix<double,3,1> p1c_b = P10 +  R01 * p1c;
            Eigen::Matrix<double,3,1> h2 {0,1,0};
            Eigen::Matrix<double,3,1> h2_b = R01 * h2;
            Eigen::Matrix<double,3,1> p12 {0,0,length[0]};
            Eigen::Matrix<double,3,1> p2c = {0,0,length[1]/2};
            Eigen::Matrix3d R12 = (hat(h2) * n2).exp();
            Eigen::Matrix3d R02 = R01 * R12;
            Eigen::Matrix<double,3,1> p2c_b = P10 +  R01 * p12 + R02 * p2c;

            Eigen::Matrix<double,3,1> h3 {0,1,0};
            Eigen::Matrix<double,3,1> h3_b = R02 * h3;
            Eigen::Matrix<double,3,1> p23 = {0,0,length[1]};
            Eigen::Matrix<double,3,1> p3c = {0,0,length[2]/2};

            Eigen::Matrix3d R23 = (hat(h3) * n3).exp();
            Eigen::Matrix3d R03 = R02 * R23;
            Eigen::Matrix<double,3,1> p3c_b = P10 +  R01 * p12 + R02 * p23 + R03 * p3c;

            Eigen::Matrix<double,3,1> h4 {0,1,0};
            Eigen::Matrix<double,3,1> h4_b = R03 * h4;
            Eigen::Matrix<double,3,1> p34 {0,0,length[2]};
            Eigen::Matrix<double,3,1> p4T {0,0,length[3]};
            Eigen::Matrix<double,3,1> p4c {0,0,length[3]/2};

            Eigen::Matrix3d R34 = (hat(h4) * n4).exp();
            Eigen::Matrix3d R04 = R03 * R34;
            Eigen::Matrix<double,3,1> p4c_b = P10 +  R01 * p12 + R02 * p23 + R03 * p34 + R04 * p4c;

            Eigen::Matrix<double,3,1> p1_b = P10 + R01 * p12;
            Eigen::Matrix<double,3,1> p2_b = P10 + R01 * p12 + R02 * p23;
            Eigen::Matrix<double,3,1> p3_b = P10 + R01 * p12 + R02 * p23 + R03 * p34;
            Eigen::Matrix<double,3,1> p4_b = P10 + R01 * p12 + R02 * p23 + R03 * p34 + R04 * p4T;

            Eigen::Matrix<double,3,4> Jw1c, Jv1c, Jw2c, Jv2c, Jw3c, Jv3c, Jw4c, Jv4c, JwT, JvT;
            
            Jw1c<<h1_b,Mat31Zero,Mat31Zero,Mat31Zero;
            Jv1c<<hat(h1_b) * (p1c_b - P10),Mat31Zero,Mat31Zero,Mat31Zero;

            Jw2c<<h1_b,h2_b,Mat31Zero,Mat31Zero;
            Jv2c<<hat(h1_b) * (p2c_b - P10),hat(h2_b) * (p2c_b - p1_b),Mat31Zero,Mat31Zero;

            Jw3c<<h1_b,h2_b,h3_b,Mat31Zero;
            Jv3c<<hat(h1_b) * (p3c_b - P10),hat(h2_b) * (p3c_b - p1_b),hat(h3_b) * (p3c_b - p2_b),Mat31Zero;

            Jw4c<<h1_b,h2_b,h3_b,h4_b;
            Jv4c<<hat(h1_b) * (p4c_b - P10),hat(h2_b) * (p4c_b - p1_b),hat(h3_b) * (p4c_b - p2_b),hat(h4_b) * (p4c_b - p3_b);

            JwT<<h1_b,h2_b,h3_b,h4_b;
            JvT<<hat(h1_b) * (p4_b- P10),hat(h2_b) * (p4_b- p1_b),hat(h3_b) * (p4_b- p2_b),hat(h4_b) * (p4_b- p3_b);

            Eigen::Matrix3d M11 = (std::accumulate(m_l.begin(), m_l.end(), 0.0) +  m_b)*Mat3Identity;
            Eigen::Matrix3d M22 = Q.transpose() * I * Q +
                                m_l[0]*Tb.transpose() * hat(Rb * p1c_b) * hat(Rb * p1c_b) * Tb + Q.transpose() * R01 * I_l[0] * R01.transpose() * Q + 
                                m_l[1]*Tb.transpose() * hat(Rb * p2c_b) * hat(Rb * p2c_b) * Tb + Q.transpose() * R02 * I_l[1] * R02.transpose() * Q + 
                                m_l[2]*Tb.transpose() * hat(Rb * p3c_b) * hat(Rb * p3c_b) * Tb + Q.transpose() * R03 * I_l[2] * R03.transpose() * Q + 
                                m_l[3]*Tb.transpose() * hat(Rb * p4c_b) * hat(Rb * p4c_b) * Tb + Q.transpose() * R04 * I_l[3] * R04.transpose() * Q;

            Eigen::Matrix<double,4,4> M33 = m_l[0]*Jv1c.transpose() * Jv1c + Jw1c.transpose() * R01 * I_l[0] * R01.transpose() * Jw1c +
                    m_l[1]*Jv2c.transpose() * Jv2c + Jw2c.transpose() * R02 * I_l[1] * R02.transpose() * Jw2c + 
                    m_l[2]*Jv3c.transpose() * Jv3c + Jw3c.transpose() * R03 * I_l[2] * R03.transpose() * Jw3c + 
                    m_l[3]*Jv4c.transpose() * Jv4c + Jw4c.transpose() * R04 * I_l[3] * R04.transpose() * Jw4c;
            
            Eigen::Matrix3d M12 = -m_l[0]*hat(Rb * p1c_b) - 
                    m_l[1]*hat(Rb * p2c_b) - 
                    m_l[2]*hat(Rb * p3c_b) - 
                    m_l[3]*hat(Rb * p4c_b);
            
            Eigen::Matrix3d M21 = M12.transpose();
            Eigen::Matrix<double,3,4> M13 = m_l[0]*Rb * Jv1c + m_l[1]*Rb * Jv2c + m_l[2]*Rb * Jv3c + m_l[3]*Rb * Jv4c;
            Eigen::Matrix<double,4,3> M31 = M13.transpose();

            Eigen::Matrix<double,3,4> M23 = Q.transpose() * R01 * I_l[0] * R01.transpose() * Jw1c - m_l[0]*Tb.transpose() * hat(Rb * p1c_b) * Rb * Jv1c +
                    Q.transpose() * R02 * I_l[1] * R02.transpose() * Jw2c - m_l[1]*Tb.transpose() * hat(Rb * p2c_b) * Rb * Jv2c + 
                    Q.transpose() * R03 * I_l[2] * R03.transpose() * Jw3c - m_l[2]*Tb.transpose() * hat(Rb * p3c_b) * Rb * Jv3c + 
                    Q.transpose() * R04 * I_l[3] * R04.transpose() * Jw4c - m_l[3]*Tb.transpose() * hat(Rb * p4c_b) * Rb * Jv4c;
            
            Eigen::Matrix<double,4,3> M32 = M23.transpose();

            Eigen::Matrix<double,3,10> M1;
            M1<<M11,M12,M13;
            Eigen::Matrix<double,3,10> M2;
            M2<<M21,M22,M23;
            Eigen::Matrix<double,4,10> M3;
            M3<<M31,M32,M33;
            Eigen::Matrix<double,10,10> M;
            M<<M1,M2,M3;

            // std::cout<<"M21 : "<<M21<<std::endl;
            // std::cout<<"M22 : "<<M22<<std::endl;
            // std::cout<<"M23 : "<<M23<<std::endl;

            // std::cout<<"M31 : "<<M31<<std::endl;
            // std::cout<<"M32 : "<<M32<<std::endl;
            // std::cout<<"M33 : "<<M33<<std::endl;

            CMat = C_matrix(m_b,m_l[0],m_l[1],m_l[2],m_l[3],I.diagonal()[0],I.diagonal()[1],I.diagonal()[2],I_l[0].diagonal()[0],I_l[0].diagonal()[1],I_l[0].diagonal()[2],I_l[1].diagonal()[0],I_l[1].diagonal()[1],I_l[1].diagonal()[2],I_l[2].diagonal()[0],I_l[2].diagonal()[1],I_l[2].diagonal()[2],I_l[3].diagonal()[0],I_l[3].diagonal()[1],I_l[3].diagonal()[2],length[0],length[1],length[2],length[3],x,y,z,phi,theta,psi,n1,n2,n3,n4,x_dot,y_dot,z_dot,phi_dot,theta_dot,psi_dot,n1_dot,n2_dot,n3_dot,n4_dot);
            
            // std::cout<<"C : "<<CMat<<std::endl;

            double G1 = 0;
            double G2 = 0;
            double G3 = (49*m_l[0])/5 + (49*m_l[1])/5 + (49*m_l[2])/5 + (49*m_l[3])/5 + (49*m_b)/5;
            double G4 = (49*m_l[2]*(cos_phi*cos_theta*((length[2]*(cos_n2*sin_n1*sin_n3 + cos_n3*sin_n1*sin_n2))/2 + length[1]*sin_n1*sin_n2) - cos_theta*sin_phi*(length[0] + (length[2]*(cos_n2*cos_n3 - sin_n2*sin_n3))/2 + length[1]*cos_n2)))/5 + (49*m_l[3]*(cos_phi*cos_theta*(length[2]*(cos_n2*sin_n1*sin_n3 + cos_n3*sin_n1*sin_n2) + (length[3]*(cos_n4*(cos_n2*sin_n1*sin_n3 + cos_n3*sin_n1*sin_n2) - sin_n4*(sin_n1*sin_n2*sin_n3 - cos_n2*cos_n3*sin_n1)))/2 + length[1]*sin_n1*sin_n2) - cos_theta*sin_phi*(length[0] + length[2]*(cos_n2*cos_n3 - sin_n2*sin_n3) + length[1]*cos_n2 + (length[3]*(cos_n4*(cos_n2*cos_n3 - sin_n2*sin_n3) - sin_n4*(cos_n2*sin_n3 + cos_n3*sin_n2)))/2)))/5 - (49*m_l[1]*(cos_theta*sin_phi*(length[0] + (length[1]*cos_n2)/2) - (length[1]*cos_phi*cos_theta*sin_n1*sin_n2)/2))/5 - (49*length[0]*m_l[0]*cos_theta*sin_phi)/10;
            double G5 = -(49*m_l[2]*(cos_theta*((length[2]*(cos_n1*cos_n2*sin_n3 + cos_n1*cos_n3*sin_n2))/2 + length[1]*cos_n1*sin_n2) + sin_phi*sin_theta*((length[2]*(cos_n2*sin_n1*sin_n3 + cos_n3*sin_n1*sin_n2))/2 + length[1]*sin_n1*sin_n2) + cos_phi*sin_theta*(length[0] + (length[2]*(cos_n2*cos_n3 - sin_n2*sin_n3))/2 + length[1]*cos_n2)))/5 - (49*m_l[1]*(cos_phi*sin_theta*(length[0] + (length[1]*cos_n2)/2) + (length[1]*cos_n1*cos_theta*sin_n2)/2 + (length[1]*sin_n1*sin_n2*sin_phi*sin_theta)/2))/5 - (49*m_l[3]*(cos_theta*((length[3]*(cos_n4*(cos_n1*cos_n2*sin_n3 + cos_n1*cos_n3*sin_n2) + sin_n4*(cos_n1*cos_n2*cos_n3 - cos_n1*sin_n2*sin_n3)))/2 + length[2]*(cos_n1*cos_n2*sin_n3 + cos_n1*cos_n3*sin_n2) + length[1]*cos_n1*sin_n2) + sin_phi*sin_theta*(length[2]*(cos_n2*sin_n1*sin_n3 + cos_n3*sin_n1*sin_n2) + (length[3]*(cos_n4*(cos_n2*sin_n1*sin_n3 + cos_n3*sin_n1*sin_n2) - sin_n4*(sin_n1*sin_n2*sin_n3 - cos_n2*cos_n3*sin_n1)))/2 + length[1]*sin_n1*sin_n2) + cos_phi*sin_theta*(length[0] + length[2]*(cos_n2*cos_n3 - sin_n2*sin_n3) + length[1]*cos_n2 + (length[3]*(cos_n4*(cos_n2*cos_n3 - sin_n2*sin_n3) - sin_n4*(cos_n2*sin_n3 + cos_n3*sin_n2)))/2)))/5 - (49*length[0]*m_l[0]*cos_phi*sin_theta)/10;
            double G6 = 0;
            double G7 = (49*m_l[2]*(sin_theta*((length[2]*(cos_n2*sin_n1*sin_n3 + cos_n3*sin_n1*sin_n2))/2 + length[1]*sin_n1*sin_n2) + cos_theta*sin_phi*((length[2]*(cos_n1*cos_n2*sin_n3 + cos_n1*cos_n3*sin_n2))/2 + length[1]*cos_n1*sin_n2)))/5 + (49*m_l[3]*(sin_theta*(length[2]*(cos_n2*sin_n1*sin_n3 + cos_n3*sin_n1*sin_n2) + (length[3]*(cos_n4*(cos_n2*sin_n1*sin_n3 + cos_n3*sin_n1*sin_n2) - sin_n4*(sin_n1*sin_n2*sin_n3 - cos_n2*cos_n3*sin_n1)))/2 + length[1]*sin_n1*sin_n2) + cos_theta*sin_phi*((length[3]*(cos_n4*(cos_n1*cos_n2*sin_n3 + cos_n1*cos_n3*sin_n2) + sin_n4*(cos_n1*cos_n2*cos_n3 - cos_n1*sin_n2*sin_n3)))/2 + length[2]*(cos_n1*cos_n2*sin_n3 + cos_n1*cos_n3*sin_n2) + length[1]*cos_n1*sin_n2)))/5 + (49*m_l[1]*((length[1]*sin_n1*sin_n2*sin_theta)/2 + (length[1]*cos_n1*cos_theta*sin_n2*sin_phi)/2))/5;
            double G8 = - (49*m_l[1]*((length[1]*cos_n1*cos_n2*sin_theta)/2 + (length[1]*cos_phi*cos_theta*sin_n2)/2 - (length[1]*cos_n2*cos_theta*sin_n1*sin_phi)/2))/5 - (49*m_l[3]*(sin_theta*((length[3]*(cos_n4*(cos_n1*cos_n2*cos_n3 - cos_n1*sin_n2*sin_n3) - sin_n4*(cos_n1*cos_n2*sin_n3 + cos_n1*cos_n3*sin_n2)))/2 + length[2]*(cos_n1*cos_n2*cos_n3 - cos_n1*sin_n2*sin_n3) + length[1]*cos_n1*cos_n2) + cos_phi*cos_theta*(length[2]*(cos_n2*sin_n3 + cos_n3*sin_n2) + (length[3]*(cos_n4*(cos_n2*sin_n3 + cos_n3*sin_n2) + sin_n4*(cos_n2*cos_n3 - sin_n2*sin_n3)))/2 + length[1]*sin_n2) + cos_theta*sin_phi*(length[2]*(sin_n1*sin_n2*sin_n3 - cos_n2*cos_n3*sin_n1) + (length[3]*(cos_n4*(sin_n1*sin_n2*sin_n3 - cos_n2*cos_n3*sin_n1) + sin_n4*(cos_n2*sin_n1*sin_n3 + cos_n3*sin_n1*sin_n2)))/2 - length[1]*cos_n2*sin_n1)))/5 - (49*m_l[2]*(sin_theta*((length[2]*(cos_n1*cos_n2*cos_n3 - cos_n1*sin_n2*sin_n3))/2 + length[1]*cos_n1*cos_n2) + cos_theta*sin_phi*((length[2]*(sin_n1*sin_n2*sin_n3 - cos_n2*cos_n3*sin_n1))/2 - length[1]*cos_n2*sin_n1) + cos_phi*cos_theta*((length[2]*(cos_n2*sin_n3 + cos_n3*sin_n2))/2 + length[1]*sin_n2)))/5;
            double G9 = - (49*m_l[3]*(sin_theta*((length[3]*(cos_n4*(cos_n1*cos_n2*cos_n3 - cos_n1*sin_n2*sin_n3) - sin_n4*(cos_n1*cos_n2*sin_n3 + cos_n1*cos_n3*sin_n2)))/2 + length[2]*(cos_n1*cos_n2*cos_n3 - cos_n1*sin_n2*sin_n3)) + cos_phi*cos_theta*(length[2]*(cos_n2*sin_n3 + cos_n3*sin_n2) + (length[3]*(cos_n4*(cos_n2*sin_n3 + cos_n3*sin_n2) + sin_n4*(cos_n2*cos_n3 - sin_n2*sin_n3)))/2) + cos_theta*sin_phi*(length[2]*(sin_n1*sin_n2*sin_n3 - cos_n2*cos_n3*sin_n1) + (length[3]*(cos_n4*(sin_n1*sin_n2*sin_n3 - cos_n2*cos_n3*sin_n1) + sin_n4*(cos_n2*sin_n1*sin_n3 + cos_n3*sin_n1*sin_n2)))/2)))/5 - (49*m_l[2]*((length[2]*sin_theta*(cos_n1*cos_n2*cos_n3 - cos_n1*sin_n2*sin_n3))/2 + (length[2]*cos_theta*sin_phi*(sin_n1*sin_n2*sin_n3 - cos_n2*cos_n3*sin_n1))/2 + (length[2]*cos_phi*cos_theta*(cos_n2*sin_n3 + cos_n3*sin_n2))/2))/5;
            double G10 = -(49*m_l[3]*((length[3]*sin_theta*(cos_n4*(cos_n1*cos_n2*cos_n3 - cos_n1*sin_n2*sin_n3) - sin_n4*(cos_n1*cos_n2*sin_n3 + cos_n1*cos_n3*sin_n2)))/2 + (length[3]*cos_phi*cos_theta*(cos_n4*(cos_n2*sin_n3 + cos_n3*sin_n2) + sin_n4*(cos_n2*cos_n3 - sin_n2*sin_n3)))/2 + (length[3]*cos_theta*sin_phi*(cos_n4*(sin_n1*sin_n2*sin_n3 - cos_n2*cos_n3*sin_n1) + sin_n4*(cos_n2*sin_n1*sin_n3 + cos_n3*sin_n1*sin_n2)))/2))/5;

            Eigen::Matrix<double,10,1> G;
            G<<G1,G2,G3,G4,G5,G6,G7,G8,G9,G10;

            // std::cout<<"G : "<<G<<std::endl;

            Eigen::Matrix<double,3,3> term1 = -hat(Rb * p4_b) * Tb;
            Eigen::Matrix<double,3,4> term2 = Rb * JvT;
            Eigen::Matrix<double,3,4> term3 = Rb * JwT;

            Eigen::Matrix<double,3,10> J_a1, J_a2, J_a3, J_a4;
            J_a1<< Mat3Identity,Mat33Zero,Eigen::Matrix<double,3,4>::Zero();
            J_a2<< Mat33Zero,Tb,Eigen::Matrix<double,3,4>::Zero();
            J_a3<< Mat3Identity,term1,term2;
            J_a4<< Mat33Zero,Tb,term3;

            Eigen::Matrix<double,12,10> J_a;
            J_a<< J_a1,J_a2,J_a3,J_a4;

            // std::cout<<"J_a : "<<J_a<<std::endl;

            double z_ext = -10;
            Eigen::Matrix<double,12,1> f_ext;
            f_ext<<0,0,sin(5*time),0,0,0,0,0,z_ext,0,0,0;
            Eigen::Matrix<double,10,1> tau_ext = J_a.transpose() * f_ext;

            // std::cout<<"tau_ext : "<<tau_ext<<std::endl;

            // std::cout<<"inverse : "<<M.inverse(); 

            Eigen::Matrix<double,10,1> qdotdot = M.inverse() * (tau + tau_ext - CMat * qdot - G);

            // std::cout<<"qdotdot : "<<qdotdot<<std::endl;

            return qdotdot;

        }

        Eigen::Matrix<double,10,1> getPoseError(const Eigen::Matrix<double,10,1> position)
        {
        
            return position - pos_des;
        }
        
        Eigen::Matrix<double,10,1> getVelError(const Eigen::Matrix<double,10,1> velocity)
        {
            
            return velocity - vel_des;
        }


        void ode_func(const Eigen::Matrix<double,30,1> & x, Eigen::Matrix<double,30,1> & dxdt, double t)
        
        {   
            delta_hat = x(Eigen::seq(0,9),0);
            vel = x(Eigen::seq(10,19),0);
            pos = x(Eigen::seq(20,29),0);

            pos_error = getPoseError(pos);
            vel_error = getVelError(vel);
            s = vel_error + lmd * pos_error;
            qrDot = vel_des - lmd * pos_error;
            qrDotDot = accl_des - lmd * vel_error;
            tau = M_hat * qrDotDot + C_hat * qrDot + G_hat + delta_hat - A * s  - K * (s.array().tanh()).matrix();
            delta_hatDot = - K_A * s;
            acceleration = solve(t, tau, pos,vel);
            dxdt(Eigen::seq(0,9),0) = delta_hatDot;
            dxdt(Eigen::seq(10,19),0) = acceleration;
            dxdt(Eigen::seq(20,29),0) = vel;
        }

        void write_states(const Eigen::Matrix<double,30,1> x, const double t)
        {
            std::cout<<" time : "<<t<<std::endl<<"position : "<<std::endl<<pos<<std::endl;
        }

        void uam_desired_stateCallback(uam_msgs::msg::Dynamics::SharedPtr uam_des_msg) 
        { 
            RCLCPP_INFO(this->ros_node_->get_logger(), "UAM desired state recieved:");

            std::vector<float> data = uam_des_msg->q_des.data;
            pos_des = convert_ros_msg_to_eigen(data);
            data = uam_des_msg->qdot_des.data;
            vel_des = convert_ros_msg_to_eigen(data);
            data = uam_des_msg->qdotdot_des.data;
            accl_des = convert_ros_msg_to_eigen(data);


            Eigen::Matrix<double,30,1> x_init;
            x_init<<delta_hat,vel,pos;

            integrate_const(
            controlled_runge_kutta<runge_kutta_cash_karp54< Eigen::Matrix<double,30,1>>>(1E-3,1E-3),
            [this](const Eigen::Matrix<double,30,1> & x, Eigen::Matrix<double,30,1> & dxdt, double t) {
                return ode_func(x, dxdt, t);
            },x_init, 0.0, 10.0, 0.01,
             [this](Eigen::Matrix<double,30,1> & x, double t) {
                return write_states(x, t);
            });
        }

        Eigen::Matrix<double,10,1> convert_ros_msg_to_eigen(std::vector<float> data)
        {
            Eigen::Map<Eigen::Matrix<float,10,1> > mat(data.data(),10,1);
            Eigen::Matrix<double,10,1> matD = mat.cast<double>();

            return matD;
        }
        Eigen::Matrix3d hat(Eigen::MatrixXd vector)
        {
            Eigen::Matrix3d result;
            result<< 0,-vector(2,0),vector(1,0),
                    vector(2,0),0,-vector(0,0),
                    -vector(1,0),vector(0,0),0;

            return result;
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
        double m_b;
        Eigen::DiagonalMatrix<double, 3> I;
        std::vector<double> m_l;
        std::vector<double> length;
        std::vector<Eigen::DiagonalMatrix<double, 3>> I_l;

        Eigen::Matrix<double,10,10> lmd;
        Eigen::Matrix<double,10,10> M_hat;
        Eigen::Matrix<double,10,10> C_hat;
        Eigen::Matrix<double,10,1> G_hat;
        Eigen::Matrix<double,10,10> K;
        Eigen::Matrix<double,10,10> A;
        Eigen::Matrix<double,10,10> CMat;
        Eigen::Matrix<double,10,10> K_A;

        Eigen::Matrix<double,10,1> pos;
        Eigen::Matrix<double,10,1> pos_des;

        Eigen::Matrix<double,10,1> vel;
        Eigen::Matrix<double,10,1> vel_des;

        Eigen::Matrix<double,10,1> accl;
        Eigen::Matrix<double,10,1> accl_des;

        Eigen::Matrix<double,10,1> delta_hat;

        Eigen::Matrix<double,10,1> pos_error;
        Eigen::Matrix<double,10,1> vel_error;
        Eigen::Matrix<double,10,1> s;
        Eigen::Matrix<double,10,1> qrDot;
        Eigen::Matrix<double,10,1> qrDotDot;
        Eigen::Matrix<double,10,1> tau;
        Eigen::Matrix<double,10,1> delta_hatDot;
        Eigen::Matrix<double,10,1> acceleration;

        std::vector<std::string> substrings;
        std::string substring;
        std::vector<double> doubles;
        
        std::string token;
        std::vector<std::string> tokens;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(UAMcontrolPush)
}