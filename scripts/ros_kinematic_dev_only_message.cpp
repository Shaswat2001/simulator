#include "ros_kinematic.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace current
{
    double time = 0.0;
    std::vector<float> position_desired(3, 0.0);
    std::vector<float> orientation_desired(3, 0.0);

    std::vector<float> position_now(3, 0.0);
    std::vector<float> orientation_now(3, 0.0);

    std::vector<float> linear_v(3, 0.0);
    std::vector<float> angular_v(3, 0.0);

    // WPose pose();
};

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
    

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twst_subscriber_;
        
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

            
            joint_subscription_ = this->ros_node_->create_subscription<std_msgs::msg::String>("/robot_joints_string", 10,std::bind(&ModelPush::robot_joint_stringCallback, this, std::placeholders::_1));
            body_subscription_ = this->ros_node_->create_subscription<std_msgs::msg::String>("/drone_body_string", 10,std::bind(&ModelPush::drone_body_stringCallback, this, std::placeholders::_1));
           
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));
            
            
        }

   
    public:
        void OnUpdate()
        {
         

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
            
        

        }

  void drone_body_stringCallback(std_msgs::msg::String::SharedPtr string_msg) 
        { 
            RCLCPP_INFO(this->ros_node_->get_logger(), "drone body message: %s", string_msg->data.c_str());
          

        }



        // Pointer to the model
    private:
        physics::ModelPtr model;
        //  double joint1_value_=0.;
        //  double joint2_value_=0.;
        //  double joint3_value_=0.;
        //  double joint4_value_=0.;
        //  double gripper_=0.;
        //  double x_value_=0.;
        //  double y_value_=0.;
        //  double z_value_=0.;
        //  double r1_value_=0.;
        //  double r2_value_=0.;
        //  double r3_value_=0.;



         std::vector<std::string> substrings;
         std::string substring;
         std::vector<double> doubles;
        
        std::string token;
        std::vector<std::string> tokens;
       




        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}