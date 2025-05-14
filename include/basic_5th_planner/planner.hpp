/*TODO: Text description*/
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "utils.h"
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

//PX4 messages
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

//ROS2 messages
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>

//Matrix lib
#include "../include/matrix/Matrix.hpp"
#include "../include/matrix/Quaternion.hpp"
#include "../include/matrix/Euler.hpp"

class QUINTIC_PLANNER : public rclcpp::Node
{
public:
    QUINTIC_PLANNER();

    void arm();
    
private:
    //Callback functions
    void odom_cb( const px4_msgs::msg::VehicleOdometry::SharedPtr );

    void run_loop();
    void client_loop();
    void takeoff_exec( float altitude );
    void land_exec();
    void goto_exec();
    void vehicle_command_publisher( uint16_t command, float param1 = 0.0f, float param2 = 0.0f );
    void publish_trajectory_setpoint();
    void publish_offboard_control_mode();

    /*Trajectory generation functions*/
    void generateTakeOffTraj( const Eigen::Vector3d& start_pos, const Eigen::Vector3d& final_pos, const float cv );
    void generateGoToTraj( const Eigen::Vector3d& start_pos, const float& start_yaw, const Eigen::Vector3d& final_pos, const float& final_yaw, const float cv );

    //Subscriptions
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr _control_mode_sub;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr _timesync_sub;
    
    //Command subscriptions
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _takeoff_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _land_sub;

    //Publishers
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_setpoint_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_control_mode_pub;

    //Input quantities
    std::atomic<uint64_t> _timestamp;   //!< common synced timestamped
    bool _takeoff_cmd{false};
    bool _land_cmd{false};
    bool _offboard_status{false};
    bool _arm_status{false};
    Eigen::Vector3d _pos_odom;
    Eigen::Vector4d _quat_odom;
    float _yaw_odom{0.0f};

    //Commanded quantities
    Eigen::Vector3d _pos_cmd;
    Eigen::Vector3d _vel_cmd;
    Eigen::Vector3d _acc_cmd;
    Eigen::Vector4d _quat_cmd;
    float _yaw_cmd{0.0f};
    float _yaw_rate_cmd{0.0f};

    int _offboard_setpoint_counter{0};
    std::string _cmd;
    Eigen::Vector3d _pos_key;
    float _yaw_key{0.0f};


    //Flags
    bool _first_odom{false};
    bool _first_traj{false};
    bool _trajectory_execution{false};
    bool _stop_trajectory{false};
    bool _takeoff_completed{false};

    //Parameters
    float _cv{0.0f}; ///< cruise velocity
    float _cv_to{0.0f}; ///< cruise velocity for takeoff
    float _cv_rot{0.0f}; ///< cruise velocity for rotation
    float _to_altitude{0.0f}; ///< takeoff altitude


    rclcpp::TimerBase::SharedPtr _timer_loop;
    rclcpp::TimerBase::SharedPtr _timer_publisher;
    rclcpp::TimerBase::SharedPtr _timer_offboard;
    rclcpp::TimerBase::SharedPtr _timer_client;

};
