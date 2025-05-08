#include "../include/basic_5th_planner/planner.hpp"

QUINTIC_PLANNER::QUINTIC_PLANNER() : Node("quintic_planner") {

    this->declare_parameter<float>("cruise_velocity", 1.0f);
    _cv = this->get_parameter("cruise_velocity").as_double();
    this->declare_parameter<float>( "cruise_velocity_takeoff", 1.0f);
    _cv_to = this->get_parameter("cruise_velocity_takeoff").as_double();
    this->declare_parameter<float>( "takeoff_altitude", 1.5f);
    _to_altitude = this->get_parameter("takeoff_altitude").as_double();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    //Inputs
    _odom_sub = 
        this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/out/vehicle_odometry", qos,
        std::bind(&QUINTIC_PLANNER::odom_cb, this, std::placeholders::_1));

    _control_mode_sub =
        this->create_subscription<px4_msgs::msg::VehicleControlMode>( "fmu/out/vehicle_control_mode", qos,
        [this]( const px4_msgs::msg::VehicleControlMode::UniquePtr msg ) {
            _offboard_status = msg->flag_control_offboard_enabled;
            _arm_status = msg->flag_armed;
        } );

    _timesync_sub = 
        this->create_subscription<px4_msgs::msg::TimesyncStatus>( "/fmu/out/timesync_status", qos,
            [this]( const px4_msgs::msg::TimesyncStatus::UniquePtr msg ) {
                _timestamp.store(msg->timestamp);
            } );

    //Outputs
    _trajectory_setpoint_pub = 
        this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("fmu/in/trajectory_setpoint", 10);
    
    _vehicle_command_pub =
        this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/in/vehicle_command", 10);
    
    _path_pub =
        this->create_publisher<nav_msgs::msg::Path>("/planner/path", 10);

    //Timer callback init
    _timer_loop =
        this->create_wall_timer(100ms, std::bind(&QUINTIC_PLANNER::run_loop, this));
    _timer_publisher =
        this->create_wall_timer(100ms, std::bind(&QUINTIC_PLANNER::publish_trajectory, this));

}

void QUINTIC_PLANNER::odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg) {
    
    _pos_odom << odom_msg->position[0], odom_msg->position[1], odom_msg->position[2];
    _quat_odom << odom_msg->q[0], odom_msg->q[1], odom_msg->q[2], odom_msg->q[3];
    if( !_first_odom ) {
        _pos_cmd = _pos_odom;
        
    }
    _first_odom = true;

}

void QUINTIC_PLANNER::run_loop() {
    if (!_first_odom) {
        RCLCPP_WARN(this->get_logger(), "Waiting for odometry data...");
        return;
    }

    if ( !_arm_status ) {
        vehicle_command_publisher( px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1 );
        RCLCPP_INFO(this->get_logger(), "Arming vehicle...");
    }

}

void QUINTIC_PLANNER::takeoff_exec( const float h ){

    Eigen::Vector3d pos_init;
    Eigen::Vector4d quat_init;
    Eigen::Vector3d pos_end_to;
    Eigen::Vector4d quat_end_to;

    if( h > 0.0f )
        h = -h;

    pos_init = _pos_odom;
    quat_init = _quat_odom;
    pos_end_to << pos_init(0), pos_init(1), h;
    quat_end_to = quat_init;
    
    //Arming command
    if ( !_arm_status ) {
        vehicle_command_publisher( px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1 );
        RCLCPP_INFO(this->get_logger(), "Arming vehicle...");
    }


} 

void QUINTIC_PLANNER::vehicle_command_publisher( uint16_t command, float param1, float param2 ) {
    
    px4_msgs::msg::VehicleCommand msg{};
    rclcpp::Time now = this->get_clock()->now();

    msg.timestamp = now.nanoseconds() / 1000.0;

    msg.param1 = param1;
    msg.param2 = param2;

    msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	
	std::cout << "Sending command\n";
    _vehicle_command_pub->publish(msg);
}

int main(int argc, char* argv[]) {
	std::cout << "Starting Quintic Polynomial Trajectory node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    rclcpp::spin( std::make_shared<QUINTIC_PLANNER>() );

    rclcpp::shutdown();

	return 0;
}