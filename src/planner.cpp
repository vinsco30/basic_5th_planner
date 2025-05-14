#include "../include/basic_5th_planner/planner.hpp"

QUINTIC_PLANNER::QUINTIC_PLANNER() : Node("quintic_planner") {

    this->declare_parameter<float>("cruise_velocity", 1.0f);
    _cv = this->get_parameter("cruise_velocity").as_double();
    this->declare_parameter<float>( "cruise_velocity_takeoff", 1.0f);
    _cv_to = this->get_parameter("cruise_velocity_takeoff").as_double();
    this->declare_parameter<float>( "takeoff_altitude", 1.5f);
    _to_altitude = this->get_parameter("takeoff_altitude").as_double();
    this->declare_parameter<float>( "cruise_velocity_rotation", 1.0f );
    _cv_rot = this->get_parameter("cruise_velocity_rotation").as_double();

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
    
    _offboard_control_mode_pub =
        this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/in/offboard_control_mode", 10);
    
        _path_pub =
        this->create_publisher<nav_msgs::msg::Path>("/planner/path", 10);

    //Timer callback init
    _timer_loop =
        this->create_wall_timer(100ms, std::bind(&QUINTIC_PLANNER::run_loop, this));
    // _timer_publisher =
    //     this->create_wall_timer(100ms, std::bind(&QUINTIC_PLANNER::publish_trajectory_setpoint, this));
    // _timer_offboard =
    //     this->create_wall_timer(100ms, std::bind(&QUINTIC_PLANNER::publish_offboard_control_mode, this));
    // _timer_client =
    //     this->create_wall_timer(100ms, std::bind(&QUINTIC_PLANNER::client_loop, this));
    boost::thread client_loop_t( &QUINTIC_PLANNER::client_loop, this );
}

void QUINTIC_PLANNER::odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg) {
    
    _pos_odom << odom_msg->position[0], odom_msg->position[1], odom_msg->position[2];
    _quat_odom << odom_msg->q[0], odom_msg->q[1], odom_msg->q[2], odom_msg->q[3];
    matrix::Quaternionf actual_att(_quat_odom(0), _quat_odom(1), _quat_odom(2), _quat_odom(3));
    _yaw_odom = matrix::Eulerf(actual_att).psi();

    _first_odom = true;

}

void QUINTIC_PLANNER::run_loop() {
    if (!_first_odom) {
        // RCLCPP_WARN(this->get_logger(), "Waiting for odometry data...");
        return;
    }

    if ( !_first_traj ) {
        _pos_cmd = _pos_odom;
        _quat_cmd = _quat_odom;
        _vel_cmd << 0.0f, 0.0f, 0.0f;
        _acc_cmd << 0.0f, 0.0f, 0.0f;
        _yaw_cmd = _yaw_odom;
        _first_traj = true;
        
        return;
    }
    if (_offboard_setpoint_counter == 10) {
		this->vehicle_command_publisher(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	}

    publish_offboard_control_mode();
    publish_trajectory_setpoint();

    if (_offboard_setpoint_counter < 11) {
		_offboard_setpoint_counter++;
	}

}

void QUINTIC_PLANNER::takeoff_exec( float altitude ){

    if(!_takeoff_completed) {

        Eigen::Vector3d pos_init;
        Eigen::Vector4d quat_init;
        Eigen::Vector3d pos_end_to;
        Eigen::Vector4d quat_end_to;
        // std::vector<double> pos_init, quat_init, pos_end_to, quat_end_to
    
        if( altitude > 0.0f )
            altitude = -altitude;
    
        pos_init = _pos_odom;
        quat_init = _quat_odom;
        pos_end_to << pos_init(0), pos_init(1), altitude;
        quat_end_to = quat_init;
        
        if( !_arm_status ) {
            arm();
        }
    
        //Takeoff trajectory
        generateTakeOffTraj( pos_init, pos_end_to, _cv_to );
        _takeoff_completed = true;
        std::cout << "Takeoff trajectory set\n";
    }
    else {
        std::cout << "Takeoff already completed\n";
    }


} 

void QUINTIC_PLANNER::goto_exec() {

    // if( _takeoff_completed ) {

        Eigen::Vector3d pos_init;
        Eigen::Vector4d quat_init;
        Eigen::Vector3d pos_end;
        Eigen::Vector4d quat_end;

        pos_init = _pos_odom;
        quat_init = _quat_odom;
        std::cout<<"step1\n";
        pos_end << _pos_key(0), _pos_key(1), _pos_key(2);

        if( pos_end(2) > 0.0 ) {
            std::cout << "Z coordinate is positive, setting it to negative\n";
            pos_end(2) = -pos_end(2);
        }

        std::cout<<"step2\n";
        generateGoToTraj( pos_init, _yaw_odom, pos_end, _yaw_key, _cv );
        std::cout << "Goto trajectory set\n";
    // }
    // else {
    //     std::cout << "Takeoff not completed yet\n";
    // }

}

void QUINTIC_PLANNER::client_loop() {
    bool exit = false;
    std::cout << "Starting client loop...\n";
    while( !exit && rclcpp::ok() ) {
        std::cout << "Enter command [arm | go | takeoff | stop]: \n"; 
        std::cin >> _cmd;
        if( _cmd == "arm" ) {
            this->arm();
        }
        else if( _cmd == "takeoff" ) {
            std::cout << "Enter takeoff altitude: \n"; 
            std::cin >> _to_altitude;
            takeoff_exec( _to_altitude );
        }
        else if( _cmd == "go" ) {
            std::cout << "Enter X coordinate: "; 
            std::cin >> _pos_key(0);
            std::cout << "Enter Y coordinate: "; 
            std::cin >> _pos_key(1);
            std::cout << "Enter Z coordinate: "; 
            std::cin >> _pos_key(2);
            std::cout << "Enter final yaw: "; 
            std::cin >> _yaw_key;
            goto_exec();
        }
        else if( _cmd == "stop" ) {
            exit = true;
            rclcpp::shutdown();
        }
        else {
            std::cout << "Unknown command;\n";
        }
    } 

}

void QUINTIC_PLANNER::publish_trajectory_setpoint() {

    if( _first_odom && _first_traj ) {

        px4_msgs::msg::TrajectorySetpoint msg{};
        rclcpp::Time now = this->get_clock()->now();
    
        msg.timestamp = now.nanoseconds() / 1000.0;
    
        msg.position[0] = _pos_cmd(0);
        msg.position[1] = _pos_cmd(1);
        msg.position[2] = _pos_cmd(2);
    
        msg.velocity[0] = _vel_cmd(0);
        msg.velocity[1] = _vel_cmd(1);
        msg.velocity[2] = _vel_cmd(2);
    
        msg.acceleration[0] = _acc_cmd(0);
        msg.acceleration[1] = _acc_cmd(1);
        msg.acceleration[2] = _acc_cmd(2);
    
        matrix::Quaternionf des_att(_quat_cmd(0), _quat_cmd(1), _quat_cmd(2), _quat_cmd(3));
        // msg.yaw = matrix::Eulerf( des_att ).psi();
        // msg.yawspeed = 0.0f;
        msg.yaw = _yaw_cmd;
        msg.yawspeed = _yaw_rate_cmd;
    
        _trajectory_setpoint_pub->publish(msg);

    }

}

void QUINTIC_PLANNER::publish_offboard_control_mode() {
    
    px4_msgs::msg::OffboardControlMode msg{};
    rclcpp::Time now = this->get_clock()->now();
    msg.timestamp = now.nanoseconds() / 1000.0;

    msg.position = true;
    msg.velocity = true;
    msg.acceleration = true;
    msg.attitude = false;
    msg.body_rate = false;

    _offboard_control_mode_pub->publish(msg);
    // std::cout<<"Pubblico offboard_control_mode\n";
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

void QUINTIC_PLANNER::arm() {
    vehicle_command_publisher( px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1 );
    RCLCPP_INFO(this->get_logger(), "Arming vehicle...");
}

int main(int argc, char* argv[]) {
	std::cout << "Starting Quintic Polynomial Trajectory node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

    rclcpp::spin( std::make_shared<QUINTIC_PLANNER>() );

    rclcpp::shutdown();

	return 0;
}