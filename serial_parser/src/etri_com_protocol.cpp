#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <map>
#include <deque>
#include <algorithm>
#include <vector>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <teb_local_planner/teb_config.h>
#include "costmap_2d/InflationPluginConfig.h"
//#include <move_base/move_base.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Char.h>
#include <std_msgs/Empty.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalStatusArray.h"

#include <fcntl.h> // for open()
#include <unistd.h> // for close()
#include <sys/ioctl.h> // for ioctl()
#include <linux/usbdevice_fs.h> // for USBDEVFS_RESET
#include <cstdlib>
#include <cstdio>
#include <sstream>
#include <array>

int get_pid(std::string port) {
    std::string command = "lsof -t " + port;
    std::array<char, 128> buffer;
    std::string result;
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        std::cerr << "Failed to run command\n";
        return -1;
    }
    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        result += buffer.data();
    }
    pclose(pipe);
    int pid;
    if (sscanf(result.c_str(), "%d", &pid) != 1) {
        std::cerr << "No process is using the port\n";
        return -1;
    }
    return pid;
}

bool kill_process(int pid) {
    std::stringstream ss;
    ss << "kill -9 " << pid;
    return system(ss.str().c_str()) == 0;
}

bool reset_usb_device(std::string device_file, int pid) {
    // First, try to kill the process that's using the device.
   // if (!kill_process(pid)) {
   //     ROS_ERROR_STREAM(" Failed to kill the process. Return false to indicate failure...");
   //     return false;
   // }

    int fd = open(device_file.c_str(), O_WRONLY);
    if (fd == -1) {
        // Cannot open device file. Return false to indicate failure.
        return false;
    }

    int rc = ioctl(fd, USBDEVFS_RESET, 0);
    if (rc == -1) {
        // USBDEVFS_RESET failed. Close the device file and return false.
        close(fd);
        return false;
    }

    close(fd);
    return true; // Successfully reset USB device.
}


class ETRI_COMM {
	
	public:

		ETRI_COMM(ros::NodeHandle* nh)
		{
			n = nh;
	        if(!n->getParam("ETRI_serial_node/device_name", device_name_)) {
				ROS_WARN("Could not get value of device_name parameter, using default value.");
				device_name_ = "/dev/ttyUSB0";
			}
			if(!n->getParam("ETRI_serial_node/default_robot_vel_x", default_robot_vel_x_)) {
				ROS_WARN("Could not get value of default_robot_vel_x parameter, using default value.");
				default_robot_vel_x_ = 0.65;
			}
			if(!n->getParam("ETRI_serial_node/default_robot_acc_x", default_robot_acc_x_)) {
				ROS_WARN("Could not get value of default_robot_acc_x parameter, using default value.");
				default_robot_acc_x_ = 0.13;
			}
			if(!n->getParam("ETRI_serial_node/frequency", frequency_)) {
				ROS_WARN("Could not get value of frequency parameter, using default value.");
				frequency_ = 5.0;
			}
			if(!n->getParam("ETRI_serial_node/baudrate", baudrate_)) {
				ROS_WARN("Could not get value of baudrate parameter, using default value.");
				baudrate_ = 115200;
			}
            if(!n->getParam("ETRI_serial_node/uart_debug", uart_debug_)) {
                ROS_WARN("Could not get value of uart_debug parameter, using default value.");
                uart_debug_ = true;
            }
			if(!n->getParam("ETRI_serial_node/goal_point", goal_point_)) {
                ROS_WARN("Could not get value of goal_point_ parameter, using default value.");
                goal_point_ = "0.0,0.0,0.0,0.0,0.0,0.0,0.0";
            }
            
			if(!n->getParam("ETRI_serial_node/pause_time", pause_time_)) {
                ROS_WARN("Could not get value of pause_time_ parameter, using default value.");
                pause_time_ = 4.0;
            }
            timer;
			param_updates = n->subscribe("/velocity_smoother/parameter_updates", 10, &ETRI_COMM::param_updates_cb, this);
			update_goal_point = n->subscribe("/freeway/update_goal_point", 10, &ETRI_COMM::update_goal_point_cb, this);
		    ai_status_pub = n->advertise<std_msgs::Int32MultiArray>("/freeway/ai_status", 10);
    	    cmd_force_pub = n->advertise<geometry_msgs::Twist>("/cmd_vel/emer", 10);
			goal_resume_pub = n->advertise<std_msgs::Empty>("/freeway/resume", 10);
			goal_pub = n->advertise<geometry_msgs::PoseStamped>("/freeway/goal", 10);
    	    move_base_force_cancel_pub = n->advertise<std_msgs::Empty>("/freeway/goal_cancel", 10);
            move_base_force_pause_pub = n->advertise<std_msgs::Empty>("/freeway/goal_pause", 10);
			cmd_array;
    	    param_update_check = false;
			param_update_time = 0.0;
			param_update_flag = false;
    	    r_command = -1;
			prev_r_command = -1;
			s_command = -1;
			prev_s_command = -1;
			s_data;
			r_data;
		}

	void update_goal_point_cb(const geometry_msgs::PoseStamped::ConstPtr& input_goal) {
		std::ostringstream ss;

    	// Extract and concatenate the values
    	ss << input_goal->pose.position.x << ","
    	   << input_goal->pose.position.y << ","
    	   << input_goal->pose.position.z << ","
    	   << input_goal->pose.orientation.x << ","
      	   << input_goal->pose.orientation.y << ","
    	   << input_goal->pose.orientation.z << ","
    	   << input_goal->pose.orientation.w;

    	goal_point_ = ss.str();
	}

    bool set_param(double default_robot_x_vel, double default_robot_x_acc, float vel_per, float acc_per) {
        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::DoubleParameter double_param_x_vel;
        dynamic_reconfigure::DoubleParameter double_param_x_acc;
        dynamic_reconfigure::Config conf;

        double_param_x_vel.name = "speed_lim_v";
        double_param_x_vel.value = default_robot_x_vel * vel_per;
        conf.doubles.push_back(double_param_x_vel);

        double_param_x_acc.name = "accel_lim_v";
        double_param_x_acc.value = default_robot_x_acc * acc_per;
        conf.doubles.push_back(double_param_x_acc);

        srv_req.config = conf;

        if (ros::service::call("/velocity_smoother_nav/set_parameters", srv_req, srv_resp)) {
            ROS_INFO("Successfully set parameters.");
            return true;
        } else {
            ROS_ERROR("Failed to call service /velocity_smoother_nav/set_parameters");
            return false;
        }
    }

	bool set_param_static(double default_robot_x_vel, double default_robot_x_acc, float vel_per, float acc_per) {

		float mod_x_vel;
		float mod_x_acc;

        if (vel_per == 0) mod_x_vel = default_robot_x_vel * vel_per;
        else mod_x_vel = default_robot_x_vel * vel_per;

        if (acc_per == 0) mod_x_acc = default_robot_x_acc * acc_per;
        else mod_x_acc = default_robot_x_acc * acc_per;

		n->setParam("/velocity_smoother_nav/speed_lim_v", mod_x_vel);
		n->setParam("/velocity_smoother_nav/accel_lim_v", mod_x_acc);
	}

	void param_updates_cb(const dynamic_reconfigure::Config& config) {
		ROS_INFO("Velocity Smoother Parameter Server is ON!!!");
		param_update_time = ros::Time::now().toSec();
		param_update_check = true;
		//dynamic_reconfigure::Config tmp = config;
	}

	void param_update_timer_check(double current_time) {
		if ((current_time - param_update_time) < 1.0) { param_update_check = true; }
		else { param_update_check = false; param_update_flag = false; 
		}
	}

    void set_param_update_check(bool param_update_check) {
        this->param_update_check = param_update_check;
    }

    double r_param_update_check() {
        return this->param_update_check;
    }

	// Utility function to compute the mode of a deque of strings
	std::string computeMode(const std::deque<std::string>& data) {
	    std::map<std::string, int> frequencyMap;
	    for (const auto& command : data) {
	        frequencyMap[command]++;
	    }
	
	    int maxCount = 0;
	    std::string mode;
	    for (const auto& pair : frequencyMap) {
	        if (pair.second > maxCount) {
	            maxCount = pair.second;
	            mode = pair.first;
	        }
	    }
	    return mode;
	}

	void parse(const std::string& input_serial) {
	    // Reset cmd_array by clearing all elements
	    cmd_array.clear();
        if(uart_debug_) { ROS_INFO_STREAM("input serial:" << input_serial); }
	    // Find the position of the first delimiter (',')
	    size_t first_delimiter_pos = input_serial.find(',');
	    if (first_delimiter_pos == std::string::npos) {
	        // First delimiter not found, handle accordingly
	        ROS_WARN("Invalid input_serial format. First delimiter not found.");
	        return;
	    }

	    // Extract the first data before the first delimiter
	    std::string first_data_str = input_serial.substr(0, first_delimiter_pos);
	    try {
	        //float first_data = std::stof(first_data_str);
	        cmd_array.push_back(first_data_str);
	    } catch (const std::exception& e) {
	        ROS_WARN("Failed to parse first_data_str: %s", e.what());
	        return;
	    }

	    // Find the position of the second delimiter ('>')
	    size_t second_delimiter_pos = input_serial.find('<', first_delimiter_pos + 1);
	    if (second_delimiter_pos == std::string::npos) {
	        // Second delimiter not found, handle accordingly
	        ROS_WARN("Invalid input_serial format. Second delimiter not found.");
	        return;
	    }

	    // Extract the second data between the first delimiter and the second delimiter
	    std::string second_data_str = input_serial.substr(first_delimiter_pos + 1, second_delimiter_pos - first_delimiter_pos - 1);
	    try {
	        //float second_data = std::stof(second_data_str);
	        cmd_array.push_back(second_data_str);
	    } catch (const std::exception& e) {
	        ROS_WARN("Failed to parse second_data_str: %s", e.what());
	        return;
	    }
	}

   void logic_controller() {
        std_msgs::Int32MultiArray r_array;
		std_msgs::Int32MultiArray s_array;
        dynamic_reconfigure::Config conf;

    	if (cmd_array.size() != 2) {
    	    ROS_WARN("cmd_array does not contain the required number of elements.");
    	    return;
    	}
	    s_data.push_back(cmd_array[0]);
	    r_data.push_back(cmd_array[1]);

		if(uart_debug_) { ROS_INFO_STREAM("cmd_array : " << cmd_array[0] << "," << cmd_array[1]); }

    	// If the data exceeds the window size, remove the oldest data
    	if (s_data.size() > 1) { // only takes one, default : 10
    	    s_data.pop_front();
    	}
    	if (r_data.size() > 1) { // only takes one, default : 7
    	    r_data.pop_front();
    	}

    	// Compute the mode for s_data and r_data
    	std::string s_mode = computeMode(s_data);
		std::string r_mode = computeMode(r_data);
        //std::string r_mode;
		//if (cmd_array[1].compare("R1") == 0) r_mode = cmd_array[1];
		//else if (cmd_array[1].compare("R2") == 0) r_mode = cmd_array[1];
		//else if (cmd_array[1].compare("R5") == 0) r_mode = cmd_array[1];
		//else r_mode = computeMode(r_data);

    	// Map the mode to the respective integer range
		int s_command = (s_mode.size() > 1) ? std::stoi(s_mode.substr(1)) : -1;
		int r_command = (r_mode.size() > 1) ? std::stoi(r_mode.substr(1)) : -1;

		if(uart_debug_) ROS_INFO_STREAM("s_command: " << s_command);
		if(uart_debug_) ROS_INFO_STREAM("r_command: " << r_command);

    	// Now, s_command will be in the range 0-10 and r_command will be in the range 0-6.
    	// You can use these values as needed in your function.


		if (r_command == 1 && (prev_r_command != r_command)) {
            //if (set_param_static(default_robot_vel_x_, default_robot_acc_x_, 1.0, 1.0)) ROS_INFO("Normal drive - set default param! -> R-Command 1");
			std::vector<double> values;
    		std::istringstream ss(goal_point_);
    		std::string token;
		
    		// Parse the comma-separated string
    		while (std::getline(ss, token, ',')) {
    		    values.push_back(std::stod(token));
    		}
    		// Check if we have all 7 values
    		if (values.size() != 7) {
    		    // Handle the error
    		    return;
    		}
    		// Create a PoseStamped message and populate its fields
    		geometry_msgs::PoseStamped goal;
		
    		goal.header.seq = 0;
    		goal.header.stamp = ros::Time::now();
    		goal.header.frame_id = "map";
		
    		goal.pose.position.x = values[0];
    		goal.pose.position.y = values[1];
    		goal.pose.position.z = values[2];
    		goal.pose.orientation.x = values[3];
    		goal.pose.orientation.y = values[4];
    		goal.pose.orientation.z = values[5];
    		goal.pose.orientation.w = values[6];

			goal_pub.publish(goal);
            if (set_param(default_robot_vel_x_, default_robot_acc_x_, 1.0, 1.0)) ROS_INFO("speed default,  acceleration default -> R-Command 1");
		  	prev_r_command = r_command;
        }
		else if (r_command == 2 && (prev_r_command != r_command)) {
            if (set_param(default_robot_vel_x_, default_robot_acc_x_, 0.6, 1.0)) ROS_INFO("speed -30per acceleration -10per -> R-Command 2");
		    prev_r_command = r_command;
        }
		else if (r_command == 3 && (prev_r_command != r_command)) {
			if (set_param(default_robot_vel_x_, default_robot_acc_x_, 1.7, 1.0)) ROS_INFO("speed +30per acceleration +10per -> R-Command 3");
				prev_r_command = r_command;
        }
		else if (r_command == 5 && (prev_r_command != r_command)) {
		    std_msgs::Empty empty_goal;
        	geometry_msgs::Twist force_stop_vel;
			ROS_INFO("Robot fully stopped -> R-Command 5");
			prev_r_command = r_command;
			move_base_force_cancel_pub.publish(empty_goal);
        	cmd_force_pub.publish(force_stop_vel);                     
        }
		else if (r_command == 6 && (prev_r_command != r_command)) {
			std_msgs::Empty empty_msg;

			move_base_force_pause_pub.publish(empty_msg);
            ROS_INFO("Goal resume -> R-Command 6");
            timer = n->createTimer(ros::Duration(pause_time_), boost::bind(&ETRI_COMM::resume_tim_cb, this, _1, goal_resume_pub), true);

            //ros::Duration(pause_time_).sleep(); // sleep for half a second
			//goal_resume_pub.publish(empty_msg);

            prev_r_command = r_command;
        }

        r_array.data.push_back(r_command);
		s_array.data.push_back(s_command);

		std_msgs::Int32MultiArray combined_array;
		combined_array.data.insert(combined_array.data.end(), s_array.data.begin(), s_array.data.end());
		combined_array.data.insert(combined_array.data.end(), r_array.data.begin(), r_array.data.end());

		ai_status_pub.publish(combined_array);     
    }

    void resume_tim_cb(const ros::TimerEvent&, ros::Publisher& pub) {
        std_msgs::Empty resume_msg;
        pub.publish(resume_msg);
    }

	std::string r_device_name() {
		return device_name_;
	}

	float r_frequency() const {
		return frequency_;
	}

	int r_baudrate() const {
		return baudrate_;
	}

private:
	ros::NodeHandle* n;
    ros::Timer timer;
	ros::Subscriber param_updates;
	ros::Subscriber update_goal_point;
	ros::Publisher goal_pub;
	ros::Publisher ai_status_pub;
    ros::Publisher cmd_force_pub;
    ros::Publisher move_base_force_cancel_pub;
    ros::Publisher move_base_force_pause_pub;
	ros::Publisher goal_resume_pub;
	std::string device_name_;
	std::string goal_point_;
	std::vector<std::string> cmd_array;
	double default_robot_vel_x_;
	double default_robot_acc_x_;
	bool param_update_flag;
	bool param_update_check;
    bool uart_debug_;
	double param_update_time;
    int r_command;
	int prev_r_command;
    int s_command;
	int prev_s_command;
	float frequency_;
    float pause_time_;
	int baudrate_;
	// Assuming these are global or member variables
	std::deque<std::string> s_data;
	std::deque<std::string> r_data;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "ETRI_serial_node");
	ros::NodeHandle nh;
    serial::Serial ser;
	ETRI_COMM ec = ETRI_COMM(&nh);

	float frequency = ec.r_frequency();
	int baudrate = ec.r_baudrate();
	std::string port = ec.r_device_name();

    ec.set_param_update_check(true);

    // Try to open the port until successful, resetting the USB device and sleeping for 5 seconds after each failure.
    while (true) {
        try
        {
            ser.setPort(port);
            ser.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
            break;  // if ser.open() succeeded, break the loop.
        }
        catch (serial::IOException& e)
        {
		int pid = get_pid(port);
                ROS_ERROR_STREAM("Unable to open port. Resetting and retrying...");
                if (!reset_usb_device(port, pid)) {
                    ROS_ERROR_STREAM("Failed to reset USB device");
                }
                ros::Duration(2.0).sleep();  // sleep for 2 seconds before retrying.
            }
        }

	if (ser.isOpen()) {
		ROS_INFO("Serial Port initialized");
	}
	else {
		return -1;
	}

	ros::Rate loop_rate(frequency);

	while (ros::ok()) {
		double current_time = ros::Time::now().toSec();
		ec.param_update_timer_check(current_time);

        if (ser.write("at+qd1?\r\n")) { // <<-in if ser.available()
            //ROS_INFO("serial write checked");  
            if(ser.available()) {
                //ROS_INFO("serial available checked");  
                std::string input_serial = ser.read(ser.available());
                ec.parse(input_serial);
                ec.logic_controller();
    	    }
    	}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
