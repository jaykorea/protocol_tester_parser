#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <map>
#include <deque>
#include <algorithm>
#include <vector>
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
	        if(!n->getParam("device_name", device_name_)) {
				ROS_WARN("Could not get value of device_name parameter, using default value.");
				device_name_ = "/dev/ttyUSB0";
			}
			if(!n->getParam("default_robot_vel_x", default_robot_vel_x_)) {
				ROS_WARN("Could not get value of default_robot_vel_x parameter, using default value.");
				default_robot_vel_x_ = 0.65;
			}
			if(!n->getParam("default_robot_acc_x", default_robot_acc_x_)) {
				ROS_WARN("Could not get value of default_robot_acc_x parameter, using default value.");
				default_robot_acc_x_ = 0.13;
			}
			if(!n->getParam("frequency", frequency_)) {
				ROS_WARN("Could not get value of frequency parameter, using default value.");
				frequency_ = 5.0;
			}
			if(!n->getParam("baudrate", baudrate_)) {
				ROS_WARN("Could not get value of baudrate parameter, using default value.");
				baudrate_ = 115200;
			}
			param_updates = n->subscribe("/velocity_smoother/parameter_updates", 10, &ETRI_COMM::param_updates_cb, this);
		    ai_status_pub = n->advertise<std_msgs::Int32MultiArray>("/freeway/ai_status", 10);
    	    cmd_force_pub = n->advertise<geometry_msgs::Twist>("/cmd_vel/emer", 10);
    	    move_base_force_cancle_pub = n->advertise<std_msgs::Empty>("/freeway/goal_cancel", 10);
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

	bool set_param(double default_robot_x_vel, double default_robot_x_acc, float vel_per, float acc_per) {
		dynamic_reconfigure::ReconfigureRequest srv_req;
		dynamic_reconfigure::ReconfigureResponse srv_resp;
		dynamic_reconfigure::DoubleParameter double_param_x_vel;
		dynamic_reconfigure::DoubleParameter double_param_x_acc;
		dynamic_reconfigure::Config conf;

		double_param_x_vel.name = "speed_lim_v";

        if (vel_per == 0) double_param_x_vel.value = default_robot_x_vel * vel_per;
        else double_param_x_vel.value = default_robot_x_vel / vel_per;

        ROS_INFO("modified_x_vel : %f", double_param_x_vel.value);

		conf.doubles.push_back(double_param_x_vel);

		double_param_x_acc.name = "accel_lim_v";
		double_param_x_acc.value = default_robot_x_acc / acc_per;
		conf.doubles.push_back(double_param_x_acc);
		srv_req.config = conf;

		ros::service::call("/move_base/TebLocalPlannerROS/set_parameters", srv_req, srv_resp);

		return true;
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
        actionlib_msgs::GoalID empty_goal;
        geometry_msgs::Twist force_stop_vel;
        dynamic_reconfigure::Config conf;
        force_stop_vel.linear.x = 0.0;
        force_stop_vel.angular.z = 0.0;

    	if (cmd_array.size() != 2) {
    	    ROS_WARN("cmd_array does not contain the required number of elements.");
    	    return;
    	}
	    s_data.push_back(cmd_array[0]);
	    r_data.push_back(cmd_array[1]);

		ROS_INFO_STREAM("cmd_array : " << cmd_array[0] << "," << cmd_array[1]);

    	// If the data exceeds the window size, remove the oldest data
    	if (s_data.size() > 10) {
    	    s_data.pop_front();
    	}
    	if (r_data.size() > 6) {
    	    r_data.pop_front();
    	}

    	// Compute the mode for s_data and r_data
    	std::string s_mode = computeMode(s_data);
    	std::string r_mode = computeMode(r_data);

    	// Map the mode to the respective integer range
		int s_command = (s_mode.size() > 1) ? std::stoi(s_mode.substr(1)) : -1;
		int r_command = (r_mode.size() > 1) ? std::stoi(r_mode.substr(1)) : -1;

		ROS_INFO_STREAM("s_command: " << s_command);
		ROS_INFO_STREAM("r_command: " << r_command);

    	// Now, s_command will be in the range 0-10 and r_command will be in the range 0-6.
    	// You can use these values as needed in your function.


		if (r_command == 1 && (prev_r_command != r_command)) {
            if (set_param_static(default_robot_vel_x_, default_robot_acc_x_, 1.0, 1.0)) ROS_INFO("Normal drive - set default param! - hash1");
		  prev_r_command = r_command;
        }
		else if (r_command == 2 && (prev_r_command != r_command)) {
            if (set_param_static(default_robot_vel_x_, default_robot_acc_x_, 0.5, 0.9)) ROS_INFO("speed -50per acceleration -10per - hash2");
		  prev_r_command = r_command;
        }
		else if (r_command == 3 && (prev_r_command != r_command)) {
		if (set_param_static(default_robot_vel_x_, default_robot_acc_x_, 1.3, 1.1)) ROS_INFO("speed +30per acceleration +10per - hash3");
		prev_r_command = r_command;
        }
		else if (r_command == 4 && (prev_r_command != r_command)) {
   		//if (set_param(default_robot_vel_x_, default_robot_acc_x_, 0.0, 2.0)) ROS_INFO("speed -> 0 acceleration -1x - hash4");
			ROS_INFO("speed -> 0 acceleration -1x - hash4");
			prev_r_command = r_command;
			move_base_force_cancle_pub.publish(empty_goal);
        	cmd_force_pub.publish(force_stop_vel);                     
        }
		else if (r_command == 5 && (prev_r_command != r_command)) {
 		//if (set_param(default_robot_vel_x_, default_robot_acc_x_, 0.0, 2.0)) ROS_INFO("speed -> 0 acceleration -2x - hash5");
			ROS_INFO("speed -> 0 acceleration -2x - hash5");
			prev_r_command = r_command;
			move_base_force_cancle_pub.publish(empty_goal); 
        	cmd_force_pub.publish(force_stop_vel);              
        }
		
        r_array.data.push_back(r_command);
		s_array.data.push_back(s_command);

		std_msgs::Int32MultiArray combined_array;
		combined_array.data.insert(combined_array.data.end(), r_array.data.begin(), r_array.data.end());
		combined_array.data.insert(combined_array.data.end(), s_array.data.begin(), s_array.data.end());

		ai_status_pub.publish(combined_array);     
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
	ros::Subscriber param_updates;
	ros::Publisher ai_status_pub;
    ros::Publisher cmd_force_pub;
    ros::Publisher move_base_force_cancle_pub;
	std::string device_name_;
	std::vector<std::string> cmd_array;
	double default_robot_vel_x_;
	double default_robot_acc_x_;
	bool param_update_flag;
	bool param_update_check;
	double param_update_time;
    int r_command;
	int prev_r_command;
    int s_command;
	int prev_s_command;
	float frequency_;
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