#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
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

#define hash_counter_check 3

class ETRI_COMM {
	
	public:

		ETRI_COMM(ros::NodeHandle* nh, const char delimiter_char):delimiter_char('>')
		{
			n = nh;
	        if(!n->getParam("device_name", device_name_)) {
				ROS_WARN("Could not get value of device_name parameter, using default value.");
				device_name_ = "/dev/ttyUSB0";
			}

			if(!n->getParam("window_size", movingAverageWindowSize)) {
				ROS_WARN("Could not get value of window_size parameter, using default value.");
				movingAverageWindowSize = 5;
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
    	    uart_counter = 0;
		    uart_payload;
		    uart_debugmode = true;
    	    delimiter_flag = false;
    	    ser_flag =false;
    	    param_update_check = false;
			param_update_time = 0.0;
			param_update_flag = false;
    	    hash_status = -1;
			prev_hash_status = -1;
			hash_values;
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

    void set_uart_counter(uint8_t uart_counter) {
        this->uart_counter = uart_counter;
    }

    double r_uart_counter() {
        return this->uart_counter;
    }

    void set_param_update_check(bool param_update_check) {
        this->param_update_check = param_update_check;
    }

    double r_param_update_check() {
        return this->param_update_check;
    }

    char* r_uart_payload() {
        return this->uart_payload;
    }

    void parse(std::string input_serial) {

        uart_payload = (char*)malloc(input_serial.length() + 1);
		uart_counter = 0;
        
        //memset(ec.uart_payload, '\0', ec.uart_payload_LEN + 1);
        while (uart_counter < input_serial.length()+1 ){
            char recChar = input_serial[uart_counter];
            if (recChar == delimiter_char)
                {
                    if(uart_debugmode) ROS_INFO("Delimiter Found");
                	delimiter_flag = true;
                    break;
                }
                delimiter_flag = false;
                uart_payload[uart_counter] = recChar;    
                uart_counter++;
            //ROS_INFO("parser while checked");
              }
			  uart_payload[uart_counter] = '\0'; // Add null terminator to the string
    }

   void logic_controller() {
        std_msgs::Int32MultiArray data_array;
        actionlib_msgs::GoalID empty_goal;
        geometry_msgs::Twist force_stop_vel;
        dynamic_reconfigure::Config conf;
        force_stop_vel.linear.x = 0.0;
        force_stop_vel.angular.z = 0.0;

		if (uart_payload > 0 && delimiter_flag == true) {

			int payloadLength = uart_counter;
    		// Check if the payload length is valid
    		if (payloadLength % 2 != 0) {
    		  // Invalid payload length, discard the data
    		  return;
    		}
			
			for (int i = 1; i < uart_counter / 2 + 1; i++) {
				data_array.data.push_back((uart_payload[2 * i - 2] - '0') * (int)10 + (uart_payload[2 * i - 1] - '0'));
				//ROS_INFO("parser pushback checked");
			}
			if (!data_array.data.empty()) { //should consider of nh::hasParam
				if (data_array.data[4] == 0 && (data_array.data[5] == 0 || data_array.data[5] == 3 || data_array.data[5] == 4)) {
					if (data_array.data[0] == 0 && data_array.data[1] != 0) { // #4 deceleration/rotation
                        hash_values.push_back(hash_4);
					}
					else if (data_array.data[0] == 0 && data_array.data[1] == 0) { // #2 deceleration
						hash_values.push_back(hash_2);
					}
				}
				if (data_array.data[4] == 0 && (data_array.data[5] == 1 || data_array.data[5] == 2)) {
						if (data_array.data[5] == 1) { 
							if (data_array.data[0] == 0 && data_array.data[2] == 0) { // #1 normal
								hash_values.push_back(hash_1);
							}
							else if (data_array.data[5] == 2 && data_array.data[0] == 0 && (data_array.data[3] == 1 && data_array.data[1] != 0)) { //#5 e_brake
                            hash_values.push_back(hash_5);
						}
							else if(data_array.data[0] == 0 && data_array.data[2] == 1) { // #3 acceleration
								hash_values.push_back(hash_3);
						}
					}
				}

				  // Check if the vector has enough elements for the moving average
  				if (hash_values.size() >= movingAverageWindowSize) {
  				  // Calculate the sum of the last 'movingAverageWindowSize' elements
  				  int sum = 0;
  				  for (int i = hash_values.size() - movingAverageWindowSize; i < hash_values.size(); i++) {
  				    sum += hash_values[i];
  				  }
				
  				  // Calculate the moving average
  				  int movingAverage = sum / movingAverageWindowSize;
				
  				  // Update the hash_status with the moving average
  				  hash_status = movingAverage;
				
  				  // Remove the oldest element from the vector
  				  hash_values.erase(hash_values.begin());
  				}
				  if (hash_status == 1 && (prev_hash_status != hash_status)) {
                      if (set_param_static(default_robot_vel_x_, default_robot_acc_x_, 1.0, 1.0)) ROS_INFO("Normal drive - set default param! - hash1");
					  prev_hash_status = hash_status;
                  }
				  else if (hash_status == 2 && (prev_hash_status != hash_status)) {
                      if (set_param_static(default_robot_vel_x_, default_robot_acc_x_, 0.5, 0.9)) ROS_INFO("speed -50per acceleration -10per - hash2");
					  prev_hash_status = hash_status;
                  }
				  else if (hash_status == 3 && (prev_hash_status != hash_status)) {
					if (set_param_static(default_robot_vel_x_, default_robot_acc_x_, 1.3, 1.1)) ROS_INFO("speed +30per acceleration +10per - hash3");
					prev_hash_status = hash_status;
                  }
				  else if (hash_status == 4 && (prev_hash_status != hash_status)) {
   					//if (set_param(default_robot_vel_x_, default_robot_acc_x_, 0.0, 2.0)) ROS_INFO("speed -> 0 acceleration -1x - hash4");
					   ROS_INFO("speed -> 0 acceleration -1x - hash4");
					   prev_hash_status = hash_status;
					   move_base_force_cancle_pub.publish(empty_goal);    
                       cmd_force_pub.publish(force_stop_vel);                     
                  }
				  else if (hash_status == 5 && (prev_hash_status != hash_status)) {
 					//if (set_param(default_robot_vel_x_, default_robot_acc_x_, 0.0, 2.0)) ROS_INFO("speed -> 0 acceleration -2x - hash5");
					   ROS_INFO("speed -> 0 acceleration -2x - hash5");
					   prev_hash_status = hash_status;
					   move_base_force_cancle_pub.publish(empty_goal); 
                       cmd_force_pub.publish(force_stop_vel);              
                  }
				//   for (int i = 0; i < hash_values.size(); i++) {
				//   	ROS_INFO("hash_values[%d]: %d", i, hash_values[i]);
			}
            data_array.data.push_back(hash_status);
			ai_status_pub.publish(data_array);
		}       
    }

    void free_uart_payload() {
        free(uart_payload);
    }

	std::string r_device_name() {
		return device_name_;
	}

	bool r_uart_debugmode() const {
		return uart_debugmode;
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
	std::vector<int> hash_values;
	int movingAverageWindowSize;
	uint8_t uart_counter;
	//uint16_t Uart_PAYLOAD_LEN;
	char* uart_payload;
	const char delimiter_char;
	bool uart_debugmode;
	bool delimiter_flag;
	bool ser_flag;
	double default_robot_vel_x_;
	double default_robot_acc_x_;
	bool param_update_flag;
	bool param_update_check;
	double param_update_time;
    const int hash_1 = 1;
	const int hash_2 = 2;
	const int hash_3 = 3;
	const int hash_4 = 4;
	const int hash_5 = 5;
    int hash_status;
    int hash_interval;
	int prev_hash_status;
    double hash_time_check;
	float frequency_;
	int baudrate_;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "ETRI_serial_node");
	ros::NodeHandle nh;
    serial::Serial ser;
	const char del = '>';
	ETRI_COMM ec = ETRI_COMM(&nh, del);

	float frequency = ec.r_frequency();
	int baudrate = ec.r_baudrate();


    ec.set_param_update_check(true);
	//dynamic_reconfigure::Client<teb_local_planner::TebLocalPlannerReconfigureConfig> client("/move_base/TebLocalPlannerROS/");
	//dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig> client2("/move_base/local_costmap/inflation_layer/");
	//teb_local_planner::TebLocalPlannerReconfigureConfig teb_config;
	//costmap_2d::InflationPluginConfig costmap_config;
	try
	{
		ser.setPort(ec.r_device_name());
		ser.setBaudrate(baudrate);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
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
		// ec.set_uart_counter(0);
		//if (!client.getCurrentConfiguration(teb_config)) ROS_INFO("waiting for costmap Configuration");
		// if ( ec.param_update_check && !ec.param_update_flag ) {
		// if (ros::param::get("/move_base/TebLocalPlannerROS/max_vel_x",default_robot_vel_x_)) ROS_INFO("Param checked");
		// if (ros::param::get("/move_base/TebLocalPlannerROS/acc_lim_x",ec.default_robot_acc_x_)) ROS_INFO("Param checked");
		// //ROS_INFO("Param checked");
		// ec.param_update_flag = true;
		// }

		// if (ser.write("at+qd1?\r\n")) ec.ser_flag =true;
		// else ec.ser_flag = false;

        if (ser.write("at+qd1?\r\n")) { // <<-in if ser.available()
            //ROS_INFO("serial write checked");  
            if(ser.available()) {
                //ROS_INFO("serial available checked");  
                std::string input_serial = ser.read(ser.available());
                ec.parse(input_serial);
                if(ec.r_uart_debugmode()) ROS_INFO("Parsed data : %s", ec.r_uart_payload());
                ec.logic_controller();
                ser.flush();
                ec.free_uart_payload();
    	    }
    	}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}