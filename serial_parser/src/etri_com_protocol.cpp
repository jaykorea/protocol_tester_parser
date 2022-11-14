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
			param_updates = nh->subscribe("/move_base/TebLocalPlannerROS/parameter_updates", 10, &ETRI_COMM::param_updates_cb, this);
		    read_pub = nh->advertise<std_msgs::Int32MultiArray>("read_pub", 100);
    	    cmd_force_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 100);
    	    move_base_force_cancle_pub = nh->advertise<actionlib_msgs::GoalID>("move_base/cancel", 100);
    	    uart_counter = 0;
		    uart_payload;
		    uart_debugmode = true;
    	    //delimiter_char = '>';
    	    delimiter_flag = false;
    	    ser_flag =false;
    	    default_robot_vel_x = 1.0;
    	    default_robot_acc_x = 0.11;
    	    param_update_check = false;
			param_update_time = 0.0;
			param_update_flag = false;
			hash_1 = 0;
			hash_2 = 0;
			hash_3 = 0;
			hash_4 = 0;
			hash_5 = 0;
    	    hash_status = -1;
    	    hash_interval = 1.5;
   			hash_time_check = 0.0;
		}

	bool set_param(double default_robot_x_vel, double default_robot_x_acc, float vel_per, float acc_per) {
		dynamic_reconfigure::ReconfigureRequest srv_req;
		dynamic_reconfigure::ReconfigureResponse srv_resp;
		dynamic_reconfigure::DoubleParameter double_param_x_vel;
		dynamic_reconfigure::DoubleParameter double_param_x_acc;
		dynamic_reconfigure::Config conf;

		double_param_x_vel.name = "max_vel_x";
        if (vel_per == 0) double_param_x_vel.value = default_robot_x_vel * vel_per;
        else double_param_x_vel.value = default_robot_x_vel / vel_per;
        ROS_INFO("modified_x_vel : %f", double_param_x_vel.value);
		conf.doubles.push_back(double_param_x_vel);

		double_param_x_acc.name = "acc_lim_x";
		double_param_x_acc.value = default_robot_x_acc / acc_per;
		conf.doubles.push_back(double_param_x_acc);
		srv_req.config = conf;

		ros::service::call("/move_base/TebLocalPlannerROS/set_parameters", srv_req, srv_resp);

		return true;
	}

	void param_updates_cb(const dynamic_reconfigure::Config& config) {
		//ROS_INFO("Teb Dynamic Server is ON!!!");
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
        
        //memset(ec.uart_payload, '\0', ec.uart_payload_LEN + 1);
        while (uart_counter < input_serial.length()+1 ){
            char recChar = input_serial[uart_counter];
            if (recChar == delimiter_char)
                {
                    if(uart_debugmode)
                    {
                        ROS_INFO("Delimiter Found");
                        delimiter_flag = true;
                    }
                    break;
                }
                delimiter_flag = false;
                uart_payload[uart_counter] = recChar;    
                uart_counter++;
            //ROS_INFO("parser while checked");
              }
    }

   void logic_controller() {
        std_msgs::Int32MultiArray data_array;
        actionlib_msgs::GoalID empty_goal;
        geometry_msgs::Twist force_stop_vel;
        dynamic_reconfigure::Config conf;
        force_stop_vel.linear.x = 0.0;
        force_stop_vel.angular.z = 0.0;

		if (uart_payload > 0 && delimiter_flag == true) {
			for (int i = 1; i < uart_counter / 2 + 1; i++) {
				data_array.data.push_back((uart_payload[2 * i - 2] - '0') * (int)10 + (uart_payload[2 * i - 1] - '0'));
				//data_array.data[i] = ec.Uart_payload[i];
				//ROS_INFO("parser pushback checked");
			}
			if (param_update_check && !data_array.data.empty()) { //should consider of nh::hasParam
			//ROS_INFO("serial param checked");
			// double mod_robot_x_vel;
			// double mod_robot_acc_x;
				if (data_array.data[4] == 0 && (data_array.data[5] == 0 || data_array.data[5] == 3 || data_array.data[5] == 4)) {
					if (data_array.data[0] == 0 && data_array.data[1] != 0) { // #4 deceleration/rotation
                        hash_4++;
					}
					else if (data_array.data[0] == 0 && data_array.data[1] == 0) { // #2 deceleration
						hash_2++;
					}
				}
			if (data_array.data[4] == 0 && (data_array.data[5] == 1 || data_array.data[5] == 2)) {
					if (data_array.data[5] == 1) { 
						if (data_array.data[0] == 0 && data_array.data[2] == 1) { // #1 normal
							hash_1++;
						}
						else if(data_array.data[0] == 0 && data_array.data[2] == 0){ // #3 acceleration
                            hash_3++;
						}
					}
					else if (data_array.data[5] == 2 && data_array.data[0] == 0 && (data_array.data[3] == 1 && data_array.data[1] != 0)) { //#5 e_brake
                            hash_5++;
					}
					else if (data_array.data[0] == 0 && data_array.data[2] == 1) { //#1 normal
							hash_1++;
					}
				}
			}
                  double current_hash_time = ros::Time::now().toSec();
                  if ((current_hash_time-hash_time_check) >= hash_interval)
                  {
                      // ROS_INFO("%f", (ros::Time::now().toNSec()-ec.hash_check)/1000000.0);
                      if (hash_1 >= hash_counter_check) {
                          if (set_param(default_robot_vel_x, default_robot_acc_x, 1.0, 1.0)) ROS_INFO("Normal drive - set default param! - hash1");
                          ROS_INFO("hash_1 : %d", hash_1);
                          hash_status = 1;
                      }
                      else if (hash_2 >= hash_counter_check) {
                          if (set_param(default_robot_vel_x, default_robot_acc_x, 2.0, 2.0)) ROS_INFO("speed -50per acceleration -1x - hash2");
                           ROS_INFO("hash_2 : %d", hash_2);
                           hash_status = 2;
                      }
                      else if (hash_3 >= hash_counter_check) {
						if (set_param(default_robot_vel_x, default_robot_acc_x, 1.3, 1.1)) ROS_INFO("speed + 30per acceleration + 1x - hash3");
                           ROS_INFO("hash_3 : %d", hash_3);
                           hash_status = 3;
                      }
                      else if (hash_4 >= hash_counter_check) {
   						if (set_param(default_robot_vel_x, default_robot_acc_x, 0.0, 2.0)) ROS_INFO("speed -> 0 acceleration -1x - hash4");
                           ROS_INFO("hash_4 : %d", hash_4);
                           hash_status = 4; 
                           cmd_force_pub.publish(force_stop_vel);
				     	   move_base_force_cancle_pub.publish(empty_goal);                         
                      }
                      else if (hash_5 >= hash_counter_check) {
 						if (set_param(default_robot_vel_x, default_robot_acc_x, 0.0, 2.0)) ROS_INFO("speed -> 0 acceleration -2x - hash5");
                           ROS_INFO("hash_5 : %d", hash_5);
                           hash_status = 5; 
                           cmd_force_pub.publish(force_stop_vel);
				     	   move_base_force_cancle_pub.publish(empty_goal);                           
                      }
                      hash_time_check = current_hash_time;
                      hash_1 = 0;
                      hash_2 = 0;
                      hash_3 = 0;
                      hash_4 = 0;
                      hash_5 = 0;
                  }
            data_array.data.push_back(hash_status);
			read_pub.publish(data_array);
		}       
    }

    void free_uart_payload() {
        free(uart_payload);
    }

private:
	ros::Subscriber param_updates;
	ros::Publisher read_pub;
    ros::Publisher cmd_force_pub;
    ros::Publisher move_base_force_cancle_pub;
	uint8_t uart_counter;
	//uint16_t Uart_PAYLOAD_LEN;
	char* uart_payload;
	bool uart_debugmode;
	const char delimiter_char;
	bool delimiter_flag;
	bool ser_flag;
	double default_robot_vel_x;
	double default_robot_acc_x;
	bool param_update_flag;
	bool param_update_check;
	double param_update_time;
    int hash_1;
	int hash_2;
	int hash_3;
	int hash_4;
	int hash_5;
    int hash_status;
    int hash_interval;
    double hash_time_check;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "ETRI_serial_node");
	ros::NodeHandle nh;
    serial::Serial ser;
	const char del = '>';
	ETRI_COMM ec = ETRI_COMM(&nh, del);
    ec.set_param_update_check(true);
	//dynamic_reconfigure::Client<teb_local_planner::TebLocalPlannerReconfigureConfig> client("/move_base/TebLocalPlannerROS/");
	//dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig> client2("/move_base/local_costmap/inflation_layer/");
	//teb_local_planner::TebLocalPlannerReconfigureConfig teb_config;
	//costmap_2d::InflationPluginConfig costmap_config;
	try
	{
		ser.setPort("/dev/ETRI");
		ser.setBaudrate(115200);
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

	ros::Rate loop_rate(5);

	while (ros::ok()) {
		double current_time = ros::Time::now().toSec();
		ec.param_update_timer_check(current_time);
		ec.set_uart_counter(0);
		//if (!client.getCurrentConfiguration(teb_config)) ROS_INFO("waiting for costmap Configuration");
		// if ( ec.param_update_check && !ec.param_update_flag ) {
		// if (ros::param::get("/move_base/TebLocalPlannerROS/max_vel_x",default_robot_vel_x)) ROS_INFO("Param checked");
		// if (ros::param::get("/move_base/TebLocalPlannerROS/acc_lim_x",ec.default_robot_acc_x)) ROS_INFO("Param checked");
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
                ROS_INFO("Parsed data : %s", ec.r_uart_payload());
                ec.logic_controller();
                ser.flush();
                ec.free_uart_payload();
                //free(ec.uart_payload);
    	    }
    	}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}