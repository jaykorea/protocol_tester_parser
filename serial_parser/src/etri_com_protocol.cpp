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
#include <move_base/move_base.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Char.h>
#include <std_msgs/Empty.h>
#include "geometry_msgs/Twist.h"
#include "actionlib_msgs/GoalStatusArray.h"

class ETRI_COMM {
    public:
        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::DoubleParameter double_param_x_vel;
        dynamic_reconfigure::DoubleParameter double_param_x_acc;
        dynamic_reconfigure::Config conf;
        ETRI_COMM(ros::NodeHandle *nh)
        {
        }
        uint8_t Uart_counter = 0;
	    //uint16_t Uart_PAYLOAD_LEN;
	    char* Uart_payload;
	    bool Uart_debugMode = true;
        const char delimiter_char = '>';
        bool delimiter_flag = false;
        bool ser_flag =false;
        double default_robot_vel_x;
        double default_robot_acc_x;
        bool param_check;

    bool set_param(dynamic_reconfigure::Config conf, double default_robot_x_vel, double default_robot_x_acc, float vel_per, float acc_per) {
        double_param_x_vel.name = "max_vel_x";
        double_param_x_vel.value = default_robot_x_vel / vel_per;
        conf.doubles.push_back(double_param_x_vel);

        double_param_x_acc.name = "acc_lim_x";
        double_param_x_acc.value = default_robot_x_acc / acc_per;
        conf.doubles.push_back(double_param_x_acc);
        srv_req.config = conf;

        if (ros::service::call("/move_base/TebLocalPlannerROS/set_parameters", srv_req, srv_resp)) return true;
        else return false;
    }

    // void set_max_x_vel(double default_robot_x_vel, float per) {
    //     double_param_x_vel.name = "max_vel_x";
    //     double_param_x_vel.value = default_robot_x_vel / per;
    //     conf.doubles.push_back(double_param_x_vel);
    //     srv_req.config = conf;
    //     ros::service::call("/move_base/TebLocalPlannerROS/set_parameters", srv_req, srv_resp);
    // }

    // void set_max_x_acc(double default_robot_x_acc, float per) {
        // double_param_x_acc.name = "acc_limit_x";
        // double_param_x_acc.value = default_robot_x_acc / acc_per;
        // conf.doubles.push_back(double_param_x_acc);
        // srv_req.config = conf;
    //     ros::service::call("/move_base/TebLocalPlannerROS/set_parameters", srv_req, srv_resp);
    // }


    double millis() {
        return ros::Time::now().toNSec();
    }

    private:
};

int main (int argc, char** argv){
    ros::init(argc, argv, "ETRI_serial_node");
    ros::NodeHandle nh;
    ros::Publisher read_pub = nh.advertise<std_msgs::Int32MultiArray>("read_pub", 100);
    ros::Publisher cmd_force_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::Publisher move_base_force_cancle_pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 100);
    serial::Serial ser;
    actionlib_msgs::GoalID empty_goal;
    geometry_msgs::Twist force_stop_vel;
    force_stop_vel.linear.x = 0.0;
    force_stop_vel.angular.z = 0.0;
    ETRI_COMM ec = ETRI_COMM(&nh);
    ec.param_check = true;

    dynamic_reconfigure::Client<teb_local_planner::TebLocalPlannerReconfigureConfig> client("/move_base/TebLocalPlannerROS/");
    dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig> client2("/move_base/local_costmap/inflation_layer/");

    teb_local_planner::TebLocalPlannerReconfigureConfig teb_config;
    costmap_2d::InflationPluginConfig costmap_config;
    //ROS_INFO_STREAM("config") << teb_config;
 
    try
    {
        ser.setPort("/dev/ETRI2");
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

    if(ser.isOpen()){
        ROS_INFO("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);

    while(ros::ok()){

        ec.Uart_counter = 0;
        std_msgs::Int32MultiArray data_array;
        // if (client.getCurrentConfiguration(teb_config)) ROS_INFO("Got Teb Configuration");
        if ((ros::param::has("/move_base/TebLocalPlannerROS/max_vel_x") && ros::param::has("/move_base/TebLocalPlannerROS/acc_lim_x")) && ec.param_check) { 
            if (ros::param::get("/move_base/TebLocalPlannerROS/max_vel_x",ec.default_robot_vel_x)) ROS_INFO("Param checked");
            if (ros::param::get("/move_base/TebLocalPlannerROS/acc_lim_x",ec.default_robot_acc_x)) ROS_INFO("Param checked");
            //ROS_INFO("Param checked");
            ec.param_check = false;
        }

        // if (ser.write("at+qd1?\r\n")) ec.ser_flag =true;
        // else ec.ser_flag = false;
                                         
        if (ser.write("at+qd1?\r\n")) { // <<-in if ser.available()
            //ROS_INFO("serial write checked");  
            if(ser.available()) {
            //ROS_INFO("serial available checked");  
            std::string input_serial = ser.read(ser.available());
            ec.Uart_payload = (char*)malloc(input_serial.length() + 1);
            
            //memset(ec.Uart_payload, '\0', ec.Uart_PAYLOAD_LEN + 1);
            while (ec.Uart_counter < input_serial.length()+1 ){

                char recChar;
                recChar = input_serial[ec.Uart_counter];

                if (recChar == ec.delimiter_char)
                {
                    if(ec.Uart_debugMode)
                    {
                        ROS_INFO("Delimiter Found");
                        ec.delimiter_flag = true;
                    }
                    break;
                }
            ec.delimiter_flag = false;
            ec.Uart_payload[ec.Uart_counter] = recChar;    
            ec.Uart_counter++;
            //ROS_INFO("parser while checked");
        }
            ROS_INFO("Parsed data : %s", ec.Uart_payload);

        if (ec.Uart_payload > 0 && ec.delimiter_flag == true) {
            for (int i=1; i<ec.Uart_counter/2+1; i++) {
                data_array.data.push_back((ec.Uart_payload[2*i-2]-'0')*(int)10 + (ec.Uart_payload[2*i-1]-'0'));
                //data_array.data[i] = ec.Uart_payload[i];
                //ROS_INFO("parser pushback checked");
            }
            if (!ec.param_check && !data_array.data.empty() && data_array.data.size() > ec.Uart_counter/2 ) { //should consider of nh::hasParam
            //ROS_INFO("serial param checked");
                // double mod_robot_x_vel;
                // double mod_robot_acc_x;
                if (data_array.data[3] >= 50) /*if value is over threshold it activates*/ {
                    if(ec.set_param(ec.conf, ec.default_robot_vel_x, ec.default_robot_acc_x, 2.0, 2.0)) ROS_INFO("speed -50per acceleration -1x");
                    // mod_robot_x_vel = ec.default_robot_x_vel/(double)2.0;
                    // mod_robot_acc_x = ec.default_robot_acc_x/(double)2.0;
                    // ros::param::set("/move_base/TebLocalPlannerROS//max_vel_x", mod_robot_x_vel);
                    // ros::param::set("/move_base/TebLocalPlannerROS//max_vel_theta", mod_robot_acc_x);
                    // speed - 50%
                    // acceleration - 1x
                }
                else if (data_array.data[0] >= 95) {
                    if(ec.set_param(ec.conf,ec.default_robot_vel_x, ec.default_robot_acc_x, 1.3, 1.1)) ROS_INFO("speed + 30per acceleration + 1x");
                    // mod_robot_x_vel = ec.default_robot_vel_x*1.3;
                    // mod_robot_acc_x = ec.default_robot_acc_x*1.1;
                    // ros::param::set("/move_base/TebLocalPlannerROS/max_vel_x", mod_robot_x_vel);
                    // ros::param::set("/move_base/TebLocalPlannerROS/max_vel_theta", mod_robot_acc_x);
                    // speed + 30%
                    // accelration + 1x
                }
                else if (data_array.data[2] >= 95 && data_array.data[4] >= 95) {
                    if(ec.set_param(ec.conf,ec.default_robot_vel_x, ec.default_robot_acc_x, 0.0, 2.0)) ROS_INFO("speed -> 0  acceleration -1x");
                    // mod_robot_x_vel = ec.default_robot_vel_x/0.0; //should be just 0.0?
                    // mod_robot_acc_x = ec.default_robot_acc_x/2.0;
                    // ros::param::set("/move_base/TebLocalPlannerROS/max_vel_x", mod_robot_x_vel);
                    // ros::param::set("/move_base/TebLocalPlannerROS/max_vel_theta", mod_robot_acc_x);   
                    // spped - 0
                    // acceleration - 1x
                    cmd_force_pub.publish(force_stop_vel);
                    move_base_force_cancle_pub.publish(empty_goal);
                }
                else if (data_array.data[4] >= 95 && data_array.data[6] >=95) {
                    if(ec.set_param(ec.conf,ec.default_robot_vel_x, ec.default_robot_acc_x, 0.0, 2.0)) ROS_INFO("speed -> 0  acceleration -2x");
                    // mod_robot_x_vel = ec.default_robot_vel_x/0.0;
                    // mod_robot_acc_x = ec.default_robot_acc_x/2.0;
                    // ros::param::set("/move_base/TebLocalPlannerROS/max_vel_x", mod_robot_x_vel);
                    // ros::param::set("/move_base/TebLocalPlannerROS/max_vel_theta", mod_robot_acc_x);   
                    // spped - 0
                    // acceleration - 2x
                    cmd_force_pub.publish(force_stop_vel);
                    move_base_force_cancle_pub.publish(empty_goal);
                }
                else {
                    if(ec.set_param(ec.conf,ec.default_robot_vel_x, ec.default_robot_acc_x, 1.0, 1.0)) ROS_INFO("set default param!");
                }
            }
                read_pub.publish(data_array);
        }
            ser.flush();
            free(ec.Uart_payload);
        }
    }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
