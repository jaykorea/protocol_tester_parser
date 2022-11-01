/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Char.h>
#include <std_msgs/Empty.h>

class ETRI_COMM {
    public:
        ETRI_COMM(ros::NodeHandle *n)
        {
        }
        uint8_t Uart_counter = 0;
	    //uint16_t Uart_PAYLOAD_LEN;
	    char* Uart_payload;
	    bool Uart_debugMode = true;
        const char delimiter_char = '>';
        bool delimiter_flag = false;

    double millis() {
        return ros::Time::now().toNSec();
    }

    private:
};

int main (int argc, char** argv){
    ros::init(argc, argv, "ETRI_serial_node");
    ros::NodeHandle nh;
    ros::Publisher read_pub = nh.advertise<std_msgs::Int32MultiArray>("read_pub", 1000);
    serial::Serial ser;

    ETRI_COMM ec = ETRI_COMM(&nh);

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

    if(ser.isOpen()){
        ROS_INFO("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();
        ec.Uart_counter = 0;
        std_msgs::Int32MultiArray data_array;
        //ser.write("at+qd1?\r\n");

        if(ser.available()) {
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
        }
            ROS_INFO("Parsed data : %s", ec.Uart_payload);
        if (ec.Uart_payload >0 && ec.delimiter_flag == true) {
            for (int i=1; i<ec.Uart_counter/2+1; i++) {
                data_array.data.push_back((ec.Uart_payload[2*i-2]-'0')*(int)10 + (ec.Uart_payload[2*i-1]-'0'));
                //data_array.data[i] = ec.Uart_payload[i];
            }
            read_pub.publish(data_array);
        }
            ser.flush();
            free(ec.Uart_payload);
        }
        loop_rate.sleep();
    }
    return 0;
}
