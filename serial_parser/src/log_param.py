import rospy
import rosparam

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('parameter_printer')

    # Loop until rospy is shutdown
    while not rospy.is_shutdown():
        # Retrieve the parameter values
        try:
            speed_lim_v = rospy.get_param('/velocity_smoother_nav/speed_lim_v')
            accel_lim_v = rospy.get_param('/velocity_smoother_nav/accel_lim_v')

            # Print the parameter values
            print("speed_lim_v:", speed_lim_v)
            print("accel_lim_v:", accel_lim_v)

        except rospy.ROSException as e:
            print("Failed to retrieve parameters:", str(e))
        
        # Sleep for a short duration before checking again
        rospy.sleep(0.05)

