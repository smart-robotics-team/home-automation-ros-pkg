// ROS message includes
#include "ros/ros.h"
#include <diagnostic_msgs/KeyValue.h>
#include <std_msgs/Float32.h>

/* protected region user include files on begin */
/* protected region user include files end */

class to_openhab_float_config
{
public:
    std::string key;
};

class to_openhab_float_data
{
// autogenerated: don't touch this class
public:
    //input data
    std_msgs::Float32 in_float;
    //output data
    diagnostic_msgs::KeyValue out_output;
    bool out_output_active;
};

class to_openhab_float_impl
{
    /* protected region user member variables on begin */
	to_openhab_float_config local_config;

	bool out_on;
	double output_value;
    /* protected region user member variables end */

public:
    to_openhab_float_impl() 
    {
        /* protected region user constructor on begin */
    	out_on = false;
    	output_value = 0.0;
        /* protected region user constructor end */
    }

    void configure(to_openhab_float_config config) 
    {
        /* protected region user configure on begin */
    	local_config = config;
        /* protected region user configure end */
    }

    void update(to_openhab_float_data &data, to_openhab_float_config config)
    {
        /* protected region user update on begin */
    	data.out_output_active = out_on;
		if(out_on)
		{
			data.out_output.key = local_config.key;
			std::ostringstream ss;
			ss << output_value;
			data.out_output.value = ss.str();
			out_on = false;
		}
        /* protected region user update end */
    }

    void topicCallback_float(const std_msgs::Float32::ConstPtr& msg)
    {
        /* protected region user implementation of subscribe callback for float on begin */
    	output_value = msg->data;
    	out_on = true;
        /* protected region user implementation of subscribe callback for float end */
    }



    /* protected region user additional functions on begin */
    /* protected region user additional functions end */
};
