// ROS message includes
#include "ros/ros.h"
#include <diagnostic_msgs/KeyValue.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

/* protected region user include files on begin */
/* protected region user include files end */

class to_openhab_button_config
{
public:
    std::string key;
    std::string value_on;
    std::string value_off;
};

class to_openhab_button_data
{
// autogenerated: don't touch this class
public:
    //input data
    std_msgs::Empty in_emptyON;
    std_msgs::Empty in_emptyOFF;
    std_msgs::Bool in_bool;
    //output data
    diagnostic_msgs::KeyValue out_output;
    bool out_output_active;
};

class to_openhab_button_impl
{
    /* protected region user member variables on begin */
	to_openhab_button_config local_config;

	bool out_on;
	bool out_off;
    /* protected region user member variables end */

public:
    to_openhab_button_impl() 
    {
        /* protected region user constructor on begin */
    	out_on = false;
    	out_off = false;
        /* protected region user constructor end */
    }

    void configure(to_openhab_button_config config) 
    {
        /* protected region user configure on begin */
    	local_config = config;
        /* protected region user configure end */
    }

    void update(to_openhab_button_data &data, to_openhab_button_config config)
    {
        /* protected region user update on begin */
    	data.out_output_active = out_on | out_off;
    	if(data.out_output_active)
    	{
    		data.out_output.key = local_config.key;
    		data.out_output.value = out_on ? local_config.value_on : local_config.value_off;
    		out_on = false;
    		out_off = false;
    	}
        /* protected region user update end */
    }

    void topicCallback_emptyON(const std_msgs::Empty::ConstPtr& msg)
    {
        /* protected region user implementation of subscribe callback for emptyON on begin */
    	out_on = true;
        /* protected region user implementation of subscribe callback for emptyON end */
    }
    void topicCallback_emptyOFF(const std_msgs::Empty::ConstPtr& msg)
    {
        /* protected region user implementation of subscribe callback for emptyOFF on begin */
    	out_off = true;
        /* protected region user implementation of subscribe callback for emptyOFF end */
    }
    void topicCallback_bool(const std_msgs::Bool::ConstPtr& msg)
    {
        /* protected region user implementation of subscribe callback for bool on begin */
    	if(msg->data)
    	{
    		out_on = true;
    	}
    	else
    	{
    		out_off = true;
    	}
        /* protected region user implementation of subscribe callback for bool end */
    }



    /* protected region user additional functions on begin */
    /* protected region user additional functions end */
};
