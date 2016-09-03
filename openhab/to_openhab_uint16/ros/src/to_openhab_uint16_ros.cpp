// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <to_openhab_uint16/to_openhab_uint16Config.h>

// ROS message includes
#include <diagnostic_msgs/KeyValue.h>
#include <std_msgs/UInt16.h>

// other includes
#include <to_openhab_uint16_common.cpp>


class to_openhab_uint16_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<to_openhab_uint16::to_openhab_uint16Config> server;
    dynamic_reconfigure::Server<to_openhab_uint16::to_openhab_uint16Config>::CallbackType f;

    ros::Publisher output_;
    ros::Subscriber input_;

    to_openhab_uint16_data component_data_;
    to_openhab_uint16_config component_config_;
    to_openhab_uint16_impl component_implementation_;

    to_openhab_uint16_ros() : np_("~")
    {
        f = boost::bind(&to_openhab_uint16_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        output_ = n_.advertise<diagnostic_msgs::KeyValue>("output", 1);
        input_ = n_.subscribe("input", 1, &to_openhab_uint16_impl::topicCallback_input, &component_implementation_);

        np_.param("key", component_config_.key, (std::string)"");
    }
    void topicCallback_input(const std_msgs::UInt16::ConstPtr& msg)
    {
        component_data_.in_input = *msg;
    }

    void configure_callback(to_openhab_uint16::to_openhab_uint16Config &config, uint32_t level)
    {
        component_config_.key = config.key;
        configure();
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_output_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_output_active)
            output_.publish(component_data_.out_output);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "to_openhab_uint16");

    to_openhab_uint16_ros node;
    node.configure();

    ros::Rate loop_rate(50.0);

    while(node.n_.ok())
    {
        node.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
