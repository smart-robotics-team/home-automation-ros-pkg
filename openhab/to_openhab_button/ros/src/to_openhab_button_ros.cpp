// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <to_openhab_button/to_openhab_buttonConfig.h>

// ROS message includes
#include <diagnostic_msgs/KeyValue.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

// other includes
#include <to_openhab_button_common.cpp>


class to_openhab_button_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<to_openhab_button::to_openhab_buttonConfig> server;
    dynamic_reconfigure::Server<to_openhab_button::to_openhab_buttonConfig>::CallbackType f;

    ros::Publisher output_;
    ros::Subscriber emptyON_;
    ros::Subscriber emptyOFF_;
    ros::Subscriber bool_;

    to_openhab_button_data component_data_;
    to_openhab_button_config component_config_;
    to_openhab_button_impl component_implementation_;

    to_openhab_button_ros() : np_("~")
    {
        f = boost::bind(&to_openhab_button_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        output_ = n_.advertise<diagnostic_msgs::KeyValue>("output", 1);
        emptyON_ = n_.subscribe("emptyON", 1, &to_openhab_button_impl::topicCallback_emptyON, &component_implementation_);
        emptyOFF_ = n_.subscribe("emptyOFF", 1, &to_openhab_button_impl::topicCallback_emptyOFF, &component_implementation_);
        bool_ = n_.subscribe("bool", 1, &to_openhab_button_impl::topicCallback_bool, &component_implementation_);

        np_.param("key", component_config_.key, (std::string)"");
        np_.param("value_on", component_config_.value_on, (std::string)"");
        np_.param("value_off", component_config_.value_off, (std::string)"");
    }
    void topicCallback_emptyON(const std_msgs::Empty::ConstPtr& msg)
    {
        component_data_.in_emptyON = *msg;
    }
    void topicCallback_emptyOFF(const std_msgs::Empty::ConstPtr& msg)
    {
        component_data_.in_emptyOFF = *msg;
    }
    void topicCallback_bool(const std_msgs::Bool::ConstPtr& msg)
    {
        component_data_.in_bool = *msg;
    }

    void configure_callback(to_openhab_button::to_openhab_buttonConfig &config, uint32_t level)
    {
        component_config_.key = config.key;
        component_config_.value_on = config.value_on;
        component_config_.value_off = config.value_off;
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

    ros::init(argc, argv, "to_openhab_button");

    to_openhab_button_ros node;
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
