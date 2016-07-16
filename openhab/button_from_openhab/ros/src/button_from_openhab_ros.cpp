// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <button_from_openhab/button_from_openhabConfig.h>

// ROS message includes
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <diagnostic_msgs/KeyValue.h>

// other includes
#include <button_from_openhab_common.cpp>


class button_from_openhab_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<button_from_openhab::button_from_openhabConfig> server;
    dynamic_reconfigure::Server<button_from_openhab::button_from_openhabConfig>::CallbackType f;

    ros::Publisher emptyON_;
    ros::Publisher emptyOFF_;
    ros::Publisher bool_;
    ros::Subscriber input_;

    button_from_openhab_data component_data_;
    button_from_openhab_config component_config_;
    button_from_openhab_impl component_implementation_;

    button_from_openhab_ros() : np_("~")
    {
        f = boost::bind(&button_from_openhab_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        emptyON_ = n_.advertise<std_msgs::Empty>("emptyON", 1);
        emptyOFF_ = n_.advertise<std_msgs::Empty>("emptyOFF", 1);
        bool_ = n_.advertise<std_msgs::Bool>("bool", 1);
        input_ = n_.subscribe("input", 1, &button_from_openhab_impl::topicCallback_input, &component_implementation_);

        np_.param("key", component_config_.key, (std::string)"");
        np_.param("value_on", component_config_.value_on, (std::string)"");
        np_.param("value_off", component_config_.value_off, (std::string)"");
    }
    void topicCallback_input(const diagnostic_msgs::KeyValue::ConstPtr& msg)
    {
        component_data_.in_input = *msg;
    }

    void configure_callback(button_from_openhab::button_from_openhabConfig &config, uint32_t level)
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
        component_data_.out_emptyON_active = true;
        component_data_.out_emptyOFF_active = true;
        component_data_.out_bool_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_emptyON_active)
            emptyON_.publish(component_data_.out_emptyON);
        if (component_data_.out_emptyOFF_active)
            emptyOFF_.publish(component_data_.out_emptyOFF);
        if (component_data_.out_bool_active)
            bool_.publish(component_data_.out_bool);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "button_from_openhab");

    button_from_openhab_ros node;
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
