// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <int32_from_openhab/int32_from_openhabConfig.h>

// ROS message includes
#include <std_msgs/Int32.h>
#include <diagnostic_msgs/KeyValue.h>

// other includes
#include <int32_from_openhab_common.cpp>


class int32_from_openhab_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<int32_from_openhab::int32_from_openhabConfig> server;
    dynamic_reconfigure::Server<int32_from_openhab::int32_from_openhabConfig>::CallbackType f;

    ros::Publisher output_;
    ros::Subscriber input_;

    int32_from_openhab_data component_data_;
    int32_from_openhab_config component_config_;
    int32_from_openhab_impl component_implementation_;

    int32_from_openhab_ros() : np_("~")
    {
        f = boost::bind(&int32_from_openhab_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        output_ = n_.advertise<std_msgs::Int32>("output", 1);
        input_ = n_.subscribe("input", 1, &int32_from_openhab_impl::topicCallback_input, &component_implementation_);

        np_.param("key", component_config_.key, (std::string)"");
    }
    void topicCallback_input(const diagnostic_msgs::KeyValue::ConstPtr& msg)
    {
        component_data_.in_input = *msg;
    }

    void configure_callback(int32_from_openhab::int32_from_openhabConfig &config, uint32_t level)
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

    ros::init(argc, argv, "int32_from_openhab");

    int32_from_openhab_ros node;
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
