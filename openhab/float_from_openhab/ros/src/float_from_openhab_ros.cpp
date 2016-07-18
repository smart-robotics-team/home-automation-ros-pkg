// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <float_from_openhab/float_from_openhabConfig.h>

// ROS message includes
#include <std_msgs/Float32.h>
#include <diagnostic_msgs/KeyValue.h>

// other includes
#include <float_from_openhab_common.cpp>


class float_from_openhab_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<float_from_openhab::float_from_openhabConfig> server;
    dynamic_reconfigure::Server<float_from_openhab::float_from_openhabConfig>::CallbackType f;

    ros::Publisher float_;
    ros::Subscriber input_;

    float_from_openhab_data component_data_;
    float_from_openhab_config component_config_;
    float_from_openhab_impl component_implementation_;

    float_from_openhab_ros() : np_("~")
    {
        f = boost::bind(&float_from_openhab_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        float_ = n_.advertise<std_msgs::Float32>("float", 1);
        input_ = n_.subscribe("input", 1, &float_from_openhab_impl::topicCallback_input, &component_implementation_);

        np_.param("key", component_config_.key, (std::string)"");
    }
    void topicCallback_input(const diagnostic_msgs::KeyValue::ConstPtr& msg)
    {
        component_data_.in_input = *msg;
    }

    void configure_callback(float_from_openhab::float_from_openhabConfig &config, uint32_t level)
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
        component_data_.out_float_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_float_active)
            float_.publish(component_data_.out_float);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "float_from_openhab");

    float_from_openhab_ros node;
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
