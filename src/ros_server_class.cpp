#include <phase_manager/ros_server_class.h>


using namespace HorizonPhases;


RosServerClass::RosServerClass(PhaseManager::Ptr pm):
    _pm(pm)

{
    _timelines = _pm->getTimelines();


    if (!rclcpp::ok())
    {
        std::cout << "Ros not initialized. Initializing." << std::endl;
        int argc = 0;
        char ** argv = nullptr;
        rclcpp::init(argc, argv);
    }

    auto _nh = std::make_unique<rclcpp::Node>("phase_manager_ros_server_class");
    _timelines_pub = _nh->create_publisher<phase_manager::msg::TimelineArray>("phase_manager/timelines", 10);
    // init_publishers();
}


void RosServerClass::init_publishers()
{

    // open publisher
    std::cout << "Opening topic for phase manager." << std::endl;
    _timelines_pub = _nh->create_publisher<phase_manager::msg::TimelineArray>("phase_manager/timelines", 10);

}

void RosServerClass::run()
{

    phase_manager::msg::TimelineArray timelines_msg;

    for (auto pair : _timelines)
    {
        phase_manager::msg::Timeline timeline_msg;

        timeline_msg.name = pair.first;
        auto phases = pair.second->getPhases();

        std::vector<std::string> phase_names;
        std::vector<int> phase_initial_nodes;
        std::vector<int> phase_durations;

        for (auto phase : phases)
        {
            phase_names.push_back(phase->getName());
            phase_initial_nodes.push_back(phase->getPosition());
            phase_durations.push_back(phase->getNNodes());
        }

        timeline_msg.phases = phase_names;
        timeline_msg.initial_nodes = phase_initial_nodes;
        timeline_msg.durations = phase_durations;

        timelines_msg.timelines.push_back(timeline_msg);
    }

    _timelines_pub->publish(timelines_msg);

}
