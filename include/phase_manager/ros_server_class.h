#ifndef ROS_SERVER_CLASS_H
#define ROS_SERVER_CLASS_H

#include <phase_manager/phase_manager.h>
#include <phase_manager/timeline.h>
#include <phase_manager/phase.h>
#include <phase_manager/msg/timeline.hpp>
#include <phase_manager/msg/timeline_array.hpp>
#include <rclcpp/rclcpp.hpp>

namespace HorizonPhases {

    class RosServerClass
    {
        public:

            typedef std::shared_ptr<RosServerClass> Ptr;
            RosServerClass(PhaseManager::Ptr pm);
            void run();


        private:

            void init_publishers();

            std::unique_ptr<rclcpp::Node> _nh;
            PhaseManager::Ptr _pm;

            std::unordered_map<std::string, Timeline::Ptr> _timelines;

            rclcpp::Publisher<phase_manager::msg::TimelineArray>::SharedPtr _timelines_pub;

    };

}
#endif // ROS_SERVER_CLASS_H
