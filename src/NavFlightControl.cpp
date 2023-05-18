#include <rclcpp/rclcpp.hpp>
//px4_msgs/msg/SensorOpticalFlow
#include "px4_msgs/msg/sensor_optical_flow.hpp"
#include "px4_msgs/msg/obstacle_distance.hpp"
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class NavFlightControl : public rclcpp::Node
{
    public:
        NavFlightControl() : Node("NavFlightControl")
        {
            distSensor = this -> create_publisher<ObstacleDistance>("/fmu/in/obstacle_distance",10);

            auto timerCallback = [this]() -> void
            {
                publishDistSensor(ObstacleDistance::MAV_DISTANCE_SENSOR_LASER,30,10);
            };
            timer_ = this->create_wall_timer(100ms, timerCallback);
        }
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<ObstacleDistance>::SharedPtr distSensor;
        rclcpp::Publisher<SensorOpticalFlow>::SharedPtr opticalSensor;

        void publishDistSensor(uint8_t sensorType, uint16_t maxDist, uint16_t minDist);


};
void NavFlightControl::publishDistSensor(uint8_t sensorType, uint16_t maxDist, uint16_t minDist)
{
    ObstacleDistance sensor;
    sensor.min_distance = minDist;
    sensor.max_distance = maxDist;
    sensor.sensor_type = sensorType;
    sensor.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    distSensor -> publish(sensor);
}

int main(int argc, char const *argv[])
{
    std::cout << "Starting..." << std::endl;
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<NavFlightControl>());
    rclcpp::shutdown();
    return 0;
}


