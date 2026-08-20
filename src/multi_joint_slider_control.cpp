#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

int main(int argc, char** argv)
{
    std::vector<double> angles;
    for (int i = 1; i < argc; ++i) {
        angles.push_back(std::stod(argv[i]));
    }

    if (angles.size() != 7) {
        std::string line;
        while (std::getline(std::cin, line)) {
            std::stringstream stream(line);
            std::vector<double> streamed_angles;
            double value = 0.0;
            while (stream >> value) {
                streamed_angles.push_back(value);
            }

            if (streamed_angles.size() != 7) {
                continue;
            }

            angles = streamed_angles;
            break;
        }
    }

    if (angles.size() != 7) {
        std::cerr << "Usage: multi_joint_slider_control <angle0> <angle1> <angle2> <angle3> <angle4> <angle5> <angle6>" << std::endl;
        return 1;
    }

    ChannelFactory::Instance()->Init(0, "eth1");
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    while (true) {
        std::string line;
        if (!std::getline(std::cin, line)) {
            break;
        }

        std::stringstream stream(line);
        std::vector<double> streamed_angles;
        double value = 0.0;
        while (stream >> value) {
            streamed_angles.push_back(value);
        }

        if (streamed_angles.size() != 7) {
            continue;
        }

        angles = streamed_angles;

        unitree_arm::msg::dds_::ArmString_ msg{};
        msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,\"angle0\":" + std::to_string(static_cast<int>(angles[0])) +
                      ",\"angle1\":" + std::to_string(static_cast<int>(angles[1])) +
                      ",\"angle2\":" + std::to_string(static_cast<int>(angles[2])) +
                      ",\"angle3\":" + std::to_string(static_cast<int>(angles[3])) +
                      ",\"angle4\":" + std::to_string(static_cast<int>(angles[4])) +
                      ",\"angle5\":" + std::to_string(static_cast<int>(angles[5])) +
                      ",\"angle6\":" + std::to_string(static_cast<int>(angles[6])) + "}}";
        publisher.Write(msg);
    }

    return 0;
}
