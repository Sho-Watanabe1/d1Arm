#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "../msg/PubServoInfo_.hpp"
#include "../msg/ArmString_.hpp"

#define TOPIC "current_servo_angle"
#define TOPIC1 "arm_Feedback"

using namespace unitree::robot;
using namespace unitree::common;

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int sock;
sockaddr_in pyAddr;

void Handler(const void* msg)
{
    const unitree_arm::msg::dds_::PubServoInfo_* pm = (const unitree_arm::msg::dds_::PubServoInfo_*)msg;
    float angles[7] = { pm->servo0_data_(), pm->servo1_data_(), pm->servo2_data_(),
                         pm->servo3_data_(), pm->servo4_data_(), pm->servo5_data_(), pm->servo6_data_() };
    sendto(sock, angles, sizeof(angles), 0, (sockaddr*)&pyAddr, sizeof(pyAddr));
    std::cout << "servo0_data:" << pm->servo0_data_() << ", servo1_data:" << pm->servo1_data_() << ", servo2_data:" << pm->servo2_data_()<< ", servo3_data:" << pm->servo3_data_()<< ", servo4_data:" << pm->servo4_data_()<< ", servo5_data:" << pm->servo5_data_()<< ", servo6_data:" << pm->servo6_data_() << std::endl;

}

void Handler1(const void* msg)
{
    const unitree_arm::msg::dds_::ArmString_* pm = (const unitree_arm::msg::dds_::ArmString_*)msg;

    std::cout << "armFeedback_data:" << pm->data_() << std::endl;
}

int main()
{
    ChannelFactory::Instance()->Init(0, "eth1");
    ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> subscriber(TOPIC);
    subscriber.InitChannel(Handler);

    ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> subscriber1(TOPIC1);
    subscriber1.InitChannel(Handler1);

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    pyAddr.sin_family = AF_INET;
    pyAddr.sin_port = htons(9999);
    inet_pton(AF_INET, "127.0.0.1", &pyAddr.sin_addr);

    while (true)
    {
        sleep(10);
    }

    return 0;
}
