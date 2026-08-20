#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include "../msg/ArmString_.hpp"

int main() {
    // 1. Setup UDP Socket Listener
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in servaddr{}, cliaddr{};
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(5005);
    bind(sockfd, (const sockaddr*)&servaddr, sizeof(servaddr));

    // 2. Setup Unitree DDS Publisher
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth1");
    unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher("rt/arm_Command");
    publisher.InitChannel();

    char buffer[1024];
    socklen_t len = sizeof(cliaddr);

    while (true) {
        // Block until Python sends data
        int n = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0, (sockaddr*)&cliaddr, &len);
        buffer[n] = '\0';

        // Publish string directly to DDS
        unitree_arm::msg::dds_::ArmString_ msg{};
        msg.data_() = std::string(buffer);
        publisher.Write(msg);
    }
    return 0;
}