/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2018 PX4 Pro Development Team
 * Copyright 2020-2022 DronePort Development Team, Uniwersity of West Bohemia, Pilsen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <queue>
#include <vector>
#include <regex>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <cassert>
#include <stdexcept>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>
#include <boost/system/system_error.hpp>

#include <iostream>
#include <random>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>

#include <Eigen/Eigen>
#include<Eigen/StdVector>

#include <mavlink/v2.0/common/mavlink.h>

static const uint32_t DEFAULT_MAVLINK_UDP_PORT = 14560;
static const uint32_t DEFAULT_MAVLINK_TCP_PORT = 4560;

class DroneportMavlinkInterface
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DroneportMavlinkInterface();
    ~DroneportMavlinkInterface();

    void getMessages(std::queue<mavlink_message_t> *messages);
    void sendMessage(const mavlink_message_t *message);
    void open();
    void close();
    void Load();
    void onSigInt();
    inline void SetMavlinkAddr(std::string mavlink_addr) { mavlink_addr_str_ = mavlink_addr; }
    inline void SetMavlinkTcpPort(int mavlink_tcp_port) { mavlink_tcp_port_ = mavlink_tcp_port; }
    inline void SetMavlinkUdpPort(int mavlink_udp_port) { mavlink_udp_port_ = mavlink_udp_port; }
    inline void SetUseTcp(bool mavlink_use_tcp) {use_tcp_ = mavlink_use_tcp; }
private:
    void acceptConnections();

    unsigned char buf_[65535];
    enum FD_TYPES {
        LISTEN_FD,
        CONNECTION_FD,
        N_FDS
    };
    struct pollfd fds_[N_FDS];
    bool use_tcp_{false};
    bool close_conn_{false};

    in_addr_t mavlink_addr_;
    std::string mavlink_addr_str_{"INADDR_ANY"};
    int mavlink_udp_port_{DEFAULT_MAVLINK_UDP_PORT}; // MAVLink refers to the PX4 simulator interface here
    int mavlink_tcp_port_{DEFAULT_MAVLINK_TCP_PORT}; // MAVLink refers to the PX4 simulator interface here

    struct sockaddr_in local_simulator_addr_;
    socklen_t local_simulator_addr_len_;
    struct sockaddr_in remote_simulator_addr_;
    socklen_t remote_simulator_addr_len_;

    int simulator_socket_fd_{0};
    int simulator_tcp_client_fd_{0};

    mavlink_status_t m_status_{};
    mavlink_message_t m_buffer_{};

    std::atomic<bool> gotSigInt_ {false};    
};
