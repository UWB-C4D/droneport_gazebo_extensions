/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2018 PX4 Development Team
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

#include "droneport_mavlink_interface.h"

DroneportMavlinkInterface::DroneportMavlinkInterface() {}

DroneportMavlinkInterface::~DroneportMavlinkInterface()
{
    close();
}

void DroneportMavlinkInterface::Load()
{
    mavlink_addr_ = htonl(INADDR_ANY);
    if (mavlink_addr_str_ != "INADDR_ANY")
    {
        mavlink_addr_ = inet_addr(mavlink_addr_str_.c_str());
        if (mavlink_addr_ == INADDR_NONE)
        {
            std::cerr << "Invalid mavlink_addr: " << mavlink_addr_str_ << ", aborting\n";
            abort();
        }
    }

    memset((char *)&remote_simulator_addr_, 0, sizeof(remote_simulator_addr_));
    remote_simulator_addr_.sin_family = AF_INET;
    remote_simulator_addr_len_ = sizeof(remote_simulator_addr_);

    memset((char *)&local_simulator_addr_, 0, sizeof(local_simulator_addr_));
    local_simulator_addr_.sin_family = AF_INET;
    local_simulator_addr_len_ = sizeof(local_simulator_addr_);

    if (use_tcp_)
    {

        local_simulator_addr_.sin_addr.s_addr = htonl(mavlink_addr_);
        local_simulator_addr_.sin_port = htons(mavlink_tcp_port_);

        if ((simulator_socket_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
            std::cerr << "Creating TCP socket failed: " << strerror(errno) << ", aborting\n";
            abort();
        }

        int yes = 1;
        int result = setsockopt(simulator_socket_fd_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
        if (result != 0)
        {
            std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
            abort();
        }

        struct linger nolinger
        {
        };
        nolinger.l_onoff = 1;
        nolinger.l_linger = 0;

        result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_LINGER, &nolinger, sizeof(nolinger));
        if (result != 0)
        {
            std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
            abort();
        }

        int socket_reuse = 1;
        result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &socket_reuse, sizeof(socket_reuse));
        if (result != 0)
        {
            std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
            abort();
        }

        // Same as above but for a given port
        result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEPORT, &socket_reuse, sizeof(socket_reuse));
        if (result != 0)
        {
            std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
            abort();
        }

        result = fcntl(simulator_socket_fd_, F_SETFL, O_NONBLOCK);
        if (result == -1)
        {
            std::cerr << "setting socket to non-blocking failed: " << strerror(errno) << ", aborting\n";
            abort();
        }

        if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0)
        {
            std::cerr << "bind failed: " << strerror(errno) << ", aborting\n";
            abort();
        }

        errno = 0;
        if (listen(simulator_socket_fd_, 0) < 0)
        {
            std::cerr << "listen failed: " << strerror(errno) << ", aborting\n";
            abort();
        }

        memset(fds_, 0, sizeof(fds_));
        fds_[LISTEN_FD].fd = simulator_socket_fd_;
        fds_[LISTEN_FD].events = POLLIN; // only listens for new connections on tcp
    }
    else
    {
        remote_simulator_addr_.sin_addr.s_addr = mavlink_addr_;
        //remote_simulator_addr_.sin_port = htons(mavlink_udp_port_);
        remote_simulator_addr_.sin_port = htons(mavlink_udp_port_);

        local_simulator_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
        //local_simulator_addr_.sin_port = htons(0);
        local_simulator_addr_.sin_port = htons(mavlink_udp_port_);

        int socket_reuse = 1;
   /*     int result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &socket_reuse, sizeof(socket_reuse));
        if (result != 0)
        {
            std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
            abort();
        }
*/
        if ((simulator_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        {
            std::cerr << "Creating UDP socket failed: " << strerror(errno) << ", aborting\n";
            abort();
        }

        // set socket to non-blocking
        int result = fcntl(simulator_socket_fd_, F_SETFL, O_NONBLOCK);
        if (result == -1)
        {
            std::cerr << "setting socket to non-blocking failed: " << strerror(errno) << ", aborting\n";
            abort();
        }

        // Same as above but for a given port
        result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEPORT, &socket_reuse, sizeof(socket_reuse));
        if (result != 0)
        {
            std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
            abort();
        }        

        if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0)
        {
            std::cerr << "bind failed: " << strerror(errno) << ", aborting\n";
            abort();
        }

        memset(fds_, 0, sizeof(fds_));
        fds_[CONNECTION_FD].fd = simulator_socket_fd_;
        fds_[CONNECTION_FD].events = POLLIN | POLLOUT; // read/write
    }
}

void DroneportMavlinkInterface::getMessages(std::queue<mavlink_message_t> *messages)
{
    if (gotSigInt_)
    {
        return;
    }

    int timeout_ms = 5000;
    int ret = ::poll(&fds_[0], N_FDS, timeout_ms);
    if (ret < 0)
    {
        std::cerr << "poll error: " << strerror(errno) << "\n";
        return;
    }

    if (ret == 0 && timeout_ms > 0)
    {
        std::cerr << "poll timeout\n";
        return;
    }

    for (int i = 0; i < N_FDS; i++)
    {
        if (fds_[i].revents == 0)
        {
            continue;
        }
        if (!(fds_[i].revents & POLLIN))
        {
            continue;
        }

        if (i == LISTEN_FD)
        { // if event is raised on the listening socket
            acceptConnections();
        }
        else
        { // if event is raised on connection socket
            int ret = recvfrom(fds_[i].fd, buf_, sizeof(buf_), 0, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);
            if (ret < 0)
            {
                // all data is read if EWOULDBLOCK is raised
                if (errno != EWOULDBLOCK)
                { // disconnected from client
                    std::cerr << "recvfrom error: " << strerror(errno) << "\n";
                }
                continue;
            }

            // client closed the connection orderly, only makes sense on tcp
            if (use_tcp_ && ret == 0)
            {
                std::cerr << "Connection closed by client."
                          << "\n";
                close_conn_ = true;
                continue;
            }

            // data received
            int len = ret;
            mavlink_message_t msg;
            mavlink_status_t status;
            for (unsigned i = 0; i < len; ++i)
            {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf_[i], &msg, &status))
                {
                    messages->push(msg);
                }
            }
        }
        return;
    }
}

void DroneportMavlinkInterface::acceptConnections()
{

    std::cout << "ACCEPT CONNECTION" << std::endl;
    if (fds_[CONNECTION_FD].fd > 0)
    {
        return;
    }

    // accepting incoming connections on listen fd
    int ret =
        accept(fds_[LISTEN_FD].fd, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);

    if (ret < 0)
    {
        if (errno != EWOULDBLOCK)
        {
            std::cerr << "accept error: " << strerror(errno) << "\n";
        }
        return;
    }

    // assign socket to connection descriptor on success
    fds_[CONNECTION_FD].fd = ret;                  // socket is replaced with latest connection
    fds_[CONNECTION_FD].events = POLLIN | POLLOUT; // read/write
}

void DroneportMavlinkInterface::sendMessage(const mavlink_message_t *message)
{
    assert(message != nullptr);

    if (gotSigInt_ || close_conn_)
    {
        return;
    }

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen = mavlink_msg_to_send_buffer(buffer, message);

    if (fds_[CONNECTION_FD].fd > 0)
    {
        int timeout_ms = 0;
        int ret = ::poll(&fds_[0], N_FDS, timeout_ms);

        if (ret < 0)
        {
            std::cerr << "poll error: " << strerror(errno) << "\n";
            return;
        }

        if (ret == 0 && timeout_ms > 0)
        {
            std::cerr << "poll timeout\n";
            return;
        }

        if (!(fds_[CONNECTION_FD].revents & POLLOUT))
        {
            std::cerr << "invalid events at fd:" << fds_[CONNECTION_FD].revents << "\n";
            return;
        }

        ssize_t len;
        if (use_tcp_)
        {
            len = send(fds_[CONNECTION_FD].fd, buffer, packetlen, 0);
        }
        else
        {
            len = sendto(fds_[CONNECTION_FD].fd, buffer, packetlen, 0, (struct sockaddr *)&remote_simulator_addr_, remote_simulator_addr_len_);
        }
        if (len < 0)
        {
            std::cerr << "Failed sending mavlink message: " << strerror(errno) << "\n";
            if (errno == ECONNRESET || errno == EPIPE)
            {
                if (use_tcp_)
                { // udp socket remains alive
                    std::cerr << "Closing connection."
                              << "\n";
                    close_conn_ = true;
                }
            }
        }
    }
}

void DroneportMavlinkInterface::close()
{

    ::close(fds_[CONNECTION_FD].fd);
    fds_[CONNECTION_FD] = {0, 0, 0};
    fds_[CONNECTION_FD].fd = -1;
}

void DroneportMavlinkInterface::onSigInt()
{
    gotSigInt_ = true;
    close();
}