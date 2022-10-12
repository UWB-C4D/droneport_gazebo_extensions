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
#include <time.h>
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
#include <boost/shared_ptr.hpp>

#include <iostream>
#include <random>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>

#include <Eigen/Eigen>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math.hh>
#include <sdf/sdf.hh>

#include <SITLGps.pb.h>

#include "droneport_mavlink_interface.h"

namespace gazebo
{

    #define DRONEPORT_COVER_CLOSED              0
    #define DRONEPORT_COVER_OPENED              1
    #define DRONEPORT_COVER_OPENNING            2
    #define DRONEPORT_COVER_CLOSING             3
    #define DRONEPORT_COVER_STOPPED             4
    #define DRONEPORT_COVER_POS_CLOSE      -0.001
    #define DRONEPORT_COVER_POS_OPEN        -0.75
    #define DRONEPORT_COVER_OPEN_DIRECTION     -1
    #define DRONEPORT_COVER_CLOSE_DIRECTION     1
    #define DRONEPORT_COVER_DEFAULT_SPEED     0.1

    // Struct for handling battery values during recharing process.
    struct Battery
    {
        int battery_id;
        time_t last_message_time;
        int value;
    };
    // GPS message type
    typedef const boost::shared_ptr<const sensor_msgs::msgs::SITLGps> SITLGps;

    // Color data type for batteries. Ensured compatibility with Gazebo 9
#if GAZEBO_MAJOR_VERSION > 9
    typedef ignition::math::Color Color;
#else
    typedef gazebo::common::Color Color;
#endif

    static const std::string kDefaultNamespace = "";

    class GazeboDroneport : public ModelPlugin
    {
    public:
        GazeboDroneport();
        ~GazeboDroneport();

        void Publish();
        void GpsCallback(SITLGps &msg);
        double GetDegrees360(const ignition::math::Angle& angle);

    protected:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void OnUpdate(const common::UpdateInfo & /* _info */);
        gazebo::msgs::Visual setBatteryColor(int battery_id, int battery_value);
        bool moveCover(int direction, float speed);
        double computeRColorChannel(double value);
        double computeGColorChannel(double value);

    private:
        // Mavlink interface object, necessary for mavlink communication.
        std::unique_ptr<DroneportMavlinkInterface> mavlink_interface_;
        float protocol_version_{2.0};

        // Node Handle for communication inside Gazebo
        transport::NodePtr node_handle_;
        std::string namespace_{kDefaultNamespace};

        // Model and World objects
        physics::ModelPtr model_{};
        physics::WorldPtr world_{nullptr};

        // Objects for transport data inside gazebo
        transport::PublisherPtr pub_visual_; 
        transport::SubscriberPtr sub_gps_;

        /// \brief Pointer to the update event connection.
        event::ConnectionPtr updateConnection_;
        // Pointer to the SigInt event connection
        event::ConnectionPtr sigIntConnection_;

        bool IsRunning();
        void onSigInt();
        std::vector<Battery> inserted_batteries_{};
        int number_of_ports_ = 0;

        // GPS time
        time_t last_gps_time;

        // True if the connection is closed
        bool close_conn_{false};

        // Target system and component ID (what system is visualized by the plugin)
        int tgt_system_id_{201};
        int tgt_component_id_{1};

        // State of the DronePort cover.
        int cover_state_{DRONEPORT_COVER_CLOSED}; // 0 - close, 1 - open, 2 -openning, 3- closing, 4 - stopped;

    };
} // namespace gazebo