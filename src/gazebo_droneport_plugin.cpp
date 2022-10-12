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

#include <gazebo_droneport_plugin.h>

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(GazeboDroneport);

    /**
     * @brief Construct a new Gazebo Droneport:: Gazebo Droneport object
     * 
     * Constructor also creates the object for communication trough mavlink protocol.
     */
    GazeboDroneport::GazeboDroneport() : ModelPlugin()
    {
        mavlink_interface_ = std::make_unique<DroneportMavlinkInterface>();
    }

    /**
     * @brief Destroy the Gazebo Droneport:: Gazebo Droneport object
     * 
     */
    GazeboDroneport::~GazeboDroneport()
    {
        mavlink_interface_->close();
        sigIntConnection_->~Connection();
        updateConnection_->~Connection();
    }

    /**
     * @brief Loading function of Gazebo DronePort model plugin
     * 
     * The function sets all parameters based on default values and on the values in gazebo model file.
     * 
     * @param _model 
     * @param _sdf 
     */
    void GazeboDroneport::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Model and world loading
        model_ = _model;
        world_ = model_->GetWorld();
        // Clear namespace_ of the robot (string)
        namespace_.clear();
        // Node handler, which provides functions to create publishers and subscribers fot communication inside Gazebo
        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init(namespace_);
        // Publisher of visual messages for editing batteries colors.
        pub_visual_ = node_handle_->Advertise<gazebo::msgs::Visual>("~/visual");

        // USE UDP or TCP based on settings inside model.sdf
        bool use_tcp = false;
        if (_sdf->HasElement("use_tcp"))
        {
            use_tcp = _sdf->GetElement("use_tcp")->Get<bool>();
        }
        mavlink_interface_->SetUseTcp(use_tcp);
        gzmsg << "Connecting to MAVLINK using " << (use_tcp ? "TCP" : "UDP") << "\n";

        // Listen to the update event. This event is broadcast every simulation iteration.
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboDroneport::OnUpdate, this, _1));
        // Listen to Ctrl+C / SIGINT.
        sigIntConnection_ = event::Events::ConnectSigInt(boost::bind(&GazeboDroneport::onSigInt, this));

        // Mavlink address, mavlink port and other settings load from model.sdf
        if (_sdf->HasElement("mavlink_addr"))
        {
            std::string mavlink_addr_str = _sdf->GetElement("mavlink_addr")->Get<std::string>();
            if (mavlink_addr_str != "INADDR_ANY")
            {
                mavlink_interface_->SetMavlinkAddr(mavlink_addr_str);
            }
        }

        // Set Mavlink  UDP and TCP port based on settings in model sdf file.
        if (_sdf->HasElement("mavlink_udp_port"))
        {
            int mavlink_udp_port = _sdf->GetElement("mavlink_udp_port")->Get<int>();
            gzmsg << "Using UDP PORT: " << mavlink_udp_port << "\n";
            mavlink_interface_->SetMavlinkUdpPort(mavlink_udp_port);
        }
        if (_sdf->HasElement("mavlink_tcp_port"))
        {
            int mavlink_tcp_port = _sdf->GetElement("mavlink_tcp_port")->Get<int>();
            mavlink_interface_->SetMavlinkTcpPort(mavlink_tcp_port);
        }
        mavlink_status_t *chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);

        // mavlink protocol_version is 2.0
        protocol_version_ = 2.0;
        chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
        gzmsg << "Using MAVLink protocol v2.0\n";

        // Set target system ID and Target Component ID based on settings in model sdf file.
        if (_sdf->HasElement("TgtSystemID"))
        {
            tgt_system_id_ = _sdf->GetElement("TgtSystemID")->Get<int>();
        }
        gzmsg << "Target system ID set to " << tgt_system_id_ << "\n";
        if (_sdf->HasElement("TgtComponentID"))
        {
            tgt_component_id_ = _sdf->GetElement("TgtComponentID")->Get<int>();
        }
        gzmsg << "Target component ID set to " << tgt_component_id_ << " (NOT USED IN CURRENT VERSION) \n";

        // Loading of Mavlink interface based on settings in model sdf file.
        mavlink_interface_->Load();

        // Set DronePort cover to Closed State
        cover_state_ = DRONEPORT_COVER_CLOSED;

        // Set number of ports in DronePort Model. Default is 4
        number_of_ports_ = 4;
        if (_sdf->HasElement("number_of_ports_for_batteries"))
        {
            number_of_ports_ = _sdf->GetElement("number_of_ports_for_batteries")->Get<int>();
        }
        gzmsg << "Model has " << number_of_ports_ << " ports for batteries. \n";

        // Fill all batteries ports by default values.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        for (int i = 0; i < number_of_ports_; i++)
        {
            pub_visual_->Publish(setBatteryColor(i, 0));
            Battery b;
            b.battery_id = 0;
            b.value = unsigned(0);
            b.last_message_time = time(NULL);
            inserted_batteries_.push_back(b);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Subscribe to GPS sensor on DronePort.
        sub_gps_ = node_handle_->Subscribe("~/" + model_->GetName() + "/link/gps0", &GazeboDroneport::GpsCallback, this);
        last_gps_time = time(NULL);
    }

    /**
     * @brief Function computed angle to be between 0 and 360 degrees.
     * 
     * @param angle 
     * @return double 
     */
    double GazeboDroneport::GetDegrees360(const ignition::math::Angle &angle)
    {
        double degrees = angle.Degree();
        while (degrees < 0.)
            degrees += 360.0;
        while (degrees >= 360.0)
            degrees -= 360.0;
        return degrees;
    }

    /**
     * @brief Callback for GPS sensor (from PX4_sitl_gazebo).
     * 
     * @param msg 
     */
    void GazeboDroneport::GpsCallback(SITLGps &msg)
    {
        if (difftime(time(NULL), last_gps_time) > 1.0)
        {
            last_gps_time = time(NULL);
            mavlink_hil_gps_t hil_gps_msg;
            hil_gps_msg.time_usec = msg->time_utc_usec();
            hil_gps_msg.fix_type = 3;
            hil_gps_msg.lat = msg->latitude_deg() * 1e7;
            hil_gps_msg.lon = msg->longitude_deg() * 1e7;
            hil_gps_msg.alt = msg->altitude() * 1000.0;
            hil_gps_msg.eph = msg->eph() * 100.0;
            hil_gps_msg.epv = msg->epv() * 100.0;
            hil_gps_msg.vel = msg->velocity() * 100.0;
            hil_gps_msg.vn = msg->velocity_north() * 100.0;
            hil_gps_msg.ve = msg->velocity_east() * 100.0;
            hil_gps_msg.vd = -msg->velocity_up() * 100.0;
            ignition::math::Angle cog(atan2(msg->velocity_east(), msg->velocity_north()));
            cog.Normalize();
            hil_gps_msg.cog = static_cast<uint16_t>(GetDegrees360(cog) * 100.0);
            hil_gps_msg.satellites_visible = 10;
            hil_gps_msg.id = 0;
            mavlink_message_t mav_msg;
            mavlink_msg_hil_gps_encode_chan(tgt_system_id_, 1, MAVLINK_COMM_0, &mav_msg, &hil_gps_msg);
            mavlink_interface_->sendMessage(&mav_msg);
        }
    }

    /**
     * @brief Function for move cover of the DronePort. 
     * 
     * When the speed is > 0, the cover is moving based on the direction.
     * Defined constants DRONEPORT_COVER_OPEN_DIRECTION and DRONEPORT_COVER_CLOSE_DIRECTION should be used
     * 
     * @param direction 
     * @param speed 
     * @return true 
     * @return false 
     */
    bool GazeboDroneport::moveCover(int direction, float speed)
    {
        physics::LinkPtr cover = model_->GetLink("cover");
        if (cover == NULL)
        {
            gzmsg << "Loaded model does not support moving top part." << std::endl;
            return false;
        }
        model_->GetJoint("front_side_to_cover")->SetVelocity(0, direction * speed);
        return true;
    }

    /**
     * @brief Function for computing Red channel of battery color value.
     * 
     * @param value 
     * @return double 
     */
    double GazeboDroneport::computeRColorChannel(double value)
    {
        return (255 - (((int)(value > 50.0)) * (value - 50) * (255 / 50))) / 255.0;
    }

    /**
     * @brief Function for computing Green channel of battery colro value.
     * 
     * @param value 
     * @return double 
     */
    double GazeboDroneport::computeGColorChannel(double value)
    {
        return (255 - (((int)(value < 50.0)) * value * (255 / 50))) / 255.0;
    }

    /**
     * @brief Function for set battery color based on its ID and its value in percents.
     * 
     * @param port_id 
     * @param battery_value 
     * @return gazebo::msgs::Visual 
     */
    gazebo::msgs::Visual GazeboDroneport::setBatteryColor(int port_id, int battery_value)
    {
        // Get visual_name and msg of the particular battery link
        std::string linkName = "battery" + std::to_string(port_id);
        std::string visual_name = "visual_" + linkName;
        physics::LinkPtr link_ = this->model_->GetLink(linkName);
        gazebo::msgs::Visual visualMsg = link_->GetVisualMessage(visual_name);

        // Set basic information to visual msg.
        visualMsg.set_name(link_->GetScopedName());
        visualMsg.set_parent_name(this->model_->GetScopedName());
        if ((!visualMsg.has_material()) || visualMsg.mutable_material() == NULL)
        {
            gazebo::msgs::Material *materialMsg = new gazebo::msgs::Material;
            visualMsg.set_allocated_material(materialMsg);
        }
        // Set color into the visual msg.
        double red_value = computeRColorChannel(battery_value);
        double green_value = computeGColorChannel(battery_value);
        Color battery_color(0.0, 0.0, 0.0, 1.0);
        if (battery_value == 0)
        {
            battery_color.Set(0.75, 0.75, 0.75, 0.0);
        }
        else
        {
            battery_color.Set(red_value, green_value, 0.0, 1.0);
        }
        gazebo::msgs::Color *colorMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(battery_color));
        gazebo::msgs::Color *diffuseMsg = new gazebo::msgs::Color(*colorMsg);
        gazebo::msgs::Color *emissiveMsg = new gazebo::msgs::Color(*colorMsg);
        Color specular_color(battery_color);
#if GAZEBO_MAJOR_VERSION > 9
        specular_color.A(0.5);
#else
        specular_color.a = 0.5;
#endif
        gazebo::msgs::Color *specularMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(specular_color));
        gazebo::msgs::Material *materialMsg = visualMsg.mutable_material();
        if (materialMsg->has_ambient())
        {
            materialMsg->clear_ambient();
        }
        materialMsg->set_allocated_ambient(colorMsg);
        if (materialMsg->has_diffuse())
        {
            materialMsg->clear_diffuse();
        }
        materialMsg->set_allocated_diffuse(diffuseMsg);
        if (materialMsg->has_emissive())
        {
            materialMsg->clear_emissive();
        }
        materialMsg->set_allocated_emissive(emissiveMsg);
        if (materialMsg->has_specular())
        {
            materialMsg->clear_specular();
        }
        materialMsg->set_allocated_specular(specularMsg);

        return visualMsg;
    }

    /**
     * @brief Function for handling onUpdate event in Gazebo.
     * 
     */
    void GazeboDroneport::OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        bool close_conn_ = false;

        // Handling mavlink messages
        std::queue<mavlink_message_t> messages;
        mavlink_interface_->getMessages(&messages);
        if (messages.empty() == false)
        {
            mavlink_message_t message = messages.front();

            if ((int)message.sysid == tgt_system_id_)
            {
                switch (message.msgid)
                {
                // Mavlink battery status -- set information about inserted batteries from mavlink messages.
                case MAVLINK_MSG_ID_BATTERY_STATUS:
                {
                    mavlink_battery_status_t battery_status;
                    mavlink_msg_battery_status_decode(&message, &battery_status);
                    bool found = false;
                    int port_number = unsigned(battery_status.battery_function) - 1;
                    if ((port_number >= 0) && (port_number <= number_of_ports_))
                    {
                        if (inserted_batteries_.at(port_number).battery_id == 0)
                        {
                            inserted_batteries_.at(port_number).battery_id = unsigned(battery_status.id);
                        }
                        inserted_batteries_.at(port_number).value = unsigned(battery_status.battery_remaining);
                        inserted_batteries_.at(port_number).last_message_time = time(NULL);
                    }
                    // TODO: It is necessary to handle situation when port_number > number_of_ports.
                    break;
                }
                // Mavlink param set -- it is used to handle DronePort cover.
                case MAVLINK_MSG_ID_PARAM_SET:
                {
                    mavlink_param_set_t param_set;
                    mavlink_msg_param_set_decode(&message, &param_set);
                    char *param_id;
                    param_id = param_set.param_id;
                    if (strcmp("cover", param_id) == 0)
                    {
                        int param_value = param_set.param_value;
                        cover_state_ = param_value;
                    }
                    break;
                }
                }
            }
        }

        // Publish visual message of all batteries based on information in inserted batteries data structure.
        for (int i; i < inserted_batteries_.size(); i++)
        {
            if (difftime(time(NULL), inserted_batteries_.at(i).last_message_time) < 1.5)
            {
                pub_visual_->Publish(setBatteryColor(i, inserted_batteries_.at(i).value));
            }
            else
            {
                pub_visual_->Publish(setBatteryColor(i, 0));
            }
        }

        // Opening and closing of DronePort model
        if (cover_state_ == DRONEPORT_COVER_OPENNING)
        {
            double cover_position = model_->GetJoint("front_side_to_cover")->Position(0);
            if (cover_position <= DRONEPORT_COVER_POS_OPEN)
            {
                cover_state_ = DRONEPORT_COVER_OPENED;
                moveCover(DRONEPORT_COVER_OPEN_DIRECTION, 0);
            }
            else
            {
                moveCover(DRONEPORT_COVER_OPEN_DIRECTION, DRONEPORT_COVER_DEFAULT_SPEED);
            }
        }
        else if (cover_state_ == DRONEPORT_COVER_CLOSING)
        {
            double cover_position = model_->GetJoint("front_side_to_cover")->Position(0);
            gzmsg << cover_position << "  " << DRONEPORT_COVER_POS_CLOSE << std::endl;
            if (cover_position >= DRONEPORT_COVER_POS_CLOSE)
            {
                cover_state_ = DRONEPORT_COVER_CLOSED;
                moveCover(DRONEPORT_COVER_CLOSE_DIRECTION, 0);
            }
            else
            {
                moveCover(DRONEPORT_COVER_CLOSE_DIRECTION, DRONEPORT_COVER_DEFAULT_SPEED);
            }
        }

        if (close_conn_)
        { // close connection if required
            mavlink_interface_->close();
        }
    }

    /**
     * @brief Function for determining if the world is running.
     * 
     * @return true 
     * @return false 
     */
    bool GazeboDroneport::IsRunning()
    {
        return world_->Running();
    }

    /**
     * @brief Function dealing with SigInt signal for application exit.
     * 
     */
    void GazeboDroneport::onSigInt()
    {
        mavlink_interface_->onSigInt();
    }

} // namespace gazebo