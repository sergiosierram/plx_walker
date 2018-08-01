/*
 * spider_pad
 * Copyright (c) 2012, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * \brief Allows to use a pad with the roboy controller, sending the messages received from the joystick device
 */

#ifndef SPIDER_PAD_H
#define SPIDER_PAD_H


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Wrench.h>

#include <unistd.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#define DEFAULT_NUM_OF_BUTTONS		16
#define DEFAULT_LEFT_AXIS_X			1
#define DEFAULT_LEFT_AXIS_Y			0
#define DEFAULT_RIGHT_AXIS_X		2
#define DEFAULT_RIGHT_AXIS_Y	    3
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		2.0
#define DEFAULT_SCALE_LINEAR_Z      1.0

////////////////////////////////////////////////////////////////////////
//                               NOTE:                                //
// This configuration is made for a THRUSTMASTER T-Wireless 3in1 Joy  //
//   please feel free to modify to adapt for your own joystick.       //
// 								      //


class SpiderPad
{
public:
    SpiderPad();
    void Update();

    private:
    void padCallback(const sensor_msgs::Joy::ConstPtr& joy);
    ros::NodeHandle nh_;

    int left_x_, left_y_, right_x_, right_y_;
    double l_scale_, a_scale_, l_scale_z_;
    //! It will publish into command velocity (for the robot) and the ptz_state (for the pantilt)
    ros::Publisher frc_left_pub_, frc_right_pub_;
    //! It will be suscribed to the joystick
    ros::Subscriber pad_sub_;
    //! Name of the topic where it will be publishing the velocity
    std::string frc_topic_l_, frc_topic_r_;
    //! Name of the service where it will be modifying the digital outputs
    std::string cmd_service_io_;
    //! Name of the topic where it will be publishing the pant-tilt values
    std::string cmd_topic_ptz_;
    double current_frc;
    //! Pad type
    std::string pad_type_;
    //! Number of the DEADMAN button
    int dead_man_button_;
    //! Number of the button for increase or decrease the speed max of the joystick
    int speed_up_button_, speed_down_button_;
    int button_output_1_, button_output_2_;
    int output_1_, output_2_;
    bool bOutput1, bOutput2;
    //! button to change kinematic mode
    int button_kinematic_mode_;
    //! kinematic mode
    int kinematic_mode_;
    //! Service to modify the kinematic mode
    ros::ServiceClient setKinematicMode;
    //! Name of the service to change the mode
    std::string cmd_set_mode_;
    //! button to start the homing service
    int button_home_;
    //! Service to start homing
    ros::ServiceClient doHome;
    //! Name of the service to do the homing
    std::string cmd_home_;
    //! buttons to the pan-tilt-zoom camera
    int ptz_tilt_up_, ptz_tilt_down_, ptz_pan_right_, ptz_pan_left_;
    int ptz_zoom_wide_, ptz_zoom_tele_;
    //! Service to modify the digital outputs
    ros::ServiceClient set_digital_outputs_client_;
    //! Number of buttons of the joystick
    int num_of_buttons_;
    //! Pointer to a vector for controlling the event when pushing the buttons
    bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];
        //! Pointer to a vector for controlling the event when pushing directional arrows (UNDER AXES ON PX4!)
        bool bRegisteredDirectionalArrows[4];

    // DIAGNOSTICS
    //! Diagnostic to control the frequency of the published command velocity topic
    diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq_l;
    diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq_r;
    //! Diagnostic to control the reception frequency of the subscribed joy topic
    diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq;
    //! General status diagnostic updater
    diagnostic_updater::Updater updater_pad;
    //! Diagnostics min freq
    double min_freq_command, min_freq_joy; //
    //! Diagnostics max freq
    double max_freq_command, max_freq_joy; //
    //! Flag to enable/disable the communication with the publishers topics
    bool bEnable;
    //! Flag to track the first reading without the deadman's button pressed.
    bool last_command_;
    //! Client of the sound play service
    //  sound_play::SoundClient sc;
    //! Pan & tilt increment (degrees)
    int pan_increment_, tilt_increment_;
    //! Zoom increment (steps)
    int zoom_increment_;
    //! Add a dead zone to the joystick that controls scissor and robot rotation (only useful for xWam)
    std::string joystick_dead_zone_;
};

#endif // SPIDER_PAD_H
