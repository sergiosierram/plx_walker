#include <spider_teleop/spider_pad.h>

SpiderPad::SpiderPad():
    linear_x_(1),
    linear_y_(0),
    angular_(2),
    linear_z_(3)
  {
      current_vel = 0.1;

      //JOYSTICK PAD TYPE
      nh_.param<std::string>("pad_type",pad_type_,"ps3");
      //
      nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
      // MOTION CONF
      nh_.param("axis_linear_x", linear_x_, DEFAULT_AXIS_LINEAR_X);
      nh_.param("axis_linear_y", linear_y_, DEFAULT_AXIS_LINEAR_Y);
      nh_.param("axis_linear_z", linear_z_, DEFAULT_AXIS_LINEAR_Z);
      nh_.param("axis_angular", angular_, DEFAULT_AXIS_ANGULAR);
      nh_.param("scale_angular", a_scale_, DEFAULT_SCALE_ANGULAR);
      nh_.param("scale_linear", l_scale_, DEFAULT_SCALE_LINEAR);
      nh_.param("scale_linear_z", l_scale_z_, DEFAULT_SCALE_LINEAR_Z);
      nh_.param("cmd_topic_vel", cmd_topic_vel_, cmd_topic_vel_);
      nh_.param("button_dead_man", dead_man_button_, dead_man_button_);
      nh_.param("button_speed_up", speed_up_button_, speed_up_button_);  //4 Thrustmaster
      nh_.param("button_speed_down", speed_down_button_, speed_down_button_); //5 Thrustmaster
      nh_.param<std::string>("joystick_dead_zone", joystick_dead_zone_, "true");

      // KINEMATIC MODE
      nh_.param("button_kinematic_mode", button_kinematic_mode_, button_kinematic_mode_);
      nh_.param("cmd_service_set_mode", cmd_set_mode_, cmd_set_mode_);
      nh_.param("cmd_service_home", cmd_home_, cmd_home_);
      kinematic_mode_ = 1;

      ROS_INFO("SummitXLPad num_of_buttons_ = %d", num_of_buttons_);
      for(int i = 0; i < num_of_buttons_; i++){
          bRegisteredButtonEvent[i] = false;
          ROS_INFO("bREG %d", i);
          }

      for(int i = 0; i < 3; i++){
        bRegisteredDirectionalArrows[i] = false;
      }

      /*ROS_INFO("Service I/O = [%s]", cmd_service_io_.c_str());
      ROS_INFO("Topic PTZ = [%s]", cmd_topic_ptz_.c_str());
      ROS_INFO("Service I/O = [%s]", cmd_topic_vel_.c_str());
      ROS_INFO("Axis linear = %d", linear_);
      ROS_INFO("Axis angular = %d", angular_);
      ROS_INFO("Scale angular = %d", a_scale_);
      ROS_INFO("Deadman button = %d", dead_man_button_);
      ROS_INFO("OUTPUT1 button %d", button_output_1_);
      ROS_INFO("OUTPUT2 button %d", button_output_2_);
      ROS_INFO("OUTPUT1 button %d", button_output_1_);
      ROS_INFO("OUTPUT2 button %d", button_output_2_);*/

      // Publish through the node handle Twist type messages to the guardian_controller/command topic
      vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);


      // Listen through the node handle sensor_msgs::Joy messages from joystick
      // (these are the references that we will sent to summit_xl_controller/command)
      pad_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &SpiderPad::padCallback, this);

      // Diagnostics
      updater_pad.setHardwareID("None");
      // Topics freq control
      min_freq_command = min_freq_joy = 5.0;
      max_freq_command = max_freq_joy = 50.0;
      sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joy", updater_pad,
                          diagnostic_updater::FrequencyStatusParam(&min_freq_joy, &max_freq_joy, 0.1, 10));

      pub_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(cmd_topic_vel_.c_str(), updater_pad,
                          diagnostic_updater::FrequencyStatusParam(&min_freq_command, &max_freq_command, 0.1, 10));


      bEnable = false;	// Communication flag disabled by default
      last_command_ = true;
  }


  /*
   *	\brief Updates the diagnostic component. Diagnostics
   *
   */
  void SpiderPad::Update(){
      updater_pad.update();
  }



  void SpiderPad::padCallback(const sensor_msgs::Joy::ConstPtr& joy)
  {
      geometry_msgs::Twist vel;

      vel.linear.x = 0.0;
      vel.linear.y = 0.0;
      vel.linear.z = 0.0;

      vel.angular.x = 0.0;
      vel.angular.y = 0.0;
      vel.angular.z = 0.0;

      bEnable = (joy->buttons[dead_man_button_] == 1);

      // Actions dependant on dead-man button
      if (joy->buttons[dead_man_button_] == 1) {
          //ROS_ERROR("SummitXLPad::padCallback: DEADMAN button %d", dead_man_button_);
          //Set the current velocity level
          if ( joy->buttons[speed_down_button_] == 1 ){

              if(!bRegisteredButtonEvent[speed_down_button_])
                  if(current_vel > 0.1){
                      current_vel = current_vel - 0.1;
                      bRegisteredButtonEvent[speed_down_button_] = true;
                      ROS_INFO("Velocity: %f%%", current_vel*100.0);
                      char buf[50]="\0";
                      int percent = (int) (current_vel*100.0);
                      sprintf(buf," %d percent", percent);
                      // sc.say(buf);
                  }
          }else{
              bRegisteredButtonEvent[speed_down_button_] = false;
           }
          //ROS_ERROR("SummitXLPad::padCallback: Passed SPEED DOWN button %d", speed_down_button_);
          if (joy->buttons[speed_up_button_] == 1){
              if(!bRegisteredButtonEvent[speed_up_button_])
                  if(current_vel < 0.9){
                      current_vel = current_vel + 0.1;
                      bRegisteredButtonEvent[speed_up_button_] = true;
                      ROS_INFO("Velocity: %f%%", current_vel*100.0);
                      char buf[50]="\0";
                      int percent = (int) (current_vel*100.0);
                      sprintf(buf," %d percent", percent);
                      // sc.say(buf);
                  }

          }else{
              bRegisteredButtonEvent[speed_up_button_] = false;
          }
          //ROS_ERROR("SummitXLPad::padCallback: Passed SPEED UP button %d", speed_up_button_);



          vel.linear.x = current_vel*l_scale_*joy->axes[linear_x_];
          vel.linear.y = current_vel*l_scale_*joy->axes[linear_y_];

          //ROS_ERROR("SummitXLPad::padCallback: Passed linear axes");

          if(joystick_dead_zone_=="true")
          {
              // limit scissor movement or robot turning (they are in the same joystick)
              if(joy->axes[angular_] == 1.0 || joy->axes[angular_] == -1.0) // if robot turning
              {
                  // Same angular velocity for the three axis
                  vel.angular.x = current_vel*(a_scale_*joy->axes[angular_]);
                  vel.angular.y = current_vel*(a_scale_*joy->axes[angular_]);
                  vel.angular.z = current_vel*(a_scale_*joy->axes[angular_]);

                  vel.linear.z = 0.0;
              }
              else if (joy->axes[linear_z_] == 1.0 || joy->axes[linear_z_] == -1.0) // if scissor moving
              {
                  vel.linear.z = current_vel*l_scale_z_*joy->axes[linear_z_]; // scissor movement

                  // limit robot turn
                  vel.angular.x = 0.0;
                  vel.angular.y = 0.0;
                  vel.angular.z = 0.0;
              }
              else
              {
                  // Same angular velocity for the three axis
                  vel.angular.x = current_vel*(a_scale_*joy->axes[angular_]);
                  vel.angular.y = current_vel*(a_scale_*joy->axes[angular_]);
                  vel.angular.z = current_vel*(a_scale_*joy->axes[angular_]);
                  vel.linear.z = current_vel*l_scale_z_*joy->axes[linear_z_]; // scissor movement
              }
          }
          else // no dead zone
          {
              vel.angular.x = current_vel*(a_scale_*joy->axes[angular_]);
              vel.angular.y = current_vel*(a_scale_*joy->axes[angular_]);
              vel.angular.z = current_vel*(a_scale_*joy->axes[angular_]);
              vel.linear.z = current_vel*l_scale_z_*joy->axes[linear_z_];
          }

          //ROS_ERROR("SummitXLPad::padCallback: Passed joystick deadzone ifelse");


          if (joy->buttons[button_kinematic_mode_] == 1) {

              if(!bRegisteredButtonEvent[button_kinematic_mode_]){
                  // Define mode (inc) - still coupled
                  kinematic_mode_ += 1;
                  if (kinematic_mode_ > 2) kinematic_mode_ = 1;
                  ROS_INFO("SummitXLJoy::joyCallback: Kinematic Mode %d ", kinematic_mode_);
                  // Call service
                  //robotnik_msgs::set_mode set_mode_srv;
                  //set_mode_srv.request.mode = kinematic_mode_;
                  //setKinematicMode.call( set_mode_srv );
                  bRegisteredButtonEvent[button_kinematic_mode_] = true;
              }
          }else{
              bRegisteredButtonEvent[button_kinematic_mode_] = false;
          }

          //ROS_ERROR("SummitXLPad::padCallback: Passed SPHERE CAM and KINEMATIC MODE");

      }
      else {
          vel.angular.x = 0.0;	vel.angular.y = 0.0; vel.angular.z = 0.0;
          vel.linear.x = 0.0; vel.linear.y = 0.0; vel.linear.z = 0.0;
      }

      sus_joy_freq->tick();	// Ticks the reception of joy events

       // Publish
      // Only publishes if it's enabled
      if(bEnable){
          vel_pub_.publish(vel);
          pub_command_freq->tick();
          last_command_ = true;
          }


      if(!bEnable && last_command_){

          vel.angular.x = 0.0;  vel.angular.y = 0.0; vel.angular.z = 0.0;
          vel.linear.x = 0.0;   vel.linear.y = 0.0; vel.linear.z = 0.0;
          vel_pub_.publish(vel);
          pub_command_freq->tick();
          last_command_ = false;
          }
  }


  int main(int argc, char** argv)
  {
      ros::init(argc, argv, "summit_xl_pad");
      SpiderPad my_spider_pad;

      ros::Rate r(10.0);

      while( ros::ok() )
      {
          // UPDATING DIAGNOSTICS
          my_spider_pad.Update();
          ros::spinOnce();
          r.sleep();
      }
  }

