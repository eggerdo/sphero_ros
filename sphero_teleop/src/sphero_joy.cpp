/*
 * Copyright (c) 2010, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class SpheroTeleop
{
public:
  SpheroTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();
  void updateMotors();

  ros::NodeHandle ph_, nh_;

  int x_axis_, y_axis_, deadman_axis_, heading_axis_;
  bool x_inverted_, y_inverted_;

  double l_scale_, a_scale_;
  ros::Publisher vel_pub_, heading_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_, heading_pressed_;
  ros::Timer timer_;

};

SpheroTeleop::SpheroTeleop():
      ph_("~"),
      x_axis_(1),
      y_axis_(0),
      x_inverted_(false),
      y_inverted_(false),
      deadman_axis_(4),
      heading_axis_(5),
      l_scale_(0.3),
      a_scale_(0.9)
{
  ph_.param("x_axis", x_axis_, x_axis_);
  ph_.param("x_inverted", x_inverted_, x_inverted_);
  ph_.param("y_axis", y_axis_, y_axis_);
  ph_.param("y_inverted", y_inverted_, y_inverted_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("axis_heading", heading_axis_, heading_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  heading_pub_ = ph_.advertise<geometry_msgs::Twist>("heading", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &SpheroTeleop::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&SpheroTeleop::publish, this));
}

void SpheroTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  if (!y_inverted_) {
    vel.linear.y = l_scale_*joy->axes[y_axis_];
  } else {
    vel.linear.y = -l_scale_*joy->axes[y_axis_];
  }
  if (!x_inverted_) {
    vel.linear.x = l_scale_*joy->axes[x_axis_];
  } else {
    vel.linear.x = -l_scale_*joy->axes[x_axis_];
  }
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];
  heading_pressed_ = joy->buttons[heading_axis_];
}

void SpheroTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (deadman_pressed_)
  {
    vel_pub_.publish(last_published_);
  }
  else if (heading_pressed_)
  {
    heading_pub_.publish(last_published_);
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Sphero_teleop");
  SpheroTeleop Sphero_teleop;

  ros::spin();
}
