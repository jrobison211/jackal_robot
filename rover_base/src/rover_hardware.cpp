/**
 *
 *  \file
 *  \brief      Class representing Rover hardware
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com> edited by Jackson Robison for use in senior design project
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include <boost/assign.hpp>
#include "rover_base/rover_hardware.h"

namespace rover_base
{

RoverHardware::RoverHardware()
{
  ros::V_string joint_names = boost::assign::list_of("FL_wheel")
      ("FR_wheel")("ML_wheel")("MR_wheel")("BL_wheel")("BR_wheel"); //changed naming convention and added middle wheels 

  for (unsigned int i = 0; i < joint_names.size(); i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
        &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  feedback_sub_ = nh_.subscribe("feedback", 1, &RoverHardware::feedbackCallback, this);

  // Realtime publisher, initializes differently from regular ros::Publisher
  cmd_drive_pub_.init(nh_, "cmd_drive", 1);
}

/**
 * Populates the internal joint state struct from the most recent Feedback message
 * received from the MCU.
 *
 * Called from the controller thread.
 */
void RoverHardware::copyJointsFromHardware()
{
  boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);
  if (feedback_msg_ && feedback_msg_lock)
  {
    for (int i = 0; i < 4; i++) //should this change to a 6? why is it a 4 not a 3?
    {
      joints_[i].position = feedback_msg_->drivers[i % 2].measured_travel;
      joints_[i].velocity = feedback_msg_->drivers[i % 2].measured_velocity;
      joints_[i].effort = 0;  // TODO(mikepurvis): determine this from amperage data.
    }
  }
}

/**
 * Populates and publishes Drive message based on the controller outputs.
 *
 * Called from the controller thread.
 */
void RoverHardware::publishDriveFromController()
{
  if (cmd_drive_pub_.trylock())
  {
    cmd_drive_pub_.msg_.mode = rover_msgs::Drive::MODE_VELOCITY;
    cmd_drive_pub_.msg_.drivers[rover_msgs::Drive::LEFT] = joints_[0].velocity_command;
    cmd_drive_pub_.msg_.drivers[rover_msgs::Drive::RIGHT] = joints_[1].velocity_command;
    cmd_drive_pub_.unlockAndPublish();
  }
}

void RoverHardware::feedbackCallback(const rover_msgs::Feedback::ConstPtr& msg)
{
  // Update the feedback message pointer to point to the current message. Block
  // until the control thread is not using the lock.
  boost::mutex::scoped_lock lock(feedback_msg_mutex_);
  feedback_msg_ = msg;
}

}  // namespace rover_base
