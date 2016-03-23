//------------------------------------------------------------------------------
// Copyright (c) 2016, RT Corporation
// All rights reserved.

// License: BSD

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of RT Corporation nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//------------------------------------------------------------------------------
#ifndef NEKONOTE_DRIVER_FOR_ROS_H
#define NEKONOTE_DRIVER_FOR_ROS_H

#include <pthread.h>

#include <deque>
#include <algorithm>
#include <boost/shared_ptr.hpp>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nekonote_msgs/Temperature.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int64.h"
#include "nekonote_msgs/Mode.h"
#include "nekonote_msgs/JointCommand.h"
#include <actionlib/server/simple_action_server.h>
#include "nekonote_msgs/JointTrajectoryAction.h"

#include "nekonote_driver/nekonote_driver.h"  //Please include this after including ros/ros.h if you use ROS

namespace rt_nekonote
{
class ThreadTools
{
public:
  class Mutex
  {
    pthread_mutex_t mutex_;

  public:
    Mutex()
    {
      pthread_mutex_init(&mutex_, NULL);
    }

    ~Mutex()
    {
      pthread_mutex_destroy(&mutex_);
    }

    inline void lock()
    {
      pthread_mutex_lock(&mutex_);
    }

    inline void unlock()
    {
      pthread_mutex_unlock(&mutex_);
    }
  };

  class Thread
  {
    pthread_t thread_;
    bool is_started_;

  public:
    Thread() : is_started_(false)
    {
    }

    void start(void* (*thread_proc)(void*), void* arg)
    {
      int res = pthread_create(&thread_, NULL, thread_proc, (void*)arg);
      if (res != 0)
        throw NekonoteException("cannot make thread");
      is_started_ = true;
    }

    void join()
    {
      if (is_started_)
        pthread_join(thread_, NULL);
      is_started_ = false;
    }
  };
};  // end class ThreadTools

class NekonoteDriverForROS
{
private:
  class LockGuard  // Convenience class to use mutex
  {
    ThreadTools::Mutex& mutex_;

  public:
    LockGuard(ThreadTools::Mutex& m) : mutex_(m)
    {
      mutex_.lock();
    }

    ~LockGuard()
    {
      mutex_.unlock();
    }
  };

  /***************Contents in request queue***************/
  class RequestItem
  {
  protected:
    ThreadTools::Mutex lock_request_;
    bool is_processed_;
    bool is_requesting_publish_;

  public:
    RequestItem() : is_processed_(false), is_requesting_publish_(false)
    {
    }

    virtual ~RequestItem()
    {
    }

    virtual void execRequest(NekonoteDriver* nekonote_ptr) = 0;

    virtual std::vector<std::string> getReturnValue()
    {
      return std::vector<std::string>();
    }

    bool isProcessed()
    {
      LockGuard _l(lock_request_);
      return is_processed_;
    }

    bool isRequestingPublish()
    {
      LockGuard _l(lock_request_);
      return is_requesting_publish_;
    }
  };

  class RequestItemGetAllData : public RequestItem
  {
  private:
    std::vector<double> angles_;
    std::vector<double> motor_temps_;
    int last_motion_id_;
    int mode_;
    std::vector<double> speed_;
    std::vector<double> torque_;

    void pushBackDoubleVectorToStringVector(const std::vector<double>& d_vec, std::vector<std::string>* s_vec)
    {
      for (std::vector<double>::const_iterator itr = d_vec.begin(); itr != d_vec.end(); ++itr)
      {
        std::ostringstream os;
        os << *itr;
        s_vec->push_back(os.str());
      }
    }

  public:
    void execRequest(NekonoteDriver* nekonote_ptr)
    {
      LockGuard _l(lock_request_);
      nekonote_ptr->getAllData(&angles_, &motor_temps_, &last_motion_id_, &mode_, &speed_, &torque_);
      is_processed_ = true;
      is_requesting_publish_ = true;
    }

    std::vector<std::string> getReturnValue()
    {
      LockGuard _l(lock_request_);
      std::vector<std::string> rvalue;
      pushBackDoubleVectorToStringVector(angles_, &rvalue);
      pushBackDoubleVectorToStringVector(motor_temps_, &rvalue);
      std::ostringstream os;
      os << last_motion_id_;
      rvalue.push_back(os.str());
      os.str("");
      os << mode_;
      rvalue.push_back(os.str());
      pushBackDoubleVectorToStringVector(speed_, &rvalue);
      pushBackDoubleVectorToStringVector(torque_, &rvalue);
      return rvalue;
    }
  };

  class RequestItemTorqueControl : public RequestItem
  {
  private:
    const std::vector<double> torque_command_;
    std::vector<double> angles_;
    std::vector<double> motor_temps_;
    int last_motion_id_;
    int mode_;
    std::vector<double> speed_;
    std::vector<double> torque_;

    void pushBackDoubleVectorToStringVector(const std::vector<double>& d_vec, std::vector<std::string>* s_vec)
    {
      for (std::vector<double>::const_iterator itr = d_vec.begin(); itr != d_vec.end(); ++itr)
      {
        std::ostringstream os;
        os << *itr;
        s_vec->push_back(os.str());
      }
    }

  public:
    RequestItemTorqueControl(const std::vector<double>& torque_command) : torque_command_(torque_command)
    {
    }

    void execRequest(NekonoteDriver* nekonote_ptr)
    {
      LockGuard _l(lock_request_);
      nekonote_ptr->torqueControl(torque_command_, &angles_, &motor_temps_, &last_motion_id_, &mode_, &speed_,
                                  &torque_);
      is_processed_ = true;
      is_requesting_publish_ = true;
    }

    std::vector<std::string> getReturnValue()
    {
      LockGuard _l(lock_request_);
      std::vector<std::string> rvalue;
      pushBackDoubleVectorToStringVector(angles_, &rvalue);
      pushBackDoubleVectorToStringVector(motor_temps_, &rvalue);
      std::ostringstream os;
      os << last_motion_id_;
      rvalue.push_back(os.str());
      os.str("");
      os << mode_;
      rvalue.push_back(os.str());
      pushBackDoubleVectorToStringVector(speed_, &rvalue);
      pushBackDoubleVectorToStringVector(torque_, &rvalue);
      return rvalue;
    }
  };

  class RequestItemPushLinkAngles : public RequestItem
  {
  private:
    const std::vector<double> angles_;
    const int tm_;
    int motion_id_;

  public:
    RequestItemPushLinkAngles(const std::vector<double>& angles, const int tm) : angles_(angles), tm_(tm)
    {
    }

    void execRequest(NekonoteDriver* nekonote_ptr)
    {
      LockGuard _l(lock_request_);
      nekonote_ptr->pushLinkAngles(angles_, tm_, &motion_id_);
      is_processed_ = true;
    }

    std::vector<std::string> getReturnValue()
    {
      LockGuard _l(lock_request_);
      std::vector<std::string> rvalue;
      std::ostringstream os;
      os << motion_id_;
      rvalue.push_back(os.str());
      return rvalue;
    }
  };

  class RequestItemPushStartAngles : public RequestItem
  {
  private:
    const std::vector<double> angles_;
    const int tm_;
    int motion_id_;

  public:
    RequestItemPushStartAngles(const std::vector<double>& angles, const int tm) : angles_(angles), tm_(tm)
    {
    }

    void execRequest(NekonoteDriver* nekonote_ptr)
    {
      LockGuard _l(lock_request_);
      nekonote_ptr->pushStartAngles(angles_, tm_, &motion_id_);
      is_processed_ = true;
    }

    std::vector<std::string> getReturnValue()
    {
      LockGuard _l(lock_request_);
      std::vector<std::string> rvalue;
      std::ostringstream os;
      os << motion_id_;
      rvalue.push_back(os.str());
      return rvalue;
    }
  };

  class RequestItemPushMiddleAngles : public RequestItem
  {
  private:
    const std::vector<double> angles_;
    const int tm_;
    int motion_id_;

  public:
    RequestItemPushMiddleAngles(const std::vector<double>& angles, const int tm) : angles_(angles), tm_(tm)
    {
    }

    void execRequest(NekonoteDriver* nekonote_ptr)
    {
      LockGuard _l(lock_request_);
      nekonote_ptr->pushMiddleAngles(angles_, tm_, &motion_id_);
      is_processed_ = true;
    }

    std::vector<std::string> getReturnValue()
    {
      LockGuard _l(lock_request_);
      std::vector<std::string> rvalue;
      std::ostringstream os;
      os << motion_id_;
      rvalue.push_back(os.str());
      return rvalue;
    }
  };

  class RequestItemPushEndAngles : public RequestItem
  {
  private:
    const std::vector<double> angles_;
    const int tm_;
    const bool is_stop_;
    int motion_id_;

  public:
    RequestItemPushEndAngles(const std::vector<double>& angles, const int tm, const bool is_stop = true)
      : angles_(angles), tm_(tm), is_stop_(is_stop)
    {
    }

    void execRequest(NekonoteDriver* nekonote_ptr)
    {
      LockGuard _l(lock_request_);
      nekonote_ptr->pushEndAngles(angles_, tm_, &motion_id_, is_stop_);
      is_processed_ = true;
    }

    std::vector<std::string> getReturnValue()
    {
      LockGuard _l(lock_request_);
      std::vector<std::string> rvalue;
      std::ostringstream os;
      os << motion_id_;
      rvalue.push_back(os.str());
      return rvalue;
    }
  };

  class RequestItemSetAsFreeMode : public RequestItem
  {
  public:
    void execRequest(NekonoteDriver* nekonote_ptr)
    {
      LockGuard _l(lock_request_);
      nekonote_ptr->setAsFreeMode();
      is_processed_ = true;
    }
  };

  class RequestItemSetAsPositionControlMode : public RequestItem
  {
  public:
    void execRequest(NekonoteDriver* nekonote_ptr)
    {
      LockGuard _l(lock_request_);
      nekonote_ptr->setAsPositionControlMode();
      is_processed_ = true;
    }
  };

  class RequestItemStartMotion : public RequestItem
  {
  public:
    void execRequest(NekonoteDriver* nekonote_ptr)
    {
      LockGuard _l(lock_request_);
      nekonote_ptr->startMotion();
      is_processed_ = true;
    }
  };

  class RequestItemStopMotion : public RequestItem
  {
  private:
    int motion_id_;

  public:
    void execRequest(NekonoteDriver* nekonote_ptr)
    {
      LockGuard _l(lock_request_);
      nekonote_ptr->stopMotion(&motion_id_);
      is_processed_ = true;
    }

    std::vector<std::string> getReturnValue()
    {
      LockGuard _l(lock_request_);
      std::vector<std::string> rvalue;
      std::ostringstream os;
      os << motion_id_;
      rvalue.push_back(os.str());
      return rvalue;
    }
  };

  class RequestItemFlushMotion : public RequestItem
  {
  public:
    void execRequest(NekonoteDriver* nekonote_ptr)
    {
      LockGuard _l(lock_request_);
      nekonote_ptr->flushMotion();
      is_processed_ = true;
    }
  };

  class RequestItemGoEmergencyState : public RequestItem
  {
  public:
    void execRequest(NekonoteDriver* nekonote_ptr)
    {
      LockGuard _l(lock_request_);
      nekonote_ptr->goEmergencyState();
      is_processed_ = true;
    }
  };
  /*******************************************************/

  typedef boost::shared_ptr<RequestItem> RequestItemPtr;
  class RequestQueue
  {
  private:
    std::deque<RequestItemPtr> deque_;
    ThreadTools::Mutex lock_queue_;

  public:
    void pushRequest(const RequestItemPtr& item)
    {
      LockGuard _l(lock_queue_);
      deque_.push_back(item);
    }

    void pushHighestPriorRequest(const RequestItemPtr& item)
    {
      LockGuard _l(lock_queue_);
      deque_.push_front(item);
    }

    bool popRequest(RequestItemPtr& item)
    {
      LockGuard _l(lock_queue_);
      if (deque_.size() > 0)
      {
        item = deque_.front();
        deque_.pop_front();
      }
      else
        return false;

      return true;
    }

    void flushQueue()
    {
      LockGuard _l(lock_queue_);
      deque_.clear();
    }
  };

  RequestQueue request_queue_;
  ThreadTools::Thread thread_comm_with_nekonote_;

  ThreadTools::Mutex lock_driver_;
  NekonoteDriver nekonote_;

  ThreadTools::Mutex lock_jta_flag_;
  bool is_sending_jt_;

  const std::vector<std::string> joint_names_;
  const std::string ns_;
  const std::string jta_ns_;
  std::string firm_version_;

  // Objects for ROS communications
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Publisher joint_states_pub_;
  ros::Publisher temperature_pub_;
  ros::Publisher last_motion_id_pub_;
  ros::Publisher mode_pub_;

  ros::Subscriber joint_cmd_sub_;
  ros::Subscriber mode_cmd_sub_;
  ros::Subscriber flush_motion_sub_;

  actionlib::SimpleActionServer<nekonote_msgs::JointTrajectoryAction> as_;

public:
  NekonoteDriverForROS(const std::string& addr, const int port, const std::vector<std::string>& joint_names,
                       const std::string& ns, const bool is_debug = true)
    : nekonote_(addr, port, is_debug, joint_names.size())
    , joint_names_(joint_names)
    , ns_(ns)
    , nh_priv_("~")
    , jta_ns_(ns + "/joint_trajectory_action")
    , as_(nh_, jta_ns_, boost::bind(&NekonoteDriverForROS::onJointTrajectoryAction, this, _1), false)
    , is_sending_jt_(false)
  {
    // ROS parameters
    nh_priv_.setParam("address", addr);
    nh_priv_.setParam("port_number", port);
    nh_priv_.setParam("joint_names", joint_names_);
    nh_priv_.setParam("namespace_of_topics", ns_);

    // ROS publishers
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>(ns_ + "/joint_states", 10);
    temperature_pub_ = nh_.advertise<nekonote_msgs::Temperature>(ns_ + "/temperature", 10);
    last_motion_id_pub_ = nh_.advertise<std_msgs::Int64>(ns_ + "/last_motion_id", 10);
    mode_pub_ = nh_.advertise<nekonote_msgs::Mode>(ns_ + "/mode", 10);

    // ROS subscribers
    joint_cmd_sub_ = nh_.subscribe(ns_ + "/joint_command", 10, &NekonoteDriverForROS::onJointCommand, this);
    mode_cmd_sub_ = nh_.subscribe(ns_ + "/mode_command", 10, &NekonoteDriverForROS::onModeCommand, this);
    flush_motion_sub_ = nh_.subscribe(ns_ + "/flush_motion", 10, &NekonoteDriverForROS::onFlushMotion, this);

    // ROS actionlib
    as_.start();
  }

  ~NekonoteDriverForROS()
  {
  }

  void start()
  {
    nekonote_.startConnection();
    nekonote_.getVersion(&firm_version_);
    ROS_INFO("NEKONOTE's firmware version: %s", firm_version_.c_str());
    ROS_INFO("Initializing NEKONOTE's state...");
    nekonote_.flushMotion();  // in order to flush and stop motion que in NEKONOTE
    nekonote_.setAsPositionControlMode();
    nekonote_.startMotion();
    ROS_INFO("NEKONOTE's state was successfully initialized");
    ROS_INFO("Now NEKONOTE's mode is POSITION_CONTROL, and motion queue in NEKONOTE is started");
    try
    {
      thread_comm_with_nekonote_.start(threadCommWithNekonoteProc, (void*)this);
      ROS_INFO("The thread communicating with NEKONOTE started.");
    }
    catch (const NekonoteException& err)
    {
      throw NekonoteException(err.what(), "NekonoteDriverForROS::start");
    }
  }

  /***************Thread functions***************/
  static void* threadCommWithNekonoteProc(void* arg)
  {
    ((NekonoteDriverForROS*)arg)->execRequestQueue();
    return 0;
  }

  void execRequestQueue()
  {
    try
    {
      while (ros::ok())
      {
        std::vector<double> angles;
        std::vector<double> motor_temps;
        int last_motion_id;
        int mode;
        std::vector<double> speed;
        std::vector<double> torque;

        RequestItemPtr request;
        bool exist_req = request_queue_.popRequest(request);
        if (!exist_req)
        {
          LockGuard _l(lock_driver_);
          nekonote_.getAllData(&angles, &motor_temps, &last_motion_id, &mode, &speed, &torque);
        }
        else
        {
          lock_driver_.lock();
          request->execRequest(&nekonote_);
          lock_driver_.unlock();
          
          if (!request->isRequestingPublish())
            continue;
          std::vector<std::string> rvalue = request->getReturnValue();
          if (rvalue.size() != (joint_names_.size() * 4 + 2))
          {
            ROS_ERROR("Publishing robot states is requested, but size of return value does not equal getAllData()");
            continue;
          }
          for (int i = 0; i < rvalue.size(); i++)
          {
            if (i < joint_names_.size())
              angles.push_back(atof(rvalue[i].c_str()));
            else if (i < (joint_names_.size() * 2))
              motor_temps.push_back(atof(rvalue[i].c_str()));
            else if (i == (joint_names_.size() * 2))
              last_motion_id = atoi(rvalue[i].c_str());
            else if (i == (joint_names_.size() * 2 + 1))
              mode = atoi(rvalue[i].c_str());
            else if (i < (joint_names_.size() * 3 + 2))
              speed.push_back(atof(rvalue[i].c_str()));
            else if (i < (joint_names_.size() * 4 + 2))
              torque.push_back(atof(rvalue[i].c_str()));
          }
        }

        sensor_msgs::JointState joint_states_msg;
        nekonote_msgs::Temperature temperature_msg;
        std_msgs::Int64 last_motion_id_msg;
        nekonote_msgs::Mode mode_msg;

        joint_states_msg.name = joint_names_;
        joint_states_msg.position = angles;
        joint_states_msg.velocity = speed;
        joint_states_msg.effort = torque;
        temperature_msg.name = joint_names_;
        temperature_msg.motor_temp = motor_temps;
        last_motion_id_msg.data = last_motion_id;
        mode_msg.state = mode;
        ros::Time now = ros::Time::now();
        joint_states_msg.header.stamp = temperature_msg.header.stamp = now;
        // Publish data
        joint_states_pub_.publish(joint_states_msg);
        temperature_pub_.publish(temperature_msg);
        last_motion_id_pub_.publish(last_motion_id_msg);
        mode_pub_.publish(mode_msg);
      }
    }
    catch (const NekonoteException& err)
    {
      ROS_ERROR("In %s() in the thread communicating with NEKONOTE: %s", err.where(), err.what());
      ROS_INFO("The thread communicating with NEKONOTE terminated. Shutting down %s ...",
               ros::this_node::getName().c_str());
      ros::shutdown();
      return;
    }
    catch (const InterruptException& err)
    {
    }
    std::cout << "The thread communicating with NEKONOTE terminated." << std::endl;
  }

  void threadJoin()
  {
    thread_comm_with_nekonote_.join();
  }
  /**********************************************/

  /***************ROS callback functions***************/
  void onJointCommand(const nekonote_msgs::JointCommand::ConstPtr& joint_command)
  {
    if (joint_command->command.size() != joint_command->name.size())
    {
      ROS_ERROR("On %s/joint_command: number of joint names and commands are not the same. Aborted.", ns_.c_str());
      return;
    }
    else if (joint_command->name.size() != joint_names_.size())
    {
      ROS_ERROR("On %s/joint_command: number of joint names does not equal joint number NekonoteDriverForROS was "
                "told. Aborted.",
                ns_.c_str());
      return;
    }

    std::vector<double> command;
    for (std::vector<std::string>::const_iterator itr_for = joint_names_.begin(); itr_for != joint_names_.end();
         ++itr_for)
    {
      std::vector<std::string>::const_iterator itr_in =
          std::find(joint_command->name.begin(), joint_command->name.end(), *itr_for);
      if (itr_in == joint_command->name.end())
      {
        ROS_ERROR("On %s/joint_command: joint \"%s\" is not found in command. Aborted.", ns_.c_str(),
                  (*itr_for).c_str());
        return;
      }
      command.push_back(joint_command->command[itr_in - joint_command->name.begin()]);
    }

    if (joint_command->mode == nekonote_msgs::JointCommand::POSITION_CONTROL)
    {
      int tm = joint_command->time;
      if (tm <= 0)
      {
        ROS_ERROR("On %s/joint_command: time's value is not more than 0. It is invalid. Aborted.", ns_.c_str());
        return;
      }
      LockGuard _l(lock_jta_flag_);
      if (is_sending_jt_)
      {
        ROS_ERROR("On %s/joint_command: a joint trajectory is being sent to NEKONOTE on %s. While that, do not send "
                  "joint position command. Aborted.",
                  ns_.c_str(), jta_ns_.c_str());
        return;
      }
      RequestItemPtr request_move = boost::make_shared<RequestItemPushLinkAngles>(command, tm);
      request_queue_.pushRequest(request_move);
    }
    else if (joint_command->mode == nekonote_msgs::JointCommand::TORQUE_CONTROL)
    {
      RequestItemPtr request_move = boost::make_shared<RequestItemTorqueControl>(command);
      request_queue_.pushRequest(request_move);
    }
    else
      ROS_ERROR("On %s/joint_command: mode number is invalid. Aborted.", ns_.c_str());
  }

  void onModeCommand(const nekonote_msgs::Mode::ConstPtr& mode_command)
  {
    if (mode_command->state == nekonote_msgs::Mode::FREE)
    {
      RequestItemPtr request = boost::make_shared<RequestItemSetAsFreeMode>();
      request_queue_.pushRequest(request);
    }
    else if (mode_command->state == nekonote_msgs::Mode::POSITION_CONTROL)
    {
      RequestItemPtr request = boost::make_shared<RequestItemSetAsPositionControlMode>();
      request_queue_.pushRequest(request);
    }
    else
      ROS_ERROR("On %s/mode_command: mode number is invalid. Aborted.", ns_.c_str());
  }

  void onFlushMotion(const std_msgs::Empty::ConstPtr& command)
  {
    LockGuard _l(lock_jta_flag_);
    if (is_sending_jt_)
    {
      ROS_ERROR("On %s/flush_motion: a joint trajectory is being sent to NEKONOTE on %s. While that, do not send "
                "flush motion command. Aborted.",
                ns_.c_str(), jta_ns_.c_str());
      return;
    }
    RequestItemPtr request_flush = boost::make_shared<RequestItemFlushMotion>();
    request_queue_.pushRequest(request_flush);
    RequestItemPtr request_start = boost::make_shared<RequestItemStartMotion>();
    request_queue_.pushRequest(request_start);
  }

  void onJointTrajectoryAction(const nekonote_msgs::JointTrajectoryGoalConstPtr& goal)
  {
    ROS_INFO("On %s: catch a joint trajectory", jta_ns_.c_str());
    nekonote_msgs::JointTrajectoryResult as_result;
    // Check number of joint names
    if (goal->trajectory.joint_names.size() != joint_names_.size())
    {
      std::string err_str("Number of joint names in the goal does not equal joint number NekonoteDriverForROS was "
                          "told.");
      ROS_ERROR("On %s: %s Aborted.", jta_ns_.c_str(), err_str.c_str());
      as_result.error_code = as_result.INVALID_JOINTS;
      as_result.error_string = err_str;
      as_.setAborted(as_result);
      return;
    }

    // Check joint name's order
    std::vector<int> joint_index;
    for (std::vector<std::string>::const_iterator itr_for = joint_names_.begin(); itr_for != joint_names_.end();
         ++itr_for)
    {
      std::vector<std::string>::const_iterator itr_in =
          std::find(goal->trajectory.joint_names.begin(), goal->trajectory.joint_names.end(), *itr_for);
      if (itr_in == goal->trajectory.joint_names.end())
      {
        std::string err_str = (std::string)("Joint \"") + (*itr_for) + "\" is not found in the goal.";
        ROS_ERROR("On %s: %s Aborted.", jta_ns_.c_str(), err_str.c_str());
        as_result.error_code = as_result.INVALID_JOINTS;
        as_result.error_string = err_str;
        as_.setAborted(as_result);
        return;
      }
      joint_index.push_back(itr_in - goal->trajectory.joint_names.begin());
    }

    // Check if the commands are valid
    if (goal->trajectory.points.size() == 0)
    {
      std::string err_str("The trajectory is empty.");
      ROS_ERROR("On %s: %s Aborted.", jta_ns_.c_str(), err_str.c_str());
      as_result.error_code = as_result.INVALID_GOAL;
      as_result.error_string = err_str;
      as_.setAborted(as_result);
      return;
    }
    ros::Duration past_time_from_start(0);
    for (int i = 0; i < goal->trajectory.points.size(); i++)
    {
      if (goal->trajectory.points[i].positions.size() != joint_names_.size())
      {
        std::string err_str("Number of joint names and size of positions in the goal are not the same.");
        ROS_ERROR("On %s: %s Aborted.", jta_ns_.c_str(), err_str.c_str());
        as_result.error_code = as_result.INVALID_GOAL;
        as_result.error_string = err_str;
        as_.setAborted(as_result);
        return;
      }
      if (i == 0)
      {
        if (goal->trajectory.points[0].time_from_start < past_time_from_start)
        {
          std::string err_str("time_from_start of the first point in the goal is less than 0.");
          ROS_ERROR("On %s: %s It is invalid. Aborted.", jta_ns_.c_str(), err_str.c_str());
          as_result.error_code = as_result.INVALID_GOAL;
          as_result.error_string = err_str;
          as_.setAborted(as_result);
          return;
        }
      }
      else if (goal->trajectory.points[i].time_from_start <= past_time_from_start)
      {
        std::string err_str("time_from_start of a point in the goal is not more than the previous point.");
        ROS_ERROR("On %s: %s It is invalid. Aborted.", jta_ns_.c_str(), err_str.c_str());
        as_result.error_code = as_result.INVALID_GOAL;
        as_result.error_string = err_str;
        as_.setAborted(as_result);
        return;
      }
      past_time_from_start = goal->trajectory.points[i].time_from_start;
    }

    // Tell other ROS callbacks to push nothing to motion queue in NEKONOTE 
    lock_jta_flag_.lock();
    is_sending_jt_ = true;
    lock_jta_flag_.unlock();

    ROS_INFO("On %s: executing requested joint trajectory...", jta_ns_.c_str());
    // Stop motion queue in NEKONOTE
    RequestItemPtr request_stop = boost::make_shared<RequestItemStopMotion>();
    request_queue_.pushRequest(request_stop);

    // Push trajectory requests to queue
    std::vector<RequestItemPtr> request_list;
    past_time_from_start = ros::Duration(0);
    int tm_err = 0;
    for (int i = 0; i < goal->trajectory.points.size(); i++)
    {
      std::vector<double> angles;
      for (int j = 0; j < joint_index.size(); j++)
        angles.push_back(goal->trajectory.points[i].positions[joint_index[j]]);
      int tm = ((goal->trajectory.points[i].time_from_start - past_time_from_start).toSec() * 1000.0);
      if (tm == 0)
      {
        tm = 1;
        tm_err += tm;
      }
      else if (tm_err > 0 && tm > tm_err)
      {
        tm -= tm_err;
        tm_err = 0;
      }
      else if (tm_err > 0)
      {
        tm_err -= (tm - 1);
        tm = 1;
      }

      if (i == 0)
      {
        RequestItemPtr request = boost::make_shared<RequestItemPushStartAngles>(angles, tm);
        request_list.push_back(request);
        request_queue_.pushRequest(request);
      }
      else if (i == (goal->trajectory.points.size() - 1))
      {
        RequestItemPtr request = boost::make_shared<RequestItemPushEndAngles>(angles, tm, false);
        request_list.push_back(request);
        request_queue_.pushRequest(request);
      }
      else
      {
        RequestItemPtr request = boost::make_shared<RequestItemPushMiddleAngles>(angles, tm);
        request_list.push_back(request);
        request_queue_.pushRequest(request);
      }
      past_time_from_start = goal->trajectory.points[i].time_from_start;
    }

    lock_jta_flag_.lock();
    is_sending_jt_ = false;
    lock_jta_flag_.unlock();

    // Get motion id and publish as feedback
    Time time;
    nekonote_msgs::JointTrajectoryFeedback feedback;
    for (std::vector<RequestItemPtr>::iterator itr = request_list.begin(); itr != request_list.end(); ++itr)
    {
      while (!((*itr)->isProcessed()) && ros::ok())
        time.msleep(1);
      if (!ros::ok())
      {
        std::cout << "On " << jta_ns_ << ": the node was interrupted. Aborted." << std::endl;
        as_.setAborted();
        return;
      }

      std::vector<std::string> result = (*itr)->getReturnValue();
      if (result.size() != 1)
      {
        ROS_ERROR("On %s: cannot get motion id. Maybe bug exists in this driver. So, emergency stop. Please shutdown "
                  "this driver and NEKONOTE. Aborted.",
                  jta_ns_.c_str());
        RequestItemPtr request = boost::make_shared<RequestItemGoEmergencyState>();
        request_queue_.pushHighestPriorRequest(request);
        as_.setAborted();
        return;
      }
      feedback.motion_id.push_back(atoi(result[0].c_str()));
    }
    as_.publishFeedback(feedback);

    // Sleep until the trajectory start time
    RequestItemPtr request_start = boost::make_shared<RequestItemStartMotion>();
    while (goal->trajectory.header.stamp > ros::Time::now() && ros::ok())
    {
      time.msleep(1);
      if (as_.isPreemptRequested())
      {
        ROS_INFO("On %s: preempt is requested by the client. Preempted...", jta_ns_.c_str());
        as_.setPreempted();
        ROS_INFO("On %s: flush motion queue in NEKONOTE to clean unnecessary joint trajectory.", jta_ns_.c_str());
        RequestItemPtr request_flush = boost::make_shared<RequestItemFlushMotion>();
        request_queue_.pushRequest(request_flush);
        request_queue_.pushRequest(request_start);
        return;
      }
    }
    if (!ros::ok())
    {
      std::cout << "On " << jta_ns_ << ": the node was interrupted. Aborted." << std::endl;
      as_.setAborted();
      return;
    }

    // Check if NEKONOTE stops at the trajectory start time
    RequestItemPtr request_get_all_data = boost::make_shared<RequestItemGetAllData>();
    request_queue_.pushRequest(request_get_all_data);
    while (!(request_get_all_data->isProcessed()) && ros::ok())
      time.msleep(1);
    if (!ros::ok())
    {
      std::cout << "On " << jta_ns_ << ": the node was interrupted. Aborted." << std::endl;
      as_.setAborted();
      return;
    }
    std::vector<std::string> result_stop = request_stop->getReturnValue();
    std::vector<std::string> result_get_all_data = request_get_all_data->getReturnValue();
    if (result_stop.size() != 1 || result_get_all_data.size() != (joint_names_.size() * 4 + 2))
    {
      ROS_ERROR("On %s: cannot get motion id. Maybe bug exists in this driver. So, emergency stop. Please shutdown "
                "this driver and NEKONOTE. Aborted.",
                jta_ns_.c_str());
      RequestItemPtr request = boost::make_shared<RequestItemGoEmergencyState>();
      request_queue_.pushHighestPriorRequest(request);
      as_.setAborted();
      return;
    }
    int stop_motion_id = atoi(result_stop[0].c_str());
    int current_last_motion_id = atoi(result_get_all_data[(joint_names_.size() * 2)].c_str());
    if (current_last_motion_id != stop_motion_id)
    {
      std::string err_str("NEKONOTE does not stop at the trajectory start time.");
      ROS_ERROR("On %s: %s Aborted...", jta_ns_.c_str(), err_str.c_str());
      as_result.error_code = as_result.INVALID_GOAL;
      as_result.error_string = err_str;
      as_.setAborted(as_result);
      ROS_INFO("On %s: flush motion queue in NEKONOTE to clean unnecessary joint trajectory.", jta_ns_.c_str());
      RequestItemPtr request_flush = boost::make_shared<RequestItemFlushMotion>();
      request_queue_.pushRequest(request_flush);
      request_queue_.pushRequest(request_start);
      return;
    }

    request_queue_.pushRequest(request_start);
    ROS_INFO("On %s: the joint trajectory was successfully started. Joint Trajectory Action succeeded.",
             jta_ns_.c_str());
    as_result.error_code = as_result.SUCCESSFUL;
    as_.setSucceeded(as_result);
  }
};  // end class NekonoteDriverForROS
};  // end namespace rt_nekonote

#endif  // NEKONOTE_DRIVER_FOR_ROS_H
