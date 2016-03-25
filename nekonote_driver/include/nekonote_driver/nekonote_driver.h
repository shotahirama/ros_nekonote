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
#ifndef NEKONOTE_DRIVER_H
#define NEKONOTE_DRIVER_H

#include <sys/time.h>
#include <sys/fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <vector>
#include <sstream>

#include <nekonote_driver/picojson.h>

#define TCP_MSS (1460)

#ifdef ROS_DEBUG
#define OUTPUT_LOG(x, ...)                                                                                             \
  {                                                                                                                    \
    if (x)                                                                                                             \
    {                                                                                                                  \
      ROS_DEBUG(__VA_ARGS__);                                                                                          \
    }                                                                                                                  \
  }
#define OUTPUT_INFO(...) ROS_INFO(__VA_ARGS__)
#define OUTPUT_WARN(x, ...)                                                                                            \
  {                                                                                                                    \
    if (x)                                                                                                             \
    {                                                                                                                  \
      ROS_WARN(__VA_ARGS__);                                                                                           \
    }                                                                                                                  \
  }
#define IS_INTERRUPT (!ros::ok())
#else
#include <stdio.h>
#define OUTPUT_LOG(x, fmt, ...)                                                                                        \
  {                                                                                                                    \
    if (x)                                                                                                             \
    {                                                                                                                  \
      printf(fmt "\n", ##__VA_ARGS__);                                                                                 \
    }                                                                                                                  \
  }                                                                      // GCC only
#define OUTPUT_INFO(fmt, ...) printf("[INFO] " fmt "\n", ##__VA_ARGS__)  // GCC only
#define OUTPUT_WARN(x, fmt, ...)                                                                                       \
  {                                                                                                                    \
    if (x)                                                                                                             \
    {                                                                                                                  \
      printf("[WARN] " fmt "\n", ##__VA_ARGS__);                                                                       \
    }                                                                                                                  \
  }  // GCC only
#define IS_INTERRUPT false
#endif  // ROS_DEBUG

namespace rt_nekonote
{
/**********************************************************************************************************
 *
 * Exceptions
 *
 **********************************************************************************************************/

class InterruptException
{
public:
  InterruptException() throw()
  {
  }

  ~InterruptException()
  {
  }
};

class NekonoteException
{
private:
  std::string err_;
  std::string loc_;

public:
  NekonoteException(const std::string err = "", const std::string loc = "") throw() : err_(err), loc_(loc)
  {
  }

  ~NekonoteException()
  {
  }

  const char* what() const throw()
  {
    return err_.c_str();
  }

  const char* where() const throw()
  {
    return loc_.c_str();
  }
};

/**********************************************************************************************************
 *
 * Time management class
 *
 **********************************************************************************************************/

class Time
{
private:
  struct timeval mark_;

public:
  Time()
  {
    setTimeMark();
  }

  void usleep(long us)
  {
    struct timespec ts;
    ts.tv_sec = us / 1000000;
    ts.tv_nsec = (us % 1000000) * 1000;
    nanosleep(&ts, NULL);
  }

  void msleep(int ms)
  {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000;
    nanosleep(&ts, NULL);
  }

  void setTimeMark()
  {
    gettimeofday(&mark_, NULL);
  }

  int getmsFromTimeMark()
  {
    struct timeval now;
    gettimeofday(&now, NULL);

    return ((now.tv_sec - mark_.tv_sec) * 1000 + (double)(now.tv_usec - mark_.tv_usec) * 0.001);
  }

  void msleepFromTimeMark(int ms)
  {
    long sleep_sec = ms / 1000;
    long sleep_nsec = (ms % 1000) * 1000000;
    struct timeval now;
    gettimeofday(&now, NULL);
    long sec_diff = sleep_sec - ((long)now.tv_sec - (long)mark_.tv_sec);
    if (sec_diff >= 0)
    {
      long nsec_diff = sleep_nsec - ((long)now.tv_usec - (long)mark_.tv_usec) * 1000;
      if (nsec_diff < 0)
      {
        sec_diff--;
        nsec_diff += 1000000000;
      }
      if (sec_diff >= 0)
      {
        struct timespec ts;
        ts.tv_sec = sec_diff;
        ts.tv_nsec = nsec_diff;
        nanosleep(&ts, NULL);
      }
    }
  }
};

/**********************************************************************************************************
 *
 * Socket communication abstraction
 *
 **********************************************************************************************************/

class SocketCommunication
{
private:
  std::string ip_addr_;  // ex) "192.168.1.54"
  int port_no_;          // ex) 1023
  int sockfd_;
  struct sockaddr_in sin_;
  Time time;

public:
  SocketCommunication(const char* addr = "", const int port = -1) : ip_addr_(addr), port_no_(port), sockfd_(-1)
  {
  }

  ~SocketCommunication()
  {
    Close();
  }

  void Open(const char* addr, const int port)
  {
    ip_addr_ = addr;
    port_no_ = port;
    Open();
  }

  void Open()
  {
    int fd = 0;

    if (sockfd_ > 0)
      return;

    fd = socket(PF_INET, SOCK_STREAM, 0);  // Make socket
    if (fd < 0)
      throw NekonoteException("cannot make socket");

    sockfd_ = fd;

    sin_.sin_family = PF_INET;
    sin_.sin_addr.s_addr = inet_addr(ip_addr_.c_str());
    sin_.sin_port = htons(port_no_);
    int val = 1;
    if (ioctl(sockfd_, FIONBIO, &val) < 0)  // Change socket to be non-blocking
      throw NekonoteException("cannot change socket to be non-blocking");
    int res = connect(sockfd_, (struct sockaddr*)&sin_, sizeof(sin_));
    if (res < 0)
    {
      if (errno == EINPROGRESS)
      {
        while (1)
        {
          const int WARN_INTERVAL = 5;  // WARN interval seconds
          for (int i = 0; i < WARN_INTERVAL; i++)
          {
            fd_set writefds;
            struct timeval timeout;
            FD_ZERO(&writefds);
            FD_SET(sockfd_, &writefds);
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;

            if (select(sockfd_ + 1, NULL, &writefds, NULL, &timeout) < 0)
              throw NekonoteException("socket cannot turn to be ready");
            if (FD_ISSET(sockfd_, &writefds))
              return;

            if (IS_INTERRUPT)
              throw InterruptException();
          }
          OUTPUT_WARN(true, "Cannot complete the socket connection within %d seconds. Retrying...", WARN_INTERVAL);
        }
      }
      throw NekonoteException("cannot connect socket to address");  // Connection error
    }
  }

  void Close()
  {
    if (sockfd_ > 0)
    {
      close(sockfd_);  // Close socket
      sockfd_ = -1;
    }
  }

  int Read(char* buf, const unsigned int buf_len, const char end_char)
  {
    unsigned int left_len = buf_len;
    char* p = buf;
    if (sockfd_ < 0)
      throw NekonoteException("socket does not open yet");

    Time time;
    do
    {
      if (IS_INTERRUPT)
        throw InterruptException();

      int size = read(sockfd_, p, left_len);
      if (size < 0)
      {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
          time.usleep(100);  //To prevent busy wait
          if (time.getmsFromTimeMark() > 5000)
          {
            OUTPUT_WARN(true, "Cannot receive data from NEKONOTE within 5 seconds. Retrying...");
            time.setTimeMark();
          }
          continue;
        }
        else
          throw NekonoteException("cannot read from socket");
      }
      else
      {
        for (int i = 0; i < size; i++)
        {
          p++;
          left_len--;
          if (*(p - 1) == end_char)
          {
            for (int j = 0; j < (size - i - 1); j++)
              *(p + j) = '\0';
            break;
          }
        }
      }
    } while (*(p - 1) != end_char);

    *(p - 1) = '\0';  // replace end character to null character

    return buf_len - left_len;  // Read device file and put the data into buf (max size is (buf_len) bytes)
  }

  int Write(char* data, const unsigned int data_len)
  {
    unsigned int left_len = data_len;
    char* p = data;
    if (sockfd_ < 0)
      throw NekonoteException("socket does not open yet");

    Time time;
    do
    {
      if (IS_INTERRUPT)
        throw InterruptException();

      int size = write(sockfd_, p, left_len);
      if (size < 0)
      {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
          time.usleep(100);  //To prevent busy wait
          if (time.getmsFromTimeMark() > 5000)
          {
            OUTPUT_WARN(true, "Cannot send data to NEKONOTE within 5 seconds. Retrying...");
            time.setTimeMark();
          }
          continue;
        }
        else
          throw NekonoteException("cannot write to socket");
      }
      else
      {
        p += size;
        left_len -= size;
      }
    } while (left_len > 0);

    return data_len - left_len;
  }
};

/**********************************************************************************************************
 *
 * NEKONOTE driver
 *
 **********************************************************************************************************/

class NekonoteDriver
{
private:
  SocketCommunication sock_comm;
  const bool is_debug_;
  const int joint_num_;
  Time time;
  const int STANDBY_TIME;  // Communication standby time[msec]
  unsigned int last_comm_id;

  /***************Message handling function***************/
  std::string makeRequest(const std::string& method_name, const std::string& params, const std::string& id) const
  {
    std::string request;
    request = (std::string)("{\"method\":") + "\"" + method_name + "\",\"params\":" + params + ",\"id\":" + id + "}";
    return request;
  }

  void sendRequest(const std::string& method_name, const std::vector<std::string>& params, const std::string& id)
  {
    std::string params_str = "[";
    if (params.size() == 0)
      throw NekonoteException("cannot make request with no params");
    for (std::vector<std::string>::const_iterator itr = params.begin(); itr != params.end(); ++itr)
    {
      std::string tmp = *itr;
      params_str += tmp;
      params_str += ",";
    }
    params_str.erase(params_str.end() - 1);
    params_str += "]";
    char send_buf[TCP_MSS];
    strcpy(send_buf, makeRequest(method_name, params_str, id).c_str());
    OUTPUT_LOG(is_debug_, "send: %s", send_buf);
    if (sock_comm.Write(send_buf, strlen(send_buf)) != strlen(send_buf))
      throw NekonoteException("cannot write full data to socket");
  }

  void parseResponse(const std::string& response, std::string* result, std::string* error, std::string* id) const
  {
    std::stringstream ss;
    ss << response;
    picojson::value v;
    ss >> v;
    if (ss.fail())
      throw NekonoteException("cannot parse response");
    else if (!v.is<picojson::object>())
      throw NekonoteException("response is not JSON object");
    picojson::object& o = v.get<picojson::object>();
    ss.str("");
    ss << o["result"];
    *result = ss.str();
    ss.str("");
    ss << o["error"];
    *error = ss.str();
    ss.str("");
    ss << o["id"];
    *id = ss.str();
  }

  void getResponse(std::string* result, std::string* error, std::string* id)
  {
    char receive_buf[TCP_MSS];
    sock_comm.Read(receive_buf, sizeof(receive_buf), '\n');
    OUTPUT_LOG(is_debug_, "receive: %s", receive_buf);
    std::string buf(receive_buf);
    parseResponse(buf, result, error, id);
  }

  void sendRequestAndGetResponse(const std::string& method_name, const std::vector<std::string>& params,
                                 std::string* result)
  {
    std::ostringstream os;
    os << last_comm_id;
    std::string send_id = (std::string)("\"") /*+ "ros"*/ + os.str() + "\"";
    sendRequest(method_name, params, send_id);
    std::string error;
    std::string received_id;
    getResponse(result, &error, &received_id);

    // Error check
    if (error != "null")
    {
      std::stringstream ss_e;
      ss_e << error;
      picojson::value v;
      ss_e >> v;
      if (ss_e.fail())
        throw NekonoteException("cannot parse error member in response");
      if (!v.is<picojson::null>())
      {
        if (v.is<picojson::object>())
        {
          picojson::object& o = v.get<picojson::object>();
          throw NekonoteException(o["message"].to_str());
        }
        else
          throw NekonoteException("some error occurred at JSON-RPC call");
      }
    }
    if (received_id != send_id)
      throw NekonoteException("received data's id is different from sended data's id");

    last_comm_id++;
  }

  /***************Common part of procedure call funciton***************/
  void getAllDataFromResult(const std::string& result, std::vector<double>* angles, std::vector<double>* motor_temps,
                            int* last_motion_id, int* mode, std::vector<double>* speed, std::vector<double>* torque)
  {
    angles->clear();
    motor_temps->clear();
    speed->clear();
    torque->clear();
    std::string _result = result;
    _result.erase(_result.begin());
    _result.erase(_result.end() - 1);
    std::stringstream ss(_result);
    std::string buf;
    int cnt = 0;
    while (std::getline(ss, buf, ','))
    {
      if (cnt < joint_num_)
        angles->push_back(atof(buf.c_str()));
      else if (cnt < (joint_num_ * 2))
        motor_temps->push_back(atof(buf.c_str()));
      else if (cnt == (joint_num_ * 2))
        *last_motion_id = atoi(buf.c_str());
      else if (cnt == (joint_num_ * 2 + 1))
        *mode = atoi(buf.c_str());
      else if (cnt < (joint_num_ * 3 + 2))
        speed->push_back(atof(buf.c_str()));
      else if (cnt < (joint_num_ * 4 + 2))
        torque->push_back(atof(buf.c_str()));
      else
        throw NekonoteException("number of joint from NEKONOTE does not equal number of joint NekonoteDriver was told");
      cnt++;
    }
    if (torque->size() != joint_num_)
      throw NekonoteException("number of joint from NEKONOTE does not equal number of joint NekonoteDriver was told");
  }

  void pushAnglesComm(const std::string& method_name, const std::vector<double>& angles, const int tm, int* motion_id,
                      const bool is_stop = false)
  {
    try
    {
      if (angles.size() != joint_num_)
        throw NekonoteException("size of angles does not equal number of joint NekonoteDriver was told");
      std::vector<std::string> params;
      for (std::vector<double>::const_iterator itr = angles.begin(); itr != angles.end(); ++itr)
      {
        std::ostringstream os;
        os << *itr;
        params.push_back(os.str());
      }
      std::ostringstream os;
      os << tm;
      params.push_back(os.str());
      if (method_name == "pushEndAngles")
      {
        os.str("");
        os << (int)(!is_stop);
        params.push_back(os.str());
      }
      time.msleepFromTimeMark(STANDBY_TIME);
      std::string result;
      sendRequestAndGetResponse(method_name, params, &result);
      time.setTimeMark();
      *motion_id = atoi(result.c_str());
    }
    catch (const NekonoteException& err)
    {
      throw NekonoteException(err.what(), (std::string)("NekonoteDriver::") + method_name);
    }
  }

  void noArgAndReturnComm(const std::string& method_name)
  {
    try
    {
      std::vector<std::string> params;
      params.push_back("0");
      time.msleepFromTimeMark(STANDBY_TIME);
      std::string result;
      sendRequestAndGetResponse(method_name, params, &result);
      time.setTimeMark();
    }
    catch (const NekonoteException& err)
    {
      throw NekonoteException(err.what(), (std::string)("NekonoteDriver::") + method_name);
    }
  }

public:
  /***************Initialization***************/
  NekonoteDriver(const std::string& addr = "", const int port = -1, const bool is_debug = true, const int joint_num = 6)
    : sock_comm(addr.c_str(), port), is_debug_(is_debug), joint_num_(joint_num), STANDBY_TIME(0), last_comm_id(0)
  {
  }

  ~NekonoteDriver()
  {
  }

  void startConnection()
  {
    try
    {
      OUTPUT_INFO("Starting connection to NEKONOTE...");
      sock_comm.Open();
      OUTPUT_INFO("Connection to NEKONOTE successfully started");
    }
    catch (const NekonoteException& err)
    {
      throw NekonoteException(err.what(), "NekonoteDriver::startConnection");
    }
  }

  void startConnection(const std::string& addr, const int port)
  {
    try
    {
      sock_comm.Open(addr.c_str(), port);
    }
    catch (const NekonoteException& err)
    {
      throw NekonoteException(err.what(), "NekonoteDriver::startConnection");
    }
  }

  /***************Procedure call funciton***************/
  void getVersion(std::string* version)
  {
    try
    {
      std::vector<std::string> params;
      params.push_back("0");
      time.msleepFromTimeMark(STANDBY_TIME);
      std::string result;
      sendRequestAndGetResponse("getVersion", params, &result);
      time.setTimeMark();
      result.erase(result.begin());
      result.erase(result.end() - 1);
      *version = result;
    }
    catch (const NekonoteException& err)
    {
      throw NekonoteException(err.what(), "NekonoteDriver::getVersion");
    }
  }

  void getLinkAngles(std::vector<double>* angles)
  {
    try
    {
      angles->clear();
      std::vector<std::string> params;
      params.push_back("0");
      time.msleepFromTimeMark(STANDBY_TIME);
      std::string result;
      sendRequestAndGetResponse("getLinkAngles", params, &result);
      time.setTimeMark();
      result.erase(result.begin());
      result.erase(result.end() - 1);
      std::stringstream ss(result);
      std::string buf;
      while (std::getline(ss, buf, ','))
        angles->push_back(atof(buf.c_str()));
      if (angles->size() != joint_num_)
      {
        angles->clear();
        throw NekonoteException("number of joint from NEKONOTE does not equal number of joint NekonoteDriver was "
                                "told");
      }
    }
    catch (const NekonoteException& err)
    {
      throw NekonoteException(err.what(), "NekonoteDriver::getLinkAngles");
    }
  }

  void getAllData(std::vector<double>* angles, std::vector<double>* motor_temps, int* last_motion_id, int* mode,
                  std::vector<double>* speed, std::vector<double>* torque)
  {
    try
    {
      std::vector<std::string> params;
      params.push_back("0");
      time.msleepFromTimeMark(STANDBY_TIME);
      std::string result;
      sendRequestAndGetResponse("getAllData", params, &result);
      time.setTimeMark();
      getAllDataFromResult(result, angles, motor_temps, last_motion_id, mode, speed, torque);
    }
    catch (const NekonoteException& err)
    {
      throw NekonoteException(err.what(), "NekonoteDriver::getAllData");
    }
  }

  void torqueControl(const std::vector<double>& torque_command, std::vector<double>* angles,
                     std::vector<double>* motor_temps, int* last_motion_id, int* mode, std::vector<double>* speed,
                     std::vector<double>* torque)
  {
    try
    {
      if (torque_command.size() != joint_num_)
        throw NekonoteException("size of command does not equal number of joint NekonoteDriver was told");
      std::vector<std::string> params;
      for (std::vector<double>::const_iterator itr = torque_command.begin(); itr != torque_command.end(); ++itr)
      {
        std::ostringstream os;
        os << *itr;
        params.push_back(os.str());
      }
      time.msleepFromTimeMark(STANDBY_TIME);
      std::string result;
      sendRequestAndGetResponse("torqueControl", params, &result);
      time.setTimeMark();
      getAllDataFromResult(result, angles, motor_temps, last_motion_id, mode, speed, torque);
    }
    catch (const NekonoteException& err)
    {
      throw NekonoteException(err.what(), "NekonoteDriver::torqueControl");
    }
  }

  void pushLinkAngles(const std::vector<double>& angles, const int tm, int* motion_id)
  {
    pushAnglesComm("pushLinkAngles", angles, tm, motion_id);
  }

  void pushStartAngles(const std::vector<double>& angles, const int tm, int* motion_id)
  {
    pushAnglesComm("pushStartAngles", angles, tm, motion_id);
  }

  void pushMiddleAngles(const std::vector<double>& angles, const int tm, int* motion_id)
  {
    pushAnglesComm("pushMiddleAngles", angles, tm, motion_id);
  }

  void pushEndAngles(const std::vector<double>& angles, const int tm, int* motion_id, const bool is_stop = true)
  {
    pushAnglesComm("pushEndAngles", angles, tm, motion_id, is_stop);
  }

  void setAsFreeMode()
  {
    noArgAndReturnComm("setAsFreeMode");
  }

  void setAsPositionControlMode()
  {
    noArgAndReturnComm("setAsPositionControlMode");
  }

  void startMotion()
  {
    noArgAndReturnComm("startMotion");
  }

  void stopMotion(int* motion_id)
  {
    try
    {
      std::vector<std::string> params;
      params.push_back("0");
      time.msleepFromTimeMark(STANDBY_TIME);
      std::string result;
      sendRequestAndGetResponse("stopMotion", params, &result);
      time.setTimeMark();
      *motion_id = atoi(result.c_str());
    }
    catch (const NekonoteException& err)
    {
      throw NekonoteException(err.what(), "NekonoteDriver::stopMotion");
    }
  }

  void flushMotion()
  {
    noArgAndReturnComm("flushMotion");
  }

  void goEmergencyState()
  {
    try
    {
      std::vector<std::string> params;
      params.push_back("\"Emergency\"");
      time.msleepFromTimeMark(STANDBY_TIME);
      std::string result;
      sendRequestAndGetResponse("transitionState", params, &result);
      time.setTimeMark();
    }
    catch (const NekonoteException& err)
    {
      throw NekonoteException(err.what(), "NekonoteDriver::goEmergencyState");
    }
  }
};
};  // end namespace rt_nekonote

#endif  // NEKONOTE_DRIVER_H
