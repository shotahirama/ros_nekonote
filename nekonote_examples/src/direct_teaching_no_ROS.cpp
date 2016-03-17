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
#include "nekonote_driver/nekonote_driver.h"  //Please include this after including ros/ros.h if you use ROS

#include <iostream>

using rt_nekonote::NekonoteDriver;
using rt_nekonote::InterruptException;
using rt_nekonote::NekonoteException;

int main(int argc, char** argv)
{
  NekonoteDriver nekonote("192.168.1.54", 1023, false);

  try
  {
    nekonote.startConnection();
    std::cout << "Connection OK" << std::endl;

    nekonote.flushMotion();  // in order to flush and stop motion que in NEKONOTE

    nekonote.setAsFreeMode();

    std::vector<std::vector<double> > angles_list;
    std::vector<double> timestamp;
    std::cout << "Start recording. Recording will automatically stop in about 10 seconds" << std::endl;
    rt_nekonote::Time time_man;
    struct timeval start_time;
    gettimeofday(&start_time, NULL);
    for (int i = 0; i < 100; i++)
    {
      time_man.setTimeMark();
      std::vector<double> angles;
      nekonote.getLinkAngles(&angles);
      angles_list.push_back(angles);
      time_man.msleepFromTimeMark(100);
      struct timeval now_time;
      gettimeofday(&now_time, NULL);
      double elapsed_time =
          (double)(now_time.tv_sec - start_time.tv_sec) + (double)(now_time.tv_usec - start_time.tv_usec) * 1.0E-6;
      timestamp.push_back(elapsed_time);
    }

    std::cout << "Stop recording. After 5 seconds, go to initial state" << std::endl;
    nekonote.setAsPositionControlMode();
    sleep(5);

    std::cout << "Go to initial state" << std::endl;
    int tm;
    int motion_id;
    nekonote.startMotion();
    nekonote.pushEndAngles(*(angles_list.begin()), 2000, &motion_id);
    angles_list.erase(angles_list.begin());

    std::cout << "Send trajectory data. Playback will automatically start after 3 seconds" << std::endl;
    time_man.setTimeMark();
    tm = (*(timestamp.begin() + 1) - *(timestamp.begin())) * 1000;
    nekonote.pushStartAngles(*(angles_list.begin()), tm, &motion_id);
    angles_list.erase(angles_list.begin());
    timestamp.erase(timestamp.begin());
    std::vector<double>::iterator itr_time = timestamp.begin();
    for (std::vector<std::vector<double> >::iterator itr_ang = angles_list.begin(); itr_ang != (angles_list.end() - 1);
         ++itr_ang, ++itr_time)
    {
      tm = (*(itr_time + 1) - *itr_time) * 1000;
      nekonote.pushMiddleAngles(*itr_ang, tm, &motion_id);
    }
    tm = (*(timestamp.end() - 1) - *(timestamp.end() - 2)) * 1000;
    nekonote.pushEndAngles(*(angles_list.end() - 1), tm, &motion_id);

    time_man.msleepFromTimeMark(3000);  // if you set this sleep time under 2000 ms, startMotion may be ignored by
                                        // NEKONOTE because executing pushEndAngles doesn't end and motion queue in
                                        // NEKONOTE doesn't stop
    std::cout << "Start playback" << std::endl;
    nekonote.startMotion();
  }
  catch (const NekonoteException& err)
  {
    std::cout << "[ERROR] In " << err.where() << "(): " << err.what() << std::endl;
  }
  catch (const InterruptException& err)
  {
  }

  return 0;
}
