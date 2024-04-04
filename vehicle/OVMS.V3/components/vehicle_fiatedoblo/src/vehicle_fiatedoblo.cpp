/*
;    Project:       Open Vehicle Monitor System
;    Date:          4nd April 2024
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2011       Michael Stegen / Stegen Electronics
;    (C) 2011-2017  Mark Webb-Johnson
;    (C) 2011       Sonny Chen @ EPRO/DX
;    (C) 2024       Raphael Mack
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#include "ovms_log.h"
static const char *TAG = "v-fiatedoblo";

#include <stdio.h>
#include "vehicle_fiatedoblo.h"

static const OvmsPoller::poll_pid_t obdii_polls[]
  =
  {
    { 0x7df, 0, VEHICLE_POLL_TYPE_OBDIICURRENT, 0x0d, {  0,  1,  10 }, 0, ISOTP_STD }, // Speed (km/h)
    { 0x7df, 0, VEHICLE_POLL_TYPE_OBDIICURRENT, 0x1f, {  0,  3,  10 }, 0, ISOTP_STD }, // runtime since engine start (s)
    { 0x7df, 0, VEHICLE_POLL_TYPE_OBDIICURRENT, 0x2f, {  0,  5,  30 }, 0, ISOTP_STD }, // Fuel level
    { 0x7df, 0, VEHICLE_POLL_TYPE_OBDIICURRENT, 0x46, {  0, 30,  30 }, 0, ISOTP_STD }, // Ambiant temp
    { 0x7df, 0, VEHICLE_POLL_TYPE_OBDIIVEHICLE, 0x02, {999, 60, 999 }, 0, ISOTP_STD }, // VIN
    POLL_LIST_END
  };

OvmsVehicleFiatEDoblo::OvmsVehicleFiatEDoblo()
  {
  ESP_LOGI(TAG, "Fiat e-Doblo vehicle module");
  RegisterCanBus(1, CAN_MODE_ACTIVE, CAN_SPEED_500KBPS);
  PollSetPidList(m_can1, obdii_polls);
  PollSetState(1);
  }

OvmsVehicleFiatEDoblo::~OvmsVehicleFiatEDoblo()
  {
  ESP_LOGI(TAG, "Shutdown Fiat e-Doblo vehicle module");
  }

class OvmsVehicleFiatEDobloInit
  {
  public: OvmsVehicleFiatEDobloInit();
} MyOvmsVehicleFiatEDobloInit  __attribute__ ((init_priority (9000)));

OvmsVehicleFiatEDobloInit::OvmsVehicleFiatEDobloInit()
  {
  ESP_LOGI(TAG, "Registering Vehicle: Fiat e-Doblo (9000)");

  MyVehicleFactory.RegisterVehicle<OvmsVehicleFiatEDoblo>("FTDO", "Fiat e-Doblo");
  }

void OvmsVehicleFiatEDoblo::IncomingFrameCan1(CAN_frame_t* p_frame)
  {
  uint8_t *d = p_frame->data.u8;
  switch (p_frame->MsgID)
    {
    case 0x3a8:
      {
	StandardMetrics.ms_v_bat_soc->SetValue((float)d[3]);
      }
    }
  }

void OvmsVehicleFiatEDoblo::IncomingPollReply(const OvmsPoller::poll_job_t &job, uint8_t* data, uint8_t length)
  {
  int value1 = (int)data[0];
  int value2 = ((int)data[0] << 8) + (int)data[1];

  switch (job.pid)
    {
    case 0x02:  // VIN (multi-line response)
      // Data in the first frame starts with 0x01 for some (all?) vehicles
      if (length > 1 && data[0] == 0x01)
        {
        ++data;
        --length;
        }
      strncat(m_vin, (char*)data, length);
      if (job.mlremain==0)
        {
        StandardMetrics.ms_v_vin->SetValue(m_vin);
        m_vin[0] = 0;
        }
      break;
    case 0x1f:  // runtime since engine start
      ESP_LOGI(TAG, "runtime since engine start %d", value2);
      StandardMetrics.ms_v_env_drivetime->SetValue(value2);
      break;
    case 0x46:  // Ambient temperature
      StandardMetrics.ms_v_env_temp->SetValue(value1 - 0x28);
      break;
    case 0x0d:  // Speed
      StandardMetrics.ms_v_pos_speed->SetValue(value1);
      break;
    case 0x2f:  // Fuel Level
      //StandardMetrics.ms_v_bat_soc->SetValue((value1 * 100) >> 8);
      ESP_LOGI(TAG, "received fuel level: %d", ((value1 * 100) >> 8));
      break;
    }
  }
