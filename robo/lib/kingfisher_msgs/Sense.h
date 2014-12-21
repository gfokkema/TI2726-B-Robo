#ifndef _ROS_kingfisher_msgs_Sense_h
#define _ROS_kingfisher_msgs_Sense_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace kingfisher_msgs
{

  class Sense : public ros::Msg
  {
    public:
      float battery;
      float current_left;
      float current_right;
      float pcb_temperature;
      bool fan_on;
      uint8_t rc;
      uint16_t rc_throttle;
      uint16_t rc_rotation;
      uint16_t rc_enable;
      enum { RC_INRANGE = 1 };
      enum { RC_INUSE = 2 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_battery;
      u_battery.real = this->battery;
      *(outbuffer + offset + 0) = (u_battery.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery);
      union {
        float real;
        uint32_t base;
      } u_current_left;
      u_current_left.real = this->current_left;
      *(outbuffer + offset + 0) = (u_current_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_left);
      union {
        float real;
        uint32_t base;
      } u_current_right;
      u_current_right.real = this->current_right;
      *(outbuffer + offset + 0) = (u_current_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_right);
      union {
        float real;
        uint32_t base;
      } u_pcb_temperature;
      u_pcb_temperature.real = this->pcb_temperature;
      *(outbuffer + offset + 0) = (u_pcb_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pcb_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pcb_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pcb_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pcb_temperature);
      union {
        bool real;
        uint8_t base;
      } u_fan_on;
      u_fan_on.real = this->fan_on;
      *(outbuffer + offset + 0) = (u_fan_on.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fan_on);
      *(outbuffer + offset + 0) = (this->rc >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rc);
      *(outbuffer + offset + 0) = (this->rc_throttle >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rc_throttle >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rc_throttle);
      *(outbuffer + offset + 0) = (this->rc_rotation >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rc_rotation >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rc_rotation);
      *(outbuffer + offset + 0) = (this->rc_enable >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rc_enable >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rc_enable);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_battery;
      u_battery.base = 0;
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery = u_battery.real;
      offset += sizeof(this->battery);
      union {
        float real;
        uint32_t base;
      } u_current_left;
      u_current_left.base = 0;
      u_current_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_left = u_current_left.real;
      offset += sizeof(this->current_left);
      union {
        float real;
        uint32_t base;
      } u_current_right;
      u_current_right.base = 0;
      u_current_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_right = u_current_right.real;
      offset += sizeof(this->current_right);
      union {
        float real;
        uint32_t base;
      } u_pcb_temperature;
      u_pcb_temperature.base = 0;
      u_pcb_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pcb_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pcb_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pcb_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pcb_temperature = u_pcb_temperature.real;
      offset += sizeof(this->pcb_temperature);
      union {
        bool real;
        uint8_t base;
      } u_fan_on;
      u_fan_on.base = 0;
      u_fan_on.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fan_on = u_fan_on.real;
      offset += sizeof(this->fan_on);
      this->rc =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->rc);
      this->rc_throttle =  ((uint16_t) (*(inbuffer + offset)));
      this->rc_throttle |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->rc_throttle);
      this->rc_rotation =  ((uint16_t) (*(inbuffer + offset)));
      this->rc_rotation |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->rc_rotation);
      this->rc_enable =  ((uint16_t) (*(inbuffer + offset)));
      this->rc_enable |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->rc_enable);
     return offset;
    }

    const char * getType(){ return "kingfisher_msgs/Sense"; };
    const char * getMD5(){ return "b0209acddc7f26d2e3dc1338d6c4df94"; };

  };

}
#endif