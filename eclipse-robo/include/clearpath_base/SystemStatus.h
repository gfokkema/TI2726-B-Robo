#ifndef _ROS_clearpath_base_SystemStatus_h
#define _ROS_clearpath_base_SystemStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace clearpath_base
{

  class SystemStatus : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint32_t uptime;
      uint8_t voltages_length;
      float st_voltages;
      float * voltages;
      uint8_t currents_length;
      float st_currents;
      float * currents;
      uint8_t temperatures_length;
      float st_temperatures;
      float * temperatures;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->uptime >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->uptime >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->uptime >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->uptime >> (8 * 3)) & 0xFF;
      offset += sizeof(this->uptime);
      *(outbuffer + offset++) = voltages_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < voltages_length; i++){
      int32_t * val_voltagesi = (int32_t *) &(this->voltages[i]);
      int32_t exp_voltagesi = (((*val_voltagesi)>>23)&255);
      if(exp_voltagesi != 0)
        exp_voltagesi += 1023-127;
      int32_t sig_voltagesi = *val_voltagesi;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_voltagesi<<5) & 0xff;
      *(outbuffer + offset++) = (sig_voltagesi>>3) & 0xff;
      *(outbuffer + offset++) = (sig_voltagesi>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_voltagesi<<4) & 0xF0) | ((sig_voltagesi>>19)&0x0F);
      *(outbuffer + offset++) = (exp_voltagesi>>4) & 0x7F;
      if(this->voltages[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      *(outbuffer + offset++) = currents_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < currents_length; i++){
      int32_t * val_currentsi = (int32_t *) &(this->currents[i]);
      int32_t exp_currentsi = (((*val_currentsi)>>23)&255);
      if(exp_currentsi != 0)
        exp_currentsi += 1023-127;
      int32_t sig_currentsi = *val_currentsi;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_currentsi<<5) & 0xff;
      *(outbuffer + offset++) = (sig_currentsi>>3) & 0xff;
      *(outbuffer + offset++) = (sig_currentsi>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_currentsi<<4) & 0xF0) | ((sig_currentsi>>19)&0x0F);
      *(outbuffer + offset++) = (exp_currentsi>>4) & 0x7F;
      if(this->currents[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      *(outbuffer + offset++) = temperatures_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < temperatures_length; i++){
      int32_t * val_temperaturesi = (int32_t *) &(this->temperatures[i]);
      int32_t exp_temperaturesi = (((*val_temperaturesi)>>23)&255);
      if(exp_temperaturesi != 0)
        exp_temperaturesi += 1023-127;
      int32_t sig_temperaturesi = *val_temperaturesi;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_temperaturesi<<5) & 0xff;
      *(outbuffer + offset++) = (sig_temperaturesi>>3) & 0xff;
      *(outbuffer + offset++) = (sig_temperaturesi>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_temperaturesi<<4) & 0xF0) | ((sig_temperaturesi>>19)&0x0F);
      *(outbuffer + offset++) = (exp_temperaturesi>>4) & 0x7F;
      if(this->temperatures[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->uptime =  ((uint32_t) (*(inbuffer + offset)));
      this->uptime |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->uptime |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->uptime |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->uptime);
      uint8_t voltages_lengthT = *(inbuffer + offset++);
      if(voltages_lengthT > voltages_length)
        this->voltages = (float*)realloc(this->voltages, voltages_lengthT * sizeof(float));
      offset += 3;
      voltages_length = voltages_lengthT;
      for( uint8_t i = 0; i < voltages_length; i++){
      uint32_t * val_st_voltages = (uint32_t*) &(this->st_voltages);
      offset += 3;
      *val_st_voltages = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_st_voltages |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_st_voltages |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_st_voltages |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_st_voltages = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_st_voltages |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_st_voltages !=0)
        *val_st_voltages |= ((exp_st_voltages)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->st_voltages = -this->st_voltages;
        memcpy( &(this->voltages[i]), &(this->st_voltages), sizeof(float));
      }
      uint8_t currents_lengthT = *(inbuffer + offset++);
      if(currents_lengthT > currents_length)
        this->currents = (float*)realloc(this->currents, currents_lengthT * sizeof(float));
      offset += 3;
      currents_length = currents_lengthT;
      for( uint8_t i = 0; i < currents_length; i++){
      uint32_t * val_st_currents = (uint32_t*) &(this->st_currents);
      offset += 3;
      *val_st_currents = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_st_currents |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_st_currents |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_st_currents |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_st_currents = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_st_currents |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_st_currents !=0)
        *val_st_currents |= ((exp_st_currents)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->st_currents = -this->st_currents;
        memcpy( &(this->currents[i]), &(this->st_currents), sizeof(float));
      }
      uint8_t temperatures_lengthT = *(inbuffer + offset++);
      if(temperatures_lengthT > temperatures_length)
        this->temperatures = (float*)realloc(this->temperatures, temperatures_lengthT * sizeof(float));
      offset += 3;
      temperatures_length = temperatures_lengthT;
      for( uint8_t i = 0; i < temperatures_length; i++){
      uint32_t * val_st_temperatures = (uint32_t*) &(this->st_temperatures);
      offset += 3;
      *val_st_temperatures = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_st_temperatures |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_st_temperatures |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_st_temperatures |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_st_temperatures = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_st_temperatures |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_st_temperatures !=0)
        *val_st_temperatures |= ((exp_st_temperatures)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->st_temperatures = -this->st_temperatures;
        memcpy( &(this->temperatures[i]), &(this->st_temperatures), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "clearpath_base/SystemStatus"; };
    const char * getMD5(){ return "b24850c808eb727058fff35ba598006f"; };

  };

}
#endif