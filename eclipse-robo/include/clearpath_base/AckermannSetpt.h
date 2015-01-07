#ifndef _ROS_clearpath_base_AckermannSetpt_h
#define _ROS_clearpath_base_AckermannSetpt_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace clearpath_base
{

  class AckermannSetpt : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float steering;
      float throttle;
      float brake;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      int32_t * val_steering = (int32_t *) &(this->steering);
      int32_t exp_steering = (((*val_steering)>>23)&255);
      if(exp_steering != 0)
        exp_steering += 1023-127;
      int32_t sig_steering = *val_steering;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_steering<<5) & 0xff;
      *(outbuffer + offset++) = (sig_steering>>3) & 0xff;
      *(outbuffer + offset++) = (sig_steering>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_steering<<4) & 0xF0) | ((sig_steering>>19)&0x0F);
      *(outbuffer + offset++) = (exp_steering>>4) & 0x7F;
      if(this->steering < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_throttle = (int32_t *) &(this->throttle);
      int32_t exp_throttle = (((*val_throttle)>>23)&255);
      if(exp_throttle != 0)
        exp_throttle += 1023-127;
      int32_t sig_throttle = *val_throttle;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_throttle<<5) & 0xff;
      *(outbuffer + offset++) = (sig_throttle>>3) & 0xff;
      *(outbuffer + offset++) = (sig_throttle>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_throttle<<4) & 0xF0) | ((sig_throttle>>19)&0x0F);
      *(outbuffer + offset++) = (exp_throttle>>4) & 0x7F;
      if(this->throttle < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_brake = (int32_t *) &(this->brake);
      int32_t exp_brake = (((*val_brake)>>23)&255);
      if(exp_brake != 0)
        exp_brake += 1023-127;
      int32_t sig_brake = *val_brake;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_brake<<5) & 0xff;
      *(outbuffer + offset++) = (sig_brake>>3) & 0xff;
      *(outbuffer + offset++) = (sig_brake>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_brake<<4) & 0xF0) | ((sig_brake>>19)&0x0F);
      *(outbuffer + offset++) = (exp_brake>>4) & 0x7F;
      if(this->brake < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t * val_steering = (uint32_t*) &(this->steering);
      offset += 3;
      *val_steering = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_steering |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_steering |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_steering |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_steering = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_steering |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_steering !=0)
        *val_steering |= ((exp_steering)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->steering = -this->steering;
      uint32_t * val_throttle = (uint32_t*) &(this->throttle);
      offset += 3;
      *val_throttle = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_throttle |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_throttle |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_throttle |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_throttle = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_throttle |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_throttle !=0)
        *val_throttle |= ((exp_throttle)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->throttle = -this->throttle;
      uint32_t * val_brake = (uint32_t*) &(this->brake);
      offset += 3;
      *val_brake = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_brake |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_brake |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_brake |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_brake = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_brake |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_brake !=0)
        *val_brake |= ((exp_brake)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->brake = -this->brake;
     return offset;
    }

    const char * getType(){ return "clearpath_base/AckermannSetpt"; };
    const char * getMD5(){ return "46d60e86aa5967736d40185bb6efb208"; };

  };

}
#endif