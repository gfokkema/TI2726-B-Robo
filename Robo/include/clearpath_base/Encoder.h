#ifndef _ROS_clearpath_base_Encoder_h
#define _ROS_clearpath_base_Encoder_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace clearpath_base
{

  class Encoder : public ros::Msg
  {
    public:
      float travel;
      float speed;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      int32_t * val_travel = (int32_t *) &(this->travel);
      int32_t exp_travel = (((*val_travel)>>23)&255);
      if(exp_travel != 0)
        exp_travel += 1023-127;
      int32_t sig_travel = *val_travel;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_travel<<5) & 0xff;
      *(outbuffer + offset++) = (sig_travel>>3) & 0xff;
      *(outbuffer + offset++) = (sig_travel>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_travel<<4) & 0xF0) | ((sig_travel>>19)&0x0F);
      *(outbuffer + offset++) = (exp_travel>>4) & 0x7F;
      if(this->travel < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_speed = (int32_t *) &(this->speed);
      int32_t exp_speed = (((*val_speed)>>23)&255);
      if(exp_speed != 0)
        exp_speed += 1023-127;
      int32_t sig_speed = *val_speed;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_speed<<5) & 0xff;
      *(outbuffer + offset++) = (sig_speed>>3) & 0xff;
      *(outbuffer + offset++) = (sig_speed>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_speed<<4) & 0xF0) | ((sig_speed>>19)&0x0F);
      *(outbuffer + offset++) = (exp_speed>>4) & 0x7F;
      if(this->speed < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t * val_travel = (uint32_t*) &(this->travel);
      offset += 3;
      *val_travel = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_travel |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_travel |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_travel |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_travel = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_travel |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_travel !=0)
        *val_travel |= ((exp_travel)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->travel = -this->travel;
      uint32_t * val_speed = (uint32_t*) &(this->speed);
      offset += 3;
      *val_speed = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_speed |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_speed |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_speed |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_speed = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_speed |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_speed !=0)
        *val_speed |= ((exp_speed)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->speed = -this->speed;
     return offset;
    }

    const char * getType(){ return "clearpath_base/Encoder"; };
    const char * getMD5(){ return "620a74fe7e8deb0e96bf85b534453633"; };

  };

}
#endif