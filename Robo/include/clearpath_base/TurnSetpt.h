#ifndef _ROS_clearpath_base_TurnSetpt_h
#define _ROS_clearpath_base_TurnSetpt_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace clearpath_base
{

  class TurnSetpt : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float trans_vel;
      float turn_radius;
      float trans_accel;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      int32_t * val_trans_vel = (int32_t *) &(this->trans_vel);
      int32_t exp_trans_vel = (((*val_trans_vel)>>23)&255);
      if(exp_trans_vel != 0)
        exp_trans_vel += 1023-127;
      int32_t sig_trans_vel = *val_trans_vel;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_trans_vel<<5) & 0xff;
      *(outbuffer + offset++) = (sig_trans_vel>>3) & 0xff;
      *(outbuffer + offset++) = (sig_trans_vel>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_trans_vel<<4) & 0xF0) | ((sig_trans_vel>>19)&0x0F);
      *(outbuffer + offset++) = (exp_trans_vel>>4) & 0x7F;
      if(this->trans_vel < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_turn_radius = (int32_t *) &(this->turn_radius);
      int32_t exp_turn_radius = (((*val_turn_radius)>>23)&255);
      if(exp_turn_radius != 0)
        exp_turn_radius += 1023-127;
      int32_t sig_turn_radius = *val_turn_radius;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_turn_radius<<5) & 0xff;
      *(outbuffer + offset++) = (sig_turn_radius>>3) & 0xff;
      *(outbuffer + offset++) = (sig_turn_radius>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_turn_radius<<4) & 0xF0) | ((sig_turn_radius>>19)&0x0F);
      *(outbuffer + offset++) = (exp_turn_radius>>4) & 0x7F;
      if(this->turn_radius < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_trans_accel = (int32_t *) &(this->trans_accel);
      int32_t exp_trans_accel = (((*val_trans_accel)>>23)&255);
      if(exp_trans_accel != 0)
        exp_trans_accel += 1023-127;
      int32_t sig_trans_accel = *val_trans_accel;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_trans_accel<<5) & 0xff;
      *(outbuffer + offset++) = (sig_trans_accel>>3) & 0xff;
      *(outbuffer + offset++) = (sig_trans_accel>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_trans_accel<<4) & 0xF0) | ((sig_trans_accel>>19)&0x0F);
      *(outbuffer + offset++) = (exp_trans_accel>>4) & 0x7F;
      if(this->trans_accel < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t * val_trans_vel = (uint32_t*) &(this->trans_vel);
      offset += 3;
      *val_trans_vel = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_trans_vel |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_trans_vel |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_trans_vel |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_trans_vel = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_trans_vel |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_trans_vel !=0)
        *val_trans_vel |= ((exp_trans_vel)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->trans_vel = -this->trans_vel;
      uint32_t * val_turn_radius = (uint32_t*) &(this->turn_radius);
      offset += 3;
      *val_turn_radius = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_turn_radius |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_turn_radius |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_turn_radius |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_turn_radius = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_turn_radius |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_turn_radius !=0)
        *val_turn_radius |= ((exp_turn_radius)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->turn_radius = -this->turn_radius;
      uint32_t * val_trans_accel = (uint32_t*) &(this->trans_accel);
      offset += 3;
      *val_trans_accel = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_trans_accel |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_trans_accel |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_trans_accel |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_trans_accel = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_trans_accel |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_trans_accel !=0)
        *val_trans_accel |= ((exp_trans_accel)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->trans_accel = -this->trans_accel;
     return offset;
    }

    const char * getType(){ return "clearpath_base/TurnSetpt"; };
    const char * getMD5(){ return "023314e739de17bd5207788d54c661df"; };

  };

}
#endif