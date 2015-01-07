#ifndef _ROS_clearpath_base_VelocitySetpt_h
#define _ROS_clearpath_base_VelocitySetpt_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace clearpath_base
{

  class VelocitySetpt : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float trans;
      float rot;
      float accel;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      int32_t * val_trans = (int32_t *) &(this->trans);
      int32_t exp_trans = (((*val_trans)>>23)&255);
      if(exp_trans != 0)
        exp_trans += 1023-127;
      int32_t sig_trans = *val_trans;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_trans<<5) & 0xff;
      *(outbuffer + offset++) = (sig_trans>>3) & 0xff;
      *(outbuffer + offset++) = (sig_trans>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_trans<<4) & 0xF0) | ((sig_trans>>19)&0x0F);
      *(outbuffer + offset++) = (exp_trans>>4) & 0x7F;
      if(this->trans < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_rot = (int32_t *) &(this->rot);
      int32_t exp_rot = (((*val_rot)>>23)&255);
      if(exp_rot != 0)
        exp_rot += 1023-127;
      int32_t sig_rot = *val_rot;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_rot<<5) & 0xff;
      *(outbuffer + offset++) = (sig_rot>>3) & 0xff;
      *(outbuffer + offset++) = (sig_rot>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_rot<<4) & 0xF0) | ((sig_rot>>19)&0x0F);
      *(outbuffer + offset++) = (exp_rot>>4) & 0x7F;
      if(this->rot < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_accel = (int32_t *) &(this->accel);
      int32_t exp_accel = (((*val_accel)>>23)&255);
      if(exp_accel != 0)
        exp_accel += 1023-127;
      int32_t sig_accel = *val_accel;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_accel<<5) & 0xff;
      *(outbuffer + offset++) = (sig_accel>>3) & 0xff;
      *(outbuffer + offset++) = (sig_accel>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_accel<<4) & 0xF0) | ((sig_accel>>19)&0x0F);
      *(outbuffer + offset++) = (exp_accel>>4) & 0x7F;
      if(this->accel < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t * val_trans = (uint32_t*) &(this->trans);
      offset += 3;
      *val_trans = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_trans |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_trans |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_trans |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_trans = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_trans |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_trans !=0)
        *val_trans |= ((exp_trans)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->trans = -this->trans;
      uint32_t * val_rot = (uint32_t*) &(this->rot);
      offset += 3;
      *val_rot = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_rot |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_rot |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_rot |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_rot = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_rot |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_rot !=0)
        *val_rot |= ((exp_rot)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->rot = -this->rot;
      uint32_t * val_accel = (uint32_t*) &(this->accel);
      offset += 3;
      *val_accel = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_accel |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_accel |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_accel |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_accel = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_accel |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_accel !=0)
        *val_accel |= ((exp_accel)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->accel = -this->accel;
     return offset;
    }

    const char * getType(){ return "clearpath_base/VelocitySetpt"; };
    const char * getMD5(){ return "7484db97a3f5854502dee1b95a48014c"; };

  };

}
#endif