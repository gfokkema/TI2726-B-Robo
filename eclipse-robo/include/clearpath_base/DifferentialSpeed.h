#ifndef _ROS_clearpath_base_DifferentialSpeed_h
#define _ROS_clearpath_base_DifferentialSpeed_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace clearpath_base
{

  class DifferentialSpeed : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float left_speed;
      float right_speed;
      float left_accel;
      float right_accel;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      int32_t * val_left_speed = (int32_t *) &(this->left_speed);
      int32_t exp_left_speed = (((*val_left_speed)>>23)&255);
      if(exp_left_speed != 0)
        exp_left_speed += 1023-127;
      int32_t sig_left_speed = *val_left_speed;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_left_speed<<5) & 0xff;
      *(outbuffer + offset++) = (sig_left_speed>>3) & 0xff;
      *(outbuffer + offset++) = (sig_left_speed>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_left_speed<<4) & 0xF0) | ((sig_left_speed>>19)&0x0F);
      *(outbuffer + offset++) = (exp_left_speed>>4) & 0x7F;
      if(this->left_speed < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_right_speed = (int32_t *) &(this->right_speed);
      int32_t exp_right_speed = (((*val_right_speed)>>23)&255);
      if(exp_right_speed != 0)
        exp_right_speed += 1023-127;
      int32_t sig_right_speed = *val_right_speed;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_right_speed<<5) & 0xff;
      *(outbuffer + offset++) = (sig_right_speed>>3) & 0xff;
      *(outbuffer + offset++) = (sig_right_speed>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_right_speed<<4) & 0xF0) | ((sig_right_speed>>19)&0x0F);
      *(outbuffer + offset++) = (exp_right_speed>>4) & 0x7F;
      if(this->right_speed < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_left_accel = (int32_t *) &(this->left_accel);
      int32_t exp_left_accel = (((*val_left_accel)>>23)&255);
      if(exp_left_accel != 0)
        exp_left_accel += 1023-127;
      int32_t sig_left_accel = *val_left_accel;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_left_accel<<5) & 0xff;
      *(outbuffer + offset++) = (sig_left_accel>>3) & 0xff;
      *(outbuffer + offset++) = (sig_left_accel>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_left_accel<<4) & 0xF0) | ((sig_left_accel>>19)&0x0F);
      *(outbuffer + offset++) = (exp_left_accel>>4) & 0x7F;
      if(this->left_accel < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_right_accel = (int32_t *) &(this->right_accel);
      int32_t exp_right_accel = (((*val_right_accel)>>23)&255);
      if(exp_right_accel != 0)
        exp_right_accel += 1023-127;
      int32_t sig_right_accel = *val_right_accel;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_right_accel<<5) & 0xff;
      *(outbuffer + offset++) = (sig_right_accel>>3) & 0xff;
      *(outbuffer + offset++) = (sig_right_accel>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_right_accel<<4) & 0xF0) | ((sig_right_accel>>19)&0x0F);
      *(outbuffer + offset++) = (exp_right_accel>>4) & 0x7F;
      if(this->right_accel < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t * val_left_speed = (uint32_t*) &(this->left_speed);
      offset += 3;
      *val_left_speed = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_left_speed |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_left_speed |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_left_speed |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_left_speed = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_left_speed |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_left_speed !=0)
        *val_left_speed |= ((exp_left_speed)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->left_speed = -this->left_speed;
      uint32_t * val_right_speed = (uint32_t*) &(this->right_speed);
      offset += 3;
      *val_right_speed = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_right_speed |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_right_speed |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_right_speed |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_right_speed = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_right_speed |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_right_speed !=0)
        *val_right_speed |= ((exp_right_speed)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->right_speed = -this->right_speed;
      uint32_t * val_left_accel = (uint32_t*) &(this->left_accel);
      offset += 3;
      *val_left_accel = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_left_accel |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_left_accel |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_left_accel |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_left_accel = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_left_accel |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_left_accel !=0)
        *val_left_accel |= ((exp_left_accel)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->left_accel = -this->left_accel;
      uint32_t * val_right_accel = (uint32_t*) &(this->right_accel);
      offset += 3;
      *val_right_accel = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_right_accel |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_right_accel |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_right_accel |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_right_accel = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_right_accel |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_right_accel !=0)
        *val_right_accel |= ((exp_right_accel)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->right_accel = -this->right_accel;
     return offset;
    }

    const char * getType(){ return "clearpath_base/DifferentialSpeed"; };
    const char * getMD5(){ return "4b62e763320bacb9c7f63f6ebd46ceba"; };

  };

}
#endif