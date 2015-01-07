#ifndef _ROS_clearpath_base_DifferentialOutput_h
#define _ROS_clearpath_base_DifferentialOutput_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace clearpath_base
{

  class DifferentialOutput : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float left;
      float right;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      int32_t * val_left = (int32_t *) &(this->left);
      int32_t exp_left = (((*val_left)>>23)&255);
      if(exp_left != 0)
        exp_left += 1023-127;
      int32_t sig_left = *val_left;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_left<<5) & 0xff;
      *(outbuffer + offset++) = (sig_left>>3) & 0xff;
      *(outbuffer + offset++) = (sig_left>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_left<<4) & 0xF0) | ((sig_left>>19)&0x0F);
      *(outbuffer + offset++) = (exp_left>>4) & 0x7F;
      if(this->left < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_right = (int32_t *) &(this->right);
      int32_t exp_right = (((*val_right)>>23)&255);
      if(exp_right != 0)
        exp_right += 1023-127;
      int32_t sig_right = *val_right;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_right<<5) & 0xff;
      *(outbuffer + offset++) = (sig_right>>3) & 0xff;
      *(outbuffer + offset++) = (sig_right>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_right<<4) & 0xF0) | ((sig_right>>19)&0x0F);
      *(outbuffer + offset++) = (exp_right>>4) & 0x7F;
      if(this->right < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t * val_left = (uint32_t*) &(this->left);
      offset += 3;
      *val_left = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_left |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_left |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_left |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_left = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_left |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_left !=0)
        *val_left |= ((exp_left)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->left = -this->left;
      uint32_t * val_right = (uint32_t*) &(this->right);
      offset += 3;
      *val_right = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_right |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_right |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_right |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_right = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_right |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_right !=0)
        *val_right |= ((exp_right)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->right = -this->right;
     return offset;
    }

    const char * getType(){ return "clearpath_base/DifferentialOutput"; };
    const char * getMD5(){ return "8f32685125452f5bdf68130369af5296"; };

  };

}
#endif