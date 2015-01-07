#ifndef _ROS_hector_uav_msgs_RawMagnetic_h
#define _ROS_hector_uav_msgs_RawMagnetic_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hector_uav_msgs
{

  class RawMagnetic : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float channel[3];

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      unsigned char * channel_val = (unsigned char *) this->channel;
      for( uint8_t i = 0; i < 3; i++){
      int32_t * val_channeli = (int32_t *) &(this->channel[i]);
      int32_t exp_channeli = (((*val_channeli)>>23)&255);
      if(exp_channeli != 0)
        exp_channeli += 1023-127;
      int32_t sig_channeli = *val_channeli;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_channeli<<5) & 0xff;
      *(outbuffer + offset++) = (sig_channeli>>3) & 0xff;
      *(outbuffer + offset++) = (sig_channeli>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_channeli<<4) & 0xF0) | ((sig_channeli>>19)&0x0F);
      *(outbuffer + offset++) = (exp_channeli>>4) & 0x7F;
      if(this->channel[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t * channel_val = (uint8_t*) this->channel;
      for( uint8_t i = 0; i < 3; i++){
      uint32_t * val_channeli = (uint32_t*) &(this->channel[i]);
      offset += 3;
      *val_channeli = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_channeli |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_channeli |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_channeli |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_channeli = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_channeli |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_channeli !=0)
        *val_channeli |= ((exp_channeli)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->channel[i] = -this->channel[i];
      }
     return offset;
    }

    const char * getType(){ return "hector_uav_msgs/RawMagnetic"; };
    const char * getMD5(){ return "babd510868ac7b486e2097c79e1384c9"; };

  };

}
#endif