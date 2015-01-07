#ifndef _ROS_clearpath_base_DifferentialControl_h
#define _ROS_clearpath_base_DifferentialControl_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace clearpath_base
{

  class DifferentialControl : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float l_p;
      float l_i;
      float l_d;
      float l_ffwd;
      float l_stic;
      float l_sat;
      float r_p;
      float r_i;
      float r_d;
      float r_ffwd;
      float r_stic;
      float r_sat;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      int32_t * val_l_p = (int32_t *) &(this->l_p);
      int32_t exp_l_p = (((*val_l_p)>>23)&255);
      if(exp_l_p != 0)
        exp_l_p += 1023-127;
      int32_t sig_l_p = *val_l_p;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_l_p<<5) & 0xff;
      *(outbuffer + offset++) = (sig_l_p>>3) & 0xff;
      *(outbuffer + offset++) = (sig_l_p>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_l_p<<4) & 0xF0) | ((sig_l_p>>19)&0x0F);
      *(outbuffer + offset++) = (exp_l_p>>4) & 0x7F;
      if(this->l_p < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_l_i = (int32_t *) &(this->l_i);
      int32_t exp_l_i = (((*val_l_i)>>23)&255);
      if(exp_l_i != 0)
        exp_l_i += 1023-127;
      int32_t sig_l_i = *val_l_i;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_l_i<<5) & 0xff;
      *(outbuffer + offset++) = (sig_l_i>>3) & 0xff;
      *(outbuffer + offset++) = (sig_l_i>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_l_i<<4) & 0xF0) | ((sig_l_i>>19)&0x0F);
      *(outbuffer + offset++) = (exp_l_i>>4) & 0x7F;
      if(this->l_i < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_l_d = (int32_t *) &(this->l_d);
      int32_t exp_l_d = (((*val_l_d)>>23)&255);
      if(exp_l_d != 0)
        exp_l_d += 1023-127;
      int32_t sig_l_d = *val_l_d;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_l_d<<5) & 0xff;
      *(outbuffer + offset++) = (sig_l_d>>3) & 0xff;
      *(outbuffer + offset++) = (sig_l_d>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_l_d<<4) & 0xF0) | ((sig_l_d>>19)&0x0F);
      *(outbuffer + offset++) = (exp_l_d>>4) & 0x7F;
      if(this->l_d < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_l_ffwd = (int32_t *) &(this->l_ffwd);
      int32_t exp_l_ffwd = (((*val_l_ffwd)>>23)&255);
      if(exp_l_ffwd != 0)
        exp_l_ffwd += 1023-127;
      int32_t sig_l_ffwd = *val_l_ffwd;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_l_ffwd<<5) & 0xff;
      *(outbuffer + offset++) = (sig_l_ffwd>>3) & 0xff;
      *(outbuffer + offset++) = (sig_l_ffwd>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_l_ffwd<<4) & 0xF0) | ((sig_l_ffwd>>19)&0x0F);
      *(outbuffer + offset++) = (exp_l_ffwd>>4) & 0x7F;
      if(this->l_ffwd < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_l_stic = (int32_t *) &(this->l_stic);
      int32_t exp_l_stic = (((*val_l_stic)>>23)&255);
      if(exp_l_stic != 0)
        exp_l_stic += 1023-127;
      int32_t sig_l_stic = *val_l_stic;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_l_stic<<5) & 0xff;
      *(outbuffer + offset++) = (sig_l_stic>>3) & 0xff;
      *(outbuffer + offset++) = (sig_l_stic>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_l_stic<<4) & 0xF0) | ((sig_l_stic>>19)&0x0F);
      *(outbuffer + offset++) = (exp_l_stic>>4) & 0x7F;
      if(this->l_stic < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_l_sat = (int32_t *) &(this->l_sat);
      int32_t exp_l_sat = (((*val_l_sat)>>23)&255);
      if(exp_l_sat != 0)
        exp_l_sat += 1023-127;
      int32_t sig_l_sat = *val_l_sat;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_l_sat<<5) & 0xff;
      *(outbuffer + offset++) = (sig_l_sat>>3) & 0xff;
      *(outbuffer + offset++) = (sig_l_sat>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_l_sat<<4) & 0xF0) | ((sig_l_sat>>19)&0x0F);
      *(outbuffer + offset++) = (exp_l_sat>>4) & 0x7F;
      if(this->l_sat < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_r_p = (int32_t *) &(this->r_p);
      int32_t exp_r_p = (((*val_r_p)>>23)&255);
      if(exp_r_p != 0)
        exp_r_p += 1023-127;
      int32_t sig_r_p = *val_r_p;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_r_p<<5) & 0xff;
      *(outbuffer + offset++) = (sig_r_p>>3) & 0xff;
      *(outbuffer + offset++) = (sig_r_p>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_r_p<<4) & 0xF0) | ((sig_r_p>>19)&0x0F);
      *(outbuffer + offset++) = (exp_r_p>>4) & 0x7F;
      if(this->r_p < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_r_i = (int32_t *) &(this->r_i);
      int32_t exp_r_i = (((*val_r_i)>>23)&255);
      if(exp_r_i != 0)
        exp_r_i += 1023-127;
      int32_t sig_r_i = *val_r_i;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_r_i<<5) & 0xff;
      *(outbuffer + offset++) = (sig_r_i>>3) & 0xff;
      *(outbuffer + offset++) = (sig_r_i>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_r_i<<4) & 0xF0) | ((sig_r_i>>19)&0x0F);
      *(outbuffer + offset++) = (exp_r_i>>4) & 0x7F;
      if(this->r_i < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_r_d = (int32_t *) &(this->r_d);
      int32_t exp_r_d = (((*val_r_d)>>23)&255);
      if(exp_r_d != 0)
        exp_r_d += 1023-127;
      int32_t sig_r_d = *val_r_d;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_r_d<<5) & 0xff;
      *(outbuffer + offset++) = (sig_r_d>>3) & 0xff;
      *(outbuffer + offset++) = (sig_r_d>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_r_d<<4) & 0xF0) | ((sig_r_d>>19)&0x0F);
      *(outbuffer + offset++) = (exp_r_d>>4) & 0x7F;
      if(this->r_d < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_r_ffwd = (int32_t *) &(this->r_ffwd);
      int32_t exp_r_ffwd = (((*val_r_ffwd)>>23)&255);
      if(exp_r_ffwd != 0)
        exp_r_ffwd += 1023-127;
      int32_t sig_r_ffwd = *val_r_ffwd;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_r_ffwd<<5) & 0xff;
      *(outbuffer + offset++) = (sig_r_ffwd>>3) & 0xff;
      *(outbuffer + offset++) = (sig_r_ffwd>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_r_ffwd<<4) & 0xF0) | ((sig_r_ffwd>>19)&0x0F);
      *(outbuffer + offset++) = (exp_r_ffwd>>4) & 0x7F;
      if(this->r_ffwd < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_r_stic = (int32_t *) &(this->r_stic);
      int32_t exp_r_stic = (((*val_r_stic)>>23)&255);
      if(exp_r_stic != 0)
        exp_r_stic += 1023-127;
      int32_t sig_r_stic = *val_r_stic;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_r_stic<<5) & 0xff;
      *(outbuffer + offset++) = (sig_r_stic>>3) & 0xff;
      *(outbuffer + offset++) = (sig_r_stic>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_r_stic<<4) & 0xF0) | ((sig_r_stic>>19)&0x0F);
      *(outbuffer + offset++) = (exp_r_stic>>4) & 0x7F;
      if(this->r_stic < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_r_sat = (int32_t *) &(this->r_sat);
      int32_t exp_r_sat = (((*val_r_sat)>>23)&255);
      if(exp_r_sat != 0)
        exp_r_sat += 1023-127;
      int32_t sig_r_sat = *val_r_sat;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_r_sat<<5) & 0xff;
      *(outbuffer + offset++) = (sig_r_sat>>3) & 0xff;
      *(outbuffer + offset++) = (sig_r_sat>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_r_sat<<4) & 0xF0) | ((sig_r_sat>>19)&0x0F);
      *(outbuffer + offset++) = (exp_r_sat>>4) & 0x7F;
      if(this->r_sat < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t * val_l_p = (uint32_t*) &(this->l_p);
      offset += 3;
      *val_l_p = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_l_p |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_l_p |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_l_p |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_l_p = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_l_p |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_l_p !=0)
        *val_l_p |= ((exp_l_p)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->l_p = -this->l_p;
      uint32_t * val_l_i = (uint32_t*) &(this->l_i);
      offset += 3;
      *val_l_i = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_l_i |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_l_i |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_l_i |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_l_i = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_l_i |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_l_i !=0)
        *val_l_i |= ((exp_l_i)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->l_i = -this->l_i;
      uint32_t * val_l_d = (uint32_t*) &(this->l_d);
      offset += 3;
      *val_l_d = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_l_d |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_l_d |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_l_d |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_l_d = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_l_d |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_l_d !=0)
        *val_l_d |= ((exp_l_d)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->l_d = -this->l_d;
      uint32_t * val_l_ffwd = (uint32_t*) &(this->l_ffwd);
      offset += 3;
      *val_l_ffwd = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_l_ffwd |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_l_ffwd |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_l_ffwd |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_l_ffwd = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_l_ffwd |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_l_ffwd !=0)
        *val_l_ffwd |= ((exp_l_ffwd)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->l_ffwd = -this->l_ffwd;
      uint32_t * val_l_stic = (uint32_t*) &(this->l_stic);
      offset += 3;
      *val_l_stic = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_l_stic |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_l_stic |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_l_stic |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_l_stic = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_l_stic |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_l_stic !=0)
        *val_l_stic |= ((exp_l_stic)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->l_stic = -this->l_stic;
      uint32_t * val_l_sat = (uint32_t*) &(this->l_sat);
      offset += 3;
      *val_l_sat = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_l_sat |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_l_sat |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_l_sat |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_l_sat = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_l_sat |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_l_sat !=0)
        *val_l_sat |= ((exp_l_sat)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->l_sat = -this->l_sat;
      uint32_t * val_r_p = (uint32_t*) &(this->r_p);
      offset += 3;
      *val_r_p = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_r_p |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_r_p |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_r_p |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_r_p = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_r_p |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_r_p !=0)
        *val_r_p |= ((exp_r_p)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->r_p = -this->r_p;
      uint32_t * val_r_i = (uint32_t*) &(this->r_i);
      offset += 3;
      *val_r_i = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_r_i |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_r_i |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_r_i |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_r_i = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_r_i |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_r_i !=0)
        *val_r_i |= ((exp_r_i)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->r_i = -this->r_i;
      uint32_t * val_r_d = (uint32_t*) &(this->r_d);
      offset += 3;
      *val_r_d = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_r_d |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_r_d |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_r_d |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_r_d = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_r_d |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_r_d !=0)
        *val_r_d |= ((exp_r_d)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->r_d = -this->r_d;
      uint32_t * val_r_ffwd = (uint32_t*) &(this->r_ffwd);
      offset += 3;
      *val_r_ffwd = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_r_ffwd |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_r_ffwd |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_r_ffwd |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_r_ffwd = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_r_ffwd |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_r_ffwd !=0)
        *val_r_ffwd |= ((exp_r_ffwd)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->r_ffwd = -this->r_ffwd;
      uint32_t * val_r_stic = (uint32_t*) &(this->r_stic);
      offset += 3;
      *val_r_stic = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_r_stic |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_r_stic |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_r_stic |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_r_stic = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_r_stic |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_r_stic !=0)
        *val_r_stic |= ((exp_r_stic)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->r_stic = -this->r_stic;
      uint32_t * val_r_sat = (uint32_t*) &(this->r_sat);
      offset += 3;
      *val_r_sat = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_r_sat |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_r_sat |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_r_sat |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_r_sat = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_r_sat |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_r_sat !=0)
        *val_r_sat |= ((exp_r_sat)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->r_sat = -this->r_sat;
     return offset;
    }

    const char * getType(){ return "clearpath_base/DifferentialControl"; };
    const char * getMD5(){ return "ae0672163e13fc0bb6491960c53a3259"; };

  };

}
#endif