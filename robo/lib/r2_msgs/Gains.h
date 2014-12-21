#ifndef _ROS_r2_msgs_Gains_h
#define _ROS_r2_msgs_Gains_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2_msgs
{

  class Gains : public ros::Msg
  {
    public:
      uint8_t joint_names_length;
      char* st_joint_names;
      char* * joint_names;
      uint8_t K_length;
      float st_K;
      float * K;
      uint8_t D_length;
      float st_D;
      float * D;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = joint_names_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < joint_names_length; i++){
      uint32_t length_joint_namesi = strlen( (const char*) this->joint_names[i]);
      memcpy(outbuffer + offset, &length_joint_namesi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->joint_names[i], length_joint_namesi);
      offset += length_joint_namesi;
      }
      *(outbuffer + offset++) = K_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < K_length; i++){
      int32_t * val_Ki = (int32_t *) &(this->K[i]);
      int32_t exp_Ki = (((*val_Ki)>>23)&255);
      if(exp_Ki != 0)
        exp_Ki += 1023-127;
      int32_t sig_Ki = *val_Ki;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_Ki<<5) & 0xff;
      *(outbuffer + offset++) = (sig_Ki>>3) & 0xff;
      *(outbuffer + offset++) = (sig_Ki>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_Ki<<4) & 0xF0) | ((sig_Ki>>19)&0x0F);
      *(outbuffer + offset++) = (exp_Ki>>4) & 0x7F;
      if(this->K[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      *(outbuffer + offset++) = D_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < D_length; i++){
      int32_t * val_Di = (int32_t *) &(this->D[i]);
      int32_t exp_Di = (((*val_Di)>>23)&255);
      if(exp_Di != 0)
        exp_Di += 1023-127;
      int32_t sig_Di = *val_Di;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_Di<<5) & 0xff;
      *(outbuffer + offset++) = (sig_Di>>3) & 0xff;
      *(outbuffer + offset++) = (sig_Di>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_Di<<4) & 0xF0) | ((sig_Di>>19)&0x0F);
      *(outbuffer + offset++) = (exp_Di>>4) & 0x7F;
      if(this->D[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t joint_names_lengthT = *(inbuffer + offset++);
      if(joint_names_lengthT > joint_names_length)
        this->joint_names = (char**)realloc(this->joint_names, joint_names_lengthT * sizeof(char*));
      offset += 3;
      joint_names_length = joint_names_lengthT;
      for( uint8_t i = 0; i < joint_names_length; i++){
      uint32_t length_st_joint_names;
      memcpy(&length_st_joint_names, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_joint_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_joint_names-1]=0;
      this->st_joint_names = (char *)(inbuffer + offset-1);
      offset += length_st_joint_names;
        memcpy( &(this->joint_names[i]), &(this->st_joint_names), sizeof(char*));
      }
      uint8_t K_lengthT = *(inbuffer + offset++);
      if(K_lengthT > K_length)
        this->K = (float*)realloc(this->K, K_lengthT * sizeof(float));
      offset += 3;
      K_length = K_lengthT;
      for( uint8_t i = 0; i < K_length; i++){
      uint32_t * val_st_K = (uint32_t*) &(this->st_K);
      offset += 3;
      *val_st_K = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_st_K |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_st_K |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_st_K |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_st_K = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_st_K |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_st_K !=0)
        *val_st_K |= ((exp_st_K)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->st_K = -this->st_K;
        memcpy( &(this->K[i]), &(this->st_K), sizeof(float));
      }
      uint8_t D_lengthT = *(inbuffer + offset++);
      if(D_lengthT > D_length)
        this->D = (float*)realloc(this->D, D_lengthT * sizeof(float));
      offset += 3;
      D_length = D_lengthT;
      for( uint8_t i = 0; i < D_length; i++){
      uint32_t * val_st_D = (uint32_t*) &(this->st_D);
      offset += 3;
      *val_st_D = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_st_D |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_st_D |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_st_D |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_st_D = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_st_D |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_st_D !=0)
        *val_st_D |= ((exp_st_D)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->st_D = -this->st_D;
        memcpy( &(this->D[i]), &(this->st_D), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "r2_msgs/Gains"; };
    const char * getMD5(){ return "689bcaebe3b1d4a08d9dd0ca1971d522"; };

  };

}
#endif