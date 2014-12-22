#ifndef _ROS_clearpath_base_DistanceTiming_h
#define _ROS_clearpath_base_DistanceTiming_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace clearpath_base
{

  class DistanceTiming : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t distances_length;
      float st_distances;
      float * distances;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = distances_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < distances_length; i++){
      int32_t * val_distancesi = (int32_t *) &(this->distances[i]);
      int32_t exp_distancesi = (((*val_distancesi)>>23)&255);
      if(exp_distancesi != 0)
        exp_distancesi += 1023-127;
      int32_t sig_distancesi = *val_distancesi;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_distancesi<<5) & 0xff;
      *(outbuffer + offset++) = (sig_distancesi>>3) & 0xff;
      *(outbuffer + offset++) = (sig_distancesi>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_distancesi<<4) & 0xF0) | ((sig_distancesi>>19)&0x0F);
      *(outbuffer + offset++) = (exp_distancesi>>4) & 0x7F;
      if(this->distances[i] < 0) *(outbuffer + offset -1) |= 0x80;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t distances_lengthT = *(inbuffer + offset++);
      if(distances_lengthT > distances_length)
        this->distances = (float*)realloc(this->distances, distances_lengthT * sizeof(float));
      offset += 3;
      distances_length = distances_lengthT;
      for( uint8_t i = 0; i < distances_length; i++){
      uint32_t * val_st_distances = (uint32_t*) &(this->st_distances);
      offset += 3;
      *val_st_distances = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_st_distances |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_st_distances |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_st_distances |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_st_distances = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_st_distances |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_st_distances !=0)
        *val_st_distances |= ((exp_st_distances)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->st_distances = -this->st_distances;
        memcpy( &(this->distances[i]), &(this->st_distances), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "clearpath_base/DistanceTiming"; };
    const char * getMD5(){ return "5cd97954696edc5d59dd80cb0218765a"; };

  };

}
#endif