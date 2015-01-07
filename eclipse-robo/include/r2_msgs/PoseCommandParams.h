#ifndef _ROS_r2_msgs_PoseCommandParams_h
#define _ROS_r2_msgs_PoseCommandParams_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2_msgs
{

  class PoseCommandParams : public ros::Msg
  {
    public:
      float maxLinVel;
      float maxRotVel;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_maxLinVel;
      u_maxLinVel.real = this->maxLinVel;
      *(outbuffer + offset + 0) = (u_maxLinVel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_maxLinVel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_maxLinVel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_maxLinVel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->maxLinVel);
      union {
        float real;
        uint32_t base;
      } u_maxRotVel;
      u_maxRotVel.real = this->maxRotVel;
      *(outbuffer + offset + 0) = (u_maxRotVel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_maxRotVel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_maxRotVel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_maxRotVel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->maxRotVel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_maxLinVel;
      u_maxLinVel.base = 0;
      u_maxLinVel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_maxLinVel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_maxLinVel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_maxLinVel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->maxLinVel = u_maxLinVel.real;
      offset += sizeof(this->maxLinVel);
      union {
        float real;
        uint32_t base;
      } u_maxRotVel;
      u_maxRotVel.base = 0;
      u_maxRotVel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_maxRotVel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_maxRotVel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_maxRotVel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->maxRotVel = u_maxRotVel.real;
      offset += sizeof(this->maxRotVel);
     return offset;
    }

    const char * getType(){ return "r2_msgs/PoseCommandParams"; };
    const char * getMD5(){ return "a159026142e288942cb08238d17789db"; };

  };

}
#endif