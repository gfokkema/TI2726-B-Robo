#ifndef _ROS_SERVICE_SetJointMode_h
#define _ROS_SERVICE_SetJointMode_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2_msgs
{

static const char SETJOINTMODE[] = "r2_msgs/SetJointMode";

  class SetJointModeRequest : public ros::Msg
  {
    public:
      char * arm_name;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_arm_name = strlen( (const char*) this->arm_name);
      memcpy(outbuffer + offset, &length_arm_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->arm_name, length_arm_name);
      offset += length_arm_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_arm_name;
      memcpy(&length_arm_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_arm_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_arm_name-1]=0;
      this->arm_name = (char *)(inbuffer + offset-1);
      offset += length_arm_name;
     return offset;
    }

    const char * getType(){ return SETJOINTMODE; };
    const char * getMD5(){ return "2ddc5460af992861cb686e55ffa09231"; };

  };

  class SetJointModeResponse : public ros::Msg
  {
    public:
      bool result;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    const char * getType(){ return SETJOINTMODE; };
    const char * getMD5(){ return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class SetJointMode {
    public:
    typedef SetJointModeRequest Request;
    typedef SetJointModeResponse Response;
  };

}
#endif
