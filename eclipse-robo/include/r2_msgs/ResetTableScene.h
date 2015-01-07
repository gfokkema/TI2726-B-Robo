#ifndef _ROS_SERVICE_ResetTableScene_h
#define _ROS_SERVICE_ResetTableScene_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2_msgs
{

static const char RESETTABLESCENE[] = "r2_msgs/ResetTableScene";

  class ResetTableSceneRequest : public ros::Msg
  {
    public:
      bool reset;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_reset;
      u_reset.real = this->reset;
      *(outbuffer + offset + 0) = (u_reset.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_reset;
      u_reset.base = 0;
      u_reset.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->reset = u_reset.real;
      offset += sizeof(this->reset);
     return offset;
    }

    const char * getType(){ return RESETTABLESCENE; };
    const char * getMD5(){ return "ba4b0b221fb425ac5eaf73f71ae34971"; };

  };

  class ResetTableSceneResponse : public ros::Msg
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

    const char * getType(){ return RESETTABLESCENE; };
    const char * getMD5(){ return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class ResetTableScene {
    public:
    typedef ResetTableSceneRequest Request;
    typedef ResetTableSceneResponse Response;
  };

}
#endif
