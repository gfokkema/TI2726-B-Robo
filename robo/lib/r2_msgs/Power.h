#ifndef _ROS_SERVICE_Power_h
#define _ROS_SERVICE_Power_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2_msgs
{

static const char POWER[] = "r2_msgs/Power";

  class PowerRequest : public ros::Msg
  {
    public:
      char * channel;
      bool state;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_channel = strlen( (const char*) this->channel);
      memcpy(outbuffer + offset, &length_channel, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->channel, length_channel);
      offset += length_channel;
      union {
        bool real;
        uint8_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_channel;
      memcpy(&length_channel, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_channel; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_channel-1]=0;
      this->channel = (char *)(inbuffer + offset-1);
      offset += length_channel;
      union {
        bool real;
        uint8_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->state = u_state.real;
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return POWER; };
    const char * getMD5(){ return "1045ad41f2608d96980e5002751cdcee"; };

  };

  class PowerResponse : public ros::Msg
  {
    public:
      bool status;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return POWER; };
    const char * getMD5(){ return "3a1255d4d998bd4d6585c64639b5ee9a"; };

  };

  class Power {
    public:
    typedef PowerRequest Request;
    typedef PowerResponse Response;
  };

}
#endif
