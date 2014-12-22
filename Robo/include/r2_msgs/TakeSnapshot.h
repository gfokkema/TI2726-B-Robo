#ifndef _ROS_SERVICE_TakeSnapshot_h
#define _ROS_SERVICE_TakeSnapshot_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2_msgs
{

static const char TAKESNAPSHOT[] = "r2_msgs/TakeSnapshot";

  class TakeSnapshotRequest : public ros::Msg
  {
    public:
      bool cmd;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_cmd;
      u_cmd.real = this->cmd;
      *(outbuffer + offset + 0) = (u_cmd.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cmd);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_cmd;
      u_cmd.base = 0;
      u_cmd.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cmd = u_cmd.real;
      offset += sizeof(this->cmd);
     return offset;
    }

    const char * getType(){ return TAKESNAPSHOT; };
    const char * getMD5(){ return "fab6d5f499de320aad10749bce1f85cf"; };

  };

  class TakeSnapshotResponse : public ros::Msg
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

    const char * getType(){ return TAKESNAPSHOT; };
    const char * getMD5(){ return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class TakeSnapshot {
    public:
    typedef TakeSnapshotRequest Request;
    typedef TakeSnapshotResponse Response;
  };

}
#endif
