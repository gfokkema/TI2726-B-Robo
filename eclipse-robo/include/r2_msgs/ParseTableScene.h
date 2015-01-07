#ifndef _ROS_SERVICE_ParseTableScene_h
#define _ROS_SERVICE_ParseTableScene_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "visualization_msgs/MarkerArray.h"

namespace r2_msgs
{

static const char PARSETABLESCENE[] = "r2_msgs/ParseTableScene";

  class ParseTableSceneRequest : public ros::Msg
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

    const char * getType(){ return PARSETABLESCENE; };
    const char * getMD5(){ return "fab6d5f499de320aad10749bce1f85cf"; };

  };

  class ParseTableSceneResponse : public ros::Msg
  {
    public:
      bool table_found;
      visualization_msgs::MarkerArray markers;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_table_found;
      u_table_found.real = this->table_found;
      *(outbuffer + offset + 0) = (u_table_found.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->table_found);
      offset += this->markers.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_table_found;
      u_table_found.base = 0;
      u_table_found.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->table_found = u_table_found.real;
      offset += sizeof(this->table_found);
      offset += this->markers.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return PARSETABLESCENE; };
    const char * getMD5(){ return "566c52328a1e033030e9a47cba63ed0d"; };

  };

  class ParseTableScene {
    public:
    typedef ParseTableSceneRequest Request;
    typedef ParseTableSceneResponse Response;
  };

}
#endif
