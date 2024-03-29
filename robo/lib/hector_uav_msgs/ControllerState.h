#ifndef _ROS_hector_uav_msgs_ControllerState_h
#define _ROS_hector_uav_msgs_ControllerState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hector_uav_msgs
{

  class ControllerState : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t source;
      uint8_t mode;
      uint8_t state;
      enum { MOTORS =  1 };
      enum { ATTITUDE =  2 };
      enum { VELOCITY =  4 };
      enum { POSITION =  8 };
      enum { HEADING =  16 };
      enum { HEIGHT =  32 };
      enum { MOTORS_RUNNING =  1 };
      enum { AIRBORNE =  2 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->source >> (8 * 0)) & 0xFF;
      offset += sizeof(this->source);
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->source =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->source);
      this->mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mode);
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "hector_uav_msgs/ControllerState"; };
    const char * getMD5(){ return "a0f668496c9bbf505c01f59674f2a0c2"; };

  };

}
#endif