#ifndef _ROS_r2_msgs_JointStatusArray_h
#define _ROS_r2_msgs_JointStatusArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "r2_msgs/JointStatus.h"

namespace r2_msgs
{

  class JointStatusArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t status_length;
      r2_msgs::JointStatus st_status;
      r2_msgs::JointStatus * status;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = status_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < status_length; i++){
      offset += this->status[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t status_lengthT = *(inbuffer + offset++);
      if(status_lengthT > status_length)
        this->status = (r2_msgs::JointStatus*)realloc(this->status, status_lengthT * sizeof(r2_msgs::JointStatus));
      offset += 3;
      status_length = status_lengthT;
      for( uint8_t i = 0; i < status_length; i++){
      offset += this->st_status.deserialize(inbuffer + offset);
        memcpy( &(this->status[i]), &(this->st_status), sizeof(r2_msgs::JointStatus));
      }
     return offset;
    }

    const char * getType(){ return "r2_msgs/JointStatusArray"; };
    const char * getMD5(){ return "2d3e86cb77e5d92be1bdb178712b3811"; };

  };

}
#endif