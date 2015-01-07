#ifndef _ROS_r2_msgs_PoseCommandArray_h
#define _ROS_r2_msgs_PoseCommandArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "r2_msgs/PoseCommand.h"

namespace r2_msgs
{

  class PoseCommandArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t command_length;
      r2_msgs::PoseCommand st_command;
      r2_msgs::PoseCommand * command;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = command_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < command_length; i++){
      offset += this->command[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t command_lengthT = *(inbuffer + offset++);
      if(command_lengthT > command_length)
        this->command = (r2_msgs::PoseCommand*)realloc(this->command, command_lengthT * sizeof(r2_msgs::PoseCommand));
      offset += 3;
      command_length = command_lengthT;
      for( uint8_t i = 0; i < command_length; i++){
      offset += this->st_command.deserialize(inbuffer + offset);
        memcpy( &(this->command[i]), &(this->st_command), sizeof(r2_msgs::PoseCommand));
      }
     return offset;
    }

    const char * getType(){ return "r2_msgs/PoseCommandArray"; };
    const char * getMD5(){ return "fd9b3b03f805dcb412e07ffc694f2684"; };

  };

}
#endif