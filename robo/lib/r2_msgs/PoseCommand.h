#ifndef _ROS_r2_msgs_PoseCommand_h
#define _ROS_r2_msgs_PoseCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace r2_msgs
{

  class PoseCommand : public ros::Msg
  {
    public:
      char * commandId;
      char * refFrame;
      geometry_msgs::Pose pose;
      char * baseFrame;
      char * toolFrame;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_commandId = strlen( (const char*) this->commandId);
      memcpy(outbuffer + offset, &length_commandId, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->commandId, length_commandId);
      offset += length_commandId;
      uint32_t length_refFrame = strlen( (const char*) this->refFrame);
      memcpy(outbuffer + offset, &length_refFrame, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->refFrame, length_refFrame);
      offset += length_refFrame;
      offset += this->pose.serialize(outbuffer + offset);
      uint32_t length_baseFrame = strlen( (const char*) this->baseFrame);
      memcpy(outbuffer + offset, &length_baseFrame, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->baseFrame, length_baseFrame);
      offset += length_baseFrame;
      uint32_t length_toolFrame = strlen( (const char*) this->toolFrame);
      memcpy(outbuffer + offset, &length_toolFrame, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->toolFrame, length_toolFrame);
      offset += length_toolFrame;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_commandId;
      memcpy(&length_commandId, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_commandId; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_commandId-1]=0;
      this->commandId = (char *)(inbuffer + offset-1);
      offset += length_commandId;
      uint32_t length_refFrame;
      memcpy(&length_refFrame, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_refFrame; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_refFrame-1]=0;
      this->refFrame = (char *)(inbuffer + offset-1);
      offset += length_refFrame;
      offset += this->pose.deserialize(inbuffer + offset);
      uint32_t length_baseFrame;
      memcpy(&length_baseFrame, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_baseFrame; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_baseFrame-1]=0;
      this->baseFrame = (char *)(inbuffer + offset-1);
      offset += length_baseFrame;
      uint32_t length_toolFrame;
      memcpy(&length_toolFrame, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_toolFrame; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_toolFrame-1]=0;
      this->toolFrame = (char *)(inbuffer + offset-1);
      offset += length_toolFrame;
     return offset;
    }

    const char * getType(){ return "r2_msgs/PoseCommand"; };
    const char * getMD5(){ return "325108abdcbe78370664839c919cbaa8"; };

  };

}
#endif