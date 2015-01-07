#ifndef _ROS_r2_msgs_PoseCommandStatus_h
#define _ROS_r2_msgs_PoseCommandStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace r2_msgs
{

  class PoseCommandStatus : public ros::Msg
  {
    public:
      ros::Time stamp;
      char * commandId;
      uint8_t status;
      char * statusMessage;
      enum { PENDING = 0 };
      enum { ACTIVE = 1 };
      enum { REJECTED = 2 };
      enum { SUCCEEDED = 3 };
      enum { FAILED = 4 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      uint32_t length_commandId = strlen( (const char*) this->commandId);
      memcpy(outbuffer + offset, &length_commandId, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->commandId, length_commandId);
      offset += length_commandId;
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      uint32_t length_statusMessage = strlen( (const char*) this->statusMessage);
      memcpy(outbuffer + offset, &length_statusMessage, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->statusMessage, length_statusMessage);
      offset += length_statusMessage;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      uint32_t length_commandId;
      memcpy(&length_commandId, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_commandId; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_commandId-1]=0;
      this->commandId = (char *)(inbuffer + offset-1);
      offset += length_commandId;
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
      uint32_t length_statusMessage;
      memcpy(&length_statusMessage, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_statusMessage; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_statusMessage-1]=0;
      this->statusMessage = (char *)(inbuffer + offset-1);
      offset += length_statusMessage;
     return offset;
    }

    const char * getType(){ return "r2_msgs/PoseCommandStatus"; };
    const char * getMD5(){ return "82493b7735e3fe414b93381d96bfd1ee"; };

  };

}
#endif