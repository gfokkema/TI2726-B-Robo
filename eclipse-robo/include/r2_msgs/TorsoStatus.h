#ifndef _ROS_r2_msgs_TorsoStatus_h
#define _ROS_r2_msgs_TorsoStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2_msgs
{

  class TorsoStatus : public ros::Msg
  {
    public:
      char * leftarm;
      char * rightarm;
      char * neck;
      char * lefthand;
      char * righthand;
      char * waist;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_leftarm = strlen( (const char*) this->leftarm);
      memcpy(outbuffer + offset, &length_leftarm, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->leftarm, length_leftarm);
      offset += length_leftarm;
      uint32_t length_rightarm = strlen( (const char*) this->rightarm);
      memcpy(outbuffer + offset, &length_rightarm, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->rightarm, length_rightarm);
      offset += length_rightarm;
      uint32_t length_neck = strlen( (const char*) this->neck);
      memcpy(outbuffer + offset, &length_neck, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->neck, length_neck);
      offset += length_neck;
      uint32_t length_lefthand = strlen( (const char*) this->lefthand);
      memcpy(outbuffer + offset, &length_lefthand, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->lefthand, length_lefthand);
      offset += length_lefthand;
      uint32_t length_righthand = strlen( (const char*) this->righthand);
      memcpy(outbuffer + offset, &length_righthand, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->righthand, length_righthand);
      offset += length_righthand;
      uint32_t length_waist = strlen( (const char*) this->waist);
      memcpy(outbuffer + offset, &length_waist, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->waist, length_waist);
      offset += length_waist;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_leftarm;
      memcpy(&length_leftarm, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_leftarm; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_leftarm-1]=0;
      this->leftarm = (char *)(inbuffer + offset-1);
      offset += length_leftarm;
      uint32_t length_rightarm;
      memcpy(&length_rightarm, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_rightarm; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_rightarm-1]=0;
      this->rightarm = (char *)(inbuffer + offset-1);
      offset += length_rightarm;
      uint32_t length_neck;
      memcpy(&length_neck, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_neck; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_neck-1]=0;
      this->neck = (char *)(inbuffer + offset-1);
      offset += length_neck;
      uint32_t length_lefthand;
      memcpy(&length_lefthand, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_lefthand; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_lefthand-1]=0;
      this->lefthand = (char *)(inbuffer + offset-1);
      offset += length_lefthand;
      uint32_t length_righthand;
      memcpy(&length_righthand, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_righthand; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_righthand-1]=0;
      this->righthand = (char *)(inbuffer + offset-1);
      offset += length_righthand;
      uint32_t length_waist;
      memcpy(&length_waist, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_waist; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_waist-1]=0;
      this->waist = (char *)(inbuffer + offset-1);
      offset += length_waist;
     return offset;
    }

    const char * getType(){ return "r2_msgs/TorsoStatus"; };
    const char * getMD5(){ return "2334ce336aad886ccb87a294113ed892"; };

  };

}
#endif