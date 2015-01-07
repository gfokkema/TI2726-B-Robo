#ifndef _ROS_r2_msgs_PDMCStatus_h
#define _ROS_r2_msgs_PDMCStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2_msgs
{

  class PDMCStatus : public ros::Msg
  {
    public:
      char * publisher;
      uint32_t registerValue;
      bool logicEnabled;
      bool motorEnabled;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_publisher = strlen( (const char*) this->publisher);
      memcpy(outbuffer + offset, &length_publisher, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->publisher, length_publisher);
      offset += length_publisher;
      *(outbuffer + offset + 0) = (this->registerValue >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->registerValue >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->registerValue >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->registerValue >> (8 * 3)) & 0xFF;
      offset += sizeof(this->registerValue);
      union {
        bool real;
        uint8_t base;
      } u_logicEnabled;
      u_logicEnabled.real = this->logicEnabled;
      *(outbuffer + offset + 0) = (u_logicEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->logicEnabled);
      union {
        bool real;
        uint8_t base;
      } u_motorEnabled;
      u_motorEnabled.real = this->motorEnabled;
      *(outbuffer + offset + 0) = (u_motorEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motorEnabled);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_publisher;
      memcpy(&length_publisher, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_publisher; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_publisher-1]=0;
      this->publisher = (char *)(inbuffer + offset-1);
      offset += length_publisher;
      this->registerValue =  ((uint32_t) (*(inbuffer + offset)));
      this->registerValue |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->registerValue |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->registerValue |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->registerValue);
      union {
        bool real;
        uint8_t base;
      } u_logicEnabled;
      u_logicEnabled.base = 0;
      u_logicEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->logicEnabled = u_logicEnabled.real;
      offset += sizeof(this->logicEnabled);
      union {
        bool real;
        uint8_t base;
      } u_motorEnabled;
      u_motorEnabled.base = 0;
      u_motorEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->motorEnabled = u_motorEnabled.real;
      offset += sizeof(this->motorEnabled);
     return offset;
    }

    const char * getType(){ return "r2_msgs/PDMCStatus"; };
    const char * getMD5(){ return "a68d9dc9991d8f304fc3adff90114aa5"; };

  };

}
#endif