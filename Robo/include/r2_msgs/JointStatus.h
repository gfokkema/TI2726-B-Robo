#ifndef _ROS_r2_msgs_JointStatus_h
#define _ROS_r2_msgs_JointStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2_msgs
{

  class JointStatus : public ros::Msg
  {
    public:
      char * publisher;
      char * joint;
      uint32_t registerValue;
      bool bridgeEnabled;
      bool motorEnabled;
      bool brakeReleased;
      bool motorPowerDetected;
      bool embeddedMotCom;
      bool jointFaulted;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_publisher = strlen( (const char*) this->publisher);
      memcpy(outbuffer + offset, &length_publisher, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->publisher, length_publisher);
      offset += length_publisher;
      uint32_t length_joint = strlen( (const char*) this->joint);
      memcpy(outbuffer + offset, &length_joint, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->joint, length_joint);
      offset += length_joint;
      *(outbuffer + offset + 0) = (this->registerValue >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->registerValue >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->registerValue >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->registerValue >> (8 * 3)) & 0xFF;
      offset += sizeof(this->registerValue);
      union {
        bool real;
        uint8_t base;
      } u_bridgeEnabled;
      u_bridgeEnabled.real = this->bridgeEnabled;
      *(outbuffer + offset + 0) = (u_bridgeEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bridgeEnabled);
      union {
        bool real;
        uint8_t base;
      } u_motorEnabled;
      u_motorEnabled.real = this->motorEnabled;
      *(outbuffer + offset + 0) = (u_motorEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motorEnabled);
      union {
        bool real;
        uint8_t base;
      } u_brakeReleased;
      u_brakeReleased.real = this->brakeReleased;
      *(outbuffer + offset + 0) = (u_brakeReleased.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->brakeReleased);
      union {
        bool real;
        uint8_t base;
      } u_motorPowerDetected;
      u_motorPowerDetected.real = this->motorPowerDetected;
      *(outbuffer + offset + 0) = (u_motorPowerDetected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motorPowerDetected);
      union {
        bool real;
        uint8_t base;
      } u_embeddedMotCom;
      u_embeddedMotCom.real = this->embeddedMotCom;
      *(outbuffer + offset + 0) = (u_embeddedMotCom.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->embeddedMotCom);
      union {
        bool real;
        uint8_t base;
      } u_jointFaulted;
      u_jointFaulted.real = this->jointFaulted;
      *(outbuffer + offset + 0) = (u_jointFaulted.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->jointFaulted);
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
      uint32_t length_joint;
      memcpy(&length_joint, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_joint; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_joint-1]=0;
      this->joint = (char *)(inbuffer + offset-1);
      offset += length_joint;
      this->registerValue =  ((uint32_t) (*(inbuffer + offset)));
      this->registerValue |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->registerValue |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->registerValue |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->registerValue);
      union {
        bool real;
        uint8_t base;
      } u_bridgeEnabled;
      u_bridgeEnabled.base = 0;
      u_bridgeEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bridgeEnabled = u_bridgeEnabled.real;
      offset += sizeof(this->bridgeEnabled);
      union {
        bool real;
        uint8_t base;
      } u_motorEnabled;
      u_motorEnabled.base = 0;
      u_motorEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->motorEnabled = u_motorEnabled.real;
      offset += sizeof(this->motorEnabled);
      union {
        bool real;
        uint8_t base;
      } u_brakeReleased;
      u_brakeReleased.base = 0;
      u_brakeReleased.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->brakeReleased = u_brakeReleased.real;
      offset += sizeof(this->brakeReleased);
      union {
        bool real;
        uint8_t base;
      } u_motorPowerDetected;
      u_motorPowerDetected.base = 0;
      u_motorPowerDetected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->motorPowerDetected = u_motorPowerDetected.real;
      offset += sizeof(this->motorPowerDetected);
      union {
        bool real;
        uint8_t base;
      } u_embeddedMotCom;
      u_embeddedMotCom.base = 0;
      u_embeddedMotCom.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->embeddedMotCom = u_embeddedMotCom.real;
      offset += sizeof(this->embeddedMotCom);
      union {
        bool real;
        uint8_t base;
      } u_jointFaulted;
      u_jointFaulted.base = 0;
      u_jointFaulted.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->jointFaulted = u_jointFaulted.real;
      offset += sizeof(this->jointFaulted);
     return offset;
    }

    const char * getType(){ return "r2_msgs/JointStatus"; };
    const char * getMD5(){ return "03efe3c55d2652ccbab38921d83a5f63"; };

  };

}
#endif