#ifndef _ROS_r2_msgs_JointControl_h
#define _ROS_r2_msgs_JointControl_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2_msgs
{

  class JointControl : public ros::Msg
  {
    public:
      char * publisher;
      float timeStamp;
      char * joint;
      uint32_t registerValue;
      bool enableBridge;
      bool enableMotor;
      bool releaseBrake;
      bool embeddedMotCom;
      uint16_t controlMode;
      bool clearFaults;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_publisher = strlen( (const char*) this->publisher);
      memcpy(outbuffer + offset, &length_publisher, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->publisher, length_publisher);
      offset += length_publisher;
      int32_t * val_timeStamp = (int32_t *) &(this->timeStamp);
      int32_t exp_timeStamp = (((*val_timeStamp)>>23)&255);
      if(exp_timeStamp != 0)
        exp_timeStamp += 1023-127;
      int32_t sig_timeStamp = *val_timeStamp;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_timeStamp<<5) & 0xff;
      *(outbuffer + offset++) = (sig_timeStamp>>3) & 0xff;
      *(outbuffer + offset++) = (sig_timeStamp>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_timeStamp<<4) & 0xF0) | ((sig_timeStamp>>19)&0x0F);
      *(outbuffer + offset++) = (exp_timeStamp>>4) & 0x7F;
      if(this->timeStamp < 0) *(outbuffer + offset -1) |= 0x80;
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
      } u_enableBridge;
      u_enableBridge.real = this->enableBridge;
      *(outbuffer + offset + 0) = (u_enableBridge.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enableBridge);
      union {
        bool real;
        uint8_t base;
      } u_enableMotor;
      u_enableMotor.real = this->enableMotor;
      *(outbuffer + offset + 0) = (u_enableMotor.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enableMotor);
      union {
        bool real;
        uint8_t base;
      } u_releaseBrake;
      u_releaseBrake.real = this->releaseBrake;
      *(outbuffer + offset + 0) = (u_releaseBrake.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->releaseBrake);
      union {
        bool real;
        uint8_t base;
      } u_embeddedMotCom;
      u_embeddedMotCom.real = this->embeddedMotCom;
      *(outbuffer + offset + 0) = (u_embeddedMotCom.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->embeddedMotCom);
      *(outbuffer + offset + 0) = (this->controlMode >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->controlMode >> (8 * 1)) & 0xFF;
      offset += sizeof(this->controlMode);
      union {
        bool real;
        uint8_t base;
      } u_clearFaults;
      u_clearFaults.real = this->clearFaults;
      *(outbuffer + offset + 0) = (u_clearFaults.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->clearFaults);
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
      uint32_t * val_timeStamp = (uint32_t*) &(this->timeStamp);
      offset += 3;
      *val_timeStamp = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_timeStamp |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_timeStamp |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_timeStamp |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_timeStamp = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_timeStamp |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_timeStamp !=0)
        *val_timeStamp |= ((exp_timeStamp)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->timeStamp = -this->timeStamp;
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
      } u_enableBridge;
      u_enableBridge.base = 0;
      u_enableBridge.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enableBridge = u_enableBridge.real;
      offset += sizeof(this->enableBridge);
      union {
        bool real;
        uint8_t base;
      } u_enableMotor;
      u_enableMotor.base = 0;
      u_enableMotor.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enableMotor = u_enableMotor.real;
      offset += sizeof(this->enableMotor);
      union {
        bool real;
        uint8_t base;
      } u_releaseBrake;
      u_releaseBrake.base = 0;
      u_releaseBrake.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->releaseBrake = u_releaseBrake.real;
      offset += sizeof(this->releaseBrake);
      union {
        bool real;
        uint8_t base;
      } u_embeddedMotCom;
      u_embeddedMotCom.base = 0;
      u_embeddedMotCom.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->embeddedMotCom = u_embeddedMotCom.real;
      offset += sizeof(this->embeddedMotCom);
      this->controlMode =  ((uint16_t) (*(inbuffer + offset)));
      this->controlMode |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->controlMode);
      union {
        bool real;
        uint8_t base;
      } u_clearFaults;
      u_clearFaults.base = 0;
      u_clearFaults.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->clearFaults = u_clearFaults.real;
      offset += sizeof(this->clearFaults);
     return offset;
    }

    const char * getType(){ return "r2_msgs/JointControl"; };
    const char * getMD5(){ return "e38c02f4ffdedb57646a4576b7e0ccf6"; };

  };

}
#endif