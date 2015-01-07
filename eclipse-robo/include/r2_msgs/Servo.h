#ifndef _ROS_SERVICE_Servo_h
#define _ROS_SERVICE_Servo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2_msgs
{

static const char SERVO[] = "r2_msgs/Servo";

  class ServoRequest : public ros::Msg
  {
    public:
      char * joint;
      bool state;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_joint = strlen( (const char*) this->joint);
      memcpy(outbuffer + offset, &length_joint, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->joint, length_joint);
      offset += length_joint;
      union {
        bool real;
        uint8_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_joint;
      memcpy(&length_joint, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_joint; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_joint-1]=0;
      this->joint = (char *)(inbuffer + offset-1);
      offset += length_joint;
      union {
        bool real;
        uint8_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->state = u_state.real;
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return SERVO; };
    const char * getMD5(){ return "84ae688f7219fea2850abe26519fc2d0"; };

  };

  class ServoResponse : public ros::Msg
  {
    public:
      int32_t status;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_status.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_status.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_status.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return SERVO; };
    const char * getMD5(){ return "86791dcf1de997ec7de5a0de7e4dcfcc"; };

  };

  class Servo {
    public:
    typedef ServoRequest Request;
    typedef ServoResponse Response;
  };

}
#endif
