#ifndef _ROS_brics_actuator_JointTorques_h
#define _ROS_brics_actuator_JointTorques_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "brics_actuator/Poison.h"
#include "brics_actuator/JointValue.h"

namespace brics_actuator
{

  class JointTorques : public ros::Msg
  {
    public:
      brics_actuator::Poison poisonStamp;
      uint8_t torques_length;
      brics_actuator::JointValue st_torques;
      brics_actuator::JointValue * torques;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->poisonStamp.serialize(outbuffer + offset);
      *(outbuffer + offset++) = torques_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < torques_length; i++){
      offset += this->torques[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->poisonStamp.deserialize(inbuffer + offset);
      uint8_t torques_lengthT = *(inbuffer + offset++);
      if(torques_lengthT > torques_length)
        this->torques = (brics_actuator::JointValue*)realloc(this->torques, torques_lengthT * sizeof(brics_actuator::JointValue));
      offset += 3;
      torques_length = torques_lengthT;
      for( uint8_t i = 0; i < torques_length; i++){
      offset += this->st_torques.deserialize(inbuffer + offset);
        memcpy( &(this->torques[i]), &(this->st_torques), sizeof(brics_actuator::JointValue));
      }
     return offset;
    }

    const char * getType(){ return "brics_actuator/JointTorques"; };
    const char * getMD5(){ return "75ccec429fa95e6b178bb9f3d06583e1"; };

  };

}
#endif