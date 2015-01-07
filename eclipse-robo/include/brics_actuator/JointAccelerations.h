#ifndef _ROS_brics_actuator_JointAccelerations_h
#define _ROS_brics_actuator_JointAccelerations_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "brics_actuator/Poison.h"
#include "brics_actuator/JointValue.h"

namespace brics_actuator
{

  class JointAccelerations : public ros::Msg
  {
    public:
      brics_actuator::Poison poisonStamp;
      uint8_t accelerations_length;
      brics_actuator::JointValue st_accelerations;
      brics_actuator::JointValue * accelerations;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->poisonStamp.serialize(outbuffer + offset);
      *(outbuffer + offset++) = accelerations_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < accelerations_length; i++){
      offset += this->accelerations[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->poisonStamp.deserialize(inbuffer + offset);
      uint8_t accelerations_lengthT = *(inbuffer + offset++);
      if(accelerations_lengthT > accelerations_length)
        this->accelerations = (brics_actuator::JointValue*)realloc(this->accelerations, accelerations_lengthT * sizeof(brics_actuator::JointValue));
      offset += 3;
      accelerations_length = accelerations_lengthT;
      for( uint8_t i = 0; i < accelerations_length; i++){
      offset += this->st_accelerations.deserialize(inbuffer + offset);
        memcpy( &(this->accelerations[i]), &(this->st_accelerations), sizeof(brics_actuator::JointValue));
      }
     return offset;
    }

    const char * getType(){ return "brics_actuator/JointAccelerations"; };
    const char * getMD5(){ return "f904d45aa2c1d97ea9e99a3ba4e3610e"; };

  };

}
#endif