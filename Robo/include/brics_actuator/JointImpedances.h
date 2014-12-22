#ifndef _ROS_brics_actuator_JointImpedances_h
#define _ROS_brics_actuator_JointImpedances_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "brics_actuator/Poison.h"
#include "brics_actuator/JointValue.h"

namespace brics_actuator
{

  class JointImpedances : public ros::Msg
  {
    public:
      brics_actuator::Poison poisonStamp;
      uint8_t dampings_length;
      brics_actuator::JointValue st_dampings;
      brics_actuator::JointValue * dampings;
      uint8_t stiffnesses_length;
      brics_actuator::JointValue st_stiffnesses;
      brics_actuator::JointValue * stiffnesses;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->poisonStamp.serialize(outbuffer + offset);
      *(outbuffer + offset++) = dampings_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < dampings_length; i++){
      offset += this->dampings[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = stiffnesses_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < stiffnesses_length; i++){
      offset += this->stiffnesses[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->poisonStamp.deserialize(inbuffer + offset);
      uint8_t dampings_lengthT = *(inbuffer + offset++);
      if(dampings_lengthT > dampings_length)
        this->dampings = (brics_actuator::JointValue*)realloc(this->dampings, dampings_lengthT * sizeof(brics_actuator::JointValue));
      offset += 3;
      dampings_length = dampings_lengthT;
      for( uint8_t i = 0; i < dampings_length; i++){
      offset += this->st_dampings.deserialize(inbuffer + offset);
        memcpy( &(this->dampings[i]), &(this->st_dampings), sizeof(brics_actuator::JointValue));
      }
      uint8_t stiffnesses_lengthT = *(inbuffer + offset++);
      if(stiffnesses_lengthT > stiffnesses_length)
        this->stiffnesses = (brics_actuator::JointValue*)realloc(this->stiffnesses, stiffnesses_lengthT * sizeof(brics_actuator::JointValue));
      offset += 3;
      stiffnesses_length = stiffnesses_lengthT;
      for( uint8_t i = 0; i < stiffnesses_length; i++){
      offset += this->st_stiffnesses.deserialize(inbuffer + offset);
        memcpy( &(this->stiffnesses[i]), &(this->st_stiffnesses), sizeof(brics_actuator::JointValue));
      }
     return offset;
    }

    const char * getType(){ return "brics_actuator/JointImpedances"; };
    const char * getMD5(){ return "14fdbb76121290f17525acd99e7242c6"; };

  };

}
#endif