#ifndef _ROS_brics_actuator_JointVelocities_h
#define _ROS_brics_actuator_JointVelocities_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "brics_actuator/Poison.h"
#include "brics_actuator/JointValue.h"

namespace brics_actuator
{

  class JointVelocities : public ros::Msg
  {
    public:
      brics_actuator::Poison poisonStamp;
      uint8_t velocities_length;
      brics_actuator::JointValue st_velocities;
      brics_actuator::JointValue * velocities;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->poisonStamp.serialize(outbuffer + offset);
      *(outbuffer + offset++) = velocities_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < velocities_length; i++){
      offset += this->velocities[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->poisonStamp.deserialize(inbuffer + offset);
      uint8_t velocities_lengthT = *(inbuffer + offset++);
      if(velocities_lengthT > velocities_length)
        this->velocities = (brics_actuator::JointValue*)realloc(this->velocities, velocities_lengthT * sizeof(brics_actuator::JointValue));
      offset += 3;
      velocities_length = velocities_lengthT;
      for( uint8_t i = 0; i < velocities_length; i++){
      offset += this->st_velocities.deserialize(inbuffer + offset);
        memcpy( &(this->velocities[i]), &(this->st_velocities), sizeof(brics_actuator::JointValue));
      }
     return offset;
    }

    const char * getType(){ return "brics_actuator/JointVelocities"; };
    const char * getMD5(){ return "9786afc900be50b3ba4113fcb65c9363"; };

  };

}
#endif