#ifndef _ROS_brics_actuator_JointPositions_h
#define _ROS_brics_actuator_JointPositions_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "brics_actuator/Poison.h"
#include "brics_actuator/JointValue.h"

namespace brics_actuator
{

  class JointPositions : public ros::Msg
  {
    public:
      brics_actuator::Poison poisonStamp;
      uint8_t positions_length;
      brics_actuator::JointValue st_positions;
      brics_actuator::JointValue * positions;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->poisonStamp.serialize(outbuffer + offset);
      *(outbuffer + offset++) = positions_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < positions_length; i++){
      offset += this->positions[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->poisonStamp.deserialize(inbuffer + offset);
      uint8_t positions_lengthT = *(inbuffer + offset++);
      if(positions_lengthT > positions_length)
        this->positions = (brics_actuator::JointValue*)realloc(this->positions, positions_lengthT * sizeof(brics_actuator::JointValue));
      offset += 3;
      positions_length = positions_lengthT;
      for( uint8_t i = 0; i < positions_length; i++){
      offset += this->st_positions.deserialize(inbuffer + offset);
        memcpy( &(this->positions[i]), &(this->st_positions), sizeof(brics_actuator::JointValue));
      }
     return offset;
    }

    const char * getType(){ return "brics_actuator/JointPositions"; };
    const char * getMD5(){ return "2b7638f5c85f5257bef140a00cd0eec3"; };

  };

}
#endif