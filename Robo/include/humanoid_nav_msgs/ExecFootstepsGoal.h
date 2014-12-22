#ifndef _ROS_humanoid_nav_msgs_ExecFootstepsGoal_h
#define _ROS_humanoid_nav_msgs_ExecFootstepsGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "humanoid_nav_msgs/StepTarget.h"

namespace humanoid_nav_msgs
{

  class ExecFootstepsGoal : public ros::Msg
  {
    public:
      uint8_t footsteps_length;
      humanoid_nav_msgs::StepTarget st_footsteps;
      humanoid_nav_msgs::StepTarget * footsteps;
      float feedback_frequency;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = footsteps_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < footsteps_length; i++){
      offset += this->footsteps[i].serialize(outbuffer + offset);
      }
      int32_t * val_feedback_frequency = (int32_t *) &(this->feedback_frequency);
      int32_t exp_feedback_frequency = (((*val_feedback_frequency)>>23)&255);
      if(exp_feedback_frequency != 0)
        exp_feedback_frequency += 1023-127;
      int32_t sig_feedback_frequency = *val_feedback_frequency;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_feedback_frequency<<5) & 0xff;
      *(outbuffer + offset++) = (sig_feedback_frequency>>3) & 0xff;
      *(outbuffer + offset++) = (sig_feedback_frequency>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_feedback_frequency<<4) & 0xF0) | ((sig_feedback_frequency>>19)&0x0F);
      *(outbuffer + offset++) = (exp_feedback_frequency>>4) & 0x7F;
      if(this->feedback_frequency < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t footsteps_lengthT = *(inbuffer + offset++);
      if(footsteps_lengthT > footsteps_length)
        this->footsteps = (humanoid_nav_msgs::StepTarget*)realloc(this->footsteps, footsteps_lengthT * sizeof(humanoid_nav_msgs::StepTarget));
      offset += 3;
      footsteps_length = footsteps_lengthT;
      for( uint8_t i = 0; i < footsteps_length; i++){
      offset += this->st_footsteps.deserialize(inbuffer + offset);
        memcpy( &(this->footsteps[i]), &(this->st_footsteps), sizeof(humanoid_nav_msgs::StepTarget));
      }
      uint32_t * val_feedback_frequency = (uint32_t*) &(this->feedback_frequency);
      offset += 3;
      *val_feedback_frequency = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_feedback_frequency |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_feedback_frequency |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_feedback_frequency |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_feedback_frequency = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_feedback_frequency |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_feedback_frequency !=0)
        *val_feedback_frequency |= ((exp_feedback_frequency)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->feedback_frequency = -this->feedback_frequency;
     return offset;
    }

    const char * getType(){ return "humanoid_nav_msgs/ExecFootstepsGoal"; };
    const char * getMD5(){ return "40a3f8092ef3bb49c3253baa6eb94932"; };

  };

}
#endif