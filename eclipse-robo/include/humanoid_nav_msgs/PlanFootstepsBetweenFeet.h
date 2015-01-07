#ifndef _ROS_SERVICE_PlanFootstepsBetweenFeet_h
#define _ROS_SERVICE_PlanFootstepsBetweenFeet_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "humanoid_nav_msgs/StepTarget.h"

namespace humanoid_nav_msgs
{

static const char PLANFOOTSTEPSBETWEENFEET[] = "humanoid_nav_msgs/PlanFootstepsBetweenFeet";

  class PlanFootstepsBetweenFeetRequest : public ros::Msg
  {
    public:
      humanoid_nav_msgs::StepTarget start_left;
      humanoid_nav_msgs::StepTarget start_right;
      humanoid_nav_msgs::StepTarget goal_left;
      humanoid_nav_msgs::StepTarget goal_right;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->start_left.serialize(outbuffer + offset);
      offset += this->start_right.serialize(outbuffer + offset);
      offset += this->goal_left.serialize(outbuffer + offset);
      offset += this->goal_right.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->start_left.deserialize(inbuffer + offset);
      offset += this->start_right.deserialize(inbuffer + offset);
      offset += this->goal_left.deserialize(inbuffer + offset);
      offset += this->goal_right.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return PLANFOOTSTEPSBETWEENFEET; };
    const char * getMD5(){ return "a081711eb51a4a4283b2b9f345efe272"; };

  };

  class PlanFootstepsBetweenFeetResponse : public ros::Msg
  {
    public:
      bool result;
      uint8_t footsteps_length;
      humanoid_nav_msgs::StepTarget st_footsteps;
      humanoid_nav_msgs::StepTarget * footsteps;
      float costs;
      float final_eps;
      float planning_time;
      int64_t expanded_states;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      *(outbuffer + offset++) = footsteps_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < footsteps_length; i++){
      offset += this->footsteps[i].serialize(outbuffer + offset);
      }
      int32_t * val_costs = (int32_t *) &(this->costs);
      int32_t exp_costs = (((*val_costs)>>23)&255);
      if(exp_costs != 0)
        exp_costs += 1023-127;
      int32_t sig_costs = *val_costs;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_costs<<5) & 0xff;
      *(outbuffer + offset++) = (sig_costs>>3) & 0xff;
      *(outbuffer + offset++) = (sig_costs>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_costs<<4) & 0xF0) | ((sig_costs>>19)&0x0F);
      *(outbuffer + offset++) = (exp_costs>>4) & 0x7F;
      if(this->costs < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_final_eps = (int32_t *) &(this->final_eps);
      int32_t exp_final_eps = (((*val_final_eps)>>23)&255);
      if(exp_final_eps != 0)
        exp_final_eps += 1023-127;
      int32_t sig_final_eps = *val_final_eps;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_final_eps<<5) & 0xff;
      *(outbuffer + offset++) = (sig_final_eps>>3) & 0xff;
      *(outbuffer + offset++) = (sig_final_eps>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_final_eps<<4) & 0xF0) | ((sig_final_eps>>19)&0x0F);
      *(outbuffer + offset++) = (exp_final_eps>>4) & 0x7F;
      if(this->final_eps < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_planning_time = (int32_t *) &(this->planning_time);
      int32_t exp_planning_time = (((*val_planning_time)>>23)&255);
      if(exp_planning_time != 0)
        exp_planning_time += 1023-127;
      int32_t sig_planning_time = *val_planning_time;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_planning_time<<5) & 0xff;
      *(outbuffer + offset++) = (sig_planning_time>>3) & 0xff;
      *(outbuffer + offset++) = (sig_planning_time>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_planning_time<<4) & 0xF0) | ((sig_planning_time>>19)&0x0F);
      *(outbuffer + offset++) = (exp_planning_time>>4) & 0x7F;
      if(this->planning_time < 0) *(outbuffer + offset -1) |= 0x80;
      union {
        int64_t real;
        uint64_t base;
      } u_expanded_states;
      u_expanded_states.real = this->expanded_states;
      *(outbuffer + offset + 0) = (u_expanded_states.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_expanded_states.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_expanded_states.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_expanded_states.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_expanded_states.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_expanded_states.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_expanded_states.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_expanded_states.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->expanded_states);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
      uint8_t footsteps_lengthT = *(inbuffer + offset++);
      if(footsteps_lengthT > footsteps_length)
        this->footsteps = (humanoid_nav_msgs::StepTarget*)realloc(this->footsteps, footsteps_lengthT * sizeof(humanoid_nav_msgs::StepTarget));
      offset += 3;
      footsteps_length = footsteps_lengthT;
      for( uint8_t i = 0; i < footsteps_length; i++){
      offset += this->st_footsteps.deserialize(inbuffer + offset);
        memcpy( &(this->footsteps[i]), &(this->st_footsteps), sizeof(humanoid_nav_msgs::StepTarget));
      }
      uint32_t * val_costs = (uint32_t*) &(this->costs);
      offset += 3;
      *val_costs = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_costs |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_costs |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_costs |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_costs = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_costs |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_costs !=0)
        *val_costs |= ((exp_costs)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->costs = -this->costs;
      uint32_t * val_final_eps = (uint32_t*) &(this->final_eps);
      offset += 3;
      *val_final_eps = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_final_eps |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_final_eps |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_final_eps |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_final_eps = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_final_eps |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_final_eps !=0)
        *val_final_eps |= ((exp_final_eps)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->final_eps = -this->final_eps;
      uint32_t * val_planning_time = (uint32_t*) &(this->planning_time);
      offset += 3;
      *val_planning_time = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_planning_time |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_planning_time |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_planning_time |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_planning_time = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_planning_time |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_planning_time !=0)
        *val_planning_time |= ((exp_planning_time)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->planning_time = -this->planning_time;
      union {
        int64_t real;
        uint64_t base;
      } u_expanded_states;
      u_expanded_states.base = 0;
      u_expanded_states.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_expanded_states.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_expanded_states.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_expanded_states.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_expanded_states.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_expanded_states.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_expanded_states.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_expanded_states.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->expanded_states = u_expanded_states.real;
      offset += sizeof(this->expanded_states);
     return offset;
    }

    const char * getType(){ return PLANFOOTSTEPSBETWEENFEET; };
    const char * getMD5(){ return "1af07cd1d4ffe34a9a731e87aa13835c"; };

  };

  class PlanFootstepsBetweenFeet {
    public:
    typedef PlanFootstepsBetweenFeetRequest Request;
    typedef PlanFootstepsBetweenFeetResponse Response;
  };

}
#endif
