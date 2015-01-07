#ifndef _ROS_scheduler_msgs_Resource_h
#define _ROS_scheduler_msgs_Resource_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "uuid_msgs/UniqueID.h"
#include "rocon_std_msgs/Remapping.h"

namespace scheduler_msgs
{

  class Resource : public ros::Msg
  {
    public:
      char * name;
      uuid_msgs::UniqueID id;
      char * platform_info;
      uint8_t remappings_length;
      rocon_std_msgs::Remapping st_remappings;
      rocon_std_msgs::Remapping * remappings;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen( (const char*) this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += this->id.serialize(outbuffer + offset);
      uint32_t length_platform_info = strlen( (const char*) this->platform_info);
      memcpy(outbuffer + offset, &length_platform_info, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->platform_info, length_platform_info);
      offset += length_platform_info;
      *(outbuffer + offset++) = remappings_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < remappings_length; i++){
      offset += this->remappings[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += this->id.deserialize(inbuffer + offset);
      uint32_t length_platform_info;
      memcpy(&length_platform_info, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_platform_info; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_platform_info-1]=0;
      this->platform_info = (char *)(inbuffer + offset-1);
      offset += length_platform_info;
      uint8_t remappings_lengthT = *(inbuffer + offset++);
      if(remappings_lengthT > remappings_length)
        this->remappings = (rocon_std_msgs::Remapping*)realloc(this->remappings, remappings_lengthT * sizeof(rocon_std_msgs::Remapping));
      offset += 3;
      remappings_length = remappings_lengthT;
      for( uint8_t i = 0; i < remappings_length; i++){
      offset += this->st_remappings.deserialize(inbuffer + offset);
        memcpy( &(this->remappings[i]), &(this->st_remappings), sizeof(rocon_std_msgs::Remapping));
      }
     return offset;
    }

    const char * getType(){ return "scheduler_msgs/Resource"; };
    const char * getMD5(){ return "865085c13bbbd476a6d6007c401ae0f5"; };

  };

}
#endif