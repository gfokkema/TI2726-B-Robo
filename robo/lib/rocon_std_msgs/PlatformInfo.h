#ifndef _ROS_rocon_std_msgs_PlatformInfo_h
#define _ROS_rocon_std_msgs_PlatformInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rocon_std_msgs/Icon.h"

namespace rocon_std_msgs
{

  class PlatformInfo : public ros::Msg
  {
    public:
      char * os;
      char * version;
      char * system;
      char * platform;
      char * name;
      rocon_std_msgs::Icon icon;
      enum { OS_ANY = * };
      enum { OS_LINUX = linux };
      enum { OS_UBUNTU = ubuntu };
      enum { OS_WINDOZE = windoze };
      enum { OS_ANDROID = android };
      enum { OS_APPLE = apple };
      enum { OS_IPHONE = iphone };
      enum { VERSION_ANY = * };
      enum { VERSION_UBUNTU_PRECISE = precise };
      enum { VERSION_ANDROID_JELLYBEAN = jellybean };
      enum { SYSTEM_ANY = * };
      enum { SYSTEM_CUSTOM = custom };
      enum { SYSTEM_ROS = ros };
      enum { SYSTEM_RQT = rqt };
      enum { SYSTEM_ROSJAVA = rosjava };
      enum { SYSTEM_ROSJS = rosjs };
      enum { SYSTEM_OPROS = opros };
      enum { PLATFORM_ANY = * };
      enum { PLATFORM_SMART_DEVICE = smart_device };
      enum { PLATFORM_PHONE = phone };
      enum { PLATFORM_TABLET = tablet };
      enum { PLATFORM_PC = pc };
      enum { PLATFORM_ROBOSEM = robosem };
      enum { PLATFORM_KOBUKI = kobuki };
      enum { PLATFORM_TURTLEBOT = turtlebot };
      enum { NAME_ANY = * };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_os = strlen( (const char*) this->os);
      memcpy(outbuffer + offset, &length_os, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->os, length_os);
      offset += length_os;
      uint32_t length_version = strlen( (const char*) this->version);
      memcpy(outbuffer + offset, &length_version, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->version, length_version);
      offset += length_version;
      uint32_t length_system = strlen( (const char*) this->system);
      memcpy(outbuffer + offset, &length_system, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->system, length_system);
      offset += length_system;
      uint32_t length_platform = strlen( (const char*) this->platform);
      memcpy(outbuffer + offset, &length_platform, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->platform, length_platform);
      offset += length_platform;
      uint32_t length_name = strlen( (const char*) this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += this->icon.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_os;
      memcpy(&length_os, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_os; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_os-1]=0;
      this->os = (char *)(inbuffer + offset-1);
      offset += length_os;
      uint32_t length_version;
      memcpy(&length_version, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_version; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_version-1]=0;
      this->version = (char *)(inbuffer + offset-1);
      offset += length_version;
      uint32_t length_system;
      memcpy(&length_system, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_system; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_system-1]=0;
      this->system = (char *)(inbuffer + offset-1);
      offset += length_system;
      uint32_t length_platform;
      memcpy(&length_platform, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_platform; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_platform-1]=0;
      this->platform = (char *)(inbuffer + offset-1);
      offset += length_platform;
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += this->icon.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "rocon_std_msgs/PlatformInfo"; };
    const char * getMD5(){ return "6eb077de6123a812533c2a069206b8d3"; };

  };

}
#endif