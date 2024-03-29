#ifndef _ROS_SERVICE_GetPlatformInfo_h
#define _ROS_SERVICE_GetPlatformInfo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rocon_std_msgs/PlatformInfo.h"

namespace rocon_std_msgs
{

static const char GETPLATFORMINFO[] = "rocon_std_msgs/GetPlatformInfo";

  class GetPlatformInfoRequest : public ros::Msg
  {
    public:

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETPLATFORMINFO; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetPlatformInfoResponse : public ros::Msg
  {
    public:
      rocon_std_msgs::PlatformInfo platform_info;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->platform_info.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->platform_info.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETPLATFORMINFO; };
    const char * getMD5(){ return "b8454ecd10f6c9bcc540342bfd0bb30c"; };

  };

  class GetPlatformInfo {
    public:
    typedef GetPlatformInfoRequest Request;
    typedef GetPlatformInfoResponse Response;
  };

}
#endif
