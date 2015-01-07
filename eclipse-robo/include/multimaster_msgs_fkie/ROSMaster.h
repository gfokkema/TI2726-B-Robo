#ifndef _ROS_multimaster_msgs_fkie_ROSMaster_h
#define _ROS_multimaster_msgs_fkie_ROSMaster_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace multimaster_msgs_fkie
{

  class ROSMaster : public ros::Msg
  {
    public:
      char * name;
      char * uri;
      float timestamp;
      float timestamp_local;
      bool online;
      char * discoverer_name;
      char * monitoruri;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen( (const char*) this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_uri = strlen( (const char*) this->uri);
      memcpy(outbuffer + offset, &length_uri, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->uri, length_uri);
      offset += length_uri;
      int32_t * val_timestamp = (int32_t *) &(this->timestamp);
      int32_t exp_timestamp = (((*val_timestamp)>>23)&255);
      if(exp_timestamp != 0)
        exp_timestamp += 1023-127;
      int32_t sig_timestamp = *val_timestamp;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_timestamp<<5) & 0xff;
      *(outbuffer + offset++) = (sig_timestamp>>3) & 0xff;
      *(outbuffer + offset++) = (sig_timestamp>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_timestamp<<4) & 0xF0) | ((sig_timestamp>>19)&0x0F);
      *(outbuffer + offset++) = (exp_timestamp>>4) & 0x7F;
      if(this->timestamp < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_timestamp_local = (int32_t *) &(this->timestamp_local);
      int32_t exp_timestamp_local = (((*val_timestamp_local)>>23)&255);
      if(exp_timestamp_local != 0)
        exp_timestamp_local += 1023-127;
      int32_t sig_timestamp_local = *val_timestamp_local;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_timestamp_local<<5) & 0xff;
      *(outbuffer + offset++) = (sig_timestamp_local>>3) & 0xff;
      *(outbuffer + offset++) = (sig_timestamp_local>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_timestamp_local<<4) & 0xF0) | ((sig_timestamp_local>>19)&0x0F);
      *(outbuffer + offset++) = (exp_timestamp_local>>4) & 0x7F;
      if(this->timestamp_local < 0) *(outbuffer + offset -1) |= 0x80;
      union {
        bool real;
        uint8_t base;
      } u_online;
      u_online.real = this->online;
      *(outbuffer + offset + 0) = (u_online.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->online);
      uint32_t length_discoverer_name = strlen( (const char*) this->discoverer_name);
      memcpy(outbuffer + offset, &length_discoverer_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->discoverer_name, length_discoverer_name);
      offset += length_discoverer_name;
      uint32_t length_monitoruri = strlen( (const char*) this->monitoruri);
      memcpy(outbuffer + offset, &length_monitoruri, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->monitoruri, length_monitoruri);
      offset += length_monitoruri;
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
      uint32_t length_uri;
      memcpy(&length_uri, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_uri; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_uri-1]=0;
      this->uri = (char *)(inbuffer + offset-1);
      offset += length_uri;
      uint32_t * val_timestamp = (uint32_t*) &(this->timestamp);
      offset += 3;
      *val_timestamp = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_timestamp |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_timestamp |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_timestamp |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_timestamp = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_timestamp |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_timestamp !=0)
        *val_timestamp |= ((exp_timestamp)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->timestamp = -this->timestamp;
      uint32_t * val_timestamp_local = (uint32_t*) &(this->timestamp_local);
      offset += 3;
      *val_timestamp_local = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_timestamp_local |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_timestamp_local |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_timestamp_local |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_timestamp_local = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_timestamp_local |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_timestamp_local !=0)
        *val_timestamp_local |= ((exp_timestamp_local)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->timestamp_local = -this->timestamp_local;
      union {
        bool real;
        uint8_t base;
      } u_online;
      u_online.base = 0;
      u_online.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->online = u_online.real;
      offset += sizeof(this->online);
      uint32_t length_discoverer_name;
      memcpy(&length_discoverer_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_discoverer_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_discoverer_name-1]=0;
      this->discoverer_name = (char *)(inbuffer + offset-1);
      offset += length_discoverer_name;
      uint32_t length_monitoruri;
      memcpy(&length_monitoruri, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_monitoruri; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_monitoruri-1]=0;
      this->monitoruri = (char *)(inbuffer + offset-1);
      offset += length_monitoruri;
     return offset;
    }

    const char * getType(){ return "multimaster_msgs_fkie/ROSMaster"; };
    const char * getMD5(){ return "08ea76968a3be2b8b1c6550b39616f72"; };

  };

}
#endif