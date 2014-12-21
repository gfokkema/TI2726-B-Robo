#ifndef _ROS_cmvision_Blob_h
#define _ROS_cmvision_Blob_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cmvision
{

  class Blob : public ros::Msg
  {
    public:
      uint32_t red;
      uint32_t green;
      uint32_t blue;
      uint32_t area;
      uint32_t x;
      uint32_t y;
      uint32_t left;
      uint32_t right;
      uint32_t top;
      uint32_t bottom;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->red >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->red >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->red >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->red >> (8 * 3)) & 0xFF;
      offset += sizeof(this->red);
      *(outbuffer + offset + 0) = (this->green >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->green >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->green >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->green >> (8 * 3)) & 0xFF;
      offset += sizeof(this->green);
      *(outbuffer + offset + 0) = (this->blue >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->blue >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->blue >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->blue >> (8 * 3)) & 0xFF;
      offset += sizeof(this->blue);
      *(outbuffer + offset + 0) = (this->area >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->area >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->area >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->area >> (8 * 3)) & 0xFF;
      offset += sizeof(this->area);
      *(outbuffer + offset + 0) = (this->x >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->x >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->x >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->x >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      *(outbuffer + offset + 0) = (this->y >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->y >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->y >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->y >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      *(outbuffer + offset + 0) = (this->left >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->left >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->left >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left);
      *(outbuffer + offset + 0) = (this->right >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->right >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->right >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right);
      *(outbuffer + offset + 0) = (this->top >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->top >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->top >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->top >> (8 * 3)) & 0xFF;
      offset += sizeof(this->top);
      *(outbuffer + offset + 0) = (this->bottom >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->bottom >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->bottom >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->bottom >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bottom);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->red =  ((uint32_t) (*(inbuffer + offset)));
      this->red |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->red |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->red |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->red);
      this->green =  ((uint32_t) (*(inbuffer + offset)));
      this->green |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->green |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->green |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->green);
      this->blue =  ((uint32_t) (*(inbuffer + offset)));
      this->blue |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->blue |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->blue |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->blue);
      this->area =  ((uint32_t) (*(inbuffer + offset)));
      this->area |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->area |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->area |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->area);
      this->x =  ((uint32_t) (*(inbuffer + offset)));
      this->x |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->x |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->x |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->x);
      this->y =  ((uint32_t) (*(inbuffer + offset)));
      this->y |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->y |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->y |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->y);
      this->left =  ((uint32_t) (*(inbuffer + offset)));
      this->left |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->left |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->left |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->left);
      this->right =  ((uint32_t) (*(inbuffer + offset)));
      this->right |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->right |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->right |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->right);
      this->top =  ((uint32_t) (*(inbuffer + offset)));
      this->top |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->top |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->top |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->top);
      this->bottom =  ((uint32_t) (*(inbuffer + offset)));
      this->bottom |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->bottom |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->bottom |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->bottom);
     return offset;
    }

    const char * getType(){ return "cmvision/Blob"; };
    const char * getMD5(){ return "56ed098ed179c6b791d85398b11f5ff6"; };

  };

}
#endif