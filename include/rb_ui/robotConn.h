// Generated by gencpp from file rb_ui/robotConn.msg
// DO NOT EDIT!


#ifndef RB_UI_MESSAGE_ROBOTCONN_H
#define RB_UI_MESSAGE_ROBOTCONN_H

#include <ros/service_traits.h>


#include <rb_ui/robotConnRequest.h>
#include <rb_ui/robotConnResponse.h>


namespace rb_ui
{

struct robotConn
{

typedef robotConnRequest Request;
typedef robotConnResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct robotConn
} // namespace rb_ui


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rb_ui::robotConn > {
  static const char* value()
  {
    return "e2cc9e9d8c464550830df49c160979ad";
  }

  static const char* value(const ::rb_ui::robotConn&) { return value(); }
};

template<>
struct DataType< ::rb_ui::robotConn > {
  static const char* value()
  {
    return "rb_ui/robotConn";
  }

  static const char* value(const ::rb_ui::robotConn&) { return value(); }
};


// service_traits::MD5Sum< ::rb_ui::robotConnRequest> should match 
// service_traits::MD5Sum< ::rb_ui::robotConn > 
template<>
struct MD5Sum< ::rb_ui::robotConnRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rb_ui::robotConn >::value();
  }
  static const char* value(const ::rb_ui::robotConnRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rb_ui::robotConnRequest> should match 
// service_traits::DataType< ::rb_ui::robotConn > 
template<>
struct DataType< ::rb_ui::robotConnRequest>
{
  static const char* value()
  {
    return DataType< ::rb_ui::robotConn >::value();
  }
  static const char* value(const ::rb_ui::robotConnRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rb_ui::robotConnResponse> should match 
// service_traits::MD5Sum< ::rb_ui::robotConn > 
template<>
struct MD5Sum< ::rb_ui::robotConnResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rb_ui::robotConn >::value();
  }
  static const char* value(const ::rb_ui::robotConnResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rb_ui::robotConnResponse> should match 
// service_traits::DataType< ::rb_ui::robotConn > 
template<>
struct DataType< ::rb_ui::robotConnResponse>
{
  static const char* value()
  {
    return DataType< ::rb_ui::robotConn >::value();
  }
  static const char* value(const ::rb_ui::robotConnResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // RB_UI_MESSAGE_ROBOTCONN_H
