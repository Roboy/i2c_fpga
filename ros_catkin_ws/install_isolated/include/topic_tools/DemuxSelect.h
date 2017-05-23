// Generated by gencpp from file topic_tools/DemuxSelect.msg
// DO NOT EDIT!


#ifndef TOPIC_TOOLS_MESSAGE_DEMUXSELECT_H
#define TOPIC_TOOLS_MESSAGE_DEMUXSELECT_H

#include <ros/service_traits.h>


#include <topic_tools/DemuxSelectRequest.h>
#include <topic_tools/DemuxSelectResponse.h>


namespace topic_tools
{

struct DemuxSelect
{

typedef DemuxSelectRequest Request;
typedef DemuxSelectResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct DemuxSelect
} // namespace topic_tools


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::topic_tools::DemuxSelect > {
  static const char* value()
  {
    return "053052240ca985e1f2eedbb0dae9b1f7";
  }

  static const char* value(const ::topic_tools::DemuxSelect&) { return value(); }
};

template<>
struct DataType< ::topic_tools::DemuxSelect > {
  static const char* value()
  {
    return "topic_tools/DemuxSelect";
  }

  static const char* value(const ::topic_tools::DemuxSelect&) { return value(); }
};


// service_traits::MD5Sum< ::topic_tools::DemuxSelectRequest> should match 
// service_traits::MD5Sum< ::topic_tools::DemuxSelect > 
template<>
struct MD5Sum< ::topic_tools::DemuxSelectRequest>
{
  static const char* value()
  {
    return MD5Sum< ::topic_tools::DemuxSelect >::value();
  }
  static const char* value(const ::topic_tools::DemuxSelectRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::topic_tools::DemuxSelectRequest> should match 
// service_traits::DataType< ::topic_tools::DemuxSelect > 
template<>
struct DataType< ::topic_tools::DemuxSelectRequest>
{
  static const char* value()
  {
    return DataType< ::topic_tools::DemuxSelect >::value();
  }
  static const char* value(const ::topic_tools::DemuxSelectRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::topic_tools::DemuxSelectResponse> should match 
// service_traits::MD5Sum< ::topic_tools::DemuxSelect > 
template<>
struct MD5Sum< ::topic_tools::DemuxSelectResponse>
{
  static const char* value()
  {
    return MD5Sum< ::topic_tools::DemuxSelect >::value();
  }
  static const char* value(const ::topic_tools::DemuxSelectResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::topic_tools::DemuxSelectResponse> should match 
// service_traits::DataType< ::topic_tools::DemuxSelect > 
template<>
struct DataType< ::topic_tools::DemuxSelectResponse>
{
  static const char* value()
  {
    return DataType< ::topic_tools::DemuxSelect >::value();
  }
  static const char* value(const ::topic_tools::DemuxSelectResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // TOPIC_TOOLS_MESSAGE_DEMUXSELECT_H
