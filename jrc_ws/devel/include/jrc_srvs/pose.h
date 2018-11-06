// Generated by gencpp from file jrc_srvs/pose.msg
// DO NOT EDIT!


#ifndef JRC_SRVS_MESSAGE_POSE_H
#define JRC_SRVS_MESSAGE_POSE_H

#include <ros/service_traits.h>


#include <jrc_srvs/poseRequest.h>
#include <jrc_srvs/poseResponse.h>


namespace jrc_srvs
{

struct pose
{

typedef poseRequest Request;
typedef poseResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct pose
} // namespace jrc_srvs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::jrc_srvs::pose > {
  static const char* value()
  {
    return "1f3af93331fc1032113dd90d9a5f0755";
  }

  static const char* value(const ::jrc_srvs::pose&) { return value(); }
};

template<>
struct DataType< ::jrc_srvs::pose > {
  static const char* value()
  {
    return "jrc_srvs/pose";
  }

  static const char* value(const ::jrc_srvs::pose&) { return value(); }
};


// service_traits::MD5Sum< ::jrc_srvs::poseRequest> should match 
// service_traits::MD5Sum< ::jrc_srvs::pose > 
template<>
struct MD5Sum< ::jrc_srvs::poseRequest>
{
  static const char* value()
  {
    return MD5Sum< ::jrc_srvs::pose >::value();
  }
  static const char* value(const ::jrc_srvs::poseRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::jrc_srvs::poseRequest> should match 
// service_traits::DataType< ::jrc_srvs::pose > 
template<>
struct DataType< ::jrc_srvs::poseRequest>
{
  static const char* value()
  {
    return DataType< ::jrc_srvs::pose >::value();
  }
  static const char* value(const ::jrc_srvs::poseRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::jrc_srvs::poseResponse> should match 
// service_traits::MD5Sum< ::jrc_srvs::pose > 
template<>
struct MD5Sum< ::jrc_srvs::poseResponse>
{
  static const char* value()
  {
    return MD5Sum< ::jrc_srvs::pose >::value();
  }
  static const char* value(const ::jrc_srvs::poseResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::jrc_srvs::poseResponse> should match 
// service_traits::DataType< ::jrc_srvs::pose > 
template<>
struct DataType< ::jrc_srvs::poseResponse>
{
  static const char* value()
  {
    return DataType< ::jrc_srvs::pose >::value();
  }
  static const char* value(const ::jrc_srvs::poseResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // JRC_SRVS_MESSAGE_POSE_H
