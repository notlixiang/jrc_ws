// Generated by gencpp from file jrc_srvs/grasp.msg
// DO NOT EDIT!


#ifndef JRC_SRVS_MESSAGE_GRASP_H
#define JRC_SRVS_MESSAGE_GRASP_H

#include <ros/service_traits.h>


#include <jrc_srvs/graspRequest.h>
#include <jrc_srvs/graspResponse.h>


namespace jrc_srvs
{

struct grasp
{

typedef graspRequest Request;
typedef graspResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct grasp
} // namespace jrc_srvs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::jrc_srvs::grasp > {
  static const char* value()
  {
    return "344890efce38d3f1768b922eb6058680";
  }

  static const char* value(const ::jrc_srvs::grasp&) { return value(); }
};

template<>
struct DataType< ::jrc_srvs::grasp > {
  static const char* value()
  {
    return "jrc_srvs/grasp";
  }

  static const char* value(const ::jrc_srvs::grasp&) { return value(); }
};


// service_traits::MD5Sum< ::jrc_srvs::graspRequest> should match 
// service_traits::MD5Sum< ::jrc_srvs::grasp > 
template<>
struct MD5Sum< ::jrc_srvs::graspRequest>
{
  static const char* value()
  {
    return MD5Sum< ::jrc_srvs::grasp >::value();
  }
  static const char* value(const ::jrc_srvs::graspRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::jrc_srvs::graspRequest> should match 
// service_traits::DataType< ::jrc_srvs::grasp > 
template<>
struct DataType< ::jrc_srvs::graspRequest>
{
  static const char* value()
  {
    return DataType< ::jrc_srvs::grasp >::value();
  }
  static const char* value(const ::jrc_srvs::graspRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::jrc_srvs::graspResponse> should match 
// service_traits::MD5Sum< ::jrc_srvs::grasp > 
template<>
struct MD5Sum< ::jrc_srvs::graspResponse>
{
  static const char* value()
  {
    return MD5Sum< ::jrc_srvs::grasp >::value();
  }
  static const char* value(const ::jrc_srvs::graspResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::jrc_srvs::graspResponse> should match 
// service_traits::DataType< ::jrc_srvs::grasp > 
template<>
struct DataType< ::jrc_srvs::graspResponse>
{
  static const char* value()
  {
    return DataType< ::jrc_srvs::grasp >::value();
  }
  static const char* value(const ::jrc_srvs::graspResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // JRC_SRVS_MESSAGE_GRASP_H
