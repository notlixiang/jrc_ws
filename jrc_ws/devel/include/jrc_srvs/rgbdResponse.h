// Generated by gencpp from file jrc_srvs/rgbdResponse.msg
// DO NOT EDIT!


#ifndef JRC_SRVS_MESSAGE_RGBDRESPONSE_H
#define JRC_SRVS_MESSAGE_RGBDRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>

namespace jrc_srvs
{
template <class ContainerAllocator>
struct rgbdResponse_
{
  typedef rgbdResponse_<ContainerAllocator> Type;

  rgbdResponse_()
    : rgb_image()
    , depth_image()  {
    }
  rgbdResponse_(const ContainerAllocator& _alloc)
    : rgb_image(_alloc)
    , depth_image(_alloc)  {
  (void)_alloc;
    }



   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _rgb_image_type;
  _rgb_image_type rgb_image;

   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _depth_image_type;
  _depth_image_type depth_image;





  typedef boost::shared_ptr< ::jrc_srvs::rgbdResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jrc_srvs::rgbdResponse_<ContainerAllocator> const> ConstPtr;

}; // struct rgbdResponse_

typedef ::jrc_srvs::rgbdResponse_<std::allocator<void> > rgbdResponse;

typedef boost::shared_ptr< ::jrc_srvs::rgbdResponse > rgbdResponsePtr;
typedef boost::shared_ptr< ::jrc_srvs::rgbdResponse const> rgbdResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jrc_srvs::rgbdResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jrc_srvs::rgbdResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace jrc_srvs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::jrc_srvs::rgbdResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jrc_srvs::rgbdResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jrc_srvs::rgbdResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jrc_srvs::rgbdResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jrc_srvs::rgbdResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jrc_srvs::rgbdResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jrc_srvs::rgbdResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "33cb5836216cc2f10faa186b171cbfdc";
  }

  static const char* value(const ::jrc_srvs::rgbdResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x33cb5836216cc2f1ULL;
  static const uint64_t static_value2 = 0x0faa186b171cbfdcULL;
};

template<class ContainerAllocator>
struct DataType< ::jrc_srvs::rgbdResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jrc_srvs/rgbdResponse";
  }

  static const char* value(const ::jrc_srvs::rgbdResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jrc_srvs::rgbdResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensor_msgs/Image rgb_image\n\
sensor_msgs/Image depth_image\n\
\n\
\n\
================================================================================\n\
MSG: sensor_msgs/Image\n\
# This message contains an uncompressed image\n\
# (0, 0) is at top-left corner of image\n\
#\n\
\n\
Header header        # Header timestamp should be acquisition time of image\n\
                     # Header frame_id should be optical frame of camera\n\
                     # origin of frame should be optical center of cameara\n\
                     # +x should point to the right in the image\n\
                     # +y should point down in the image\n\
                     # +z should point into to plane of the image\n\
                     # If the frame_id here and the frame_id of the CameraInfo\n\
                     # message associated with the image conflict\n\
                     # the behavior is undefined\n\
\n\
uint32 height         # image height, that is, number of rows\n\
uint32 width          # image width, that is, number of columns\n\
\n\
# The legal values for encoding are in file src/image_encodings.cpp\n\
# If you want to standardize a new string format, join\n\
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\
\n\
string encoding       # Encoding of pixels -- channel meaning, ordering, size\n\
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n\
\n\
uint8 is_bigendian    # is this data bigendian?\n\
uint32 step           # Full row length in bytes\n\
uint8[] data          # actual matrix data, size is (step * rows)\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::jrc_srvs::rgbdResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jrc_srvs::rgbdResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.rgb_image);
      stream.next(m.depth_image);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct rgbdResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jrc_srvs::rgbdResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jrc_srvs::rgbdResponse_<ContainerAllocator>& v)
  {
    s << indent << "rgb_image: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.rgb_image);
    s << indent << "depth_image: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.depth_image);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JRC_SRVS_MESSAGE_RGBDRESPONSE_H
