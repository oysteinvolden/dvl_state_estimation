// Generated by gencpp from file sbg_driver/SbgGpsHdt.msg
// DO NOT EDIT!


#ifndef SBG_DRIVER_MESSAGE_SBGGPSHDT_H
#define SBG_DRIVER_MESSAGE_SBGGPSHDT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace sbg_driver
{
template <class ContainerAllocator>
struct SbgGpsHdt_
{
  typedef SbgGpsHdt_<ContainerAllocator> Type;

  SbgGpsHdt_()
    : header()
    , time_stamp(0)
    , status(0)
    , tow(0)
    , true_heading(0.0)
    , true_heading_acc(0.0)
    , pitch(0.0)
    , pitch_acc(0.0)  {
    }
  SbgGpsHdt_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , time_stamp(0)
    , status(0)
    , tow(0)
    , true_heading(0.0)
    , true_heading_acc(0.0)
    , pitch(0.0)
    , pitch_acc(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint32_t _time_stamp_type;
  _time_stamp_type time_stamp;

   typedef uint16_t _status_type;
  _status_type status;

   typedef uint32_t _tow_type;
  _tow_type tow;

   typedef float _true_heading_type;
  _true_heading_type true_heading;

   typedef float _true_heading_acc_type;
  _true_heading_acc_type true_heading_acc;

   typedef float _pitch_type;
  _pitch_type pitch;

   typedef float _pitch_acc_type;
  _pitch_acc_type pitch_acc;





  typedef boost::shared_ptr< ::sbg_driver::SbgGpsHdt_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sbg_driver::SbgGpsHdt_<ContainerAllocator> const> ConstPtr;

}; // struct SbgGpsHdt_

typedef ::sbg_driver::SbgGpsHdt_<std::allocator<void> > SbgGpsHdt;

typedef boost::shared_ptr< ::sbg_driver::SbgGpsHdt > SbgGpsHdtPtr;
typedef boost::shared_ptr< ::sbg_driver::SbgGpsHdt const> SbgGpsHdtConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sbg_driver::SbgGpsHdt_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sbg_driver::SbgGpsHdt_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::sbg_driver::SbgGpsHdt_<ContainerAllocator1> & lhs, const ::sbg_driver::SbgGpsHdt_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.time_stamp == rhs.time_stamp &&
    lhs.status == rhs.status &&
    lhs.tow == rhs.tow &&
    lhs.true_heading == rhs.true_heading &&
    lhs.true_heading_acc == rhs.true_heading_acc &&
    lhs.pitch == rhs.pitch &&
    lhs.pitch_acc == rhs.pitch_acc;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::sbg_driver::SbgGpsHdt_<ContainerAllocator1> & lhs, const ::sbg_driver::SbgGpsHdt_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace sbg_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::sbg_driver::SbgGpsHdt_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sbg_driver::SbgGpsHdt_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sbg_driver::SbgGpsHdt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sbg_driver::SbgGpsHdt_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sbg_driver::SbgGpsHdt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sbg_driver::SbgGpsHdt_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sbg_driver::SbgGpsHdt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e132ccfa1c1c41e27b0c5998a0ca02cd";
  }

  static const char* value(const ::sbg_driver::SbgGpsHdt_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe132ccfa1c1c41e2ULL;
  static const uint64_t static_value2 = 0x7b0c5998a0ca02cdULL;
};

template<class ContainerAllocator>
struct DataType< ::sbg_driver::SbgGpsHdt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sbg_driver/SbgGpsHdt";
  }

  static const char* value(const ::sbg_driver::SbgGpsHdt_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sbg_driver::SbgGpsHdt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# SBG Ellipse Messages\n"
"Header header\n"
"\n"
"# Time since sensor is powered up us \n"
"uint32 time_stamp\n"
"\n"
"# GPS True Heading status.\n"
"# 0 SOL_COMPUTED		A valid solution has been computed.\n"
"# 1 INSUFFICIENT_OBS	Not enough valid SV to compute a solution.\n"
"# 2 INTERNAL_ERROR		An internal error has occurred.\n"
"# 3 HEIGHT_LIMIT		The height limit has been exceeded.\n"
"uint16 status\n"
"\n"
"# GPS Time of Week ms\n"
"uint32 tow\n"
"\n"
"# True heading angle (0 to 360 deg).\n"
"float32 true_heading\n"
"\n"
"# 1 sigma True heading estimated accuracy (0 to 360 deg).\n"
"float32 true_heading_acc\n"
"\n"
"# Pitch angle from the master to the rover\n"
"float32 pitch\n"
"\n"
"# 1 sigma pitch estimated accuracy\n"
"float32 pitch_acc\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::sbg_driver::SbgGpsHdt_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sbg_driver::SbgGpsHdt_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.time_stamp);
      stream.next(m.status);
      stream.next(m.tow);
      stream.next(m.true_heading);
      stream.next(m.true_heading_acc);
      stream.next(m.pitch);
      stream.next(m.pitch_acc);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SbgGpsHdt_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sbg_driver::SbgGpsHdt_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sbg_driver::SbgGpsHdt_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "time_stamp: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.time_stamp);
    s << indent << "status: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.status);
    s << indent << "tow: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.tow);
    s << indent << "true_heading: ";
    Printer<float>::stream(s, indent + "  ", v.true_heading);
    s << indent << "true_heading_acc: ";
    Printer<float>::stream(s, indent + "  ", v.true_heading_acc);
    s << indent << "pitch: ";
    Printer<float>::stream(s, indent + "  ", v.pitch);
    s << indent << "pitch_acc: ";
    Printer<float>::stream(s, indent + "  ", v.pitch_acc);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SBG_DRIVER_MESSAGE_SBGGPSHDT_H
