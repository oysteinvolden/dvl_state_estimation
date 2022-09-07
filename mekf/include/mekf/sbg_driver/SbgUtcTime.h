// Generated by gencpp from file sbg_driver/SbgUtcTime.msg
// DO NOT EDIT!


#ifndef SBG_DRIVER_MESSAGE_SBGUTCTIME_H
#define SBG_DRIVER_MESSAGE_SBGUTCTIME_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <sbg_driver/SbgUtcTimeStatus.h>

namespace sbg_driver
{
template <class ContainerAllocator>
struct SbgUtcTime_
{
  typedef SbgUtcTime_<ContainerAllocator> Type;

  SbgUtcTime_()
    : header()
    , time_stamp(0)
    , clock_status()
    , year(0)
    , month(0)
    , day(0)
    , hour(0)
    , min(0)
    , sec(0)
    , nanosec(0)
    , gps_tow(0)  {
    }
  SbgUtcTime_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , time_stamp(0)
    , clock_status(_alloc)
    , year(0)
    , month(0)
    , day(0)
    , hour(0)
    , min(0)
    , sec(0)
    , nanosec(0)
    , gps_tow(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint32_t _time_stamp_type;
  _time_stamp_type time_stamp;

   typedef  ::sbg_driver::SbgUtcTimeStatus_<ContainerAllocator>  _clock_status_type;
  _clock_status_type clock_status;

   typedef uint16_t _year_type;
  _year_type year;

   typedef uint8_t _month_type;
  _month_type month;

   typedef uint8_t _day_type;
  _day_type day;

   typedef uint8_t _hour_type;
  _hour_type hour;

   typedef uint8_t _min_type;
  _min_type min;

   typedef uint8_t _sec_type;
  _sec_type sec;

   typedef uint32_t _nanosec_type;
  _nanosec_type nanosec;

   typedef uint32_t _gps_tow_type;
  _gps_tow_type gps_tow;





  typedef boost::shared_ptr< ::sbg_driver::SbgUtcTime_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sbg_driver::SbgUtcTime_<ContainerAllocator> const> ConstPtr;

}; // struct SbgUtcTime_

typedef ::sbg_driver::SbgUtcTime_<std::allocator<void> > SbgUtcTime;

typedef boost::shared_ptr< ::sbg_driver::SbgUtcTime > SbgUtcTimePtr;
typedef boost::shared_ptr< ::sbg_driver::SbgUtcTime const> SbgUtcTimeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sbg_driver::SbgUtcTime_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sbg_driver::SbgUtcTime_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::sbg_driver::SbgUtcTime_<ContainerAllocator1> & lhs, const ::sbg_driver::SbgUtcTime_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.time_stamp == rhs.time_stamp &&
    lhs.clock_status == rhs.clock_status &&
    lhs.year == rhs.year &&
    lhs.month == rhs.month &&
    lhs.day == rhs.day &&
    lhs.hour == rhs.hour &&
    lhs.min == rhs.min &&
    lhs.sec == rhs.sec &&
    lhs.nanosec == rhs.nanosec &&
    lhs.gps_tow == rhs.gps_tow;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::sbg_driver::SbgUtcTime_<ContainerAllocator1> & lhs, const ::sbg_driver::SbgUtcTime_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace sbg_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::sbg_driver::SbgUtcTime_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sbg_driver::SbgUtcTime_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sbg_driver::SbgUtcTime_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sbg_driver::SbgUtcTime_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sbg_driver::SbgUtcTime_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sbg_driver::SbgUtcTime_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sbg_driver::SbgUtcTime_<ContainerAllocator> >
{
  static const char* value()
  {
    return "89495f07708fa38e487b6509c4edabaa";
  }

  static const char* value(const ::sbg_driver::SbgUtcTime_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x89495f07708fa38eULL;
  static const uint64_t static_value2 = 0x487b6509c4edabaaULL;
};

template<class ContainerAllocator>
struct DataType< ::sbg_driver::SbgUtcTime_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sbg_driver/SbgUtcTime";
  }

  static const char* value(const ::sbg_driver::SbgUtcTime_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sbg_driver::SbgUtcTime_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# SBG Ellipse Messages\n"
"Header header\n"
"\n"
"# Time since sensor is powered up (us)\n"
"uint32 time_stamp\n"
"\n"
"# General UTC time and clock sync status\n"
"SbgUtcTimeStatus clock_status\n"
"\n"
"# Year\n"
"uint16 year\n"
"\n"
"# Month in Year [1 ... 12]\n"
"uint8 month\n"
"\n"
"# Day in Month [1 ... 31]\n"
"uint8 day\n"
"\n"
"# Hour in day [0 ... 23]\n"
"uint8 hour\n"
"\n"
"# Minute in hour [0 ... 59]\n"
"uint8 min\n"
"\n"
"# Second in minute [0 ... 60], Note 60 is when a leap second is added.\n"
"uint8 sec\n"
"\n"
"# Nanosecond of second.\n"
"uint32 nanosec\n"
"\n"
"# GPS Time of week (ms)\n"
"uint32 gps_tow\n"
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
"\n"
"================================================================================\n"
"MSG: sbg_driver/SbgUtcTimeStatus\n"
"# SBG Ellipse Messages\n"
"\n"
"# True when a clock input can be used to synchronize the internal clock.\n"
"bool clock_stable\n"
"\n"
"# Define the internal clock estimation status\n"
"# 0 An error has occurred on the clock estimation.\n"
"# 1 The clock is only based on the internal crystal.\n"
"# 2 A PPS has been detected and the clock is converging to it.\n"
"# 3 The clock has converged to the PPS and is within 500ns.\n"
"uint8 clock_status\n"
"\n"
"# True if UTC time is synchronized with a PPS\n"
"bool clock_utc_sync\n"
"\n"
"# UTC validity status\n"
"# 0 The UTC time is not known, we are just propagating the UTC time internally.\n"
"# 1 We have received valid UTC time information but we don't have the leap seconds information.\n"
"# 2 We have received valid UTC time data with valid leap seconds.\n"
"uint8 clock_utc_status\n"
;
  }

  static const char* value(const ::sbg_driver::SbgUtcTime_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sbg_driver::SbgUtcTime_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.time_stamp);
      stream.next(m.clock_status);
      stream.next(m.year);
      stream.next(m.month);
      stream.next(m.day);
      stream.next(m.hour);
      stream.next(m.min);
      stream.next(m.sec);
      stream.next(m.nanosec);
      stream.next(m.gps_tow);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SbgUtcTime_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sbg_driver::SbgUtcTime_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sbg_driver::SbgUtcTime_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "time_stamp: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.time_stamp);
    s << indent << "clock_status: ";
    s << std::endl;
    Printer< ::sbg_driver::SbgUtcTimeStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.clock_status);
    s << indent << "year: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.year);
    s << indent << "month: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.month);
    s << indent << "day: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.day);
    s << indent << "hour: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.hour);
    s << indent << "min: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.min);
    s << indent << "sec: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.sec);
    s << indent << "nanosec: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.nanosec);
    s << indent << "gps_tow: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.gps_tow);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SBG_DRIVER_MESSAGE_SBGUTCTIME_H
