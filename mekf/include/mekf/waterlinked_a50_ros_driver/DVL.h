// Generated by gencpp from file waterlinked_a50_ros_driver/DVL.msg
// DO NOT EDIT!


#ifndef WATERLINKED_A50_ROS_DRIVER_MESSAGE_DVL_H
#define WATERLINKED_A50_ROS_DRIVER_MESSAGE_DVL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include "DVLBeam.h" // defined locally

namespace waterlinked_a50_ros_driver
{
template <class ContainerAllocator>
struct DVL_
{
  typedef DVL_<ContainerAllocator> Type;

  DVL_()
    : header()
    , time(0.0)
    , velocity()
    , fom(0.0)
    , altitude(0.0)
    , beams()
    , velocity_valid(false)
    , status(0)
    , form()  {
    }
  DVL_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , time(0.0)
    , velocity(_alloc)
    , fom(0.0)
    , altitude(0.0)
    , beams(_alloc)
    , velocity_valid(false)
    , status(0)
    , form(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _time_type;
  _time_type time;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _velocity_type;
  _velocity_type velocity;

   typedef double _fom_type;
  _fom_type fom;

   typedef double _altitude_type;
  _altitude_type altitude;

   typedef std::vector< ::waterlinked_a50_ros_driver::DVLBeam_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::waterlinked_a50_ros_driver::DVLBeam_<ContainerAllocator> >::other >  _beams_type;
  _beams_type beams;

   typedef uint8_t _velocity_valid_type;
  _velocity_valid_type velocity_valid;

   typedef int64_t _status_type;
  _status_type status;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _form_type;
  _form_type form;





  typedef boost::shared_ptr< ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator> const> ConstPtr;

}; // struct DVL_

typedef ::waterlinked_a50_ros_driver::DVL_<std::allocator<void> > DVL;

typedef boost::shared_ptr< ::waterlinked_a50_ros_driver::DVL > DVLPtr;
typedef boost::shared_ptr< ::waterlinked_a50_ros_driver::DVL const> DVLConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator1> & lhs, const ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.time == rhs.time &&
    lhs.velocity == rhs.velocity &&
    lhs.fom == rhs.fom &&
    lhs.altitude == rhs.altitude &&
    lhs.beams == rhs.beams &&
    lhs.velocity_valid == rhs.velocity_valid &&
    lhs.status == rhs.status &&
    lhs.form == rhs.form;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator1> & lhs, const ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace waterlinked_a50_ros_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dea1debc1aca7804a62c507714ec3777";
  }

  static const char* value(const ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdea1debc1aca7804ULL;
  static const uint64_t static_value2 = 0xa62c507714ec3777ULL;
};

template<class ContainerAllocator>
struct DataType< ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator> >
{
  static const char* value()
  {
    return "waterlinked_a50_ros_driver/DVL";
  }

  static const char* value(const ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"#Time\n"
"float64 time\n"
"#Measured velocity [m/s]\n"
"geometry_msgs/Vector3 velocity\n"
"#Figure of Merit\n"
"float64 fom\n"
"#Altitude of the sensor\n"
"float64 altitude\n"
"#Beams/Transducers\n"
"DVLBeam[] beams\n"
"#Validity of velocity\n"
"bool velocity_valid\n"
"#Status message\n"
"int64 status\n"
"#Formatting of json\n"
"string form\n"
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
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: waterlinked_a50_ros_driver/DVLBeam\n"
"#Transducer ID\n"
"int64 id\n"
"#Velocity reported by transducer\n"
"float64 velocity\n"
"#Distance value\n"
"float64 distance\n"
"#RSSI\n"
"float64 rssi\n"
"#NSD\n"
"float64 nsd\n"
"#Report if beam is locked on and providing reliable data\n"
"bool valid\n"
;
  }

  static const char* value(const ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.time);
      stream.next(m.velocity);
      stream.next(m.fom);
      stream.next(m.altitude);
      stream.next(m.beams);
      stream.next(m.velocity_valid);
      stream.next(m.status);
      stream.next(m.form);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DVL_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::waterlinked_a50_ros_driver::DVL_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
    s << indent << "velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
    s << indent << "fom: ";
    Printer<double>::stream(s, indent + "  ", v.fom);
    s << indent << "altitude: ";
    Printer<double>::stream(s, indent + "  ", v.altitude);
    s << indent << "beams[]" << std::endl;
    for (size_t i = 0; i < v.beams.size(); ++i)
    {
      s << indent << "  beams[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::waterlinked_a50_ros_driver::DVLBeam_<ContainerAllocator> >::stream(s, indent + "    ", v.beams[i]);
    }
    s << indent << "velocity_valid: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.velocity_valid);
    s << indent << "status: ";
    Printer<int64_t>::stream(s, indent + "  ", v.status);
    s << indent << "form: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.form);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WATERLINKED_A50_ROS_DRIVER_MESSAGE_DVL_H
