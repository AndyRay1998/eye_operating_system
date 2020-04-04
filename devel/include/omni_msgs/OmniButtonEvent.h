// Generated by gencpp from file omni_msgs/OmniButtonEvent.msg
// DO NOT EDIT!


#ifndef OMNI_MSGS_MESSAGE_OMNIBUTTONEVENT_H
#define OMNI_MSGS_MESSAGE_OMNIBUTTONEVENT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace omni_msgs
{
template <class ContainerAllocator>
struct OmniButtonEvent_
{
  typedef OmniButtonEvent_<ContainerAllocator> Type;

  OmniButtonEvent_()
    : grey_button(0)
    , white_button(0)  {
    }
  OmniButtonEvent_(const ContainerAllocator& _alloc)
    : grey_button(0)
    , white_button(0)  {
  (void)_alloc;
    }



   typedef int32_t _grey_button_type;
  _grey_button_type grey_button;

   typedef int32_t _white_button_type;
  _white_button_type white_button;





  typedef boost::shared_ptr< ::omni_msgs::OmniButtonEvent_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::omni_msgs::OmniButtonEvent_<ContainerAllocator> const> ConstPtr;

}; // struct OmniButtonEvent_

typedef ::omni_msgs::OmniButtonEvent_<std::allocator<void> > OmniButtonEvent;

typedef boost::shared_ptr< ::omni_msgs::OmniButtonEvent > OmniButtonEventPtr;
typedef boost::shared_ptr< ::omni_msgs::OmniButtonEvent const> OmniButtonEventConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::omni_msgs::OmniButtonEvent_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::omni_msgs::OmniButtonEvent_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace omni_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': True, 'HasHeader': False}
// {'omni_msgs': ['/home/andy/eye_op_robot_mixed/src/omni_packages/omni_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::omni_msgs::OmniButtonEvent_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::omni_msgs::OmniButtonEvent_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::omni_msgs::OmniButtonEvent_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::omni_msgs::OmniButtonEvent_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::omni_msgs::OmniButtonEvent_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::omni_msgs::OmniButtonEvent_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::omni_msgs::OmniButtonEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fb77877e6b639935c3360838062f05f0";
  }

  static const char* value(const ::omni_msgs::OmniButtonEvent_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfb77877e6b639935ULL;
  static const uint64_t static_value2 = 0xc3360838062f05f0ULL;
};

template<class ContainerAllocator>
struct DataType< ::omni_msgs::OmniButtonEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "omni_msgs/OmniButtonEvent";
  }

  static const char* value(const ::omni_msgs::OmniButtonEvent_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::omni_msgs::OmniButtonEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 grey_button\n\
int32 white_button\n\
";
  }

  static const char* value(const ::omni_msgs::OmniButtonEvent_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::omni_msgs::OmniButtonEvent_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.grey_button);
      stream.next(m.white_button);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct OmniButtonEvent_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::omni_msgs::OmniButtonEvent_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::omni_msgs::OmniButtonEvent_<ContainerAllocator>& v)
  {
    s << indent << "grey_button: ";
    Printer<int32_t>::stream(s, indent + "  ", v.grey_button);
    s << indent << "white_button: ";
    Printer<int32_t>::stream(s, indent + "  ", v.white_button);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OMNI_MSGS_MESSAGE_OMNIBUTTONEVENT_H