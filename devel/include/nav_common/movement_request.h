// Generated by gencpp from file nav_common/movement_request.msg
// DO NOT EDIT!


#ifndef NAV_COMMON_MESSAGE_MOVEMENT_REQUEST_H
#define NAV_COMMON_MESSAGE_MOVEMENT_REQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace nav_common
{
template <class ContainerAllocator>
struct movement_request_
{
  typedef movement_request_<ContainerAllocator> Type;

  movement_request_()
    : category()
    , subclass()  {
    }
  movement_request_(const ContainerAllocator& _alloc)
    : category(_alloc)
    , subclass(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _category_type;
  _category_type category;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _subclass_type;
  _subclass_type subclass;




  typedef boost::shared_ptr< ::nav_common::movement_request_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nav_common::movement_request_<ContainerAllocator> const> ConstPtr;

}; // struct movement_request_

typedef ::nav_common::movement_request_<std::allocator<void> > movement_request;

typedef boost::shared_ptr< ::nav_common::movement_request > movement_requestPtr;
typedef boost::shared_ptr< ::nav_common::movement_request const> movement_requestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nav_common::movement_request_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::nav_common::movement_request_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace nav_common

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'nav_common': ['/home/bart/workspace/AERO/src/nav_common/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::nav_common::movement_request_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nav_common::movement_request_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav_common::movement_request_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav_common::movement_request_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav_common::movement_request_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav_common::movement_request_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nav_common::movement_request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f034dc06870086475e1cf8ef445bd4da";
  }

  static const char* value(const ::nav_common::movement_request_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf034dc0687008647ULL;
  static const uint64_t static_value2 = 0x5e1cf8ef445bd4daULL;
};

template<class ContainerAllocator>
struct DataType< ::nav_common::movement_request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nav_common/movement_request";
  }

  static const char* value(const ::nav_common::movement_request_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nav_common::movement_request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string category\n\
string subclass\n\
";
  }

  static const char* value(const ::nav_common::movement_request_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nav_common::movement_request_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.category);
      stream.next(m.subclass);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct movement_request_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nav_common::movement_request_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::nav_common::movement_request_<ContainerAllocator>& v)
  {
    s << indent << "category: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.category);
    s << indent << "subclass: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.subclass);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAV_COMMON_MESSAGE_MOVEMENT_REQUEST_H
