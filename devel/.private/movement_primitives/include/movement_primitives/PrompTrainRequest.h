// Generated by gencpp from file movement_primitives/PrompTrainRequest.msg
// DO NOT EDIT!


#ifndef MOVEMENT_PRIMITIVES_MESSAGE_PROMPTRAINREQUEST_H
#define MOVEMENT_PRIMITIVES_MESSAGE_PROMPTRAINREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace movement_primitives
{
template <class ContainerAllocator>
struct PrompTrainRequest_
{
  typedef PrompTrainRequest_<ContainerAllocator> Type;

  PrompTrainRequest_()
    : promp_name()  {
    }
  PrompTrainRequest_(const ContainerAllocator& _alloc)
    : promp_name(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _promp_name_type;
  _promp_name_type promp_name;





  typedef boost::shared_ptr< ::movement_primitives::PrompTrainRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::movement_primitives::PrompTrainRequest_<ContainerAllocator> const> ConstPtr;

}; // struct PrompTrainRequest_

typedef ::movement_primitives::PrompTrainRequest_<std::allocator<void> > PrompTrainRequest;

typedef boost::shared_ptr< ::movement_primitives::PrompTrainRequest > PrompTrainRequestPtr;
typedef boost::shared_ptr< ::movement_primitives::PrompTrainRequest const> PrompTrainRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::movement_primitives::PrompTrainRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::movement_primitives::PrompTrainRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::movement_primitives::PrompTrainRequest_<ContainerAllocator1> & lhs, const ::movement_primitives::PrompTrainRequest_<ContainerAllocator2> & rhs)
{
  return lhs.promp_name == rhs.promp_name;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::movement_primitives::PrompTrainRequest_<ContainerAllocator1> & lhs, const ::movement_primitives::PrompTrainRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace movement_primitives

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::movement_primitives::PrompTrainRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::movement_primitives::PrompTrainRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::movement_primitives::PrompTrainRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::movement_primitives::PrompTrainRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::movement_primitives::PrompTrainRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::movement_primitives::PrompTrainRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::movement_primitives::PrompTrainRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "401e24503097a56d5cd6705a1036c46d";
  }

  static const char* value(const ::movement_primitives::PrompTrainRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x401e24503097a56dULL;
  static const uint64_t static_value2 = 0x5cd6705a1036c46dULL;
};

template<class ContainerAllocator>
struct DataType< ::movement_primitives::PrompTrainRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "movement_primitives/PrompTrainRequest";
  }

  static const char* value(const ::movement_primitives::PrompTrainRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::movement_primitives::PrompTrainRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string promp_name\n"
"#string path_to_training_data\n"
;
  }

  static const char* value(const ::movement_primitives::PrompTrainRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::movement_primitives::PrompTrainRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.promp_name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PrompTrainRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::movement_primitives::PrompTrainRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::movement_primitives::PrompTrainRequest_<ContainerAllocator>& v)
  {
    s << indent << "promp_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.promp_name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOVEMENT_PRIMITIVES_MESSAGE_PROMPTRAINREQUEST_H
