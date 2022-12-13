// Generated by gencpp from file dmp/LearnDMPFromDemoResponse.msg
// DO NOT EDIT!


#ifndef DMP_MESSAGE_LEARNDMPFROMDEMORESPONSE_H
#define DMP_MESSAGE_LEARNDMPFROMDEMORESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <dmp/DMPData.h>

namespace dmp
{
template <class ContainerAllocator>
struct LearnDMPFromDemoResponse_
{
  typedef LearnDMPFromDemoResponse_<ContainerAllocator> Type;

  LearnDMPFromDemoResponse_()
    : dmp_list()
    , tau(0.0)  {
    }
  LearnDMPFromDemoResponse_(const ContainerAllocator& _alloc)
    : dmp_list(_alloc)
    , tau(0.0)  {
  (void)_alloc;
    }



   typedef std::vector< ::dmp::DMPData_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::dmp::DMPData_<ContainerAllocator> >> _dmp_list_type;
  _dmp_list_type dmp_list;

   typedef double _tau_type;
  _tau_type tau;





  typedef boost::shared_ptr< ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator> const> ConstPtr;

}; // struct LearnDMPFromDemoResponse_

typedef ::dmp::LearnDMPFromDemoResponse_<std::allocator<void> > LearnDMPFromDemoResponse;

typedef boost::shared_ptr< ::dmp::LearnDMPFromDemoResponse > LearnDMPFromDemoResponsePtr;
typedef boost::shared_ptr< ::dmp::LearnDMPFromDemoResponse const> LearnDMPFromDemoResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator1> & lhs, const ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator2> & rhs)
{
  return lhs.dmp_list == rhs.dmp_list &&
    lhs.tau == rhs.tau;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator1> & lhs, const ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dmp

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d2dccae00aae58574694dfa33e62fac1";
  }

  static const char* value(const ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd2dccae00aae5857ULL;
  static const uint64_t static_value2 = 0x4694dfa33e62fac1ULL;
};

template<class ContainerAllocator>
struct DataType< ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dmp/LearnDMPFromDemoResponse";
  }

  static const char* value(const ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"# Returns a DMP for each DOF, intended to be linked together with a single phase variable\n"
"DMPData[] dmp_list\n"
"\n"
"# A time constant (in seconds) that will cause the DMPs to replay at the same speed they were demonstrated. \n"
"float64 tau\n"
"\n"
"\n"
"================================================================================\n"
"MSG: dmp/DMPData\n"
"float64 k_gain\n"
"float64 d_gain\n"
"float64[] weights\n"
"float64[] f_domain\n"
"float64[] f_targets\n"
"\n"
;
  }

  static const char* value(const ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.dmp_list);
      stream.next(m.tau);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LearnDMPFromDemoResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dmp::LearnDMPFromDemoResponse_<ContainerAllocator>& v)
  {
    s << indent << "dmp_list[]" << std::endl;
    for (size_t i = 0; i < v.dmp_list.size(); ++i)
    {
      s << indent << "  dmp_list[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::dmp::DMPData_<ContainerAllocator> >::stream(s, indent + "    ", v.dmp_list[i]);
    }
    s << indent << "tau: ";
    Printer<double>::stream(s, indent + "  ", v.tau);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DMP_MESSAGE_LEARNDMPFROMDEMORESPONSE_H