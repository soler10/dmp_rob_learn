// Generated by gencpp from file dmp/DMPTraj.msg
// DO NOT EDIT!


#ifndef DMP_MESSAGE_DMPTRAJ_H
#define DMP_MESSAGE_DMPTRAJ_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <dmp/DMPPoint.h>

namespace dmp
{
template <class ContainerAllocator>
struct DMPTraj_
{
  typedef DMPTraj_<ContainerAllocator> Type;

  DMPTraj_()
    : points()
    , times()  {
    }
  DMPTraj_(const ContainerAllocator& _alloc)
    : points(_alloc)
    , times(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::dmp::DMPPoint_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::dmp::DMPPoint_<ContainerAllocator> >> _points_type;
  _points_type points;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _times_type;
  _times_type times;





  typedef boost::shared_ptr< ::dmp::DMPTraj_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dmp::DMPTraj_<ContainerAllocator> const> ConstPtr;

}; // struct DMPTraj_

typedef ::dmp::DMPTraj_<std::allocator<void> > DMPTraj;

typedef boost::shared_ptr< ::dmp::DMPTraj > DMPTrajPtr;
typedef boost::shared_ptr< ::dmp::DMPTraj const> DMPTrajConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dmp::DMPTraj_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dmp::DMPTraj_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dmp::DMPTraj_<ContainerAllocator1> & lhs, const ::dmp::DMPTraj_<ContainerAllocator2> & rhs)
{
  return lhs.points == rhs.points &&
    lhs.times == rhs.times;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dmp::DMPTraj_<ContainerAllocator1> & lhs, const ::dmp::DMPTraj_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dmp

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dmp::DMPTraj_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dmp::DMPTraj_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dmp::DMPTraj_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dmp::DMPTraj_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dmp::DMPTraj_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dmp::DMPTraj_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dmp::DMPTraj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1d088d86ab60cf6a2671bc3c0e99932b";
  }

  static const char* value(const ::dmp::DMPTraj_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1d088d86ab60cf6aULL;
  static const uint64_t static_value2 = 0x2671bc3c0e99932bULL;
};

template<class ContainerAllocator>
struct DataType< ::dmp::DMPTraj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dmp/DMPTraj";
  }

  static const char* value(const ::dmp::DMPTraj_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dmp::DMPTraj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# points and times should be the same length\n"
"DMPPoint[] points\n"
"\n"
"# Times of observations, in seconds, starting at zero\n"
"float64[] times\n"
"\n"
"\n"
"\n"
"================================================================================\n"
"MSG: dmp/DMPPoint\n"
"# Positions and velocities of DOFs\n"
"#Velocity is only used for movement plans, not for giving demonstrations.\n"
"float64[] positions\n"
"float64[] velocities\n"
"\n"
"\n"
;
  }

  static const char* value(const ::dmp::DMPTraj_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dmp::DMPTraj_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.points);
      stream.next(m.times);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DMPTraj_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dmp::DMPTraj_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dmp::DMPTraj_<ContainerAllocator>& v)
  {
    s << indent << "points[]" << std::endl;
    for (size_t i = 0; i < v.points.size(); ++i)
    {
      s << indent << "  points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::dmp::DMPPoint_<ContainerAllocator> >::stream(s, indent + "    ", v.points[i]);
    }
    s << indent << "times[]" << std::endl;
    for (size_t i = 0; i < v.times.size(); ++i)
    {
      s << indent << "  times[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.times[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DMP_MESSAGE_DMPTRAJ_H
