// Generated by gencpp from file traj_utils/MultiBsplines.msg
// DO NOT EDIT!


#ifndef TRAJ_UTILS_MESSAGE_MULTIBSPLINES_H
#define TRAJ_UTILS_MESSAGE_MULTIBSPLINES_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <traj_utils/Bspline.h>

namespace traj_utils
{
template <class ContainerAllocator>
struct MultiBsplines_
{
  typedef MultiBsplines_<ContainerAllocator> Type;

  MultiBsplines_()
    : drone_id_from(0)
    , traj()  {
    }
  MultiBsplines_(const ContainerAllocator& _alloc)
    : drone_id_from(0)
    , traj(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _drone_id_from_type;
  _drone_id_from_type drone_id_from;

   typedef std::vector< ::traj_utils::Bspline_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::traj_utils::Bspline_<ContainerAllocator> >> _traj_type;
  _traj_type traj;





  typedef boost::shared_ptr< ::traj_utils::MultiBsplines_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::traj_utils::MultiBsplines_<ContainerAllocator> const> ConstPtr;

}; // struct MultiBsplines_

typedef ::traj_utils::MultiBsplines_<std::allocator<void> > MultiBsplines;

typedef boost::shared_ptr< ::traj_utils::MultiBsplines > MultiBsplinesPtr;
typedef boost::shared_ptr< ::traj_utils::MultiBsplines const> MultiBsplinesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::traj_utils::MultiBsplines_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::traj_utils::MultiBsplines_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::traj_utils::MultiBsplines_<ContainerAllocator1> & lhs, const ::traj_utils::MultiBsplines_<ContainerAllocator2> & rhs)
{
  return lhs.drone_id_from == rhs.drone_id_from &&
    lhs.traj == rhs.traj;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::traj_utils::MultiBsplines_<ContainerAllocator1> & lhs, const ::traj_utils::MultiBsplines_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace traj_utils

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::traj_utils::MultiBsplines_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::traj_utils::MultiBsplines_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::traj_utils::MultiBsplines_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::traj_utils::MultiBsplines_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traj_utils::MultiBsplines_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traj_utils::MultiBsplines_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::traj_utils::MultiBsplines_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7f45adafc838893dace82d2af415aae3";
  }

  static const char* value(const ::traj_utils::MultiBsplines_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7f45adafc838893dULL;
  static const uint64_t static_value2 = 0xace82d2af415aae3ULL;
};

template<class ContainerAllocator>
struct DataType< ::traj_utils::MultiBsplines_<ContainerAllocator> >
{
  static const char* value()
  {
    return "traj_utils/MultiBsplines";
  }

  static const char* value(const ::traj_utils::MultiBsplines_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::traj_utils::MultiBsplines_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 drone_id_from\n"
"\n"
"Bspline[] traj\n"
"\n"
"\n"
"================================================================================\n"
"MSG: traj_utils/Bspline\n"
"int32 drone_id\n"
"\n"
"int32 order\n"
"int64 traj_id\n"
"time start_time\n"
"\n"
"float64[] knots\n"
"geometry_msgs/Point[] pos_pts\n"
"\n"
"float64[] yaw_pts\n"
"float64 yaw_dt\n"
"\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::traj_utils::MultiBsplines_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::traj_utils::MultiBsplines_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.drone_id_from);
      stream.next(m.traj);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MultiBsplines_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::traj_utils::MultiBsplines_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::traj_utils::MultiBsplines_<ContainerAllocator>& v)
  {
    s << indent << "drone_id_from: ";
    Printer<int32_t>::stream(s, indent + "  ", v.drone_id_from);
    s << indent << "traj[]" << std::endl;
    for (size_t i = 0; i < v.traj.size(); ++i)
    {
      s << indent << "  traj[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::traj_utils::Bspline_<ContainerAllocator> >::stream(s, indent + "    ", v.traj[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // TRAJ_UTILS_MESSAGE_MULTIBSPLINES_H
