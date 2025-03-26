// Generated by gencpp from file dy_avoidobs/Multilocaltrajs.msg
// DO NOT EDIT!


#ifndef DY_AVOIDOBS_MESSAGE_MULTILOCALTRAJS_H
#define DY_AVOIDOBS_MESSAGE_MULTILOCALTRAJS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <dy_avoidobs/localtraj.h>

namespace dy_avoidobs
{
template <class ContainerAllocator>
struct Multilocaltrajs_
{
  typedef Multilocaltrajs_<ContainerAllocator> Type;

  Multilocaltrajs_()
    : car_id_from(0)
    , traj()  {
    }
  Multilocaltrajs_(const ContainerAllocator& _alloc)
    : car_id_from(0)
    , traj(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _car_id_from_type;
  _car_id_from_type car_id_from;

   typedef std::vector< ::dy_avoidobs::localtraj_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::dy_avoidobs::localtraj_<ContainerAllocator> >> _traj_type;
  _traj_type traj;





  typedef boost::shared_ptr< ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator> const> ConstPtr;

}; // struct Multilocaltrajs_

typedef ::dy_avoidobs::Multilocaltrajs_<std::allocator<void> > Multilocaltrajs;

typedef boost::shared_ptr< ::dy_avoidobs::Multilocaltrajs > MultilocaltrajsPtr;
typedef boost::shared_ptr< ::dy_avoidobs::Multilocaltrajs const> MultilocaltrajsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator1> & lhs, const ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator2> & rhs)
{
  return lhs.car_id_from == rhs.car_id_from &&
    lhs.traj == rhs.traj;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator1> & lhs, const ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dy_avoidobs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3c3a990cdcf595dfcffe37275da7c4d3";
  }

  static const char* value(const ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3c3a990cdcf595dfULL;
  static const uint64_t static_value2 = 0xcffe37275da7c4d3ULL;
};

template<class ContainerAllocator>
struct DataType< ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dy_avoidobs/Multilocaltrajs";
  }

  static const char* value(const ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 car_id_from\n"
"\n"
"localtraj[] traj\n"
"\n"
"\n"
"================================================================================\n"
"MSG: dy_avoidobs/localtraj\n"
"int32 car_id\n"
"\n"
"int64 traj_id\n"
"time start_time\n"
"\n"
"\n"
"geometry_msgs/Point[] pos_pts\n"
"\n"
"\n"
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

  static const char* value(const ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.car_id_from);
      stream.next(m.traj);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Multilocaltrajs_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dy_avoidobs::Multilocaltrajs_<ContainerAllocator>& v)
  {
    s << indent << "car_id_from: ";
    Printer<int32_t>::stream(s, indent + "  ", v.car_id_from);
    s << indent << "traj[]" << std::endl;
    for (size_t i = 0; i < v.traj.size(); ++i)
    {
      s << indent << "  traj[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::dy_avoidobs::localtraj_<ContainerAllocator> >::stream(s, indent + "    ", v.traj[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DY_AVOIDOBS_MESSAGE_MULTILOCALTRAJS_H
