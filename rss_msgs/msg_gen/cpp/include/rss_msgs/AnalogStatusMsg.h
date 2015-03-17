/* Auto-generated by genmsg_cpp for file /home/rss-student/RSS-I-group/rss_msgs/msg/AnalogStatusMsg.msg */
#ifndef RSS_MSGS_MESSAGE_ANALOGSTATUSMSG_H
#define RSS_MSGS_MESSAGE_ANALOGSTATUSMSG_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace rss_msgs
{
template <class ContainerAllocator>
struct AnalogStatusMsg_ {
  typedef AnalogStatusMsg_<ContainerAllocator> Type;

  AnalogStatusMsg_()
  : values()
  {
    values.assign(0.0);
  }

  AnalogStatusMsg_(const ContainerAllocator& _alloc)
  : values()
  {
    values.assign(0.0);
  }

  typedef boost::array<double, 8>  _values_type;
  boost::array<double, 8>  values;


  ROS_DEPRECATED uint32_t get_values_size() const { return (uint32_t)values.size(); }
private:
  static const char* __s_getDataType_() { return "rss_msgs/AnalogStatusMsg"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "7e6caa6f77d06950d365d446a6483a22"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float64[8] values\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, values);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, values);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(values);
    return size;
  }

  typedef boost::shared_ptr< ::rss_msgs::AnalogStatusMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rss_msgs::AnalogStatusMsg_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct AnalogStatusMsg
typedef  ::rss_msgs::AnalogStatusMsg_<std::allocator<void> > AnalogStatusMsg;

typedef boost::shared_ptr< ::rss_msgs::AnalogStatusMsg> AnalogStatusMsgPtr;
typedef boost::shared_ptr< ::rss_msgs::AnalogStatusMsg const> AnalogStatusMsgConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::rss_msgs::AnalogStatusMsg_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::rss_msgs::AnalogStatusMsg_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace rss_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rss_msgs::AnalogStatusMsg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rss_msgs::AnalogStatusMsg_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rss_msgs::AnalogStatusMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "7e6caa6f77d06950d365d446a6483a22";
  }

  static const char* value(const  ::rss_msgs::AnalogStatusMsg_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x7e6caa6f77d06950ULL;
  static const uint64_t static_value2 = 0xd365d446a6483a22ULL;
};

template<class ContainerAllocator>
struct DataType< ::rss_msgs::AnalogStatusMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rss_msgs/AnalogStatusMsg";
  }

  static const char* value(const  ::rss_msgs::AnalogStatusMsg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rss_msgs::AnalogStatusMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64[8] values\n\
";
  }

  static const char* value(const  ::rss_msgs::AnalogStatusMsg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::rss_msgs::AnalogStatusMsg_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rss_msgs::AnalogStatusMsg_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.values);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct AnalogStatusMsg_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rss_msgs::AnalogStatusMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::rss_msgs::AnalogStatusMsg_<ContainerAllocator> & v) 
  {
    s << indent << "values[]" << std::endl;
    for (size_t i = 0; i < v.values.size(); ++i)
    {
      s << indent << "  values[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.values[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // RSS_MSGS_MESSAGE_ANALOGSTATUSMSG_H
