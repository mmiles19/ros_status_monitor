#include <ros_type_introspection/ros_introspection.hpp>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Float32.h>
#include <boost/shared_ptr.hpp>

using namespace RosIntrospection;
using topic_tools::ShapeShifter;

XmlRpc::XmlRpcValue monitor_list;
std::map<std::string,std::pair<ros::Time,double>> monitor_log; // topic_name, last_time, rate at last_time
std::map<std::string,boost::shared_ptr<ros::Publisher>> publishers;

void topicCallback(const ShapeShifter::ConstPtr& msg,
                   const std::string &topic_name,
                   RosIntrospection::Parser& parser)
{
  ros::Time curr_time = ros::Time::now();
  std::map<std::string,std::pair<ros::Time,double>>::iterator monitor_log_it = monitor_log.find(topic_name);
  if (monitor_log_it!=monitor_log.end()){
    std::pair<ros::Time,double>* monitor_log_pair = &(monitor_log_it->second);
    ros::Time last_time = monitor_log_pair->first;
    double diff_time = curr_time.toSec()-last_time.toSec();
    double last_rate = 1.0/diff_time;
    monitor_log_pair->first = curr_time; // update msg last_time
    monitor_log_pair->second = last_rate; // update msg rate at last_time
    return;
  }
  else {
    monitor_log[topic_name] = std::pair<ros::Time,double>(curr_time,0.0); //  add first instance of msg with rate 0
    return;
  }
}

void checkLoopFunc(const ros::TimerEvent& event){
  ros::Time curr_time = ros::Time::now();
  for (uint i=0; i<monitor_list.size(); i++) {
    XmlRpc::XmlRpcValue monitor_it = monitor_list[i];
    ROS_ASSERT(monitor_it.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    ROS_ASSERT(monitor_it["topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string topic_name = monitor_it["topic"];

    std::map<std::string,std::pair<ros::Time,double>>::iterator monitor_log_it = monitor_log.find(topic_name);
    if (monitor_log_it==monitor_log.end()){
      // have not received any msgs from this topic
      ROS_WARN("Have not received any msgs yet on %s.", topic_name.c_str());
      continue;
    }
    else{
      std::pair<ros::Time,double>* monitor_log_pair = &(monitor_log_it->second);
      if (monitor_log_pair->second==0.0){
        // have received this msg just once so far
        // ROS_INFO("Have received a single msg on %s.", topic_name.c_str());
        continue;
      }
      else{
        // have received at least two msgs on this topic
        ros::Time last_time = monitor_log_pair->first;
        double last_rate = monitor_log_pair->second;
        // ROS_INFO("Have received msgs on %s. Last time %f. Last rate %f.", topic_name.c_str(), last_time.toSec(), last_rate);

        // ROS_INFO("pubbing freq");
        std_msgs::Float32 freq_msg;
        freq_msg.data = last_rate;
        publishers[topic_name]->publish(freq_msg);

        ROS_ASSERT(monitor_it["expected_hz"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double expected_hz = monitor_it["expected_hz"];
        ROS_ASSERT(monitor_it["tolerance_hz"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double tolerance_hz = monitor_it["tolerance_hz"];

        if (fabs(expected_hz-last_rate)>tolerance_hz){
          ROS_WARN("Topic %s behaving unexpectedly. Expected rate of %f but had %f at %f.", topic_name.c_str(), expected_hz, last_rate, last_time.toSec());
          continue;
        }
        if (fabs(curr_time.toSec()-last_time.toSec())>2.0/tolerance_hz){
          ROS_WARN("Topic %s behaving unexpectedly. Have not received since %f.", topic_name.c_str(), last_time.toSec());
          continue;
        }
      }
    }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "status_monitor");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("Started.");

  double rate;
  nh.param("rate", rate, (double)1.0);
  // std::string param_file;
  // nh.param("param_file", param_file, "");
  // nh.getParam(param_file);
  nh.getParam("monitors", monitor_list);
  ROS_ASSERT(monitor_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

  ROS_INFO("Parameters loaded.");
  ROS_INFO("%d monitors found.", monitor_list.size());

  std::vector<ros::Subscriber> subscribers;

  for (uint i=0; i<monitor_list.size(); i++)
  {
    XmlRpc::XmlRpcValue monitor_it = monitor_list[i];
    ROS_ASSERT(monitor_it.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(monitor_it["topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string topic_name = monitor_it["topic"];

    Parser parser;
    boost::function<void(const ShapeShifter::ConstPtr&)> callback = [&parser, topic_name](const ShapeShifter::ConstPtr& msg){topicCallback(msg, topic_name, parser);};
    ros::Subscriber subscriber = nh.subscribe(topic_name, 10, callback);
    subscribers.push_back(subscriber);

    boost::shared_ptr<ros::Publisher> publisher (new ros::Publisher(nh.advertise<std_msgs::Float32>(topic_name+"/frequency", 10)));
    publishers[topic_name] = publisher;

    ROS_INFO("Added monitor to %s.", topic_name.c_str());
  }

  ros::Timer checkLoopTimer = nh.createTimer(ros::Duration(1/rate), checkLoopFunc);

  ros::spin();

  return 0;
};
