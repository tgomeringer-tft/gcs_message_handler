#include "ros/ros.h"
#include "mavros_msgs/Mavlink.h"
#include <mavconn/interface.h>
#include <mavros_msgs/mavlink_convert.h>
#include <algorithm>

#include <iostream>
#include <boost/thread/thread.hpp>
#include <mutex>

#include <iostream>
#include <fstream>
using namespace std;

ofstream logfile;

std::mutex gcs_send_mutex;
std::mutex log_mutex;
std::mutex print_mutex;

mavconn::MAVConnInterface::Ptr ip_;
mavconn::MAVConnInterface::Ptr gcs_serial_;
mavconn::MAVConnInterface::Ptr fcu_serial_;

mavlink::common::msg::DEBUG db{};

using mavlink::mavlink_message_t;

bool sendAll = false;
bool use_gcs_serial = false;
bool use_fcu_serial = false;
bool use_ip = false;

bool enable_debug_message=false;
bool log_uplink = false;
bool log_downlink = false;
bool print_uplink_ids = false;
bool print_downlink_ids = false;

std::string gcs_serial_url="";
std::string fcu_serial_url="";
std::string gcs_url="";

int toFCU[] = {0, 20, 21, 39, 40, 43, 44, 45, 47, 76, 233};
int toGCS[] = {0, 1, 24, 30, 32, 39, 40, 42, 43, 44, 47, 74, 77, 127, 141, 147, 253, 254};

void uplink_Publisher(int* publish_rate)
{

  ros::Rate loop_rate(*publish_rate);

  while (ros::ok())
  {

    if(use_ip) {


        ip_->message_received_cb = [&](const mavlink::mavlink_message_t * msg, const mavconn::Framing framing) {

        ROS_INFO("Received msg %4u from QGC", msg->msgid);

        if ( (std::find(std::begin(toFCU), std::end(toFCU), msg->msgid) != std::end(toFCU) ) || sendAll)
        {

          mavros_msgs::Mavlink uplink_msg;

                   
          if(print_uplink_ids) {
            print_mutex.lock();
            //ROS_INFO("Send msg %4u to RFGate Home GCS_1", msg->msgid);
            ROS_INFO("Send msg %4u to FTDI 1", msg->msgid);


            print_mutex.unlock();
          } 
          
          if(log_uplink) {         
           log_mutex.lock();       
           logfile << "up: " << msg->msgid << "\n";
           log_mutex.unlock();      
          }

          if(use_fcu_serial) {
             //fcu_send_mutex.lock(); 
             //mavlink::mavlink_message_t mvmsg;
             //mavros_msgs::mavlink::convert(*msg, mvmsg);
             fcu_serial_->send_message_ignore_drop(msg);
             //fcu_send_mutex.unlock();
          }

        }
        else
        {

             ROS_INFO("Rejecting %4u to FTDI 1", msg->msgid);

        }

      };
    
    }

    if(use_gcs_serial) {

      gcs_serial_->message_received_cb = [&](const mavlink::mavlink_message_t * msg, const mavconn::Framing framing) {

        if ( (std::find(std::begin(toFCU), std::end(toFCU), msg->msgid) != std::end(toFCU) ) || sendAll)
        {

          //mavros_msgs::Mavlink uplink_msg;

          //ROS_INFO("Forwarding message %4u to FCU", msg->msgid);
          //mavros_msgs::mavlink::convert(*msg, uplink_msg);
          //pub.publish(uplink_msg);
          //loop_rate.sleep();

        }

      };

    } // if(use_gcs_serial)



  }; //while (ros::ok())

} // uplink_Publisher

void downlink_Publisher(int* publish_rate){

  ros::Rate loop_rate(*publish_rate);

  while (ros::ok())
  {

    if(use_fcu_serial) {

      fcu_serial_->message_received_cb = [&](const mavlink::mavlink_message_t * msg, const mavconn::Framing framing) {

        if ( (std::find(std::begin(toGCS), std::end(toGCS), msg->msgid) != std::end(toGCS)) || sendAll)
        {

          if(print_downlink_ids) {
            print_mutex.lock();
            //ROS_INFO("Rcvd msg %4u from RFGate Home GCS_1", msg->msgid);
            ROS_INFO("Rcvd msg %4u from FTDI 1", msg->msgid);
            print_mutex.unlock();
          } 

          ip_->send_message_ignore_drop(msg);

        }

      };

    } // if(use_gcs_serial)

  }

}

int main(int argc, char **argv){

  logfile.open("GCS_messages.txt");

  int rate = 100; // 100 Hz

  //std::string url="udp://:14999@10.0.1.22:15000";
  std::string url="udp://:14999@127.0.0.1:15000";

  ros::init(argc, argv, "gcs_message_handler");

  ros::NodeHandle n;

  n.getParam("/gcs_message_handler/gcs_serial_url",gcs_serial_url);
  n.getParam("/gcs_message_handler/fcu_serial_url",fcu_serial_url);
  n.getParam("/gcs_message_handler/gcs_url",gcs_url);
  //n.getParam("/gcs_message_handler/gcs_url",fcu_url);
  n.getParam("/gcs_message_handler/use_ip",use_ip);
  n.getParam("/gcs_message_handler/use_gcs_serial",use_gcs_serial);
  n.getParam("/gcs_message_handler/use_fcu_serial",use_fcu_serial);
  n.getParam("/gcs_message_handler/sendAll",sendAll);
  n.getParam("/gcs_message_handler/enable_debug_message",enable_debug_message);

  n.getParam("/gcs_message_handler/print_downlink_ids",print_downlink_ids);
  n.getParam("/gcs_message_handler/log_downlink",log_downlink);
  n.getParam("/gcs_message_handler/print_uplink_ids",print_uplink_ids);
  n.getParam("/gcs_message_handler/log_uplink",log_uplink);

  ROS_INFO("use ip: %d",use_ip); 
  //if(use_ip) {
    ROS_INFO("gcs_message_handler connecting to %s", gcs_url.c_str()); 
    ip_ = mavconn::MAVConnInterface::open_url(gcs_url);
  //}

  if(use_gcs_serial) {
    ROS_INFO("gcs_message_handler connecting to %s", gcs_serial_url.c_str()); 
    gcs_serial_ = mavconn::MAVConnInterface::open_url(gcs_serial_url);
  }

  if(use_fcu_serial) {
    ROS_INFO("fcu_message_handler connecting to %s", fcu_serial_url.c_str()); 
    fcu_serial_ = mavconn::MAVConnInterface::open_url(fcu_serial_url);
  }

  // spawn thread
  ROS_INFO("Spawning uplink thread"); 
  boost::thread thread_b(uplink_Publisher, &rate);
  ROS_INFO("Spawning downlink thread"); 
  boost::thread thread_c(downlink_Publisher, &rate);

  ros::spin();
  logfile.close();
  return 0;

}
