#include <string>
#include <stdio.h>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Vector3.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{
  class ContactPlugin : public SensorPlugin
  {
    /////////////////////////////////////////////////
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
      // Get the parent sensor.
      this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

      // Make sure the parent sensor is valid.
      if (!this->parentSensor)
      {
        gzerr << "ContactPlugin requires a ContactSensor.\n";
        return;
      }

      // Make sure the parent sensor is active.
      this->parentSensor->SetActive(true);

      // Connect to the sensor update event.
      this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&ContactPlugin::OnUpdate, this));

      this->topic = "/custom";
      if (!_sdf->HasElement("topic")) {
        ROS_WARN_NAMED(this->parentSensor->Name(), "contacts Plugin (ns = %s) missing <topic>, defaults to %s",
        this->parentSensor->Name().c_str(), this->topic.c_str());

      } else {
        this->topic = _sdf->GetElement("topic")->Get<std::string>();

      }

      if(!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, this->parentSensor->Name(), ros::init_options::NoSigintHandler);
      }

      this->rosnode_ = new ros::NodeHandle(this->parentSensor->Name());
      this->pub_ = this->rosnode_->advertise<gazebo_msgs::ContactsState>(this->topic.c_str(), 100);
    }

    public: void OnUpdate()
    {
      // Get all the contacts.
      msgs::Contacts contacts;
      contacts = this->parentSensor->Contacts();

      //  Get number of contacts
      int nbContacts = contacts.contact_size();

      //contactStamped2Pub.header.stamp = contacts.contact(0).time();
      contactStamped2Pub.states.clear();
    
      contact2Pub.contact_normals.clear();
      contact2Pub.contact_positions.clear();

      if( nbContacts > 0 )
      {
        int size = contacts.contact(0).position_size();

        for (unsigned int j = 0; j < size; ++j)
        {
          //  Get contact position
          this->position.x = contacts.contact(0).position(j).x();
          this->position.y = contacts.contact(0).position(j).y();
          this->position.z = contacts.contact(0).position(j).z();

          //  Get contact normal
          this->normal.x = contacts.contact(0).normal(j).x();
          this->normal.y = contacts.contact(0).normal(j).y();
          this->normal.z = contacts.contact(0).normal(j).z();

          contact2Pub.info = "ON";
          contact2Pub.contact_positions.push_back(this->position);
          contact2Pub.contact_normals.push_back(this->normal);
        }

        contactStamped2Pub.states.push_back(contact2Pub);
      }
      else {
        contact2Pub.info = "OFF";
        contactStamped2Pub.states.push_back(contact2Pub);
      }

      //  Publish latest contacts
      this->pub_.publish(contactStamped2Pub);
    }

    // Pointer to the model
    private: sensors::ContactSensorPtr parentSensor;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: ros::CallbackQueue queue_;
    private: boost::thread callback_queue_thread_;
    private: std::string topic;
    private: gazebo_msgs::ContactsState contactStamped2Pub;
    private: gazebo_msgs::ContactState contact2Pub;
    private: geometry_msgs::Vector3 normal, position;
    private: int size;
  };

  GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)
}