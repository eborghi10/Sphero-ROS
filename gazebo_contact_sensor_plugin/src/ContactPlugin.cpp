#include "ContactPlugin.h"
#include <sphero_node/SpheroCollision.h>
#include <gazebo/common/Plugin.hh>


namespace gazebo
{
    GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)
    
    /////////////////////////////////////////////////
    ContactPlugin::ContactPlugin() : 
    SensorPlugin(),
    _nh("contact_sensor_plugin")
    {
        _contactPub = _nh.advertise<sphero_node::SpheroCollision>("/collision",1);
    }
    
    /////////////////////////////////////////////////
    ContactPlugin::~ContactPlugin()
    {
        ROS_DEBUG_STREAM_NAMED("contact_sensor","Unloaded");
    }
    
    /////////////////////////////////////////////////
    void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
    {
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
          ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load contact plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
          return;
        }
        
      // Get the parent sensor.
        this->_parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);
    
      // Make sure the parent sensor is valid.
      if (!this->_parentSensor)
      {
        gzerr << "ContactPlugin requires a ContactSensor.\n";
        return;
      }
    
      // Connect to the sensor update event.
      this->_updateConnection = this->_parentSensor->ConnectUpdated( boost::bind(&ContactPlugin::OnUpdate, this) );
    
      // Make sure the parent sensor is active.
      this->_parentSensor->SetActive(true);
    }
    
    /////////////////////////////////////////////////
    void ContactPlugin::OnUpdate()
    {
      // Get all the contacts.
      msgs::Contacts contacts;
//      contacts = this->_parentSensor->GetContacts();
      contacts = this->_parentSensor->Contacts();
      for (unsigned int i = 0; i < contacts.contact_size(); ++i)
      {
        std::cout << "Collision between[" << contacts.contact(i).collision1()
                  << "] and [" << contacts.contact(i).collision2() << "]\n";
    
        for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
        {
          std::cout << j << "  Position:"
                    << contacts.contact(i).position(j).x() << " "
                    << contacts.contact(i).position(j).y() << " "
                    << contacts.contact(i).position(j).z() << "\n";
          std::cout << "   Normal:"
                    << contacts.contact(i).normal(j).x() << " "
                    << contacts.contact(i).normal(j).y() << " "
                    << contacts.contact(i).normal(j).z() << "\n";
          std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
        }
      }
      
      // Publish a contact message if robot collided
      if (contacts.contact_size() > 0)
      {
        static int seq=0;
        
        sphero_node::SpheroCollision msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "base_link";
        msg.header.seq = seq;
        msg.x = contacts.contact(0).position(0).x();
        msg.y = contacts.contact(0).position(0).y();
        msg.z = contacts.contact(0).position(0).z();
        
        _contactPub.publish(msg);
        
        seq++;
      }
    }
}