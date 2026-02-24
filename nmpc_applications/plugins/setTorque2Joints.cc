#include <string>
#include <stdio.h>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{
    class SetLinkWrenches : public ModelPlugin
    {
        /////////////////////////////////////////////////
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Get the parent sensor.
            this->model = _parent;
            
            // Make sure the parent sensor is valid.
            if (!this->model)
            {
                gzerr << "ModelPlugin requires a model.\n";
                return;
            }
            
            // Connect to the sensor update event.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&SetLinkWrenches::OnUpdate, this));
            
            if(!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, this->model->GetName(), ros::init_options::NoSigintHandler);
            }
            
            /*this->bl_link = this->model->GetLink(this->blName);
            this->blw_link = this->model->GetLink(this->blwName);
            this->flw_link = this->model->GetLink(this->flwName);
            this->brw_link = this->model->GetLink(this->brwName);
            this->frw_link = this->model->GetLink(this->frwName);*/

            this->blw_joint = this->model->GetJoint("back_left_wheel_joint");
            this->flw_joint = this->model->GetJoint("front_left_wheel_joint");
            this->brw_joint = this->model->GetJoint("back_right_wheel_joint");
            this->frw_joint = this->model->GetJoint("front_right_wheel_joint");

            //std::cout << this->blw_joint->GetName().c_str() << std::endl;

            //  Forces and moments for 3-D motion
            this->force.X(1.0);
            this->force.Y(0.02);
            this->force.Z(0.04);
            
            this->torque.X(-0.03);
            this->torque.Y(0.02);
            this->torque.Z(0.2);

            //  Forces and moments for planar motion
            /*this->force.X(2.0);
            this->force.Y(0.0);
            this->force.Z(0.0);
            
            this->torque.X(0.0);
            this->torque.Y(0.0);
            this->torque.Z(0.2);*/
            
            // Get the model's name
            //std::string modelName = this->model->GetName();
            
            // Print it
            //std::cout << "Loaded model: " << modelName << std::endl;
        }
        
        public: void OnUpdate()
        {
            std::cout << "Found joint: " << this->blw_joint->GetName() << std::endl;
        }

        // Pointer to the model
        private: physics::ModelPtr model;

        //  Joint pointer
        private: physics::JointPtr blw_joint, flw_joint, brw_joint, frw_joint;

        //  Link pointer
        private: physics::LinkPtr bl_link, blw_link, flw_link, brw_link, frw_link;

        private: ignition::math::Vector3d force, torque, offset;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
        
        private: ros::NodeHandle* rosnode_;
    };

    //  Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(SetLinkWrenches)
}