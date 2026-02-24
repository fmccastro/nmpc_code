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
            
            this->blName = "base_link";
            if (!_sdf->HasElement("baseLink")) {
                ROS_WARN_NAMED("Model Plugin (ns = %s) missing <baseLink>, defaults to %s", this->model->GetName().c_str(), this->blName.c_str());

            } else {
                this->blName = _sdf->GetElement("baseLink")->Get<std::string>();
                std::cout << "Link name is " << this->blName << std::endl;

            }

            this->blwName = "back_left_wheel";
            if (!_sdf->HasElement("backLeftWheelLink")) {
                ROS_WARN_NAMED("Model Plugin (ns = %s) missing <backLeftWheelLink>, defaults to %s", this->model->GetName().c_str(), this->blwName.c_str());

            } else {
                this->blwName = _sdf->GetElement("backLeftWheelLink")->Get<std::string>();
                std::cout << "Link name is " << this->blwName << std::endl;

            }

            this->flwName = "front_left_wheel";
            if (!_sdf->HasElement("frontLeftWheelLink")) {
                ROS_WARN_NAMED("Model Plugin (ns = %s) missing <frontLeftWheelLink>, defaults to %s", this->model->GetName().c_str(), this->flwName.c_str());

            } else {
                this->flwName = _sdf->GetElement("frontLeftWheelLink")->Get<std::string>();
                std::cout << "Link name is " << this->flwName << std::endl;

            }

            this->brwName = "back_right_wheel";
            if (!_sdf->HasElement("backRightWheelLink")) {
                ROS_WARN_NAMED("Model Plugin (ns = %s) missing <backRightWheelLink>, defaults to %s", this->model->GetName().c_str(), this->brwName.c_str());

            } else {
                this->brwName = _sdf->GetElement("backRightWheelLink")->Get<std::string>();
                std::cout << "Link name is " << this->brwName << std::endl;

            }

            this->frwName = "front_right_wheel";
            if (!_sdf->HasElement("frontRightWheelLink")) {
                ROS_WARN_NAMED("Model Plugin (ns = %s) missing <frontRightWheelLink>, defaults to %s", this->model->GetName().c_str(), this->frwName.c_str());

            } else {
                this->frwName = _sdf->GetElement("frontRightWheelLink")->Get<std::string>();
                std::cout << "Link name is " << this->frwName << std::endl;

            }
            
            if(!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, this->model->GetName(), ros::init_options::NoSigintHandler);
            }
            
            this->bl_link = this->model->GetLink(this->blName);
            this->blw_link = this->model->GetLink(this->blwName);
            this->flw_link = this->model->GetLink(this->flwName);
            this->brw_link = this->model->GetLink(this->brwName);
            this->frw_link = this->model->GetLink(this->frwName);

            this->offset.X(-4.37153e-6);
            this->offset.Y(0.0);
            this->offset.Z(0.1635730);

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
            this->bl_link->AddLinkForce(this->force, this->offset);
            this->bl_link->AddRelativeTorque(this->torque);
        }

        // Pointer to the model
        private: physics::ModelPtr model;

        //  Link pointer
        private: physics::LinkPtr bl_link, blw_link, flw_link, brw_link, frw_link;
        private: std::string blName, blwName, flwName, brwName, frwName;

        private: ignition::math::Vector3d force, torque, offset;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
        
        private: ros::NodeHandle* rosnode_;
    };

    //  Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(SetLinkWrenches)
}