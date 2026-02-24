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
    class GetLinkWrenches : public ModelPlugin
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
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GetLinkWrenches::OnUpdate, this));
            
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
            
            // Get the model's name
            //std::string modelName = this->model->GetName();
            
            // Print it
            //std::cout << "Loaded model: " << modelName << std::endl;
            
            this->rosnode_ = new ros::NodeHandle("getForce2Links");
            this->pub_base_wrench = this->rosnode_->advertise<geometry_msgs::WrenchStamped>("/base_wrench", 100);
            this->pub_back_left_wrench = this->rosnode_->advertise<geometry_msgs::WrenchStamped>("/back_left_wheel_wrench", 100);
            this->pub_front_left_wrench = this->rosnode_->advertise<geometry_msgs::WrenchStamped>("/front_left_wheel_wrench", 100);
            this->pub_back_right_wrench = this->rosnode_->advertise<geometry_msgs::WrenchStamped>("/back_right_wheel_wrench", 100);
            this->pub_front_right_wrench = this->rosnode_->advertise<geometry_msgs::WrenchStamped>("/front_right_wheel_wrench", 100);

            this->pub_base_wrench_rel_pose = this->rosnode_->advertise<geometry_msgs::Vector3>("/base_wrench/rel_pose", 100);
            this->pub_back_left_wrench_rel_pose = this->rosnode_->advertise<geometry_msgs::Vector3>("/back_left_wheel_wrench/rel_pose", 100);
            this->pub_front_left_wrench_rel_pose = this->rosnode_->advertise<geometry_msgs::Vector3>("/front_left_wheel_wrench/rel_pose", 100);
            this->pub_back_right_wrench_rel_pose = this->rosnode_->advertise<geometry_msgs::Vector3>("/back_right_wheel_wrench/rel_pose", 100);
            this->pub_front_right_wrench_rel_pose = this->rosnode_->advertise<geometry_msgs::Vector3>("/front_right_wheel_wrench/rel_pose", 100);
        }
        
        public: void OnUpdate()
        {   
            this->base_wrench_rel_pose = this->bl_link->RelativePose();
            this->back_left_wrench_rel_pose = this->blw_link->RelativePose();
            this->front_left_wrench_rel_pose = this->flw_link->RelativePose();
            this->back_right_wrench_rel_pose= this->brw_link->RelativePose();
            this->front_right_wrench_rel_pose = this->frw_link->RelativePose();
            
            this->b_force = this->bl_link->RelativeForce();
            this->bl_force = this->blw_link->RelativeForce();
            this->fl_force = this->flw_link->RelativeForce();
            this->br_force = this->brw_link->RelativeForce();
            this->fr_force = this->frw_link->RelativeForce();
            
            this->b_torque = this->bl_link->RelativeTorque();
            this->bl_torque = this->blw_link->RelativeTorque();
            this->fl_torque = this->flw_link->RelativeTorque();
            this->br_torque = this->brw_link->RelativeTorque();
            this->fr_torque = this->frw_link->RelativeTorque();
            
            //  Assign base wrench
            this->base_wrench.header.stamp = ros::Time::now();
            this->base_wrench.header.frame_id = "base_link";

            this->base_wrench.wrench.force.x = this->b_force.X();
            this->base_wrench.wrench.force.y = this->b_force.Y();
            this->base_wrench.wrench.force.z = this->b_force.Z();
            
            this->base_wrench.wrench.torque.x = this->b_torque.X();
            this->base_wrench.wrench.torque.y = this->b_torque.Y();
            this->base_wrench.wrench.torque.z = this->b_torque.Z();

            this->base_euler.x = this->base_wrench_rel_pose.Roll();
            this->base_euler.y = this->base_wrench_rel_pose.Pitch();
            this->base_euler.z = this->base_wrench_rel_pose.Yaw();

            this->pub_base_wrench.publish( this->base_wrench );
            this->pub_base_wrench_rel_pose.publish( this->base_euler );
            
            //  Assign back left wheel wrench
            this->back_left_wrench.header.stamp = ros::Time::now();
            this->back_left_wrench.header.frame_id = "back_left_wheel";

            this->back_left_wrench.wrench.force.x = this->bl_force.X();
            this->back_left_wrench.wrench.force.y = this->bl_force.Y();
            this->back_left_wrench.wrench.force.z = this->bl_force.Z();
            
            this->back_left_wrench.wrench.torque.x = this->bl_torque.X();
            this->back_left_wrench.wrench.torque.y = this->bl_torque.Y();
            this->back_left_wrench.wrench.torque.z = this->bl_torque.Z();

            this->back_left_euler.x = this->back_left_wrench_rel_pose.Roll();
            this->back_left_euler.y = this->back_left_wrench_rel_pose.Pitch();
            this->back_left_euler.z = this->back_left_wrench_rel_pose.Yaw();
            
            this->pub_back_left_wrench.publish( this->back_left_wrench );
            this->pub_back_left_wrench_rel_pose.publish( this->back_left_euler );

            //  Assign front left wheel wrench
            this->front_left_wrench.header.stamp = ros::Time::now();
            this->front_left_wrench.header.frame_id = "front_left_wheel";

            this->front_left_wrench.wrench.force.x = this->fl_force.X();
            this->front_left_wrench.wrench.force.y = this->fl_force.Y();
            this->front_left_wrench.wrench.force.z = this->fl_force.Z();
            
            this->front_left_wrench.wrench.torque.x = this->fl_torque.X();
            this->front_left_wrench.wrench.torque.y = this->fl_torque.Y();
            this->front_left_wrench.wrench.torque.z = this->fl_torque.Z();

            this->front_left_euler.x = this->front_left_wrench_rel_pose.Roll();
            this->front_left_euler.y = this->front_left_wrench_rel_pose.Pitch();
            this->front_left_euler.z = this->front_left_wrench_rel_pose.Yaw();
            
            this->pub_front_left_wrench.publish( this->front_left_wrench );
            this->pub_front_left_wrench_rel_pose.publish( this->front_left_euler );

            //  Assign back right wheel wrench
            this->back_right_wrench.header.stamp = ros::Time::now();
            this->back_right_wrench.header.frame_id = "back_right_wheel";

            this->back_right_wrench.wrench.force.x = this->br_force.X();
            this->back_right_wrench.wrench.force.y = this->br_force.Y();
            this->back_right_wrench.wrench.force.z = this->br_force.Z();
            
            this->back_right_wrench.wrench.torque.x = this->br_torque.X();
            this->back_right_wrench.wrench.torque.y = this->br_torque.Y();
            this->back_right_wrench.wrench.torque.z = this->br_torque.Z();

            this->back_right_euler.x = this->back_right_wrench_rel_pose.Roll();
            this->back_right_euler.y = this->back_right_wrench_rel_pose.Pitch();
            this->back_right_euler.z = this->back_right_wrench_rel_pose.Yaw();
            
            this->pub_back_right_wrench.publish( this->back_right_wrench );
            this->pub_back_right_wrench_rel_pose.publish( this->back_right_euler );

            //  Assign front right wheel wrench
            this->front_right_wrench.header.stamp = ros::Time::now();
            this->front_right_wrench.header.frame_id = "front_right_wheel";
            
            this->front_right_wrench.wrench.force.x = this->fr_force.X();
            this->front_right_wrench.wrench.force.y = this->fr_force.Y();
            this->front_right_wrench.wrench.force.z = this->fr_force.Z();
            
            this->front_right_wrench.wrench.torque.x = this->fr_torque.X();
            this->front_right_wrench.wrench.torque.y = this->fr_torque.Y();
            this->front_right_wrench.wrench.torque.z = this->fr_torque.Z();

            this->front_right_euler.x = this->front_right_wrench_rel_pose.Roll();
            this->front_right_euler.y = this->front_right_wrench_rel_pose.Pitch();
            this->front_right_euler.z = this->front_right_wrench_rel_pose.Yaw();
            
            this->pub_front_right_wrench.publish( this->front_right_wrench );
            this->pub_front_right_wrench_rel_pose.publish( this->front_right_euler );
        }

        // Pointer to the model
        private: physics::ModelPtr model;

        //  Link pointer
        private: physics::LinkPtr bl_link, blw_link, flw_link, brw_link, frw_link;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
        
        private: ros::NodeHandle* rosnode_;
        private: ros::Publisher pub_base_wrench, pub_back_left_wrench, pub_front_left_wrench, pub_back_right_wrench, pub_front_right_wrench;
        private: ros::Publisher pub_base_wrench_rel_pose, pub_back_left_wrench_rel_pose, pub_front_left_wrench_rel_pose, pub_back_right_wrench_rel_pose, pub_front_right_wrench_rel_pose;
        private: std::string topic, modelName, blName, blwName, flwName, brwName, frwName;
        private: ignition::math::Pose3d base_wrench_rel_pose, back_left_wrench_rel_pose, front_left_wrench_rel_pose, back_right_wrench_rel_pose, front_right_wrench_rel_pose;
        private: geometry_msgs::WrenchStamped base_wrench, back_left_wrench, front_left_wrench, back_right_wrench, front_right_wrench;
        private: geometry_msgs::Vector3 base_euler, back_left_euler, front_left_euler, back_right_euler, front_right_euler;
        private: ignition::math::Vector3d b_force, bl_force, fl_force, br_force, fr_force, b_torque, bl_torque, fl_torque, br_torque, fr_torque;
    };

    //  Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(GetLinkWrenches)
}