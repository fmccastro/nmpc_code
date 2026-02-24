#include <functional>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

namespace gazebo {
    class ContactSteering : public ModelPlugin {
        public:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
                model_ = _parent;
                world_ = model_->GetWorld();

                if (!_sdf->HasElement("back_left_contact_joint")) {
                    ROS_ERROR(
                        "No back_left_contact_joint element present. DifferentialPlugin could not be loaded.");
                    return;
                }
                back_left_contact_joint_name_ = _sdf->GetElement("back_left_contact_joint")->Get<std::string>();

                //

                if (!_sdf->HasElement("front_left_contact_joint")) {
                    ROS_ERROR(
                        "No front_left_contact_joint element present. DifferentialPlugin could not be loaded.");
                    return;
                }
                front_left_contact_joint_name_ = _sdf->GetElement("front_left_contact_joint")->Get<std::string>();

                //

                if (!_sdf->HasElement("back_right_contact_joint")) {
                    ROS_ERROR(
                        "No front_left_contact_joint element present. DifferentialPlugin could not be loaded.");
                    return;
                }
                back_right_contact_joint_name_ = _sdf->GetElement("back_right_contact_joint")->Get<std::string>();

                //

                if (!_sdf->HasElement("front_right_contact_joint")) {
                    ROS_ERROR(
                        "No front_right_contact_joint element present. DifferentialPlugin could not be loaded.");
                    return;
                }
                front_right_contact_joint_name_ = _sdf->GetElement("front_right_contact_joint")->Get<std::string>();

                //

                if (!_sdf->HasElement("forceConstant")) {
                    ROS_ERROR("No forceConstant element present. DifferentialPlugin could "
                                "not be loaded.");
                    return;
                }
                force_constant_ = _sdf->GetElement("forceConstant")->Get<double>();

                back_left_contact_joint = model_->GetJoint(back_left_contact_joint_name_);
                if (!back_left_contact_joint) {
                    ROS_ERROR_STREAM("No joint named \""
                                    << back_left_contact_joint_name_
                                    << "\". DifferentialPlugin could not be loaded.");
                    return;
                }

                //

                front_left_contact_joint = model_->GetJoint(front_left_contact_joint_name_);
                if (!front_left_contact_joint) {
                    ROS_ERROR_STREAM("No joint named \""
                                    << front_left_contact_joint_name_
                                    << "\". DifferentialPlugin could not be loaded.");
                    return;
                }

                //

                back_right_contact_joint = model_->GetJoint(back_right_contact_joint_name_);
                if (!back_right_contact_joint) {
                    ROS_ERROR_STREAM("No joint named \""
                                    << back_right_contact_joint_name_
                                    << "\". DifferentialPlugin could not be loaded.");
                    return;
                }

                //

                front_right_contact_joint = model_->GetJoint(front_right_contact_joint_name_);
                if (!front_right_contact_joint) {
                    ROS_ERROR_STREAM("No joint named \""
                                    << front_right_contact_joint_name_
                                    << "\". DifferentialPlugin could not be loaded.");
                    return;
                }

                //

                this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&ContactSteering::OnUpdate, this));

                ROS_INFO_STREAM("DifferentialPlugin loaded! Back left contact joint: \""
                                << back_left_contact_joint_name_ << "\", Front left contact joint: \"" << front_left_contact_joint_name_ << "\", Back right contact joint: \"" << back_right_contact_joint_name_
                                << "\", Front right contact joint: \"" << front_right_contact_joint_name_
                                << "\", Force Constant: " << force_constant_);
            }

            void OnUpdate() {
                double bl_c_p = back_left_contact_joint->Position();
                double fl_c_p = front_left_contact_joint->Position();
                double br_c_p = back_right_contact_joint->Position();
                double fr_c_p = front_right_contact_joint->Position();

                back_left_contact_joint->SetForce(0, -bl_c_p * force_constant_);
                back_right_contact_joint->SetForce(0, -br_c_p * force_constant_);
                front_left_contact_joint->SetForce(0, -fl_c_p * force_constant_);
                front_right_contact_joint->SetForce(0, -fr_c_p * force_constant_);
            }

            private:
                std::string back_left_contact_joint_name_, front_left_contact_joint_name_, back_right_contact_joint_name_, front_right_contact_joint_name_;
                double force_constant_;

                physics::JointPtr back_left_contact_joint, front_left_contact_joint, back_right_contact_joint, front_right_contact_joint;
                physics::ModelPtr model_;
                physics::WorldPtr world_;

                event::ConnectionPtr updateConnection;
    };

    GZ_REGISTER_MODEL_PLUGIN(ContactSteering)
}