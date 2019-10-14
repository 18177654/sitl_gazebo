#include "gazebo_forces_moments_aerodynamics_plugin.h"

namespace gazebo {
GazeboForcesMomentsAeroPlugin::~GazeboForcesMomentsAeroPlugin() {
    update_connection_->~Connection();
}

void GazeboForcesMomentsAeroPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    // Store the pointer to the model and world.
    model_ = _model;
    world_ = model_->GetWorld();

    // Default values.
    cdx_ = 0;
    cdy_ = 0;
    cdz_ = 0;
    rho_ = 1.225;

    // Get parameters.
    getSdfParam<std::string>(_sdf, "robotNamespace", namespace_, namespace_, true);
    getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_, true);

    getSdfParam<double>(_sdf, "cdx", cdx_, cdx_, true);
    getSdfParam<double>(_sdf, "cdy", cdy_, cdy_, true);
    getSdfParam<double>(_sdf, "cdz", cdz_, cdz_, true);

    getSdfParam<double>(_sdf, "rho", rho_, rho_, true);

    // Initialize the node_handle.
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    // Pointer to the link, to be able to apply forces and torques on it.
    link_ = model_->GetLink(link_name_);
    if (link_ == NULL)
        gzthrow("[gazebo_forces_moments_aerodynamics_plugin] Couldn't find specified link \"" << link_name_ << "\".");

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboForcesMomentsAeroPlugin::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboForcesMomentsAeroPlugin::OnUpdate(const common::UpdateInfo& _info) {
    // get linear velocity at cp in inertial frame
#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d velI = model_->WorldLinearVel();
#else
    ignition::math::Vector3d velI = ignitionFromGazeboMath(model_->GetWorldLinearVel());
#endif

    // pose of body
#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d pose = model_->WorldPose();
#else
    ignition::math::Pose3d pose = ignitionFromGazeboMath(model_->GetWorldPose());
#endif

    // rotate inertial velocity to body
    ignition::math::Vector3d velB = pose.Inverse().Rot().RotateVector(velI);

    // calculate aerodynamic force
    double dragX = 0.5 * rho_ * (-velB.X() * abs(velB.X())) * cdx_;
    double dragY = 0.5 * rho_ * (-velB.Y() * abs(velB.Y())) * cdy_;
    double dragZ = 0.5 * rho_ * (-velB.Z() * abs(velB.Z())) * cdz_;

    ignition::math::Vector3d force = ignition::math::Vector3d(dragX, dragY, dragZ);

    // apply force
    link_->AddRelativeForce(force);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboForcesMomentsAeroPlugin)
} // namespace gazebo
