#ifndef _GAZEBO_FORCES_MOMENTS_AERODYNAMICS_PLUGIN_H_
#define _GAZEBO_FORCES_MOMENTS_AERODYNAMICS_PLUGIN_H_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include <ignition/math.hh>
#include "common.h"

namespace gazebo {
  class GazeboForcesMomentsAeroPlugin : public ModelPlugin
  {
    public:
        GazeboForcesMomentsAeroPlugin() :  ModelPlugin() {};
        virtual ~GazeboForcesMomentsAeroPlugin();

    protected:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void OnUpdate(const common::UpdateInfo& /*_info*/);

    private:
        event::ConnectionPtr update_connection_;
        physics::WorldPtr world_;
        physics::ModelPtr model_;
        physics::LinkPtr link_;
        std::string namespace_;
        std::string link_name_;
        transport::NodePtr node_handle_;

        double cdx_;
        double cdy_;
        double cdz_;

        double rho_;
  };
} // namespace gazebo
#endif // _GAZEBO_FORCES_MOMENTS_AERODYNAMICS_PLUGIN_H_
