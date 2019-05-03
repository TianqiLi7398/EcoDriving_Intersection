#ifndef _VEHICLE_PLUGIN_HH_
#define _VEHICLE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>


namespace gazebo
{
  /// \brief A plugin to control a Vehicle sensor.
  class VehiclePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VehiclePlugin() {}

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    private: physics::LinkPtr link;

    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      this->SetVelocity(_msg->x());
    }

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
    if (_model->GetJointCount() == 0)
    {
     std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
     return;
    }
    this->model = _model;
    math::Vector3 worldForce;
    worldForce.x = 10000;
    worldForce.y = 0;
    worldForce.z = 0;
    //
    //
    // Link =  _model->GetLink("chassis");
    // Link->AddForce(worldForce);

    this->link = _model->GetLinks()[0];
    double velocity = 10;
    this->SetVelocity(velocity);

    // this->link->AddForce(worldForce );
    // this->model->GetLink("chassis")->SetLinearVel({10,0,0});
    // this->model->SetLinearAccel({10,0,0});
    // this->link->SetLinearVel({10,0,0});

    // this->joint = _model->GetJoints()[0];
    // // Setup a P-controller, with a gain of 0.1.
    // this->pid = common::PID(0.1, 0, 0);
    //
    // // Apply the P-controller to the joint.
    // this->model->GetJointController()->SetVelocityPID(
    //   this->joint->GetScopedName(), this->pid);
    //
    //   // Set the joint's target velocity. This target velocity is just
    //   // for demonstration purposes.
    //   this->model->GetJointController()->SetVelocityTarget(
    //     this->joint->GetScopedName(), 10.0);
    // // Default to zero velocity
    // double velocity = 0;
    //
    // // Check that the velocity element exists, then read the value
    // if (_sdf->HasElement("velocity"))
    //   velocity = _sdf->Get<double>("velocity");
    //
    // // Set the joint's target velocity. This target velocity is just
    // // for demonstration purposes.
    // this->model->GetJointController()->SetVelocityTarget(
    //     this->joint->GetScopedName(), velocity);
    // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(this->model->GetWorld()->GetName());
      #else
      this->node->Init(this->model->GetWorld()->Name());
      #endif

      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName,
         &VehiclePlugin::OnMsg, this);

    }

    public: void SetVelocity(const double &_vel)
    {
      // Set the joint's target velocity.
      // this->model->GetJointController()->SetVelocityTarget(
      //     this->joint->GetScopedName(), _vel);
      this->model->SetLinearVel({_vel,0,0});
    }

    
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VehiclePlugin)
}
#endif
