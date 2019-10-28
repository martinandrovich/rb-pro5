#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo {

class MarbleContactPlugin : public SensorPlugin {
  /// \brief Constructor.
public:
  MarbleContactPlugin();

  virtual ~MarbleContactPlugin();

  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

  virtual void Init();

  /// \brief Callback that receives the contact sensor's update signal.
private:
  virtual void OnUpdate();

  sensors::ContactSensorPtr parentSensor;
  event::ConnectionPtr updateConnection;
};

} // namespace gazebo
#endif
