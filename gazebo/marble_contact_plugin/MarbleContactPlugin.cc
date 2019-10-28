#include "MarbleContactPlugin.hh"

#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/World.hh>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(MarbleContactPlugin)

/////////////////////////////////////////////////
MarbleContactPlugin::MarbleContactPlugin() : SensorPlugin() {}

/////////////////////////////////////////////////
MarbleContactPlugin::~MarbleContactPlugin() {}

/////////////////////////////////////////////////
void MarbleContactPlugin::Load(sensors::SensorPtr _sensor,
                               sdf::ElementPtr /*_sdf*/) {
  // Get the parent sensor.
  this->parentSensor =
      std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor) {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&MarbleContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void MarbleContactPlugin::Init() {}

/////////////////////////////////////////////////
void MarbleContactPlugin::OnUpdate() {

  //  // Get all the contacts.
  msgs::Contacts contacts = this->parentSensor->Contacts();
  if (contacts.contact_size() > 0) {

    std::string linkName = this->parentSensor->ParentName();
    std::string marbleName = linkName.substr(0, linkName.find(":"));

    std::cout << "Removing " << marbleName << std::endl;

    physics::WorldPtr world =
        physics::get_world(this->parentSensor->WorldName());

    world->RemoveModel(marbleName);
  }
}
