#include "RovoTeleport.hh"

#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>

#include <cmath>
#include <sstream>

namespace rovo_gz_teleport
{

RovoTeleport::RovoTeleport() = default;
RovoTeleport::~RovoTeleport() = default;

void RovoTeleport::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Rovo Teleport";

  if (_pluginElem)
  {
    if (auto e = _pluginElem->FirstChildElement("model"))
      if (e->GetText())
        this->modelName = e->GetText();
    if (auto e = _pluginElem->FirstChildElement("world"))
      if (e->GetText())
        this->worldName = e->GetText();
    if (auto e = _pluginElem->FirstChildElement("drop"))
      e->QueryDoubleText(&this->dropHeight);
    if (auto e = _pluginElem->FirstChildElement("spawn_x"))
      e->QueryDoubleText(&this->spawnX);
    if (auto e = _pluginElem->FirstChildElement("spawn_y"))
      e->QueryDoubleText(&this->spawnY);
    if (auto e = _pluginElem->FirstChildElement("spawn_z"))
      e->QueryDoubleText(&this->spawnZ);
    if (auto e = _pluginElem->FirstChildElement("spawn_yaw"))
      e->QueryDoubleText(&this->spawnYaw);
    if (auto e = _pluginElem->FirstChildElement("follow_on_start"))
      e->QueryBoolText(&this->followOnStart);
    if (auto e = _pluginElem->FirstChildElement("follow_offset_x"))
      e->QueryDoubleText(&this->followX);
    if (auto e = _pluginElem->FirstChildElement("follow_offset_y"))
      e->QueryDoubleText(&this->followY);
    if (auto e = _pluginElem->FirstChildElement("follow_offset_z"))
      e->QueryDoubleText(&this->followZ);
  }

  // worldName is filled by the first Update() from the ECM, so the SDF and
  // this panel cannot drift apart.
  gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
  this->SetStatus("waiting for the world");
}

QString RovoTeleport::ModelName() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return QString::fromStdString(this->modelName);
}

void RovoTeleport::SetModelName(const QString &_name)
{
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->modelName = _name.toStdString();
  }
  this->ModelNameChanged();
}

QString RovoTeleport::StatusText() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return QString::fromStdString(this->status);
}

bool RovoTeleport::Picking() const
{
  return this->picking;
}

void RovoTeleport::SetPicking(bool _picking)
{
  this->picking = _picking;
  this->SetStatus(_picking ? "click a point in the scene" : "ready");
  this->PickingChanged();
}

void RovoTeleport::SetStatus(const std::string &_text)
{
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->status = _text;
  }
  this->StatusTextChanged();
}

void RovoTeleport::Update(const gz::sim::UpdateInfo &,
                          gz::sim::EntityComponentManager &_ecm)
{
  if (this->worldName.empty())
  {
    _ecm.Each<gz::sim::components::World,
              gz::sim::components::Name>(
        [&](const gz::sim::Entity &, const gz::sim::components::World *,
            const gz::sim::components::Name *_name) -> bool
        {
          this->worldName = _name->Data();
          return false;
        });
    if (!this->worldName.empty())
      this->SetStatus("ready");
  }

  std::string want;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    want = this->modelName;
  }
  this->haveCurrent = false;
  _ecm.Each<gz::sim::components::Model,
            gz::sim::components::Name>(
      [&](const gz::sim::Entity &_entity, const gz::sim::components::Model *,
          const gz::sim::components::Name *_name) -> bool
      {
        if (_name->Data() != want)
          return true;
        const auto pose = gz::sim::worldPose(_entity, _ecm);
        this->curX = pose.Pos().X();
        this->curY = pose.Pos().Y();
        this->curZ = pose.Pos().Z();
        this->curYaw = pose.Rot().Yaw();
        this->haveCurrent = true;
        return false;
      });

  // Follow can only be applied once the GUI knows the entity; asking earlier
  // returns success and quietly does nothing, which is why this waits for the
  // model to show up in the ECM instead of firing at load time.
  if (this->followOnStart && !this->followApplied && this->haveCurrent)
  {
    this->followApplied = true;
    this->SetFollowing(true);
  }
}

void RovoTeleport::Teleport(double _x, double _y, double _z, double _yawDeg,
                            bool _applyYaw)
{
  if (this->worldName.empty())
  {
    this->SetStatus("no world name; cannot call set_pose");
    return;
  }

  // z below the sentinel means "keep whatever the model has", used when a
  // scene pick has no surface height to offer.
  if (_z < -1e5)
    _z = this->haveCurrent ? this->curZ : 0.0;

  double yaw = _applyYaw ? _yawDeg * M_PI / 180.0
                         : (this->haveCurrent ? this->curYaw : 0.0);

  gz::msgs::Pose req;
  req.set_name(this->modelName);
  req.mutable_position()->set_x(_x);
  req.mutable_position()->set_y(_y);
  req.mutable_position()->set_z(_z);
  req.mutable_orientation()->set_x(0.0);
  req.mutable_orientation()->set_y(0.0);
  req.mutable_orientation()->set_z(std::sin(yaw / 2.0));
  req.mutable_orientation()->set_w(std::cos(yaw / 2.0));

  gz::msgs::Boolean rep;
  bool ok{false};
  const std::string service = "/world/" + this->worldName + "/set_pose";
  if (this->node.Request(service, req, 5000, rep, ok) && ok && rep.data())
  {
    std::ostringstream ss;
    ss.precision(2);
    ss << std::fixed << "moved to (" << _x << ", " << _y << ", " << _z
       << ") yaw " << yaw * 180.0 / M_PI << " deg";
    this->SetStatus(ss.str());
  }
  else
  {
    this->SetStatus("set_pose failed on " + service);
  }
}

bool RovoTeleport::Following() const
{
  return this->following;
}

void RovoTeleport::SetFollowing(bool _following)
{
  std::string target;
  if (_following)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    target = this->modelName;
  }

  // An empty target is how gz is told to stop following.
  gz::msgs::StringMsg req;
  req.set_data(target);
  gz::msgs::Boolean rep;
  bool ok{false};
  if (!this->node.Request("/gui/follow", req, 3000, rep, ok) || !ok)
  {
    this->SetStatus("camera follow request failed");
    return;
  }

  if (_following)
  {
    gz::msgs::Vector3d off;
    off.set_x(this->followX);
    off.set_y(this->followY);
    off.set_z(this->followZ);
    gz::msgs::Boolean offRep;
    bool offOk{false};
    this->node.Request("/gui/follow/offset", off, 3000, offRep, offOk);
  }

  this->following = _following;
  this->SetStatus(_following ? "camera locked on the model"
                             : "camera free");
  this->FollowingChanged();
}

void RovoTeleport::SetHeading(double _yawDeg)
{
  if (!this->haveCurrent)
  {
    this->SetStatus("model pose not known yet");
    return;
  }
  this->Teleport(this->curX, this->curY, -1e6, _yawDeg, true);
}

void RovoTeleport::ResetPose()
{
  this->Teleport(this->spawnX, this->spawnY, this->spawnZ,
                 this->spawnYaw * 180.0 / M_PI, true);
}

bool RovoTeleport::eventFilter(QObject *_obj, QEvent *_event)
{
  if (this->picking &&
      _event->type() == gz::gui::events::LeftClickToScene::kType)
  {
    auto ev = reinterpret_cast<gz::gui::events::LeftClickToScene *>(_event);
    if (ev)
    {
      const auto p = ev->Point();
      // The click point sits ON the surface under the cursor, so the drop is
      // the only clearance needed -- no heightfield lookup, and it works on
      // the lower level of a two-level street because that is what was hit.
      this->Teleport(p.X(), p.Y(), p.Z() + this->dropHeight, 0.0, false);
      this->SetPicking(false);
    }
  }
  return QObject::eventFilter(_obj, _event);
}

}  // namespace rovo_gz_teleport

GZ_ADD_PLUGIN(rovo_gz_teleport::RovoTeleport, gz::gui::Plugin)
