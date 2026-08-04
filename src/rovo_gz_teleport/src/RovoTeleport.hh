#ifndef ROVO_GZ_TELEPORT__ROVOTELEPORT_HH_
#define ROVO_GZ_TELEPORT__ROVOTELEPORT_HH_

#include <QString>
#include <gz/sim/gui/GuiSystem.hh>
#include <gz/transport/Node.hh>

#include <mutex>
#include <string>

namespace rovo_gz_teleport
{
/// \brief Gazebo GUI panel that moves a model to a pose, the way RViz's
/// "2D Pose Estimate" moves a localisation guess.
///
/// Two ways in, both landing on the same set_pose call:
///   - arm "Pick in scene", then click the ground; the click already carries
///     the 3D point under the cursor, so the height comes from the scene
///     itself and needs no heightfield lookup.
///   - type X / Y / Yaw and press Teleport.
///
/// The model keeps its current yaw on a scene pick unless "Apply yaw" is on,
/// so a click is a move, not a move-and-spin.
class RovoTeleport : public gz::sim::GuiSystem
{
  Q_OBJECT

  Q_PROPERTY(QString modelName READ ModelName WRITE SetModelName
             NOTIFY ModelNameChanged)
  Q_PROPERTY(QString statusText READ StatusText NOTIFY StatusTextChanged)
  Q_PROPERTY(bool picking READ Picking WRITE SetPicking NOTIFY PickingChanged)
  Q_PROPERTY(bool following READ Following WRITE SetFollowing
             NOTIFY FollowingChanged)

  public: RovoTeleport();
  public: ~RovoTeleport() override;

  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief Reads the model's live pose straight out of the ECM, so the panel
  /// can keep the parts of a pose the user did not ask to change without
  /// guessing at a service or topic name.
  public: void Update(const gz::sim::UpdateInfo &_info,
                      gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Teleport to a typed pose. Z < -1e5 means "keep the model's
  /// current z", which is what the scene pick uses when it has no better idea.
  public: Q_INVOKABLE void Teleport(double _x, double _y, double _z,
                                    double _yawDeg, bool _applyYaw);

  /// \brief Turn the model in place, leaving its position alone.
  public: Q_INVOKABLE void SetHeading(double _yawDeg);

  /// \brief Put the model back where the world spawned it.
  public: Q_INVOKABLE void ResetPose();

  public: QString ModelName() const;
  public: void SetModelName(const QString &_name);
  public: QString StatusText() const;
  public: bool Picking() const;
  public: void SetPicking(bool _picking);
  public: bool Following() const;
  public: void SetFollowing(bool _following);

  signals: void ModelNameChanged();
  signals: void StatusTextChanged();
  signals: void PickingChanged();
  signals: void FollowingChanged();

  /// \brief Catches LeftClickToScene while armed.
  protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

  private: void SetStatus(const std::string &_text);
  private: bool haveCurrent{false};
  private: double curX{0}, curY{0}, curZ{0}, curYaw{0};

  private: gz::transport::Node node;
  private: std::string worldName;
  private: std::string modelName{"rovo"};
  private: double dropHeight{0.3};
  private: double spawnX{0.0}, spawnY{0.0}, spawnZ{0.0}, spawnYaw{0.0};
  private: bool picking{false};
  private: bool following{false};
  private: bool followApplied{false};
  private: bool followOnStart{true};
  private: double followX{-6.0}, followY{0.0}, followZ{9.0};
  private: std::string status{"idle"};
  private: mutable std::mutex mutex;
};
}  // namespace rovo_gz_teleport

#endif  // ROVO_GZ_TELEPORT__ROVOTELEPORT_HH_
