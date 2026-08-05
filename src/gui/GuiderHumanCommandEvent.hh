#ifndef GZ_HUMAN_SIM_GUIDERHUMANCOMMANDEVENT_HH_
#define GZ_HUMAN_SIM_GUIDERHUMANCOMMANDEVENT_HH_

#include <QEvent>
#include <QString>
#include <QStringList>
#include <QVariantMap>

// Hand-mirrored copy of guide_robot/src/GuiderHumanCommandEvent.hh.
// See GuiderViewpointRequestEvent.hh (this directory) for why this is a
// duplicated header rather than an #include, and why that is safe here.
// The two copies must stay identical, kType and every kCmd* string
// included -- but note that, unlike the roster (罠4), a MISMATCH here
// degrades loudly rather than silently: an unknown command comes back
// with Handled()==false and the pad says so.

namespace guider
{
  namespace events
  {
    /// \brief Sent by GuiderPadController, consumed by gz_human_sim's
    /// HumanControlPanel: "do this to a person".
    ///
    /// \par Why this exists
    /// With `pad_ui:=true` the sidebar carries no panels at all
    /// (config/gui_pad.config), so every human-side control that used to
    /// live on HumanControlPanel's own QML -- spawn, jump, walking speed,
    /// follow mode, waypoints -- is unreachable. The pad is the only UI,
    /// so the pad has to be able to ask for those, and the only channel
    /// between two gz-gui plugins in different packages is a QEvent.
    ///
    /// \par Why one event and not one per action
    /// The QEvent::Type ledger in CLAUDE.md is a scarce, hand-managed
    /// resource (MaxUser-64 .. -74 were already spent). More importantly,
    /// the roster's '|'-separated fixed-length format taught the lesson
    /// this deliberately avoids (罠4): a positional format breaks
    /// silently when one side gains a field. Args and Reply are MAPS, so
    /// a key either side does not know is simply not read -- adding a
    /// command, or an argument to an existing one, can never make the
    /// other package misread what it already understood.
    ///
    /// \par Synchronous, like the two query events
    /// App()->sendEvent() is QCoreApplication::sendEvent, i.e. a direct
    /// call, so the sender can read Reply() and Handled() the instant it
    /// returns -- that is how kCmdModels asks what human models exist
    /// instead of hand-mirroring the list (which is exactly the mistake
    /// 罠4 documents). postEvent() would queue it and the sender would
    /// read an empty reply; do not change it.
    ///
    /// \par Not a duplicate of DeleteRecoverRequest
    /// Deleting and recovering a person already works from the pad
    /// through GuiderDeleteRecoverEvent, which both panels answer with
    /// the same semantics. Nothing here repeats it.
    class HumanCommandRequest : public QEvent
    {
      /// \brief Next free slot after GuiderElevatorQueryEvent's
      /// MaxUser-74 (see CLAUDE.md's event-type ledger; MaxUser-76 is the
      /// next one after this).
      public: static const QEvent::Type kType =
          QEvent::Type(QEvent::MaxUser - 75);

      public: HumanCommandRequest(
          const QString &_command, const QString &_target = QString(),
          const QVariantMap &_args = QVariantMap())
        : QEvent(kType), command(_command), target(_target), args(_args)
      {
      }

      /// \brief One of the kCmd* constants below.
      public: const QString &Command() const { return this->command; }

      /// \brief Which person, by gz entity name (the roster's `name`).
      /// Empty for commands that are not about one particular person
      /// (kCmdModels, kCmdSpawn, kCmdSpeed).
      public: const QString &Target() const { return this->target; }

      public: const QVariantMap &Args() const { return this->args; }

      /// \brief Written by the receiver. Read it straight after
      /// sendEvent() returns.
      public: QVariantMap &Reply() { return this->reply; }
      public: const QVariantMap &Reply() const { return this->reply; }

      /// \brief False when no HumanControlPanel is loaded at all, or it
      /// did not understand the command. Lets the pad say "人物パネルが
      /// 読み込まれていません" instead of appearing to do nothing --
      /// which is exactly how the QML property-owner bug (罠13) managed
      /// to ship.
      public: bool Handled() const { return this->handled; }
      public: void SetHandled(bool _handled) { this->handled = _handled; }

      private: QString command;
      private: QString target;
      private: QVariantMap args;
      private: QVariantMap reply;
      private: bool handled{false};
    };

    // Command names. Strings rather than an enum so that a package built
    // against an older copy of this header simply fails to match an
    // unknown one (and reports Handled()==false), instead of matching
    // some other command that happens to share its number.

    /// \brief Ask what human models can be spawned. No target, no args.
    /// Reply: "labels" (QStringList, index = model index),
    ///        "defaultZ" (QVariantList of double, ground offset per model),
    ///        "movable" (QVariantList of bool, false = static pose-only).
    inline const char *const kCmdModels = "models";

    /// \brief Spawn one person. No target.
    /// Args: "model" (int index into kCmdModels' reply),
    ///       "x", "y", "z" (double, world; z is the FLOOR height -- the
    ///       receiver adds the model's own ground offset, because only it
    ///       knows that per model), "yaw" (double, optional).
    /// Reply: "name" (QString) -- the receiver names it, since it is the
    /// one that knows which names are already taken.
    inline const char *const kCmdSpawn = "spawn";

    /// \brief Make the target jump, at the panel's current jump height.
    inline const char *const kCmdJump = "jump";

    /// \brief Set the walking-speed multiplier. No target: it is a single
    /// panel-wide setting, the same one the QML slider drives.
    /// Args: "value" (double). Reply: "value" (double, as clamped).
    inline const char *const kCmdSpeed = "speed";

    /// \brief Read the panel-wide walking-speed multiplier.
    /// Reply: "value" (double).
    inline const char *const kCmdSpeedQuery = "speed?";

    /// \brief Set the target's follow mode.
    /// Args: "mode" (QString, "auto" or "path").
    /// Reply: "mode" (QString, as applied).
    inline const char *const kCmdFollowMode = "followMode";

    /// \brief Read the target's follow mode. Reply: "mode" (QString).
    inline const char *const kCmdFollowModeQuery = "followMode?";

    /// \brief Walk the target to a world point (a one-point path, the
    /// same channel the QML's click-to-walk uses).
    /// Args: "x", "y" (double, world).
    inline const char *const kCmdWaypoint = "waypoint";

    /// \brief Show or hide the target's collision capsule.
    /// Args: "on" (bool). Reply: "on" (bool, as applied).
    inline const char *const kCmdShowCollision = "showCollision";

    /// \brief Read whether the target's collision capsule is shown.
    /// Reply: "on" (bool).
    inline const char *const kCmdShowCollisionQuery = "showCollision?";

    /// \brief Read what the target is doing right now, as a display
    /// string. The authority is the server (罠14), which the panel
    /// already subscribes to -- the pad must not re-derive it.
    /// Reply: "text" (QString).
    inline const char *const kCmdStateQuery = "state?";
  }  // namespace events
}  // namespace guider

#endif  // GZ_HUMAN_SIM_GUIDERHUMANCOMMANDEVENT_HH_
