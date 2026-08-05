#ifndef GZ_HUMAN_SIM_GUIDERVIEWPOINTREQUESTEVENT_HH_
#define GZ_HUMAN_SIM_GUIDERVIEWPOINTREQUESTEVENT_HH_

#include <QEvent>
#include <QString>

// Hand-mirrored copy of guide_robot/src/GuiderViewpointRequestEvent.hh.
// gz_human_sim must not gain a build dependency on guide_robot (same rule
// GuiderTargetRoster.hh's format is under -- see HumanControlPanel.cc's
// PublishRoster() comment), so this class is duplicated here field-for-
// field rather than included. Any change to the original must be mirrored
// here by hand, including the QEvent::Type value.
//
// This is safe across the two plugins' separately-compiled .so files
// (rather than merely "textually identical") because both are built in
// the same colcon workspace against the same Qt5/compiler ABI -- the
// same precondition GuiderFloorViewEvent.hh/GuiderJointBridgeEvent.hh
// already rely on for their guide_robot-internal cross-plugin delivery,
// just extended across a package boundary that happens to share a build
// environment.
namespace guider
{
  namespace events
  {
    class ViewpointRequest : public QEvent
    {
      public: static const QEvent::Type kType =
          QEvent::Type(QEvent::MaxUser - 66);

      public: ViewpointRequest(
          const QString &_targetName, const QString &_kind, int _viewIndex)
        : QEvent(kType), targetName(_targetName), kind(_kind),
          viewIndex(_viewIndex)
      {
      }

      public: const QString &TargetName() const { return this->targetName; }

      public: const QString &Kind() const { return this->kind; }

      public: int ViewIndex() const { return this->viewIndex; }

      private: QString targetName;

      private: QString kind;

      private: int viewIndex{0};
    };
  }  // namespace events
}  // namespace guider

#endif  // GZ_HUMAN_SIM_GUIDERVIEWPOINTREQUESTEVENT_HH_
