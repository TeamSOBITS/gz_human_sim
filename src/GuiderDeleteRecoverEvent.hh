#ifndef GZ_HUMAN_SIM_GUIDERDELETERECOVEREVENT_HH_
#define GZ_HUMAN_SIM_GUIDERDELETERECOVEREVENT_HH_

#include <QEvent>
#include <QString>

// Hand-mirrored copy of guide_robot/src/GuiderDeleteRecoverEvent.hh.
// See GuiderViewpointRequestEvent.hh (this directory) for why this is a
// duplicated header rather than an #include, and why that is safe here.
namespace guider
{
  namespace events
  {
    class DeleteRecoverRequest : public QEvent
    {
      public: static const QEvent::Type kType =
          QEvent::Type(QEvent::MaxUser - 68);

      public: DeleteRecoverRequest(
          const QString &_targetName, const QString &_kind, bool _recover)
        : QEvent(kType), targetName(_targetName), kind(_kind),
          recover(_recover)
      {
      }

      public: const QString &TargetName() const { return this->targetName; }

      public: const QString &Kind() const { return this->kind; }

      public: bool Recover() const { return this->recover; }

      private: QString targetName;

      private: QString kind;

      private: bool recover{false};
    };
  }  // namespace events
}  // namespace guider

#endif  // GZ_HUMAN_SIM_GUIDERDELETERECOVEREVENT_HH_
