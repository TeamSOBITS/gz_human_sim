#include "HumanControlPanel.hh"

#include <algorithm>
#include <string>

#include <QString>

#include "HumanControlPanelInternal.hh"

// 視点操作の QML 窓口。カメラそのものを動かすのは CameraController で、
// ここがやるのは **「何番の人物か」を名前と数値に翻訳すること**だけ。
//
// 分けた理由は CameraController.hh の頭に書いてある（構想書 §13 段階4）。
// 要するに、追従カメラは guide_robot にもある共通機能なので、人物固有の
// 型（Human / HumanRegistry）を混ぜないでおきたい。その境界がこのファイル。

namespace gz_human_sim
{
void HumanControlPanel::setViewpoint(int _index, int _viewIndex, double _distance)
{
  if (_viewIndex < 0 || _viewIndex >= kViewCount)
    return;

  // Record this as _index's current view (for ActiveViewIndex()/
  // ActiveViewDistance(), which the global viewpoint combo/distance field
  // bind to) whenever _index is a real human, regardless of which branch
  // below actually runs -- including kViewFree, so switching back to this
  // human later shows "自由視点" rather than a stale prior selection.
  if (this->humans.valid(_index))
  {
    this->humans.at(_index).camera.index = _viewIndex;
    this->humans.at(_index).camera.distance = _distance;
    if (_index == this->activeHumanIndex)
      this->activeViewIndexChanged();
  }

  if (_viewIndex == kViewFree)
  {
    this->cameraController.ReleaseToFreeView();
    return;
  }

  if (!this->humans.valid(_index))
  {
    this->SetStatus("視点変更：対象の人物がありません");
    return;
  }

  const auto &human = this->humans.at(_index);

  // 一人称視点でだけ使う。**このモデル自身の原点から見た**目の高さを渡す。
  // 絶対高さ kEyeHeight をそのまま渡さないのは、モデルによって原点の高さが
  // 違うため（理由は kEyeHeight のコメントに書いてある）。この計算だけは
  // 人物モデル表を知っている必要があるので、カメラ側ではなくここでやる。
  const double eyeOffset = std::max(kMinEyeOffset,
      kEyeHeight - DefaultZForModelName(human.model));

  this->cameraController.Follow(_viewIndex, _distance, human.name, eyeOffset);
}

void HumanControlPanel::resetToInitialView()
{
  // The active human's combo should reflect reality: nothing is being
  // followed anymore once this takes effect.
  if (this->humans.valid(this->activeHumanIndex))
  {
    this->humans.at(this->activeHumanIndex).camera.index = kViewFree;
    this->activeViewIndexChanged();
  }
  this->cameraController.ResetToInitialView();
}
}  // namespace gz_human_sim
