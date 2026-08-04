#ifndef GZ_HUMAN_SIM_GUI_COLLISIONBODYCONTROLLER_HH_
#define GZ_HUMAN_SIM_GUI_COLLISIONBODYCONTROLLER_HH_

#include <string>

#include <gz/rendering/RenderTypes.hh>

// 当たり判定モデル（human_collision_body）の SDF 組み立てと、
// そのデバッグ表示のオン・オフ。
//
// ---------------------------------------------------------------------------
// なぜ HumanControlPanel から分けたのか（構想書 §13 段階4）
// ---------------------------------------------------------------------------
// CameraController / SpawnMarkerRenderer と同じで、**このクラスは人物を
// 知りません。** モデル名の文字列と寸法だけを受け取ります。
//
// もう一つ、こちらには実利があります。テンプレートの穴埋めが
// **2 箇所に同じ形で書かれていました**（当たり判定サイズ変更と、スポーン
// 安全判定のプローブ）。片方だけ直すと静かにずれるので、ここへ寄せました。
//
// ---------------------------------------------------------------------------
// スレッド
// ---------------------------------------------------------------------------
// Qt スレッド    : SetTemplate() / HasTemplate() / BuildSdf()
// レンダースレッド: SetVisible() / LogMissing()
// ---------------------------------------------------------------------------

namespace gz_human_sim
{
/// \brief 当たり判定ビジュアルがシーンに現れるのを待つ上限フレーム数。
///
/// 当たり判定モデルは人物本体とは別に spawn されるので、人物が見えてから
/// 数フレーム遅れて現れる。見つからないまま毎フレーム全ビジュアルを
/// 走査し続けないよう、ここで打ち切る（約 10 秒ぶん）。
inline constexpr int kCollisionVisualMaxRetries = 600;

class CollisionBodyController
{
  /// \brief models/human_collision_body/model.sdf の中身を丸ごと渡す。
  /// 空のままなら BuildSdf() は使えない（HasTemplate() が false）。
  public: void SetTemplate(std::string _sdf);

  public: bool HasTemplate() const;

  /// \brief テンプレートから、_modelName 用の SDF を作る。
  ///
  /// モデル名と VelocityControl のトピックを差し替える。トピックを
  /// 差し替えないと、**同時に存在する当たり判定モデルとプローブが全部
  /// 同じトピックを奪い合う。**
  ///
  /// \param[in] _radius _length 負なら寸法は差し替えない（テンプレートの
  ///            既定値のまま）。プローブはこちらを使う。
  public: std::string BuildSdf(const std::string &_modelName,
              double _radius = -1.0, double _length = -1.0) const;

  /// \brief _modelName の当たり判定ビジュアルの表示を切り替える。
  /// 1 つも見つからなければ false（呼び出し側がリトライ回数を持つ）。
  public: bool SetVisible(const gz::rendering::ScenePtr &_scene,
              const std::string &_modelName, bool _visible) const;

  /// \brief 見つからなかったときの診断ダンプ。リトライのたびではなく、
  /// **探索の最初の 1 回だけ**呼ぶこと（呼び出し側の責任）。
  public: void LogMissing(const gz::rendering::ScenePtr &_scene,
              const std::string &_modelName) const;

  private: std::string sdfTemplate;
};
}  // namespace gz_human_sim
#endif  // GZ_HUMAN_SIM_GUI_COLLISIONBODYCONTROLLER_HH_
