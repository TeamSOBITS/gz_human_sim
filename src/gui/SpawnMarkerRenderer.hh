#ifndef GZ_HUMAN_SIM_GUI_SPAWNMARKERRENDERER_HH_
#define GZ_HUMAN_SIM_GUI_SPAWNMARKERRENDERER_HH_

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include <QString>

#include <gz/rendering/RenderTypes.hh>

// 床に描く「魔法陣」スポーンマーカーの描画。
//
// ---------------------------------------------------------------------------
// なぜ HumanControlPanel から分けたのか（構想書 §13 段階4）
// ---------------------------------------------------------------------------
// CameraController と同じ理由。マーカー描画は guide_robot にもあり
// （spawn_marker.png は両パッケージに同一バイト列でコピーされている）、
// 共通の GUI 基盤パッケージへ出す対象（§12）。だから **このクラスは Human
// も HumanRegistry も知らない。** 名前・座標・色番号だけを受け取る。
//
// 「どの人物にどのマーカーが要るか」を決めるのはパネルの仕事として
// HumanControlPanelOverlay.cc に残してある。人物ごとの Visual の持ち主も
// Human::marker.visual のままで、ここは作り方と後始末だけを担当する。
//
// ---------------------------------------------------------------------------
// スレッド
// ---------------------------------------------------------------------------
// Qt スレッド    : QueueRemoval() / InvalidatePending() / ColorHex()
// レンダースレッド: BeginFrame() / Create() / ClearPending() / AddPending()
//                   / SpinPending()
//
// Visual の生成・破棄はレンダースレッド専用。Qt 側で人物を消したときは
// 名前を QueueRemoval() に積んでおき、次のフレームで BeginFrame() が
// まとめて破棄する。
// ---------------------------------------------------------------------------

namespace gz_human_sim
{
// Spawn-marker palette (see HumanControlPanel::setShowSpawnMarker()):
// one colour per human, assigned in spawn order and wrapping around, so a
// group of people spawned together can be told apart by their markers.
// Deliberately saturated and well separated in hue rather than a smooth
// ramp -- these are identity labels, not a scale.
inline const double kMarkerColors[][3] = {
  {0.95, 0.26, 0.21},  // red
  {0.13, 0.59, 0.95},  // blue
  {0.30, 0.69, 0.31},  // green
  {1.00, 0.76, 0.03},  // amber
  {0.61, 0.15, 0.69},  // purple
  {0.00, 0.74, 0.83},  // cyan
  {1.00, 0.44, 0.00},  // orange
  {0.91, 0.12, 0.39},  // pink
};
inline constexpr int kMarkerColorCount =
    static_cast<int>(sizeof(kMarkerColors) / sizeof(kMarkerColors[0]));

// Spawn-marker disc geometry: a cylinder squashed flat and laid just above
// the floor, wide enough to read as "a person starts here" from the default
// overview camera without hiding the person standing on it.
// Half-width of the magic-circle marker. Deliberately much bigger than
// the 0.12 the old flat disc used: this panel is normally driven in a
// whole building, and at a camera distance that shows a floor plan a
// person-sized dot on the ground is simply not visible. 1.2 m across
// reads from across a room without burying the spot it marks.
inline constexpr double kMarkerRadius = 0.6;
inline constexpr double kMarkerZ = 0.02;

// Radians per second the circles turn. Slow on purpose: an idle animation
// to catch the eye, not something competing with the humans for attention.
inline constexpr double kMarkerSpinRate = 0.35;
// Not-yet-spawned picked points are drawn in the same shape but neutral
// white and more transparent -- they aren't anybody's marker yet.
// Near-solid: at the camera distances these panels are actually used from,
// anything much lower washes the circle out against a light floor. The
// pending marker stays a little softer than a placed one purely so the two
// are still tellable apart at a glance, not to make it subtle.
inline constexpr double kMarkerAlpha = 0.97;
inline constexpr double kPendingMarkerAlpha = 0.85;

class SpawnMarkerRenderer
{
  /// \brief media/spawn_marker.png への絶対パス。空なら、テクスチャ無しの
  /// 単色の板にフォールバックする。
  public: void SetTexturePath(const std::string &_path);

  /// \brief Qt スレッド: _name のマーカーを次フレームで破棄するよう積む。
  /// Visual の破棄はレンダースレッドでしかできないためのキュー。
  public: void QueueRemoval(const std::string &_name);

  /// \brief Qt スレッド: 未スポーンの候補点の並びが変わった。
  ///
  /// 個数の比較ではなくこの明示フラグで判定しているのは、1 点取り消して
  /// 別の 1 点を打つと個数が変わらないまま全マーカーの位置が変わるため。
  public: void InvalidatePending();

  /// \brief レンダースレッド: 溜まった破棄を実行し、このフレームの
  /// 回転角 [rad] を返す。
  ///
  /// 角度は経過実時間から出しているので、GUI のフレームレートが変わっても
  /// 回転速度は変わらない。
  public: double BeginFrame(const gz::rendering::ScenePtr &_scene);

  /// \brief レンダースレッド: マーカーを 1 枚作る。作れなければ nullptr。
  /// \param[in] _pending true なら「まだ誰のものでもない候補点」として
  ///            色を付けず半透明の白で描く。
  public: gz::rendering::VisualPtr Create(
      const gz::rendering::ScenePtr &_scene, const std::string &_name,
      double _x, double _y, double _z, int _colorIndex, bool _pending) const;

  /// \brief レンダースレッド: InvalidatePending() 以降で初めての呼び出しなら
  /// true を返し、フラグを落とす。
  public: bool ConsumePendingDirty();

  /// \brief レンダースレッド: 候補点のマーカーを全部消す。
  public: void ClearPending(const gz::rendering::ScenePtr &_scene);

  /// \brief レンダースレッド: 候補点のマーカーを 1 枚足す。
  public: void AddPending(const gz::rendering::ScenePtr &_scene,
      std::size_t _index, double _x, double _y, double _z);

  /// \brief レンダースレッド: 候補点のマーカーを _angle まで回す。
  /// 人物のマーカーはパネル側が同じ角度で回す。
  public: void SpinPending(double _angle);

  /// \brief 色番号を QML 用の "#rrggbb" にする。
  public: static QString ColorHex(int _colorIndex);

  private: std::string texturePath;

  /// \brief Render-scene visuals for the panel's pending spawn points.
  private: std::vector<gz::rendering::VisualPtr> pendingVisuals;
  private: bool pendingDirty{false};

  private: std::mutex removalMutex;
  private: std::vector<std::string> removalQueue;

  /// \brief Time the spin angle of every marker is derived from, so they
  /// rotate together at a rate independent of the GUI's frame rate.
  private: std::chrono::steady_clock::time_point epoch;
  private: bool epochValid{false};
};
}  // namespace gz_human_sim
#endif  // GZ_HUMAN_SIM_GUI_SPAWNMARKERRENDERER_HH_
