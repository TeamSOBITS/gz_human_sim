#ifndef GZ_HUMAN_SIM_GUI_CAMERACONTROLLER_HH_
#define GZ_HUMAN_SIM_GUI_CAMERACONTROLLER_HH_

#include <functional>
#include <mutex>
#include <string>

#include <QString>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/rendering/RenderTypes.hh>

// GUI カメラ（ユーザーカメラ）の視点制御。
//
// ---------------------------------------------------------------------------
// なぜ HumanControlPanel から分けたのか（構想書 §13 段階4）
// ---------------------------------------------------------------------------
// このクラスは **人物のことを何も知りません。** 追従先をモデル名の文字列で
// 受け取るだけです。guide_robot にもほぼ同じ視点制御があり（§12）、共通の
// GUI 基盤パッケージへ出すのはこのクラスです。そのとき「人物用の型が混じって
// いて持っていけない」とならないよう、いま境界を引いておきます。
//
// したがって **ここに Human や HumanRegistry を持ち込まないこと。**
// 「何番の人物か」「その人物のモデルは何か」を解決するのはパネルの仕事で、
// このクラスへは解決済みの名前と数値だけを渡します。
//
// ---------------------------------------------------------------------------
// スレッド
// ---------------------------------------------------------------------------
// Qt スレッド    : Follow() / ReleaseToFreeView() / ResetToInitialView()
// レンダースレッド: ApplyPending()
//
// Ogre2 のシーンに触れてよいのはレンダースレッドだけなので、Qt 側は
// 「やってほしいこと」を ViewCommand に書き置くだけにして、実際の
// SetFollowTarget() はレンダースレッドで行います。受け渡しは viewMutex。
// ---------------------------------------------------------------------------

namespace gz_human_sim
{
/// \brief 視点の種類。QML の ComboBox の並び順、kViewpointLabels、
/// CameraController::Follow() の switch の 3 つが同じ順序であること。
enum ViewIndex
{
  kViewFree = 0,
  kViewFirstPerson,
  kViewBehind,
  kViewFront,
  kViewRight,
  kViewLeft,
  kViewTop,
  kViewFrontRightUp,
  kViewFrontLeftUp,
  kViewCount,
};

inline const char *const kViewpointLabels[kViewCount] = {
  "自由視点", "一人称（本人視点）", "後方追従", "前方から", "右横から", "左横から",
  "俯瞰（真上）", "右奥上から（斜め上）", "左奥上から（斜め上）",
};

// How eagerly the chase camera (SetFollowTarget/SetTrackTarget's pgain)
// catches up to the human's current world position each frame. The
// background-swings-when-the-body-turns problem this used to be tuned for
// is now fixed properly via ViewCommand::worldFrame instead (see
// ApplyPending()) -- the human's own rotation no longer moves the
// camera at all, so this only smooths the camera's translation as the
// human actually walks somewhere. Kept a bit below the original 0.35 for
// a gentle trailing feel without being sluggish to snap onto a
// freshly-selected human.
inline constexpr double kChasePGain = 0.25;

class CameraController
{
  /// \brief ステータス行への出力口。パネルの SetStatus() を挿してもらう。
  /// このクラスが Qt のシグナルを持たずに済むよう、あえて std::function に
  /// してある（QObject にすると §12 で持ち出すときに moc が要る）。
  public: using StatusCallback = std::function<void(const QString &)>;

  public: void SetStatusCallback(StatusCallback _callback);

  /// \brief _name のモデルを _viewIndex の視点で追従する。
  /// \param[in] _viewIndex kViewFree と kViewCount 以上は呼び出し側で弾く
  ///            こと（ここでは default: で単に何もしない）。
  /// \param[in] _distance 追従距離 [m]。kViewFirstPerson では使わない。
  /// \param[in] _name 追従先のモデル名（シーン上のノード名）。
  /// \param[in] _eyeOffset kViewFirstPerson 専用。**そのモデル自身の原点から
  ///            見た**目の高さ [m]。モデルによって原点の高さが違うため、
  ///            絶対高さではなくこの相対値を受け取る（呼び出し側が
  ///            kEyeHeight から計算する）。
  public: void Follow(int _viewIndex, double _distance,
              const std::string &_name, double _eyeOffset);

  /// \brief 追従をやめ、カメラを自由視点に戻す（位置はそのまま）。
  public: void ReleaseToFreeView();

  /// \brief 追従をやめたうえで、ワールド読み込み直後の俯瞰位置へ戻す。
  /// ApplyPending() がまだ一度もユーザーカメラを見つけていない場合は
  /// 戻る先が無いので、その旨をステータスに出すだけになる。
  public: void ResetToInitialView();

  /// \brief レンダースレッド専用。溜まっている ViewCommand を適用する。
  public: void ApplyPending();

  /// \brief 最後に確認した GUI カメラの XY を返す。まだ一度もカメラを
  /// 見つけていなければ false。
  ///
  /// スポーン安全判定（ProbeSafeSpawnPosition）が使う。オペレータが
  /// クリックできた点なのだから、カメラからその点への視線は必ず通って
  /// いる。だから「カメラの側から寄っていく」のが、家具にぶつからない
  /// 唯一確実な進入方向になる。
  ///
  /// \note 更新されるのは ApplyPending() が実際に命令を処理したフレーム
  /// だけで、毎フレームではない（元の ApplyViewpoint() が命令なしで
  /// 即 return していた挙動をそのまま維持している）。
  public: bool GroundPosition(double &_x, double &_y) const;

  /// \brief シーンから GUI カメラを見つける。初回は初期姿勢も控える。
  private: bool EnsureUserCamera(const gz::rendering::ScenePtr &_scene);

  /// \brief Pending camera command, written on the Qt thread by
  /// Follow() and consumed on the render thread by ApplyPending().
  /// `engage` false means "release the camera back to free view".
  private: struct ViewCommand
  {
    bool pending{false};
    bool engage{false};
    std::string target;
    gz::math::Vector3d followOffset{0.0, 0.0, 0.0};
    gz::math::Vector3d trackOffset{0.0, 0.0, 0.6};
    // True (the default, set for every view except kViewFirstPerson) means
    // followOffset/trackOffset are fixed WORLD-frame vectors, so the
    // camera's position/look-at direction don't rotate when the human's
    // body turns -- only the human's own position (which the offsets are
    // still added to every frame) moves the camera, keeping the
    // background visually stable while the body turns in place (a chase
    // view that rotated with the body would swing the whole background
    // around on every turn, which reads as disorienting rather than as a
    // 3D-game-style chase camera). kViewFirstPerson sets this false
    // instead, since a head/eye view SHOULD turn with the body.
    bool worldFrame{true};
    QString label;
    int retries{0};
    // Only meaningful together with engage=false: also snap the camera
    // back to initialCameraPose instead of just releasing follow/track
    // and leaving it wherever it drifted to (see ResetToInitialView()).
    bool resetPose{false};
  };

  private: StatusCallback statusCallback;

  private: gz::rendering::CameraPtr userCamera;
  private: std::mutex viewMutex;
  private: ViewCommand viewCommand;

  /// \brief 現在追従しているモデル名。
  /// \note 書かれるだけで、いまはどこからも読まれていない。分離にあたって
  /// 挙動を変えないため、元のまま残してある。
  private: std::string viewpointTarget;

  // Captured once, the first time ApplyPending() finds the user camera
  // (i.e. still at whatever gui.config's MinimalScene <camera_pose>
  // placed it at) -- see ResetToInitialView().
  private: gz::math::Pose3d initialCameraPose;
  private: bool initialCameraPoseCaptured{false};

  private: mutable std::mutex cameraPosMutex;
  private: double cameraX{0.0};
  private: double cameraY{0.0};
  private: bool cameraPosValid{false};
};
}  // namespace gz_human_sim
#endif  // GZ_HUMAN_SIM_GUI_CAMERACONTROLLER_HH_
