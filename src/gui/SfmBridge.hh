#ifndef GZ_HUMAN_SIM_GUI_SFMBRIDGE_HH_
#define GZ_HUMAN_SIM_GUI_SFMBRIDGE_HH_

#include <string>
#include <utility>
#include <vector>

#include <gz/transport/Node.hh>

// SfmCrowdSystem（サーバー側のワールド単位システム）への登録・解除。
//
// ---------------------------------------------------------------------------
// なぜ HumanControlPanel から分けたのか（構想書 §13 段階4）
// ---------------------------------------------------------------------------
// **このクラスは人物を知りません。** 名前と経路を受け取るだけです。
// 「誰を登録するのか」を決めるのは呼び出し側の関心事。
//
// もう一つの狙いは、**電文の書式をここ 1 箇所に閉じ込めること。**
// 書式は SfmCrowdSystem::ParseRegistration() と一致していなければならず、
// パネルのあちこちに散らばっていると片方だけ直して静かに壊れます。
//
// ---------------------------------------------------------------------------
// トピック名を両側でハードコードしている理由
// ---------------------------------------------------------------------------
// SfmCrowdSystem 側の register_topic / unregister_topic の SDF 既定値と
// 同じ文字列を、このパネルもハードコードしています。ワールドの SDF から
// 発見する作りにしていないのは、SfmCrowdSystem はふつうワールドに 1 つ
// しか居らず、**どちらの側からも grep 一発で追える**ほうが得だからです。
// ---------------------------------------------------------------------------

namespace gz_human_sim
{
class SfmBridge
{
  /// \brief SfmCrowdSystem がこのワールドに居るか問い合わせ、結果を控える。
  ///
  /// SfmCrowdSystem は読み込まれたときに register トピックを subscribe
  /// します。それをするのは他に居ません。だから gz-transport の discovery
  /// だけで「このワールドに居るか」が分かり、**専用の heartbeat を
  /// どちらの側も出さずに済みます。**
  public: bool Available(gz::transport::Node &_node);

  /// \brief 最後に Available() が返した値。問い合わせは行わない。
  public: bool CachedAvailable() const;

  /// \brief _name の人物を、_route を巡回する対象として登録する。
  ///
  /// \param[in] _cyclic 経路を巡回するか、端で止まるか。
  /// \param[in] _route **確定済みの経路**を渡すこと。障害物回避が有効な
  ///            ときは、回避後の経路でなければ人物は壁を突っ切る。
  public: void Register(gz::transport::Node &_node, const std::string &_name,
              bool _cyclic,
              const std::vector<std::pair<double, double>> &_route);

  /// \brief _name の人物の登録を解除する。
  ///
  /// 登録されていなくても無害（向こうで名前が一致しないだけ）。削除時は
  /// 常に送ること。**同名で作り直した人物が、以前の経路を引き継いで
  /// しまうのを防ぐため。**
  public: void Unregister(gz::transport::Node &_node, const std::string &_name);

  /// \brief 2 本の publisher を初回だけ advertise する。
  ///
  /// コンストラクタでやらないのは、その時点ではまだ node が使える保証が
  /// ないため。「遅延・初回のみ」はこのパネルの他の transport 設定と同じ流儀。
  private: void EnsurePublishers(gz::transport::Node &_node);

  private: gz::transport::Node::Publisher registerPublisher;
  private: gz::transport::Node::Publisher unregisterPublisher;
  private: bool available{false};
};
}  // namespace gz_human_sim
#endif  // GZ_HUMAN_SIM_GUI_SFMBRIDGE_HH_
