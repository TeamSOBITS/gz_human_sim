// Instantiates the QML that is actually inside the built plugin, from
// inside an ApplicationWindow. This is the only check that catches the
// failures a build and qmllint both pass:
//
//   * a section .qml failing to resolve as a type -- the whole panel then
//     disappears at startup with "XxxSection is not a type". Adding a file
//     to gui/qml/ without adding it to BOTH gui/qml/qmldir and
//     gui/HumanControlPanel.qrc does exactly this (CLAUDE.md 罠1)
//   * a binding that reads a property which does not exist on the object
//     it is qualified with. That is silently `undefined` on read and
//     "Cannot assign to non-existent property" on write, and QML does not
//     stop -- the rest of the handler block just never runs. The 配置方法
//     radio buttons shipped broken this way for a while (`root.placingByClick`
//     was declared on the inner ColumnLayout, not on `root`)
//
// Not wired into CMake: it needs a display-less Qt run and links Qt5Quick
// directly, which the package itself does not. Build and run it by hand
// after touching anything under gui/:
//
//   cd src/gz_human_sim
//   g++ -fPIC test/qml_instantiation_check.cc -o /tmp/humancheck \
//       $(pkg-config --cflags --libs Qt5Quick Qt5Qml Qt5Gui Qt5Core) -ldl
//   QT_QPA_PLATFORM=offscreen QT_QUICK_BACKEND=software /tmp/humancheck \
//       ../../build/gz_human_sim/libHumanControlPanel.so
//
// Exit 0 and "unexpected-warning count = 0" means every section resolved
// as a type and every binding evaluated. The stub below stands in for the
// C++ plugin object, so this exercises the bindings rather than just
// parsing them -- which is why it has to carry the whole QML-facing API.
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlComponent>
#include <QQmlContext>
#include <QQuickItem>
#include <QQuickWindow>
#include <QFile>
#include <QDir>
#include <dlfcn.h>
#include <cstdio>

static int qmlErrors = 0;
static void handler(QtMsgType type, const QMessageLogContext &, const QString &msg)
{
  if (msg.contains("XDG_RUNTIME_DIR"))
    return;
  fprintf(stderr, "[%d] %s\n", int(type), qPrintable(msg));
  if (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg)
    ++qmlErrors;
}

int main(int argc, char **argv)
{
  qInstallMessageHandler(handler);
  QGuiApplication app(argc, argv);

  if (argc < 2)
  {
    fprintf(stderr, "usage: %s <libHumanControlPanel.so>\n", argv[0]);
    return 2;
  }
  if (!dlopen(argv[1], RTLD_NOW))
  {
    fprintf(stderr, "dlopen failed: %s\n", dlerror());
    return 2;
  }

  QQmlApplicationEngine engine;

  // Stands in for the plugin object. Written as a QML QtObject rather than
  // a QQmlPropertyMap so the property types QML sees are the ones the real
  // Q_PROPERTYs would give it. Every name here appears in gui/**.qml; a
  // missing one shows up as a warning rather than passing quietly.
  const char *stubQml = R"QML(
    import QtQuick 2.9
    QtObject {
      property string status: "テスト"
      property var humanModels: ["walking_actor", "standing"]
      property var posePresets: ["initial_pose", "sit_on_chair"]
      property var followModeLabels: ["テレオペ", "経路専用"]
      property var viewpointLabels: ["自由視点", "一人称", "後方追従"]
      property var pathTemplateLabels: ["円", "往復", "四角"]
      // 2体。1体だけだと「対象かどうか」の分岐が片側しか回らない。
      property var humanList: ["walker_1", "walker_2"]
      property int activeHumanIndex: 0
      property int activeViewIndex: 2
      property real activeViewDistance: 3.5
      property int activeFollowModeIndex: 0
      property string activeLockedPose: "sit_on_chair"
      property string heldPose: ""
      property bool spawnPicking: true
      property var pendingSpawnPoints: ["1.00, 2.00", "3.00, 4.00"]
      property var pendingRoutePoints: ["1.00, 2.00"]
      property bool routeRecording: true
      property int routeTargetCount: 1
      property bool useSfm: true
      property bool cyclicRoute: true
      property bool avoidObstacles: true
      // 「入っていないワールド」側の警告文を一度は描かせたいので false。
      property bool sfmAvailable: false
      property bool avoidObstaclesAvailable: false
      property bool dualsenseModeEnabled: true
      property string dualsenseStatusText: "接続済み"
      property bool invertCameraY: true
      property real jumpHeight: 1.2
      property real speedMultiplier: 1.5
      property bool shiftHeld: false
      property bool ctrlHeld: false
      property bool sHeld: true
      property bool jHeld: false

      // 実物の Q_SIGNAL（src/HumanControlPanel.hh）のうち、**上のプロパティから
      // 自動生成されない名前だけ**をここで宣言する。`property bool spawnPicking`
      // は spawnPickingChanged を自動で持つので、重ねて書くと
      // "Duplicate signal name" でスタブごと生成に失敗する。
      // 宣言し忘れると Connections が "no signal of the target matches the
      // name" を出す。**それは本物との名前の食い違いを示す有効な警告**なので、
      // 出たらまず本物のシグナル名を確認すること。
      signal humansChanged()
      signal activeHumanChanged()
      signal activeFollowModeChanged()
      signal pendingRouteChanged()
      signal routeSettingsChanged()
      signal sfmModeChanged()
      signal dualsenseModeChanged()
      signal dualsenseStatusChanged()

      function modelDescription(i) { return "説明" }
      function isCustomHuman(i) { return true }
      function isActorModel(i) { return true }
      function isHumanActorAt(i) { return true }
      function isPoseCapableHumanAt(i) { return true }
      function poseLabel(i) { return "立位" }
      function defaultZ(i) { return 0.05 }
      function followModeValue(i) { return "auto" }
      function nextSpawnX() { return 1.0 }
      function nextSpawnY() { return 2.0 }
      function spawnMarkerColorAt(i) { return "#2f6fd0" }
      function showSpawnMarkerAt(i) { return true }
      function showCollisionAt(i) { return true }
      function collisionRadiusAt(i) { return 0.3 }
      function collisionLengthAt(i) { return 1.7 }
      function routeTargetAt(i) { return true }
      // サーバーからの状態（構想書 §13 段階3）。"-" は未受信の見た目。
      function humanStateLabel(i) { return i === 0 ? "移動中" : "-" }

      function spawnHumans() {}
      function removeHuman(i) {}
      function setActiveHuman(i) {}
      function setSpawnPicking(v) {}
      function clearSpawnPoints() {}
      function undoLastSpawnPoint() {}
      function setViewpoint(i, v, d) {}
      function resetToInitialView() {}
      function setFollowMode(i, v) {}
      function setShowSpawnMarker(i, v) {}
      function setShowSpawnMarkerAll(v) {}
      function setShowCollision(i, v) {}
      function setShowCollisionAll(v) {}
      function applyCollisionSize(i, r, l) {}
      function togglePoseLock(i) {}
      function setJumpHeight(v) {}
      function setSpeedMultiplier(v) {}
      function teleopMove(i, x, y) {}
      function teleopDirection(i, k, a, b) {}
      function teleopRotate(i, left) {}
      function teleopStop(i) {}
      function teleopJump(i) {}
      function setRouteRecording(v) {}
      function clearPendingRoute() {}
      function undoLastRoutePoint() {}
      function confirmRoute() {}
      function generatePathTemplate(i) {}
      function setRouteTarget(i, v) {}
      function setAllRouteTargets(v) {}
      function setUseSfm(v) {}
      function setCyclicRoute(v) {}
      function setAvoidObstacles(v) {}
      function setSfmEnabledForTargets(v) {}
    }
  )QML";

  QQmlComponent stubComponent(&engine);
  stubComponent.setData(stubQml, QUrl("qrc:/stub.qml"));
  QObject *stub = stubComponent.create();
  if (!stub)
  {
    for (const auto &e : stubComponent.errors())
      fprintf(stderr, "STUB ERROR: %s\n", qPrintable(e.toString()));
    return 2;
  }
  engine.rootContext()->setContextProperty("HumanControlPanel", stub);

  // The panel has to live inside an ApplicationWindow: anything that
  // reaches for ApplicationWindow.overlay resolves to null outside one,
  // and null is not an error QML reports.
  const char *wrapper = R"QML(
    import QtQuick 2.9
    import QtQuick.Controls 2.2
    ApplicationWindow { width: 420; height: 900; visible: true }
  )QML";
  QQmlComponent shell(&engine);
  shell.setData(wrapper, QUrl("qrc:/wrapper.qml"));
  QObject *window = shell.create();
  if (!window)
  {
    for (const auto &e : shell.errors())
      fprintf(stderr, "WRAPPER ERROR: %s\n", qPrintable(e.toString()));
    return 2;
  }

  // Copied out of the resource under a DIFFERENT basename before loading,
  // so the component's base URL is NOT its qrc directory. That is the
  // strict case, and the one gz-gui puts a plugin in: implicit
  // same-directory types and relative imports both stop working, and only
  // the absolute `import "qrc:/HumanControlPanel/qml"` still resolves.
  // Loading it in place would additionally make the file an implicitly
  // available type named HumanControlPanel that shadows the context
  // property of the same name, turning every binding into `undefined`.
  QFile embedded(":/HumanControlPanel/HumanControlPanel.qml");
  if (!embedded.open(QIODevice::ReadOnly))
  {
    fprintf(stderr, "resource missing from the .so\n");
    return 2;
  }
  const QString copied = QDir::tempPath() + "/HumanPanelCheck.qml";
  QFile out(copied);
  out.open(QIODevice::WriteOnly);
  out.write(embedded.readAll());
  out.close();

  QQmlComponent plugin(&engine, QUrl::fromLocalFile(copied));
  if (plugin.isError())
  {
    for (const auto &e : plugin.errors())
      fprintf(stderr, "PLUGIN QML ERROR: %s\n", qPrintable(e.toString()));
    return 3;
  }
  QObject *item = plugin.create(engine.rootContext());
  if (!item)
  {
    for (const auto &e : plugin.errors())
      fprintf(stderr, "PLUGIN CREATE ERROR: %s\n", qPrintable(e.toString()));
    return 4;
  }
  auto *content = qobject_cast<QQuickWindow *>(window)->contentItem();
  if (auto *asItem = qobject_cast<QQuickItem *>(item))
  {
    asItem->setParentItem(content);
    asItem->setWidth(420);
    asItem->setHeight(900);
  }

  // Let bindings, Connections and delegates settle.
  for (int i = 0; i < 3; ++i)
    QCoreApplication::processEvents();

  // Every section must actually be in the tree. "is not a type" would have
  // failed above, but a section dropped from the root QML by accident
  // would not -- the panel would just quietly lose a block.
  static const char *kSections[] = {
    "SpawnSection", "MarkerSection", "ViewpointSection", "MoveParamsSection",
    "DualsenseSection", "FollowModeSection", "PoseSection", "CollisionSection",
    "RouteSection", "HelpSection"};
  int missing = 0;
  const auto children = item->findChildren<QObject *>();
  for (const char *name : kSections)
  {
    bool found = false;
    for (auto *c : children)
    {
      if (QString(c->metaObject()->className()).contains(name))
      {
        found = true;
        break;
      }
    }
    if (!found)
    {
      fprintf(stderr, "SECTION MISSING: %s\n", name);
      ++missing;
    }
  }

  printf("sections found = %d/%d, unexpected-warning count = %d\n",
         int(sizeof(kSections) / sizeof(*kSections)) - missing,
         int(sizeof(kSections) / sizeof(*kSections)), qmlErrors);
  return (missing == 0 && qmlErrors == 0) ? 0 : 5;
}
