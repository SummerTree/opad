// A click takes the body in front, not a selected one behind it; case in tools/bench_cases/viewer.py.
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QTimer>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepTools.hxx>
#include <functional>
#include <map>
#include <sstream>

#include "BenchRegistry.hpp"
#include "Jobs.hpp"
#include "MainWindow.hpp"
#include "Viewport.hpp"

namespace {
bool until(const std::function<bool()>& done, int ms) {
  QElapsedTimer clock;
  clock.start();
  while (!done() && clock.elapsed() < ms) {
    QEventLoop loop;
    QTimer::singleShot(20, &loop, &QEventLoop::quit);
    loop.exec();
  }
  return done();
}

std::string brep(const TopoDS_Shape& shape) {
  std::ostringstream out;
  BRepTools::Write(shape, out);
  return out.str();
}
}  // namespace

// OPAD_BENCH_PICKFRONT=1 on an empty document: a 40 x 40 mm slab Back and a 20 mm block Front standing over its middle, seen
// from the top. A selected body is drawn raised (Topmost with the X-ray highlight, Top without), and OCCT ranks layers before
// depth: a click on Front took the selected Back behind it. With Back selected, a click on Front selects Front; where clicks
// add and take out (a tool's picks, Thermal setup's Add), a click on Front adds it and Back stays; a click on Back where
// nothing is in front still reaches it (it is taken out). The same with the X-ray highlight off; with Select through on, the
// old order is kept (Back, drawn over Front, is what the click takes).
OPAD_BENCH(OPAD_BENCH_PICKFRONT, pickfront) {
  bool all = true;
  auto require = [&all](bool ok, const QString& what) {
    trace::log(QString("bench: pick front: %1 %2").arg(what, ok ? "PASS" : "FAIL"));
    all = all && ok;
  };
  Viewport* v = w.m_viewport;
  w.m_doc->run("import_brep", {{"brep", brep(BRepPrimAPI_MakeBox(40, 40, 10).Shape())}, {"name", "Back"}});
  w.m_doc->run("import_brep", {{"brep", brep(BRepPrimAPI_MakeBox(gp_Pnt(10, 10, 20), 20, 20, 10).Shape())}, {"name", "Front"}});
  if (!until([v] { return v->displayedCount() >= 2 && v->remainingBodies() == 0; }, 30000)) {
    require(false, "the two bodies are displayed");
    return QCoreApplication::exit(2), true;
  }
  std::map<QString, std::string> ids;
  for (const auto& id : w.m_doc->scene.all_bodies()) ids[w.m_doc->nodeName(id)] = id;
  const std::string back = ids["Back"], front = ids["Front"];
  v->standardView("top");
  v->fitAll();
  const QPointF overFront = v->widgetPoint({20, 20, 30}), overBackOnly = v->widgetPoint({4, 4, 10});
  auto selected = [v] {
    std::vector<std::string> out;
    for (const auto& r : v->selection()) out.push_back(r.body);
    std::sort(out.begin(), out.end());
    return out;
  };
  auto settle = [&w] { until([&w] { return !w.m_jobs->busy(); }, 10000); };
  auto selectBack = [&] {
    opad::Ref r;
    r.body = back;
    v->selectRefs({r});
    settle();  // the selection's layers: Back raised
  };
  auto sorted = [](std::vector<std::string> v) {
    std::sort(v.begin(), v.end());
    return v;
  };
  for (const bool xray : {true, false}) {
    v->setXrayHighlight(xray);
    const QString how = xray ? "X-ray highlight" : "depth-tested highlight";
    // a plain click: Front, not Back behind it
    selectBack();
    v->benchClickAt(overFront);
    settle();
    require(selected() == std::vector<std::string>{front}, how + ": Back selected, a click on Front selects Front");
    // clicks that add and take out
    v->setPickAccumulate(true);
    selectBack();
    v->benchClickAt(overFront);
    settle();
    require(selected() == sorted({back, front}), how + ": picks that add and take out: a click on Front adds it, Back stays");
    v->benchClickAt(overBackOnly);
    settle();
    require(selected() == std::vector<std::string>{front}, how + ": a click on Back where nothing is in front still reaches it (taken out)");
    v->setPickAccumulate(false);
    v->clearSelection();
    settle();
  }
  // Select through (a setting): what the selection draws over is what a click takes, as before
  v->setXrayHighlight(true);
  v->setSelectThrough(true);
  selectBack();
  v->benchClickAt(overFront);
  settle();
  require(selected() == std::vector<std::string>{back}, "Select through on: Back, drawn over Front, is what the click takes");
  v->setSelectThrough(false);
  QCoreApplication::exit(all ? 0 : 2);
  return true;
}
