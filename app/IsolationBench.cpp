// Isolate mode as the browser and the tools use it; case in tools/bench_cases/views.py.
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QTimer>

#include <functional>

#include "AppDocument.hpp"
#include "BenchRegistry.hpp"
#include "BrowserPanel.hpp"
#include "MainWindow.hpp"
#include "Tracking.hpp"
#include "Viewport.hpp"
#include "opad/design/sketch.hpp"

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
}  // namespace

// OPAD_BENCH_ISOLATION=1 on three boxes: the first isolated; a box and a sketch made while isolated join the isolation and
// are drawn (they were made and never seen); the browser's eye of the second box adds it (its own visibility flag kept),
// clicked again takes it out; Space on a selection toggles it in and out; the eye of every row out of the isolation ends it.
OPAD_BENCH(OPAD_BENCH_ISOLATION, isolation) {
  bool all = true;
  auto require = [&all](bool ok, const QString& what) {
    trace::log(QString("bench: isolation: %1 %2").arg(what, ok ? "PASS" : "FAIL"));
    all = all && ok;
  };
  AppDocument* doc = w.m_doc;
  Viewport* view = w.m_viewport;
  const std::vector<std::string> bodies = doc->scene.all_bodies();
  if (bodies.size() < 3) {
    require(false, "three boxes");
    return QCoreApplication::exit(2), true;
  }
  const std::string a = bodies[0], b = bodies[1], c = bodies[2];
  view->isolate({a});
  require(view->isIsolated() && view->inIsolation(a) && !view->inIsolation(b), "the first box isolated");
  require(until([&] { return view->drawsBody(a) && !view->drawsBody(b); }, 5000), "only it drawn");
  const opad::json made = doc->run("feature", {{"kind", "box"}, {"inputs", {{"x", "0 mm"}, {"y", "40 mm"}, {"length", "10 mm"}, {"width", "10 mm"}, {"height", "10 mm"}}}});
  const std::string fresh = made.value("body_ids", opad::json::array()).empty() ? std::string() : made["body_ids"][0].get<std::string>();
  require(!fresh.empty() && until([&] { return view->drawsBody(fresh); }, 5000) && view->inIsolation(fresh), "a box made while isolated joins it and is drawn");
  opad::design::Sketch sk;
  sk.add_circle(sk.add_point(0, 0), 5);
  const std::string sketch = doc->run("sketch", {{"plane", {{"base", "xy"}}}, {"geometry", sk.to_json()}}).value("sketch_id", std::string());
  require(until([&] { return view->drawsSketch(sketch); }, 5000) && view->inIsolation(sketch), "a sketch made while isolated joins it and is drawn");
  require(!view->drawsBody(b) && !view->drawsBody(c), "the boxes left out stay out");
  // The browser's eye: the isolation's while it is on.
  emit w.m_browser->tree()->eyeClicked(b);
  require(view->inIsolation(b) && until([&] { return view->drawsBody(b); }, 5000) && doc->node(b)->visible, "the eye of a box out of it adds it (its own flag kept)");
  emit w.m_browser->tree()->eyeClicked(b);
  require(!view->inIsolation(b) && until([&] { return !view->drawsBody(b); }, 5000) && doc->node(b)->visible, "clicked again: taken out");
  w.m_browser->toggleVisibility({b, c});
  require(view->inIsolation(b) && view->inIsolation(c), "Space on two out of it: both in");
  w.m_browser->toggleVisibility({b, c});
  require(!view->inIsolation(b) && !view->inIsolation(c), "Space again: both out");
  for (const std::string& id : {a, fresh, sketch}) emit w.m_browser->tree()->eyeClicked(id);
  require(!view->isIsolated() && until([&] { return view->drawsBody(b) && view->drawsBody(c); }, 5000), "the last one out ends it: everything shown as its flags say");
  emit w.m_browser->tree()->eyeClicked(b);
  require(!doc->node(b)->visible, "out of isolate mode the eye hides again");
  QCoreApplication::exit(all ? 0 : 2);
  return true;
}
