// Extrude To face driven as a user does it; case in tools/bench_cases/design.py.
#include <QComboBox>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QTimer>

#include <functional>

#include "AppDocument.hpp"
#include "BenchRegistry.hpp"
#include "DesignController.hpp"
#include "DesignPanels.hpp"
#include "Jobs.hpp"
#include "MainWindow.hpp"
#include "Tracking.hpp"
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
}  // namespace

// OPAD_BENCH_EXTRUDE_TOFACE=1 on extrude-curved (a cylinder, a sphere, a box, the sketch Low with a rectangle under each):
// Extent chosen in its combo box (after the profile, and before it), then clicks on what Up to face takes: a face, an origin
// plane. Then each profile up to every face of every body, the face extended or not, each End at: every preview planned or
// refused in words, the app still running (a crash report of To face could not be reproduced on Linux; this keeps watch).
OPAD_BENCH(OPAD_BENCH_EXTRUDE_TOFACE, extrudeToFace) {
  bool all = true;
  auto require = [&all](bool ok, const QString& what) {
    trace::log(QString("bench: extrude to face: %1 %2").arg(what, ok ? "PASS" : "FAIL"));
    all = all && ok;
  };
  w.setWorkspace("design");
  DesignController* design = w.m_design;
  FeaturePanel* form = design->featurePanel();
  Viewport* view = w.m_viewport;
  auto settle = [&] { until([&] { return !design->previewing() && !w.m_jobs->busy(); }, 20000); };
  view->standardView("iso");
  view->fitAll();
  auto clickOn = [&](const std::function<bool(const std::string&, const opad::Ref&)>& want, const QString& what) {
    int x = 0, y = 0;
    if (!until([&] { return view->benchPickPoint(want, x, y, true); }, 8000)) {
      require(false, what + " is found in the view");
      return false;
    }
    trace::log("bench: extrude to face: clicking " + what);
    view->benchClickAt(x, y);
    settle();
    trace::log("bench: extrude to face: after " + what + ": " + QString::fromStdString(form->inputs().dump()) + " status " + form->statusText());
    return true;
  };
  auto region = [](double x) {
    return [x](const std::string& c, const opad::Ref&) {
      if (c.find("\"at\"") == std::string::npos) return false;
      const opad::json j = opad::json::parse(c);
      return std::abs(j["at"][0].get<double>() - x) < 6;
    };
  };
  std::vector<std::string> bodies = w.m_doc->scene.all_bodies();
  for (const bool profileFirst : {true, false}) {
    design->startFeature("extrude");
    settle();
    if (profileFirst) clickOn(region(0), "the region under the cylinder");
    QComboBox* extent = nullptr;
    for (auto* box : form->findChildren<QComboBox*>())
      if (box->findData("to_face") >= 0) extent = box;
    if (!extent) {
      require(false, "the Extent combo box");
      break;
    }
    extent->setCurrentIndex(extent->findData("to_face"));
    settle();
    clickOn([](const std::string& c, const opad::Ref& r) { return c.empty() && r.kind == opad::Ref::Kind::Face; }, "a face");
    clickOn([](const std::string& c, const opad::Ref&) { return c.find("\"base\"") != std::string::npos; }, "an origin plane");
    require(true, QString("the app still runs (%1)").arg(profileFirst ? "profile first" : "To face first"));
  }
  // Every profile up to every face of every body, the face extended or not, each End at.
  for (const double x : {0.0, -40.0, 45.0}) {
    design->startFeature("extrude");
    settle();
    const std::string sketch = w.m_doc->scene.sketches.front().id;
    form->setPicks("profiles", opad::json::array({{{"sketch", sketch}, {"at", {x, 0}}}}));
    form->setValue("extent", "to_face");
    settle();
    for (const auto& body : bodies)
      for (int face = 0; face < 4; ++face) {
        opad::Ref r;
        r.body = body;
        r.kind = opad::Ref::Kind::Face;
        r.index = face;
        for (const bool extend : {true, false})
          for (const char* end : {"follow_face", "farthest_contact"}) {
            form->setPicks("extent_face", opad::json::array({r.to_json()}));
            form->setValues({{"extend", extend}, {"extent_end", end}});
            settle();
          }
        trace::log(QString("bench: extrude to face: x %1 up to %2 face %3: %4").arg(x).arg(w.m_doc->nodeName(body)).arg(face).arg(form->statusText()));
      }
  }
  require(true, "every profile up to every face: the app still runs");
  QCoreApplication::exit(all ? 0 : 2);
  return true;
}
