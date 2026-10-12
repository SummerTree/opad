// OPAD_BENCH_SECTION_PICK=<prefix> on a document with one box: the Section panel's Pick face takes one face and is done.
// Pick face waits (the tool runs, faces are picked); a top face clicked sets the plane, Pick face waits no more and the
// filter is back; another face selected afterwards leaves the plane where it was (it used to follow every face picked);
// Pick face again waits again, and a second click on it stops it without a pick; Exit section (the footer's main button)
// turns the section off and closes the panel, Save as named section beside it.
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QPushButton>
#include <QTimer>
#include <QToolButton>
#include <cmath>
#include <functional>
#include <memory>

#include "BenchRegistry.hpp"
#include "Jobs.hpp"
#include "MainWindow.hpp"
#include "PanelFooter.hpp"
#include "SectionPanel.hpp"
#include "ToolPanel.hpp"
#include "opad/inspect.hpp"

OPAD_BENCH(OPAD_BENCH_SECTION_PICK, sectionPick) {
  bool all = true;
  auto check = [&all](bool ok, const QString& what) {
    trace::log(QString("bench: section pick: %1 %2").arg(what, ok ? "PASS" : "FAIL"));
    all = all && ok;
  };
  auto until = [](const std::function<bool()>& done, int ms) {
    QElapsedTimer t;
    t.start();
    while (!done() && t.elapsed() < ms) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
  };
  const auto bodies = w.m_doc->scene.all_bodies();
  if (bodies.empty()) {
    check(false, "the document has a body");
    QCoreApplication::exit(2);
    return true;
  }
  // The box's faces by the way they face: its top (+Z) and a side (+X).
  opad::Ref top, side;
  for (int i = 0; i < 6; ++i) {
    opad::Ref r;
    r.body = bodies.front();
    r.kind = opad::Ref::Kind::Face;
    r.index = i;
    const auto info = opad::inspect_ref(w.m_doc->doc, w.m_doc->scene, r);
    if (!info.contains("normal")) continue;
    if (info["normal"][2].get<double>() > 0.5) top = r;
    if (info["normal"][0].get<double>() > 0.5) side = r;
  }
  QToolButton* pick = nullptr;
  for (auto* b : w.m_section->findChildren<QToolButton*>())
    if (b->text() == SectionPanel::tr("Pick face")) pick = b;
  auto click = [&w, &until](const opad::Ref& face) {  // as the view's click does: the selection, then its handler
    // the faces are pickable once the filter has reached the view (its owners are made on the way)
    until([&w, &face] {
      w.m_viewport->selectRefs({face});
      return w.m_viewport->selection().size() == 1;
    }, 10000);
    w.onViewportSelection();
  };
  until([&w] { return w.m_viewport->displayedCount() > 0 && !w.m_jobs->busy(); }, 20000);
  w.action("inspect.section")->setChecked(true);
  check(pick && w.m_sectionPanel->isVisible() && !top.body.empty() && !side.body.empty(), "the Section panel is open with Pick face; the box has a top and a side");
  if (!pick || top.body.empty() || side.body.empty()) {
    QCoreApplication::exit(2);
    return true;
  }
  // The footer: Exit section is the main button, Save as named section a side one.
  PanelFooter* footer = w.m_section->footer();
  QPushButton* save = w.m_section->findChild<QPushButton*>("sectionSave");
  check(footer && footer->primaryText() == SectionPanel::tr("Exit section") && footer->primary()->isVisibleTo(w.m_section) && save &&
            save->isVisibleTo(w.m_section) && !footer->cancel()->isVisibleTo(w.m_section),
        "the footer: Exit section is the main button, Save as named section beside it");
  pick->click();
  check(w.m_section->picking() && w.m_tool.id == "sectionface" && w.m_viewport->selectionFilter() == Viewport::SelFilter::Face, "Pick face waits for a face (faces picked)");
  click(top);
  until([&w] { return !w.m_sectionJob; }, 20000);
  opad::Vec3 o = w.m_section->origin(), n = w.m_section->normal();
  check(w.m_section->fromFace() && std::abs(std::abs(n[2]) - 1) < 1e-9 && std::abs(o[2] - 10) < 0.05, QString("the top face sets the plane (z %1)").arg(o[2]));
  check(!w.m_section->picking() && w.m_tool.id.isEmpty() && w.m_viewport->selectionFilter() == Viewport::SelFilter::Body && !pick->isDown(),
        "then Pick face waits no more: no tool, the filter back to bodies");
  // Another face selected (another tool's pick, a selection): the plane stays.
  w.m_viewport->setSelectionFilter(Viewport::SelFilter::Face);
  click(side);
  until([&w] { return !w.m_sectionJob; }, 2000);
  n = w.m_section->normal();
  check(!w.m_sectionJob && std::abs(std::abs(n[2]) - 1) < 1e-9 && std::abs(w.m_section->origin()[2] - 10) < 0.05, "a face selected afterwards leaves the plane on the top face");
  w.m_viewport->clearSelection();
  w.m_viewport->setSelectionFilter(Viewport::SelFilter::Body);
  // Pick face again: another face, or (clicked once more) none after all.
  pick->click();
  check(w.m_section->picking() && w.m_tool.id == "sectionface", "Pick face again waits for another face");
  pick->click();
  check(!w.m_section->picking() && w.m_tool.id.isEmpty() && w.m_section->fromFace() && pick->isChecked(), "a second click stops it; the plane is still the top face's");
  pick->click();
  click(side);
  until([&w] { return !w.m_sectionJob; }, 20000);
  n = w.m_section->normal();
  check(std::abs(std::abs(n[0]) - 1) < 1e-9 && !w.m_section->picking(), "picked again: the side face's plane");
  // Exit section.
  footer->primary()->click();
  check(!w.m_section->enabled() && !w.m_sectionPanel->isVisible() && !w.action("inspect.section")->isChecked(), "Exit section turns the section off and closes the panel");
  QCoreApplication::exit(all ? 0 : 2);
  return true;
}
