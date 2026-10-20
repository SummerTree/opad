// OPAD_BENCH_COOLING_WALLS=<prefix>: Thermal setup on an enclosure of two bodies, a tray and the cover on it (built here on an
// empty document: a board, a 3 W chip, a heatsink with a fin (the component Cooler), a fan block with a hub (the component Fan
// unit), a desk under it). Everything is picked through the app's selection (what clicks in the view and rows of the browser
// make). Nothing is guessed: the box's list opens empty, the heat table holds only the chip the case already heats, the Air page
// no fan. Each list grows by its Add button, which picks while it is on (it says Done): the tray and the cover added; a click
// that would toggle one out keeps it; Remove takes out the row chosen; Done stops. The Heat page: the board added (no power,
// ticked Board), a face of it added through Add face (the Faces filter on) with its power, kept when the filter is switched by
// hand; a part added and removed. The Air page: Apply refuses fans with none; a fan added (a component, one part), the face its
// air goes through (Flip turns it round; its rectangle the outlet, drawn with the air's way, sized narrower), a custom fan, the
// heatsink (a component) and its material. Apply writes the enclosure, the heat loads, the fan load, the board and the heatsink's
// material; opened again, everything is as it was set. Natural convection: no fan load, the air rising (buoyancy); opened again,
// it is chosen. Fans and warm air rising: the fan's load back with buoyancy, at Quick (a new study's quality), the run's estimate
// saying it takes longer. Frames: <prefix>.box.png, .heat.png, .air.png.
#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QRadioButton>
#include <QTableWidget>
#include <QTimer>

#include <memory>

#include "AppDocument.hpp"
#include "BenchRegistry.hpp"
#include "DesignController.hpp"
#include "DesignPanels.hpp"
#include "Jobs.hpp"
#include "MainWindow.hpp"
#include "SimulateArea.hpp"
#include "SimulateCooling.hpp"
#include "Tracking.hpp"
#include "Viewport.hpp"
#include "opad/design/sketch_geom.hpp"

OPAD_BENCH(OPAD_BENCH_COOLING_WALLS, coolingWalls) {
  struct State {
    int phase = 0, ticks = 0;
    bool all = true;
    std::string tray, cover, board, chip, sink, fan, desk, hub, fin, fanUnit, sinkUnit;
    opad::Vec3 way{0, 0, 0};
    std::string sketch;
  };
  auto st = std::make_shared<State>();
  const QString prefix = value;
  auto* timer = new QTimer(&w);
  timer->setInterval(100);
  QObject::connect(timer, &QTimer::timeout, &w, [&w, st, timer, prefix] {
    AppDocument* doc = w.m_doc;
    Simulate* area = w.findChild<Simulate*>();
    CoolingAssistant* c = w.findChild<CoolingAssistant*>();
    auto require = [st](bool ok, const QString& what) {
      trace::log(QString("bench: cooling walls: %1 %2").arg(what, ok ? "PASS" : "FAIL"));
      st->all = st->all && ok;
      return ok;
    };
    auto finish = [timer, st] {
      timer->stop();
      QCoreApplication::exit(st->all ? 0 : 2);
    };
    if (++st->ticks > 900) {
      require(false, QString("timed out in phase %1").arg(st->phase));
      return finish();
    }
    if (doc->loading || doc->designBusy) return;
    auto box = [doc](const char* name, opad::Vec3 at, double l, double wd, double h, const std::string& cutFrom = {}) {
      opad::json inputs = {{"plane", {{"origin", at}, {"normal", {0, 0, 1}}}}, {"length", l}, {"width", wd}, {"height", h}, {"centered", false}};
      if (!cutFrom.empty()) inputs["operation"] = "cut", inputs["targets"] = {cutFrom};
      opad::json args = {{"kind", "box"}, {"inputs", inputs}};
      if (name) args["name"] = name;
      const opad::json r = doc->run("feature", args);
      return cutFrom.empty() && r.contains("body_ids") && !r["body_ids"].empty() ? r["body_ids"][0].get<std::string>() : cutFrom;
    };
    auto has = [](const std::vector<std::string>& v, const std::string& id) { return std::find(v.begin(), v.end(), id) != v.end(); };
    auto pick = [area](const std::vector<std::string>& ids) {  // as clicks in the view or rows of the browser select them
      std::vector<opad::Ref> refs;
      for (const auto& id : ids) {
        opad::Ref r;
        r.body = id;
        r.kind = opad::Ref::Kind::Body;
        refs.push_back(r);
      }
      area->services().select(refs);
    };
    auto body = [](const std::string& id) {
      opad::Ref r;
      r.body = id;
      return r;
    };
    auto face = [](const std::string& body, int index) {
      opad::Ref r;
      r.body = body;
      r.kind = opad::Ref::Kind::Face;
      r.index = index;
      return r;
    };
    auto row = [c](const std::string& id) {  // the heat table's row of a part
      for (int r = 0; r < c->heatTable()->rowCount(); ++r)
        if (c->heatTable()->item(r, 0)->data(Qt::UserRole).toString().toStdString() == id) return r;
      return -1;
    };
    auto pickBox = [c](const char* name) { return c->findChild<PickBox*>(name); };
    auto button = [c](const char* name) { return c->findChild<QPushButton*>(name); };
    using Pick = CoolingAssistant::Pick;
    switch (st->phase) {
      case 0: {  // the model
        if (!require(area && doc->hasDocument, "an empty document and the Simulate area")) return finish();
        st->tray = box(nullptr, {0, 0, 0}, 100, 80, 25);
        box(nullptr, {3, 3, 3}, 94, 74, 25, st->tray);
        doc->run("rename", {{"target", st->tray}, {"name", "Tray"}});
        st->cover = box(nullptr, {0, 0, 25}, 100, 80, 15);
        box(nullptr, {3, 3, 24}, 94, 74, 13, st->cover);
        doc->run("rename", {{"target", st->cover}, {"name", "Cover"}});
        st->board = box("Board", {10, 10, 8}, 60, 50, 1.6);
        st->chip = box("Chip", {30, 30, 9.6}, 10, 10, 2);
        st->sink = box("Heatsink", {28, 28, 11.6}, 14, 14, 20);
        st->fan = box("Fan", {3, 30, 5}, 10, 20, 20);
        st->desk = box("Desk", {-50, -50, -10}, 200, 180, 10);
        st->hub = box("Hub", {5, 38, 13}, 4, 4, 4);
        st->fin = box("Fin", {28, 42, 11.6}, 14, 2, 18);
        st->fanUnit = doc->run("component", {{"name", "Fan unit"}}).value("id", "");
        doc->run("reparent", {{"targets", {st->fan, st->hub}}, {"parent", st->fanUnit}});
        st->sinkUnit = doc->run("component", {{"name", "Cooler"}}).value("id", "");
        doc->run("reparent", {{"targets", {st->sink, st->fin}}, {"parent", st->sinkUnit}});
        doc->run("load", {{"kind", "heat"}, {"case", CoolingAssistant::kCase}, {"on", {st->chip}}, {"value", 3}});
        require(!st->tray.empty() && !st->cover.empty() && !st->sink.empty() && !st->desk.empty() && !st->fanUnit.empty() && !st->sinkUnit.empty(),
                "a tray, its cover, a board, a chip, a heatsink and a fin (a component), a fan block and a hub (a component), a desk");
        st->phase = 1;
        return;
      }
      case 1: {
        QAction* a = area->services().action("simulate.cooling");
        if (!require(a && a->isEnabled(), "Thermal setup is enabled")) return finish();
        a->trigger();
        st->phase = 2;
        return;
      }
      case 2: {  // nothing guessed: an empty box, the chip the case heats already, no picking until Add
        if (!c || !c->isVisible()) return;
        require(c->pickedBodies().empty() && c->wallChoice()->count() == 0, "the box's list opens empty (nothing guessed)");
        require(c->heatBodies() == std::vector<std::string>{st->chip}, "the heat table holds the chip its load already heats, nothing more");
        require(!c->picking() && !c->boxAdd()->isChecked() && c->boxAdd()->text() == CoolingAssistant::tr("Add"), "nothing picks until Add is clicked");
        require(!c->findChild<PickBox*>("coolingBoxPick") && !c->findChild<QListWidget*>("coolingInside"), "no pick box of names, no list of what is inside");
        c->boxAdd()->click();
        require(c->pickingWhat() == Pick::Box && c->boxAdd()->isChecked() && c->boxAdd()->text() == CoolingAssistant::tr("Done") &&
                    w.action("select.bodies")->isChecked(),
                "Add picks (it says Done) with the Bodies filter");
        pick({st->tray});
        st->phase = 3;
        return;
      }
      case 3: {
        if (c->pickedBodies() != std::vector<std::string>{st->tray}) return;
        pick({st->tray, st->cover});  // the cover clicked: added
        st->phase = 4;
        return;
      }
      case 4: {
        if (c->pickedBodies().size() != 2) return;
        require(c->pickedBodies() == std::vector<std::string>{st->tray, st->cover} && c->wallChoice()->count() == 2, "the tray and the cover added, two rows");
        c->grab().save(prefix + ".box.png");
        pick({st->cover});  // a click that toggled the tray out of the selection: the list only grows
        st->phase = 5;
        return;
      }
      case 5: {  // the tray kept, and selected again
        const auto sel = area->services().selection().ids;
        if (sel.size() != 2) return;
        require(c->pickedBodies() == std::vector<std::string>{st->tray, st->cover} && has(sel, st->tray), "a click never takes a body out: the tray kept, shown selected again");
        c->wallChoice()->item(1)->setSelected(true);  // the cover's row chosen
        require(button("coolingBoxRemove")->isEnabled(), "Remove enabled with a row chosen");
        button("coolingBoxRemove")->click();
        require(c->pickedBodies() == std::vector<std::string>{st->tray}, "Remove takes out the row chosen");
        pick({st->tray, st->cover});
        st->phase = 6;
        return;
      }
      case 6: {
        if (c->pickedBodies().size() != 2) return;
        c->boxAdd()->click();  // Done
        require(!c->picking() && !c->boxAdd()->isChecked() && c->boxAdd()->text() == CoolingAssistant::tr("Add"), "Done: it picks no more");
        c->open(1);
        require(!c->picking(), "the Heat page picks nothing until an Add button");
        c->partAdd()->click();
        require(c->pickingWhat() == Pick::Parts && w.action("select.bodies")->isChecked(), "Add part picks bodies");
        pick({st->chip, st->board});  // the board clicked
        st->phase = 7;
        return;
      }
      case 7: {
        if (row(st->board) < 0) return;
        require(c->heatBodies() == std::vector<std::string>{st->chip, st->board}, "the board added after the chip");
        const int b = row(st->board);
        c->heatTable()->item(b, 1)->setData(Qt::EditRole, 0.0);  // a board making no heat
        c->heatTable()->item(b, 2)->setCheckState(Qt::Checked);
        c->heatTable()->item(b, 3)->setData(Qt::EditRole, 6);
        pick({st->chip, st->board, st->desk});  // the desk added, then removed
        st->phase = 8;
        return;
      }
      case 8: {
        if (row(st->desk) < 0) return;
        c->heatTable()->clearSelection();
        c->heatTable()->selectRow(row(st->desk));
        button("coolingHeatRemove")->click();
        require(row(st->desk) < 0 && c->heatBodies().size() == 2, "Remove takes the desk's row out");
        c->faceAdd()->click();
        require(c->pickingFaces() && w.action("select.faces")->isChecked() && !c->partAdd()->isChecked(), "Add face picks faces (the Faces filter on)");
        require(c->pickedBodies().size() == 2, "the box's bodies kept through the filter's switch");
        area->services().select({face(st->board, 5)});
        st->phase = 9;
        return;
      }
      case 9: {
        if (c->heatFaces().empty()) {  // the faces' pick targets come once the filter is on (a sliced job): clicked then
          area->services().select({face(st->board, 5)});
          return;
        }
        const int r = c->heatTable()->rowCount() - 1;
        require(c->heatTable()->item(r, 0)->text().contains("Board") && !(c->heatTable()->item(r, 2)->flags() & Qt::ItemIsEnabled),
                "the face a row of the heat table, named after its body, no board tick: " + c->heatTable()->item(r, 0)->text());
        c->heatTable()->item(r, 1)->setData(Qt::EditRole, 2.5);
        w.action("select.bodies")->trigger();  // the filter switched by hand: the selection cleared, the face kept
        require(c->heatFaces().size() == 1, "the Bodies filter switched to by hand: the face kept");
        w.action("select.faces")->trigger();
        st->phase = 10;
        return;
      }
      case 10: {  // back in the Faces filter, the face is shown selected again (a moment later)
        const auto refs = area->services().selection().refs;
        if (refs.size() != 1) return;
        require(refs[0].kind == opad::Ref::Kind::Face && refs[0].body == st->board && refs[0].index == 5 && c->heatFaces().size() == 1,
                "back in the Faces filter: the face shown selected again");
        c->grab().save(prefix + ".heat.png");
        c->open(2);
        require(!c->picking() && w.action("select.bodies")->isChecked(), "the Air page picks nothing until asked; the filter as it was");
        require(c->fans().empty(), "no fan guessed");
        auto* fans = c->findChild<QRadioButton*>("coolingFans");
        auto* natural = c->findChild<QRadioButton*>("coolingNatural");
        require(fans && natural && fans->isChecked() && !natural->isChecked(), "how the air moves: Fans or Natural convection, Fans chosen");
        require(!c->apply() && c->step() == 2, "Apply with fans but none added: refused, on the Air page");
        for (auto* b : c->findChildren<QPushButton*>())
          if (b->text() == CoolingAssistant::tr("Add a fan")) b->click();
        require(c->pickingWhat() == Pick::Outlet && c->fans().size() == 1 && w.action("select.faces")->isChecked(),
                "Add a fan: a fan, picking its opening (faces and the sketches' profiles)");
        require(pickBox("coolingInletPick") && !pickBox("coolingInletPick")->isVisibleTo(c), "an axial fan has no inlet to draw");
        st->phase = 11;
        return;
      }
      case 11: {  // its opening: a face of the fan block, once the faces are pickable; its normal read on a worker
        if (c->fans()[0].outlet.is_null()) {
          area->services().select({face(st->fan, 0)});
          return;
        }
        const auto& spec = c->fans()[0];
        if (std::hypot(spec.normal[0], spec.normal[1], spec.normal[2]) < 0.5 || spec.outletLoops.empty()) return;
        require(spec.outlet == face(st->fan, 0).to_json() && spec.on.empty(), "the opening is the face clicked, no body stands for the fan");
        const opad::Vec3 v = spec.vector();
        st->way = v;
        require(std::fabs(std::fabs(v[0]) + std::fabs(v[1]) + std::fabs(v[2]) - 1) < 1e-6, QString("the air goes across the face: along an axis of the block (%1, %2, %3)").arg(v[0]).arg(v[1]).arg(v[2]));
        require(area->fanGlyphsShown() == 1, "the opening's outline and the air's way drawn in the view");
        w.m_viewport->fitAll();
        w.m_viewport->grabImage().save(prefix + ".outlet.png");
        c->findChild<QPushButton*>("coolingWayFlip")->click();
        const opad::Vec3 f = c->fans()[0].vector();
        require(f[0] == -v[0] && f[1] == -v[1] && f[2] == -v[2], "Flip turns the way round");
        // Its datasheet: max airflow in CFM and max static pressure in inH2O (the defaults), two points off its chart, 80 % speed;
        // the pressure then shown in Pa, the same fan.
        auto* model = c->findChild<QComboBox*>("coolingFanModel");
        auto* flow = c->findChild<QDoubleSpinBox*>("coolingFanFlow");
        model->setCurrentIndex(model->count() - 1);  // From its datasheet
        require(flow->isEnabled() && c->findChild<QComboBox*>("coolingFanFlowUnit")->currentData() == "cfm" &&
                    c->findChild<QComboBox*>("coolingFanPressureUnit")->currentData() == "inH2O",
                "From its datasheet: its max airflow (CFM) and max static pressure (inH2O) to type");
        flow->setValue(5);
        c->findChild<QDoubleSpinBox*>("coolingFanPressure")->setValue(0.2);
        auto* curve = c->findChild<QLineEdit*>("coolingFanCurve");
        curve->setText("2 0.15; 4 0.06");
        emit curve->editingFinished();
        c->findChild<QDoubleSpinBox*>("coolingFanSpeed")->setValue(80);
        auto* pressureUnit = c->findChild<QComboBox*>("coolingFanPressureUnit");
        pressureUnit->setCurrentIndex(pressureUnit->findData("Pa"));
        const auto& g = c->fans()[0];
        require(std::fabs(g.pressure - 0.2 * 249.08891) < 1e-6 && g.curve.size() == 2 && std::fabs(g.curve[0][1] - 0.15 * 249.08891) < 1e-6 && g.flow == 5,
                QString("the pressure in Pa: %1 Pa, its chart's points converted too").arg(g.pressure));
        pickBox("coolingSinkPick")->click();
        require(c->pickingWhat() == Pick::Sink && w.action("select.bodies")->isChecked(), "Heatsink clicked: it picks bodies or a component");
        st->phase = 12;
        return;
      }
      case 12: {
        if (c->fans()[0].sink != std::vector<std::string>{st->sinkUnit}) {
          area->services().select({body(st->sinkUnit)});
          return;
        }
        auto* material = c->findChild<QComboBox*>("coolingSinkMaterial");
        material->setCurrentIndex(material->findData("copper"));
        require(c->fans()[0].material == "copper", "copper chosen for the whole heatsink");
        // A blower: its outlet a sketch's profile clicked in the view (a circle sketched above the cover), its inlet missing.
        opad::design::Sketch slot;
        slot.add_circle(slot.add_point(0, 0), 12);
        st->sketch = doc->run("sketch", {{"plane", {{"origin", {80, 40, 45}}, {"normal", {0, 0, 1}}, {"x", {1, 0, 0}}}}, {"geometry", slot.to_json()}, {"name", "Blower outlet"}})
                         .value("sketch_id", std::string());
        for (auto* b : c->findChildren<QPushButton*>())
          if (b->text() == CoolingAssistant::tr("Add a fan")) b->click();
        auto* type = c->findChild<QComboBox*>("coolingFanType");
        type->setCurrentIndex(type->findData("blower"));
        require(c->fans().size() == 2 && c->fans()[1].type == "blower" && pickBox("coolingInletPick")->isVisibleTo(c) && c->pickingWhat() == Pick::Outlet,
                "a second fan, a blower: its outlet and its inlet to draw");
        w.m_viewport->standardView("top");
        w.m_viewport->fitAll();
        st->phase = 13;
        return;
      }
      case 13: {  // the sketch's region offered among the candidates, clicked in the view
        if (c->fans()[1].outlet.is_null()) {
          int x = 0, y = 0;
          if (w.m_viewport->benchPickPoint([](const std::string& cand, const opad::Ref&) { return cand.find("\"sketch\"") != std::string::npos; }, x, y, false))
            w.m_viewport->benchClickAt(x, y);
          return;
        }
        require(c->fans()[1].outlet.value("sketch", std::string()) == st->sketch, "the blower's outlet: the sketch's profile clicked");
        require(!c->apply() && c->step() == 2 && c->currentFan() == 1, "Apply with a blower without its inlet: refused, on that fan");
        // Draw… starts New sketch (its plane picked first); Esc there comes back here.
        c->findChild<QPushButton*>("coolingInletDraw")->click();
        require(w.m_design->pickingPlane(), "Draw… asks for the sketch's plane (the blower's face)");
        w.m_design->escape();
        require(c->isVisible() && c->step() == 2 && !w.m_design->pickingPlane(), "Esc: back on the Air page");
        trace::log(QString("bench: cooling walls: back from Draw…: %1 fan(s), fan %2 current").arg(c->fans().size()).arg(c->currentFan() + 1));
        c->fanList()->setCurrentRow(1);
        button("coolingFanRemove")->click();
        require(c->fans().size() == 1, "the blower removed");
        c->grab().save(prefix + ".air.png");
        require(!c->findChild<QLabel*>("coolingVents")->text().isEmpty(), "the vents' estimate said: " + c->findChild<QLabel*>("coolingVents")->text());
        c->open(3);
        require(!c->picking() && w.action("select.bodies")->isChecked() && !c->findChild<QPushButton*>("coolingNext")->isVisible(),
                "the Run page picks nothing, the view's filter as it was; no Next on the last page");
        require(c->apply(), "Apply writes the case");
        st->phase = 14;
        return;
      }
      case 14: {  // what Apply wrote; opened again, the same
        const opad::Study* study = nullptr;
        for (const auto& s : doc->scene.studies)
          if (s.name == CoolingAssistant::studyName().toStdString()) study = &s;
        const opad::json settings = study ? study->def.value("settings", opad::json::object()) : opad::json::object();
        require(settings.value("cfd", opad::json::object()).value("enclosure", opad::json()) == opad::json::array({st->tray, st->cover}),
                "the study's enclosure is the tray and the cover");
        int onFace = 0, onChip = 0, onBoard = 0;
        for (const auto& l : doc->scene.loads)
          if (l.kind == "heat" && l.load_case == CoolingAssistant::kCase && l.refs.size() == 1) {
            const auto& r = l.refs.front();
            onFace += r.kind == opad::Ref::Kind::Face && r.body == st->board && r.index == 5 && l.def.value("value", 0.0) == 2.5;
            onChip += r.kind == opad::Ref::Kind::Body && r.body == st->chip && l.def.value("value", 0.0) == 3;
            onBoard += r.kind == opad::Ref::Kind::Body && r.body == st->board;
          }
        require(onFace == 1 && onChip == 1 && onBoard == 0, QString("heat loads: 2.5 W on the board's face, 3 W in the chip, none in the board (%1, %2, %3)").arg(onFace).arg(onChip).arg(onBoard));
        const opad::json mats = settings.value("materials", opad::json::object());
        require(mats.value(st->board, opad::json()).value("pcb", opad::json()).value("layers", 0) == 6, "the board a PCB of 6 copper layers");
        require(mats.value(st->sink, opad::json()).value("material", "") == "copper" && mats.value(st->fin, opad::json()).value("material", "") == "copper",
                "the heatsink's two bodies both copper");
        const opad::Load* fan = nullptr;
        for (const auto& l : doc->scene.loads)
          if (l.kind == "fan" && l.load_case == CoolingAssistant::kCase) fan = &l;
        const opad::json spec = fan ? fan->def.value("fan", opad::json()) : opad::json();
        require(fan && fan->refs.empty() && fan->def.value("outlet", opad::json()) == face(st->fan, 0).to_json() && fan->def.value("type", "") == "axial" &&
                    fan->def.value("heatsink", opad::json()) == opad::json::array({st->sinkUnit}) &&
                    fan->def.value("vector", opad::Vec3{}) == opad::Vec3{-st->way[0], -st->way[1], -st->way[2]} &&
                    !settings.value("cfd", opad::json::object()).value("buoyancy", true),
                "the fan load: by its opening, through the heatsink, the way flipped; no buoyancy");
        require(spec.value("cfm", 0.0) == 5 && std::fabs(spec.value("pressure", 0.0) - 0.2 * 249.08891) < 1e-6 && spec.value("speed", 0.0) == 80 &&
                    spec.value("curve", opad::json::array()).size() == 4,
                "its datasheet: 5 CFM, 0.2 inH2O as Pa, its chart's two points between the ends, at 80 %: " + QString::fromStdString(spec.dump()));
        c->dismiss();
        require(!c->picking(), "closed: the view selects again");
        c->open(1);
        require(c->pickedBodies() == std::vector<std::string>{st->tray, st->cover}, "opened again: the tray and the cover");
        const int b = row(st->board);
        require(c->heatBodies() == std::vector<std::string>{st->chip, st->board} && b >= 0 && c->heatTable()->item(b, 2)->checkState() == Qt::Checked &&
                    c->heatTable()->item(b, 3)->data(Qt::EditRole).toInt() == 6 && c->heatFaces().size() == 1,
                "opened again: the chip, the board (a PCB of 6 layers) and the face");
        const auto& f = c->fans();
        require(f.size() == 1 && f[0].outlet == face(st->fan, 0).to_json() && f[0].sink == std::vector<std::string>{st->sinkUnit} && f[0].material == "copper" &&
                    f[0].flowUnit == "cfm" && f[0].pressureUnit == "Pa" && f[0].curve.size() == 2 && f[0].speed == 80,
                "opened again: the fan as set, in the units it was given in");
        // Fans and the warm air rising: the fan's load kept, buoyancy on; the estimate says it takes longer.
        c->open(2);
        auto* estimate = c->findChild<QLabel*>("coolingEstimate");
        const QString fansOnly = estimate ? estimate->text() : QString();
        c->findChild<QRadioButton*>("coolingMixed")->setChecked(true);
        require(c->fanList()->isEnabled() && estimate && estimate->text() != fansOnly,
                "Fans and warm air rising: the fans still set, the estimate longer: " + (estimate ? estimate->text() : QString()));
        require(c->apply(), "Apply with fans and the warm air rising");
        st->phase = 30;
        return;
      }
      case 30: {
        int fans = 0;
        opad::json cfd;
        for (const auto& l : doc->scene.loads) fans += l.kind == "fan" && l.load_case == CoolingAssistant::kCase;
        for (const auto& s : doc->scene.studies)
          if (s.name == CoolingAssistant::studyName().toStdString()) cfd = s.def.value("settings", opad::json::object()).value("cfd", opad::json::object());
        require(fans == 1 && cfd.value("buoyancy", false), "Fans and warm air rising written: the fan's load and buoyancy together");
        require(cfd.value("quality", "") == "quick", "a new study runs at Quick (a first answer in minutes): " + QString::fromStdString(cfd.value("quality", "")));
        c->dismiss();
        c->open(2);
        require(c->findChild<QRadioButton*>("coolingMixed")->isChecked(), "opened again: Fans and warm air rising chosen");
        // Natural convection: no fans, the air rising
        c->findChild<QRadioButton*>("coolingNatural")->setChecked(true);
        require(!c->fanList()->isEnabled() && c->naturalConvection(), "Natural convection: the fans set aside");
        require(c->apply(), "Apply with natural convection");
        st->phase = 15;
        return;
      }
      case 15: {
        int fans = 0;
        for (const auto& l : doc->scene.loads) fans += l.kind == "fan" && l.load_case == CoolingAssistant::kCase;
        bool buoyant = false;
        for (const auto& s : doc->scene.studies)
          if (s.name == CoolingAssistant::studyName().toStdString())
            buoyant = s.def.value("settings", opad::json::object()).value("cfd", opad::json::object()).value("buoyancy", false);
        require(fans == 0 && buoyant, "natural convection written: no fan load, the air rising (buoyancy)");
        c->dismiss();
        c->open(2);
        require(c->findChild<QRadioButton*>("coolingNatural")->isChecked(), "opened again: Natural convection chosen");
        c->dismiss();
        require(w.action("select.bodies")->isChecked(), "closed: the Bodies filter as before");
        return finish();
      }
    }
  });
  timer->start();
  return true;
}

// OPAD_BENCH_COOLING_PERF=<prefix>, meant for the Engine (or any big model): once its bodies are on screen (unhidden in memory,
// never saved), Thermal setup opened from its command: the command returns at once and nothing stalls the UI over 250 ms; a
// body added takes a few milliseconds. (It once measured every body's shape per candidate box on the UI thread: a minute.)
OPAD_BENCH(OPAD_BENCH_COOLING_PERF, coolingPerf) {
  struct State {
    int phase = 0, ticks = 0, settled = 0;
    bool all = true;
    QElapsedTimer clock;
  };
  auto st = std::make_shared<State>();
  auto* timer = new QTimer(&w);
  timer->setInterval(50);
  QObject::connect(timer, &QTimer::timeout, &w, [&w, st, timer] {
    AppDocument* doc = w.m_doc;
    Simulate* area = w.findChild<Simulate*>();
    CoolingAssistant* c = w.findChild<CoolingAssistant*>();
    auto require = [st](bool ok, const QString& what) {
      trace::log(QString("bench: cooling perf: %1 %2").arg(what, ok ? "PASS" : "FAIL"));
      st->all = st->all && ok;
      return ok;
    };
    auto finish = [timer, st] {
      timer->stop();
      QCoreApplication::exit(st->all ? 0 : 2);
    };
    if (++st->ticks > 12000) {
      require(false, QString("timed out in phase %1").arg(st->phase));
      return finish();
    }
    const bool busy = doc->loading || doc->designBusy || w.m_displayJob || w.m_meshRemaining > 0;
    switch (st->phase) {
      case 0: {  // its bodies on screen
        if (busy) return;
        const auto bodies = doc->scene.all_bodies();
        if (std::none_of(bodies.begin(), bodies.end(), [doc](const std::string& id) { return doc->scene.effectively_visible(id); })) {
          trace::log("bench: cooling perf: nothing visible, unhiding (in memory, never saved)");
          std::vector<std::string> hidden;
          for (const auto& [id, node] : doc->scene.nodes)
            if (!node.visible) hidden.push_back(id);
          for (const auto& id : hidden) doc->run("appearance", {{"target", id}, {"visible", true}});
          return;
        }
        if (++st->settled < 20 || w.m_jobs->busy()) return;
        trace::resetStalls();
        QElapsedTimer t;
        t.start();
        area->services().action("simulate.cooling")->trigger();
        const qint64 call = t.elapsed();
        require(call < 250, QString("the command returns in %1 ms with %2 bodies").arg(call).arg(doc->scene.all_bodies().size()));
        st->clock.start();
        st->phase = 1;
        return;
      }
      case 1: {  // opened at once, the UI never held; a body added and removed quickly
        if (!c || !c->isVisible()) return;
        const trace::Stalls stalls = trace::stalls();
        require(stalls.longest < 250, QString("opened in %1 ms, the longest stall meanwhile %2 ms").arg(st->clock.elapsed()).arg(stalls.longest));
        c->boxAdd()->click();
        require(c->picking() && c->pickedBodies().empty(), "the box's list opens empty and Add picks");
        std::string first;
        for (const auto& id : doc->scene.all_bodies())
          if (doc->scene.effectively_visible(id)) {
            first = id;
            break;
          }
        QElapsedTimer t;
        t.start();
        c->selectionChanged({first}, {});
        require(t.elapsed() < 250 && c->pickedBodies() == std::vector<std::string>{first}, QString("a body added in %1 ms").arg(t.elapsed()));
        c->dismiss();
        return finish();
      }
    }
  });
  timer->start();
  return true;
}
