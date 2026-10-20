// Thermal setup writes as it goes, and the Simulation folder's rows light what they stand for; case in
// tools/bench_cases/simulate.py.
#include <QAction>
#include <QCoreApplication>
#include <QPainter>
#include <QPushButton>
#include <QRadioButton>
#include <QTableWidget>
#include <QTimer>

#include <memory>

#include "AppDocument.hpp"
#include "BenchRegistry.hpp"
#include "BrowserPanel.hpp"
#include "MainWindow.hpp"
#include "SimulateArea.hpp"
#include "SimulateCooling.hpp"
#include "Tracking.hpp"
#include "Viewport.hpp"

// OPAD_BENCH_SIMLIT=<prefix> on an empty document (a box, a board, a chip, a heatsink and a fan block built here): Thermal setup
// driven through its Add buttons, each addition a row of the browser's Simulation folder a moment later, as the manual
// commands' are (the box, the chip's heat source, the board, the fan, its heatsink, the air); a power typed changes the same
// load in place; natural convection takes the fan's load out. Then each row picked in the browser lights its target by its
// nature: the heat source red with its flame badge, the fan, the heatsink, the board, the box; a hidden chip is not lit;
// nothing picked, nothing lit. Del on a row deletes it: a heat source (Undo brings it back), the Board row (the board no PCB),
// the whole case, the study. In Thermal setup itself, a row chosen in a list lights which part or face it is: the heat
// table's chip (red), board, and a face of the board (that face alone); the box's row; the current fan with its heatsink.
// Frames: <prefix>.heat.png, .case.png, .face-row.png (the view with its badges).
OPAD_BENCH(OPAD_BENCH_SIMLIT, simlit) {
  struct State {
    int phase = 0, ticks = 0, wait = 0;
    bool all = true;
    std::string box, board, chip, sink, fan, study, heat, fanLoad;
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
      trace::log(QString("bench: simulate lit: %1 %2").arg(what, ok ? "PASS" : "FAIL"));
      st->all = st->all && ok;
      return ok;
    };
    auto finish = [timer, st] {
      timer->stop();
      QCoreApplication::exit(st->all ? 0 : 2);
    };
    if (++st->ticks > 600) {
      require(false, QString("timed out in phase %1").arg(st->phase));
      return finish();
    }
    if (doc->loading || doc->designBusy) return;
    auto make = [doc](const char* name, opad::Vec3 at, double l, double wd, double h) {
      const opad::json r = doc->run("feature", {{"kind", "box"}, {"name", name},
                                                {"inputs", {{"plane", {{"origin", at}, {"normal", {0, 0, 1}}}}, {"length", l}, {"width", wd}, {"height", h}, {"centered", false}}}});
      return r["body_ids"][0].get<std::string>();
    };
    auto pick = [area](const std::vector<std::string>& ids) {
      std::vector<opad::Ref> refs;
      for (const auto& id : ids) {
        opad::Ref r;
        r.body = id;
        refs.push_back(r);
      }
      area->services().select(refs);
    };
    auto row = [&w](const std::string& id) { return w.m_browser->isProvided(id); };
    auto caseLoad = [doc](const char* kind) -> const opad::Load* {
      for (const auto& l : doc->scene.loads)
        if (l.kind == kind && l.load_case == CoolingAssistant::kCase) return &l;
      return nullptr;
    };
    auto studyId = [doc] {
      for (const auto& s : doc->scene.studies)
        if (QString::fromStdString(s.name) == CoolingAssistant::studyName()) return s.id;
      return std::string();
    };
    auto lights = [area](const std::string& rowId, const std::string& nature, const std::string& body) {
      for (const auto& l : area->lit())
        if (l.row == rowId && l.nature == nature && l.target.body == body && l.shown) return true;
      return false;
    };
    auto badges = [&w] {
      int n = 0;
      for (QWidget* b : w.findChildren<QWidget*>("simBadge")) n += b->isVisible();
      return n;
    };
    auto shot = [&w, prefix](const char* name) {  // the view, its badges painted where they float
      QImage image = w.m_viewport->grabImage();
      if (image.isNull()) return;
      QPainter p(&image);
      const qreal dpr = image.width() / qreal(std::max(1, w.m_viewport->width()));
      p.scale(dpr, dpr);
      for (QWidget* b : w.findChildren<QWidget*>("simBadge"))
        if (b->isVisible()) p.drawPixmap(b->pos(), b->grab());
      p.end();
      image.save(prefix + "." + name + ".png");
    };
    switch (st->phase) {
      case 0: {
        st->box = make("Box", {0, 0, 0}, 100, 80, 40);
        st->board = make("Board", {10, 10, 50}, 60, 50, 1.6);
        st->chip = make("Chip", {30, 30, 51.6}, 10, 10, 2);
        st->sink = make("Heatsink", {28, 28, 53.6}, 14, 14, 10);
        st->fan = make("Fan", {120, 30, 50}, 10, 20, 20);
        w.action("simulate.cooling")->trigger();
        st->phase = 1;
        return;
      }
      case 1: {
        if (!c || !c->isVisible()) return;
        c->boxAdd()->click();
        pick({st->box});
        st->phase = 2;
        return;
      }
      case 2: {  // the box: the study, its box row (once written, a moment later)
        st->study = studyId();
        if (st->study.empty() || !row("sim:x:" + st->study + ":box")) return;
        require(row("sim:s:" + st->study), "the box added: the study and its Box row are in the browser at once");
        c->boxAdd()->click();
        c->open(1);
        c->partAdd()->click();
        pick({st->chip});
        st->phase = 3;
        return;
      }
      case 3: {
        const opad::Load* l = caseLoad("heat");
        if (!l || !row("sim:l:" + l->id)) return;
        st->heat = l->id;
        require(l->refs.size() == 1 && l->refs[0].body == st->chip, "the chip added: its heat source is a row of the case: " + QString::fromStdString(l->name));
        for (int r = 0; r < c->heatTable()->rowCount(); ++r)
          if (c->heatTable()->item(r, 0)->data(Qt::UserRole).toString().toStdString() == st->chip) c->heatTable()->item(r, 1)->setData(Qt::EditRole, 3.0);
        pick({st->chip, st->board});
        st->phase = 4;
        return;
      }
      case 4: {
        const opad::Load* l = caseLoad("heat");
        int boardRow = -1;
        for (int r = 0; r < c->heatTable()->rowCount(); ++r)
          if (c->heatTable()->item(r, 0)->data(Qt::UserRole).toString().toStdString() == st->board) boardRow = r;
        if (!l || l->def.value("value", 0.0) != 3.0 || boardRow < 0) return;
        require(l->id == st->heat, "the power typed: the same load changed in place (3 W)");
        c->heatTable()->item(boardRow, 1)->setData(Qt::EditRole, 0.0);
        c->heatTable()->item(boardRow, 2)->setCheckState(Qt::Checked);
        st->phase = 5;
        return;
      }
      case 5: {
        if (!row("sim:x:" + st->study + ":board:" + st->board)) return;
        int heats = 0;
        for (const auto& l : doc->scene.loads) heats += l.kind == "heat" && l.load_case == CoolingAssistant::kCase;
        require(heats == 1, "the board ticked: its Board row, and no heat source for it (0 W)");
        c->partAdd()->click();  // Done
        // Choosing a row of the heat table lights which part it is.
        auto chooseRow = [c](const std::string& id) {
          for (int r = 0; r < c->heatTable()->rowCount(); ++r)
            if (c->heatTable()->item(r, 0)->data(Qt::UserRole).toString().toStdString() == id) c->heatTable()->selectRow(r);
        };
        c->heatTable()->clearSelection();
        chooseRow(st->chip);
        require(lights("setup", "heat", st->chip) && area->lit().size() == 1 && badges() == 1, "the chip's row chosen in the heat table: the chip lit red, with its badge");
        c->heatTable()->clearSelection();
        chooseRow(st->board);
        require(lights("setup", "board", st->board) && area->lit().size() == 1, "the board's row chosen: the board lit as one");
        c->heatTable()->clearSelection();
        require(area->lit().empty() && badges() == 0, "no row chosen: nothing lit");
        c->faceAdd()->click();
        st->phase = 50;
        return;
      }
      case 50: {  // a face of the board added (its targets come once the Faces filter is applied)
        if (c->heatFaces().empty()) {
          opad::Ref f;
          f.body = st->board;
          f.kind = opad::Ref::Kind::Face;
          f.index = 5;
          area->services().select({f});
          return;
        }
        c->faceAdd()->click();  // Done
        const int last = c->heatTable()->rowCount() - 1;
        c->heatTable()->clearSelection();
        c->heatTable()->selectRow(last);
        bool face = false;
        for (const auto& l : area->lit()) face = face || (l.nature == "heat" && l.target.kind == opad::Ref::Kind::Face && l.target.body == st->board && l.shown);
        require(face && area->lit().size() == 1, "the face's row chosen: that face of the board lit (not the whole board), whatever the filter");
        st->phase = 51;
        return;
      }
      case 51: {  // Done: nothing stays selected over it (a selection's blue would turn the heat source's red purple)
        if (!area->services().selection().refs.empty() && ++st->wait < 30) return;
        require(area->services().selection().refs.empty(), QString("Done: the picks no longer selected, the face lit in its own red (%1 selected)").arg(area->services().selection().refs.size()));
        shot("face-row");
        c->open(0);
        c->wallChoice()->item(0)->setSelected(true);
        require(lights("setup", "box", st->box), "the box's row chosen: the box lit");
        c->open(2);
        for (auto* b : c->findChildren<QPushButton*>())
          if (b->text() == CoolingAssistant::tr("Add a fan")) b->click();
        st->phase = 6;
        return;
      }
      case 6: {
        if (c->fans().empty()) return;
        if (c->fans()[0].outlet.is_null()) {  // its opening: a face of the fan block, once the faces' pick targets are built
          opad::Ref r;
          r.body = st->fan, r.kind = opad::Ref::Kind::Face, r.index = 0;
          area->services().select({r});
          return;
        }
        c->findChild<PickBox*>("coolingSinkPick")->click();
        st->phase = 61;
        return;
      }
      case 61: {  // the heatsink, once the Bodies filter is back from Faces (a sliced job)
        if (c->fans()[0].sink.empty()) {
          pick({st->sink});
          return;
        }
        st->phase = 7;
        return;
      }
      case 7: {
        const opad::Load* f = caseLoad("fan");
        if (!f || !row("sim:x:" + st->study + ":sink:" + st->sink)) return;
        st->fanLoad = f->id;
        require(row("sim:l:" + f->id) && row("sim:x:" + st->study + ":air"), "the fan and its heatsink: the fan's load, the Heatsink and Air rows");
        require(lights("setup", "fan", st->fan) && lights("setup", "heatsink", st->sink), "the Air page: the current fan lit as a fan, its heatsink as one");
        c->dismiss();
        w.m_viewport->fitAll();  // the whole model (view.fit fits a selection while there is one)
        w.m_browser->selectIds({"sim:l:" + st->heat});
        st->phase = 8;
        return;
      }
      case 8: {  // the rows light their targets
        require(lights("sim:l:" + st->heat, "heat", st->chip) && area->lit().size() == 1 && badges() == 1, "the heat source's row: the chip lit, with its badge");
        require(Simulate::natureColor("heat") == QColor("#e5484d") && Simulate::natureIcon("heat") == "simHeat", "a heat source is red, its badge a flame");
        shot("heat");
        w.m_browser->selectIds({"sim:l:" + st->fanLoad});
        require(lights("sim:l:" + st->fanLoad, "fan", st->fan) && badges() == 1, "the fan's row: the fan block lit as a fan");
        w.m_browser->selectIds({"sim:x:" + st->study + ":sink:" + st->sink});
        require(lights("sim:x:" + st->study + ":sink:" + st->sink, "heatsink", st->sink), "the Heatsink row: the heatsink lit as one");
        w.m_browser->selectIds({"sim:x:" + st->study + ":board:" + st->board});
        require(lights("sim:x:" + st->study + ":board:" + st->board, "board", st->board), "the Board row: the board lit as one");
        w.m_browser->selectIds({"sim:x:" + st->study + ":box"});
        require(lights("sim:x:" + st->study + ":box", "box", st->box), "the Box row: the box lit");
        w.m_browser->selectIds({"sim:c:" + std::string(CoolingAssistant::kCase)});
        require(lights("sim:c:Cooling", "heat", st->chip) && lights("sim:c:Cooling", "fan", st->fan) && badges() == int(area->lit().size()), "the case's row: each of its loads lit by its nature, each badged");
        shot("case");
        doc->run("appearance", {{"target", st->chip}, {"visible", false}});
        st->phase = 9;
        return;
      }
      case 9: {
        w.m_browser->selectIds({"sim:l:" + st->heat});
        require(!area->lit().empty() && !area->lit()[0].shown && badges() == 0, "the chip hidden: its heat source lights nothing, no badge");
        doc->run("appearance", {{"target", st->chip}, {"visible", true}});
        w.m_browser->selectIds({});
        require(area->lit().empty() && badges() == 0, "nothing picked: nothing lit");
        c->open(2);
        c->findChild<QRadioButton*>("coolingNatural")->setChecked(true);
        st->phase = 10;
        return;
      }
      case 10: {
        if (caseLoad("fan")) return;
        require(row("sim:x:" + st->study + ":air") && !row("sim:l:" + st->fanLoad), "natural convection: the fan's load gone from the browser, the Air row kept");
        c->dismiss();
        // Del on the folder's rows deletes what they stand for (it once only lit them).
        w.m_browser->selectIds({"sim:l:" + st->heat});
        w.action("edit.delete")->trigger();
        require(!doc->scene.load(st->heat) && !row("sim:l:" + st->heat), "Del on the chip's heat source row: the load and its row gone");
        w.action("edit.undo")->trigger();
        require(doc->scene.load(st->heat) && row("sim:l:" + st->heat), "Undo: the heat source back");
        w.m_browser->selectIds({"sim:x:" + st->study + ":board:" + st->board});
        w.action("edit.delete")->trigger();
        const opad::Study* s = doc->scene.study(st->study);
        require(s && !row("sim:x:" + st->study + ":board:" + st->board) &&
                    !s->def["settings"].value("materials", opad::json::object()).value(st->board, opad::json::object()).contains("pcb"),
                "Del on the Board row: the board is no PCB any more, the study kept");
        w.m_browser->selectIds({"sim:c:" + std::string(CoolingAssistant::kCase)});
        w.action("edit.delete")->trigger();
        int left = 0;
        for (const auto& l : doc->scene.loads) left += l.load_case == CoolingAssistant::kCase;
        require(left == 0 && !row("sim:c:Cooling") && row("sim:s:" + st->study), "Del on the case's row: every load of it gone, the study kept");
        w.m_browser->selectIds({"sim:s:" + st->study});
        w.action("edit.delete")->trigger();
        require(!doc->scene.study(st->study) && !row("sim:s:" + st->study), "Del on the study's row: the study gone");
        return finish();
      }
    }
  });
  timer->start();
  return true;
}
