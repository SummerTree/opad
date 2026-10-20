// The Simulation folder's rows in the browser: a thermal study's setup (its box, boards, heatsinks and air) as rows of their own,
// and what a picked row stands for lit in the view in the colour of its nature, with its badge (a heat source red with a
// flame, a fan blue, a heatsink steel, a board green, the box sand, a support purple, a force orange). A hidden part is not lit.
#include <QBitmap>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>

#include <map>
#include <set>

#include "AppDocument.hpp"
#include "I18n.hpp"
#include "Icons.hpp"
#include "Jobs.hpp"
#include "SimulateArea.hpp"
#include "Tracking.hpp"
#include "Viewport.hpp"
#include "opad/materials.hpp"

namespace {
using opad::json;

// A round native child over the view (it cannot be translucent), the nature's icon in white on its colour.
class Badge : public QFrame {
 public:
  Badge(QWidget* parent, const QColor& color, const QString& icon) : QFrame(parent) {
    setObjectName("simBadge");
    setAttribute(Qt::WA_NativeWindow);
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setFixedSize(24, 24);
    setStyleSheet(QString("#simBadge { background: %1; border: none; }").arg(color.name()));
    auto* layout = new QHBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);
    auto* label = new QLabel(this);
    label->setPixmap(icons::pixmap(icon, Qt::white, 16, devicePixelRatioF()));
    layout->addWidget(label);
    QBitmap mask(size());
    mask.fill(Qt::color0);
    QPainter p(&mask);
    p.setPen(Qt::NoPen);
    p.setBrush(Qt::color1);
    p.drawEllipse(QRectF(0, 0, width(), height()));
    p.end();
    setMask(mask);
  }
};

constexpr size_t kMaxBadges = 40;  // a heat source on hundreds of parts: every one lit, the first ones badged
}  // namespace

QColor Simulate::natureColor(const std::string& nature) {
  if (nature == "heat") return QColor("#e5484d");
  if (nature == "temperature") return QColor("#d6409f");
  if (nature == "convection" || nature == "natural") return QColor("#29a3c2");
  if (nature == "radiation") return QColor("#f5a623");
  if (nature == "fan") return QColor("#3e63dd");
  if (nature == "heatsink") return QColor("#7c8a9e");
  if (nature == "board") return QColor("#30a46c");
  if (nature == "box") return QColor("#b8956a");
  if (nature == "fixed" || nature == "displacement") return QColor("#8e4ec6");
  return QColor("#f76b15");  // force, pressure, moment, gravity, a bolt's preload
}

QString Simulate::natureIcon(const std::string& nature) {
  if (nature == "heat") return "simHeat";
  if (nature == "temperature") return "simTemperature";
  if (nature == "convection") return "simConvection";
  if (nature == "natural") return "simNatural";
  if (nature == "radiation") return "simRadiation";
  if (nature == "fan") return "simFan";
  if (nature == "heatsink") return "simHeatsink";
  if (nature == "board") return "simBoard";
  if (nature == "box") return "simBox";
  if (nature == "fixed" || nature == "displacement") return "simFixed";
  if (nature == "pressure") return "simPressure";
  if (nature == "bolt_preload") return "simBolt";
  if (nature == "gravity") return "simGravity";
  return "simForce";
}

std::vector<browser::Item> Simulate::setupRows(const opad::Study& st) const {
  std::vector<browser::Item> rows;
  const AppDocument* doc = services().document();
  const opad::Scene& s = doc->scene;
  const json set = st.def.value("settings", json::object());
  if (set.value("air", std::string()) != "cfd") return rows;
  const json cfd = set.value("cfd", json::object());
  const json mats = set.value("materials", json::object());
  const std::string prefix = "sim:x:" + st.id + ":";
  const std::string loadCase = set.value("case", std::string());
  auto name = [&](const std::string& id) { return s.node(id) ? doc->nodeName(id) : QString::fromStdString(id); };
  const json e = cfd.value("enclosure", json());
  QStringList walls;
  for (const auto& w : e.is_array() ? e : json::array({e}))
    if (w.is_string()) walls << name(w.get<std::string>());
  if (!walls.isEmpty())
    rows.push_back({prefix + "box", tr("Box: %1").arg(walls.join(" + ")), "simBox", tr("The box the air is solved in, with a margin of the room around it: its holes are the vents"), {}});
  for (const auto& [id, m] : mats.items())
    if (m.is_object() && m.contains("pcb"))
      rows.push_back({prefix + "board:" + id, tr("Board: %1 · %n copper layer(s)", "", m["pcb"].value("layers", 4)).arg(name(id)), "simBoard",
                      tr("A printed circuit board: its copper spreads the heat along it far better than through it"), {}});
  int fans = 0;
  std::set<std::string> sinks;
  for (const auto& l : s.loads) {
    if (l.load_case != loadCase || l.kind != "fan") continue;
    ++fans;
    for (const auto& h : l.def.value("heatsink", json::array())) {
      if (!h.is_string() || !sinks.insert(h.get<std::string>()).second) continue;
      const std::string id = h.get<std::string>();
      const opad::Node* n = s.node(id);
      const std::string first = n && n->kind != opad::Node::Kind::Body && !s.bodies_under(id).empty() ? s.bodies_under(id).front() : id;
      QString text = tr("Heatsink: %1").arg(name(id));
      if (const opad::Material* m = opad::material(mats.value(first, json::object()).value("material", std::string()))) text += " · " + i18n::t(QString::fromStdString(m->name));
      rows.push_back({prefix + "sink:" + id, text, "simHeatsink", tr("What a fan blows through: its fins get fine cells"), {}});
    }
  }
  const bool natural = cfd.value("buoyancy", false) && fans == 0;
  rows.push_back({prefix + "air", natural ? tr("Air: natural convection") : tr("Air: moved by %n fan(s)", "", fans), natural ? "simNatural" : "simFan",
                  natural ? tr("No fans: the air rises where the parts warm it and leaves through the vents") : tr("The fans' loads are in the case's folder"), {}});
  return rows;
}

void Simulate::addTarget(std::vector<Lit>& out, const std::string& row, const opad::Ref& r, const std::string& nature) const {
  if (r.body.empty()) return;
  const opad::Scene& s = services().document()->scene;
  const opad::Node* n = s.node(r.body);
  if (n && n->kind != opad::Node::Kind::Body) {  // a component (a fan's own model): each of its bodies
    for (const auto& b : s.bodies_under(r.body)) {
      opad::Ref body;
      body.body = b;
      out.push_back({row, nature, body});
    }
    return;
  }
  out.push_back({row, nature, r});
}

std::vector<Simulate::Lit> Simulate::rowTargets(const std::string& row) const {
  std::vector<Lit> out;
  const opad::Scene& s = services().document()->scene;
  auto add = [&](const opad::Ref& r, const std::string& nature) { addTarget(out, row, r, nature); };
  auto body = [](const std::string& id) {
    opad::Ref r;
    r.body = id;
    return r;
  };
  auto ofLoad = [&](const opad::Load& l) {
    for (const auto& r : l.refs) add(r, l.kind);
  };
  if (row.rfind("sim:l:", 0) == 0) {
    if (const opad::Load* l = s.load(row.substr(6))) ofLoad(*l);
  } else if (row.rfind("sim:c:", 0) == 0) {
    for (const auto& l : s.loads)
      if (l.load_case == row.substr(6)) ofLoad(l);
  } else if (row.rfind("sim:x:", 0) == 0) {
    const std::string rest = row.substr(6);
    const size_t colon = rest.find(':');
    const opad::Study* st = s.study(rest.substr(0, colon));
    if (!st || colon == std::string::npos) return out;
    const std::string what = rest.substr(colon + 1);
    const json cfd = st->def.value("settings", json::object()).value("cfd", json::object());
    if (what == "box" || what == "air") {  // the air: where it moves, the box
      const json e = cfd.value("enclosure", json());
      for (const auto& w : e.is_array() ? e : json::array({e}))
        if (w.is_string()) add(body(w.get<std::string>()), what == "box" ? "box" : cfd.value("buoyancy", false) ? "natural" : "fan");
    } else if (what.rfind("board:", 0) == 0) {
      add(body(what.substr(6)), "board");
    } else if (what.rfind("sink:", 0) == 0) {
      add(body(what.substr(5)), "heatsink");
    }
  }
  return out;
}

void Simulate::lightRows(const std::vector<std::string>& rows) {
  m_litRefs.clear();
  std::vector<Lit> targets;
  for (const auto& row : rows) {
    const auto more = rowTargets(row);
    targets.insert(targets.end(), more.begin(), more.end());
  }
  lightTargets(std::move(targets));
}

void Simulate::lightRefs(const std::vector<std::pair<opad::Ref, std::string>>& refs) {
  m_litRefs = refs;
  std::vector<Lit> targets;
  for (const auto& [r, nature] : refs) addTarget(targets, "setup", r, nature);
  lightTargets(std::move(targets));
}

void Simulate::lightTargets(std::vector<Lit> targets) {
  Viewport* view = services().viewport();
  if (!view) return;
  view->clearSimTargets();
  for (auto& [at, badge] : std::exchange(m_badges, {})) delete badge;
  m_lit.clear();
  for (Lit t : targets) {
    opad::Vec3 centre{0, 0, 0};
    t.shown = view->showSimTarget(t.target, natureColor(t.nature), &centre);
    if (t.shown && m_badges.size() < kMaxBadges) m_badges.push_back({centre, new Badge(view, natureColor(t.nature), natureIcon(t.nature))});
    m_lit.push_back(t);
  }
  placeBadges();
}

void Simulate::placeBadges() {
  const Viewport* view = services().viewport();
  for (auto& [at, badge] : m_badges) {
    const QPoint p = view->widgetPoint(at);
    const bool in = view->rect().contains(p);
    badge->move(p + QPoint(10, -10 - badge->height()));  // beside its target's middle, up and to the right: the part stays seen
    badge->setVisible(in);
    if (in) badge->raise();
  }
}

bool Simulate::removeRows(const std::vector<std::string>& rows) {
  AppDocument* doc = services().document();
  const opad::Scene& s = doc->scene;
  std::vector<std::string> deleted;  // op ids: loads, studies, joints
  auto drop = [&](const std::string& id) {
    if (std::find(deleted.begin(), deleted.end(), id) == deleted.end()) deleted.push_back(id);
  };
  std::map<std::string, json> settings;        // a study's settings as its setup rows leave them
  std::map<std::string, json> heatsinks;       // a fan load's heatsink list without the heatsink deleted
  for (const auto& row : rows) {
    if (row.rfind("sim:l:", 0) == 0 && s.load(row.substr(6))) drop(row.substr(6));
    else if (row.rfind("sim:s:", 0) == 0 && s.study(row.substr(6))) drop(row.substr(6));
    else if (row.rfind("sim:j:", 0) == 0 && s.joint(row.substr(6))) drop(row.substr(6));
    else if (row.rfind("sim:c:", 0) == 0) {
      for (const auto& l : s.loads)
        if (l.load_case == row.substr(6)) drop(l.id);
    } else if (row.rfind("sim:x:", 0) == 0) {
      const std::string rest = row.substr(6);
      const size_t colon = rest.find(':');
      const opad::Study* st = colon == std::string::npos ? nullptr : s.study(rest.substr(0, colon));
      if (!st) continue;
      const std::string what = rest.substr(colon + 1);
      json& set = settings.try_emplace(st->id, st->def.value("settings", json::object())).first->second;
      if (what == "box") {
        if (set.contains("cfd")) set["cfd"].erase("enclosure");
      } else if (what.rfind("board:", 0) == 0) {
        const std::string id = what.substr(6);
        if (set.contains("materials") && set["materials"].contains(id)) {
          set["materials"][id].erase("pcb");
          if (set["materials"][id].empty()) set["materials"].erase(id);
        }
      } else if (what.rfind("sink:", 0) == 0) {
        const std::string id = what.substr(5);
        for (const auto& l : s.loads) {  // off the fans that blow through it
          if (l.load_case != set.value("case", std::string()) || l.kind != "fan") continue;
          json& list = heatsinks.try_emplace(l.id, l.def.value("heatsink", json::array())).first->second;
          for (auto it = list.begin(); it != list.end();) it = *it == id ? list.erase(it) : it + 1;
        }
        std::vector<std::string> bodies{id};  // its material, set for it as one part
        if (const opad::Node* n = s.node(id); n && n->kind != opad::Node::Kind::Body) bodies = s.bodies_under(id);
        for (const auto& b : bodies)
          if (set.contains("materials") && set["materials"].contains(b) && set["materials"][b].contains("material")) set["materials"].erase(b);
      }
    }
  }
  std::vector<std::pair<std::string, json>> writes;
  for (const auto& id : deleted) writes.push_back({"delete", {{"target", id}}});
  for (const auto& [id, set] : settings)
    if (std::find(deleted.begin(), deleted.end(), id) == deleted.end() && set != s.study(id)->def.value("settings", json::object()))
      writes.push_back({"study", {{"id", id}, {"settings", set}, {"run", false}}});
  for (const auto& [id, list] : heatsinks)
    if (std::find(deleted.begin(), deleted.end(), id) == deleted.end() && list != s.load(id)->def.value("heatsink", json::array()))
      writes.push_back({"load", {{"id", id}, {"heatsink", list}}});
  if (writes.empty()) return false;
  if (std::find(deleted.begin(), deleted.end(), m_study) != deleted.end()) chooseStudy(std::string());  // its results put away
  doc->batch(rows.size() == 1 ? tr("Delete") : tr("Delete %n item(s)", "", int(rows.size())), [&] {
    for (const auto& [command, args] : writes) doc->run(command, args);
  });
  lightRows({});
  m_picked.clear();
  return true;
}
