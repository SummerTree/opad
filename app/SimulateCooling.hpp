#pragma once
// Thermal setup (simulate.cooling, the CoolingAssistant), the content of the "cooling" ToolPanel, its steps as tabs: the guided
// setup of a thermal study of a board, its chips and a heatsink in a vented box, cooled by fans, by fans with the warm air rising
// too (mixed convection, slower), or by natural convection: the box (the bodies it is made of: a base and its lid, a frame and
// its panels), the parts and faces that make heat (a chip joined into its board is a face) with their power and which part is a
// board, how the air moves (fans, each a body or a component of its own model; the face the air crosses, whose rectangle, sized,
// is the fan's outlet, drawn in the view with the air's way; the heatsink it blows through with its material), and the run (the
// air solved with OpenFOAM, the parts with CalculiX: sim/cfd.hpp; Quick by default, with an estimate of how long) with its
// results. Every list starts empty (or as the study left it) and is filled by its Add button, which picks in the view and the
// browser while it is on: a click adds, never takes out (Remove does, for the rows chosen). What it sets up is ordinary loads
// (case "Cooling") and a study, written as it is set (each change one step to undo, the loads changed in place), so they show in
// the browser's Simulation folder as the manual commands' do, and can be changed in the Simulation panel afterwards; an AI agent
// sees the same.
#include <QPointer>
#include <QWidget>

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "opad/sim/cfd.hpp"
#include "opad/sim/study.hpp"

class AppDocument;
class Job;
class JobRunner;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QListWidget;
class QPushButton;
class QRadioButton;
class QSpinBox;
class QStackedWidget;
class QTableWidget;

class CoolingAssistant : public QWidget {
  Q_OBJECT
 public:
  struct Hooks {
    std::function<AppDocument*()> document;
    std::function<opad::json(const std::string&, const opad::json&, const QString&)> write;  // a command on the document, undoable
    std::function<void(const QString&, const std::function<void()>&)> batch;  // the writes inside as one step to undo (none: each its own)
    std::function<void(const std::string&)> run;           // runs a study in the background (the Simulation panel shows it)
    std::function<bool()> busy;                            // a study is running
    std::function<void(const QString&)> showField;         // the result map with that field (temperature, air_speed)
    std::function<void(const std::string&, bool)> setVisible;  // a body shown or hidden
    std::function<JobRunner*()> jobs;                      // where the fan's face is read (a worker)
    std::function<void(const std::vector<opad::Ref>&)> select;  // bodies or faces selected in the view and the browser
    // The rows chosen in a list lit in the view by what they are ("heat", "board", "box", "fan", "heatsink"): a face too,
    // whatever the selection filter; none: nothing lit.
    std::function<void(const std::vector<std::pair<opad::Ref, std::string>>&)> light;
    std::function<void(bool)> panel;
    std::function<void(const std::vector<opad::json>&)> fanGlyphs;  // fans ({refs, vector, outlet}) drawn with their plane and way  // its panel shown (as the other panels: the one shown) or hidden; none: a window of its own
    std::function<void(bool)> accumulate;                  // the view's clicks add and take out (picking) or select
    std::function<QString(const QString&)> filter;         // the view's selection filter set to that action (empty: kept); the one before
    std::function<bool()> filterSwitching;                 // a filter switch is being applied (its cleared selection is no pick)
    std::function<void(QObject*, std::function<void()>)> onFilterApplied;  // each time the view's filter has built its pick targets
  };
  CoolingAssistant(Hooks hooks, QWidget* parent = nullptr);

  void open(int step = -1);  // its panel shown, at that step (-1: where it was)
  void dismiss();            // its panel closed (what is still to write written first)
  void escape();             // Esc: the Add button that picks goes off, else the panel closes
  int step() const;
  int steps() const;
  // A study finished (or failed: run null): its results onto the Run page.
  void runFinished(const std::string& study, std::shared_ptr<const opad::sim::StudyRun> run, const QString& error);

  // The names it gives what it makes.
  static constexpr const char* kCase = "Cooling";
  static QString studyName();

  // For benches.
  QListWidget* wallChoice() const { return m_walls; }  // the box's bodies, by name (their ids in Qt::UserRole)
  const std::vector<std::string>& pickedBodies() const { return m_picked; }
  bool picking() const { return m_picking != Pick::None; }
  bool pickingFaces() const { return m_picking == Pick::Faces; }
  std::vector<std::string> heatBodies() const;  // the heat table's parts (bodies), in its order
  std::vector<opad::Ref> heatFaces() const;     // its faces (a chip joined into its board)
  // The app's selection changed (the view, the browser): while an Add button is on, what was clicked is added to its list.
  void selectionChanged(const std::vector<std::string>& ids, const std::vector<opad::Ref>& refs);
  QTableWidget* heatTable() const { return m_heat; }
  bool naturalConvection() const;  // no fans: the air moves by rising where it is warm
  bool warmAirRises() const;       // natural convection, or fans and the air rising too (cfd.buoyancy: slower)
  QString runKind() const;         // its quality and how the air moves, as the last runs' durations are kept
  void showEstimate();             // how long Run would take, said before it starts
  // A fan as the Air page sets it up: written as a fan load (on, fan, vector, across, heatsink) and the heatsink's material.
  struct FanSpec {
    std::vector<std::string> on;    // the fan: bodies or a component (its model's bodies, one part)
    opad::Ref across;               // the flat face the air crosses (body empty: none, the axis below)
    opad::Vec3 normal{0, 0, 0};     // that face's normal in the world, measured on a worker
    bool flip = false;
    int axis = 0;                   // ways(), without a face
    QString model = "40x10";        // a library fan, empty: custom
    double flow = 10, pressure = 20;  // custom: m3/h, Pa
    std::vector<std::string> sink;  // the heatsink it blows through: bodies or a component (one part)
    // Its outlet: the rectangle the air leaves by, fitted to the face it goes through (a blower's side) and sized from there;
    // the solver's pressure jump is across it (load "outlet"). Without one: a disk through the middle of the fan's body.
    bool outlet = false;
    opad::Vec3 centre{0, 0, 0}, xdir{1, 0, 0};
    double width = 0, height = 0;
    opad::json outletJson() const;  // as the load has it ({centre, normal, x, width, height}); null without one
    QString material;               // the heatsink's library material, empty: as modelled
    opad::Vec3 vector() const;      // the way the air goes
  };
  const std::vector<FanSpec>& fans() const { return m_fanSpecs; }
  int currentFan() const { return m_fan; }
  QListWidget* fanList() const { return m_fanList; }
  QLabel* resultText() const { return m_result; }
  QPushButton* runButton() const { return m_runButton; }
  bool apply();         // the loads and the study written now; false when something is missing (said on the page)
  void sync();          // what the pages hold written now (as far as it goes: a fan not picked yet is no load); each change schedules it
  bool startRun();      // apply, then run the study
  void buildExample();  // the example board in its vented box, into an empty document

 protected:
  void keyPressEvent(class QKeyEvent* e) override;
  void hideEvent(class QHideEvent* e) override;

 private:
  void reload();      // the pages from the document as it is now
  void scheduleSync();  // a change of the pages: written a moment later (typing a power writes once)
  void showStep(int i);
  void lightChosen();  // the rows chosen on the page shown (the box's, the heat table's, the current fan) lit in the view
  std::vector<std::string> walls() const;  // the box's bodies
 public:
  // What the view and the browser pick: the box's bodies, the parts or faces that make heat, a fan, the face its air crosses,
  // its heatsink. Each page picks with one of them at a time; its pick boxes choose which.
  enum class Pick { None, Box, Parts, Faces, Fan, Way, Sink };
  Pick pickingWhat() const { return m_picking; }
 private:
  void startPicking(Pick what);
  void stopPicking();
  void reselect();      // what the list being added to holds, shown selected
  void showPicked();    // the box's list
  void syncAdd();       // the Add buttons say whether they pick
  bool pickable(const std::string& id) const;  // a shown body of the document
  void addHeatBody(const std::string& id, double watts, bool board, int layers);
  void addHeatFace(const opad::Ref& face, double watts);
  void loadFans(const opad::Scene& s);  // the fans the study's loads set
  void showFans();      // the fan list and the current fan's form
  void showFan();       // the current fan's form
  void measureWay(int fan);  // the normal of the face the fan's air crosses, on a worker
  std::string studyId() const;

  Hooks m_hooks;
  class QTabBar* m_steps = nullptr;  // Box, Heat, Air, Run
  QStackedWidget* m_pages = nullptr;
  QLabel* m_engine = nullptr;
  QListWidget* m_walls = nullptr;   // the box's bodies, by name
  QPushButton* m_boxAdd = nullptr;  // on: clicks in the view and the browser add bodies to it
  QPushButton* m_boxRemove = nullptr;
  std::vector<std::string> m_picked;
  Pick m_picking = Pick::None;
  QString m_filterBefore;     // the view's filter before picking
  bool m_awaiting = false;    // the filter switched: the picks shown again once its targets are built
  QString pickFilter() const;  // the filter the page picks with
  void filterApplied();
  QPushButton* m_partAdd = nullptr;  // on: clicks add parts that make heat (bodies)
  QPushButton* m_faceAdd = nullptr;  // on: clicks add faces that make heat (a chip joined into its board)
  QPushButton* m_heatRemove = nullptr;
  QPushButton* m_example = nullptr;
  QTableWidget* m_heat = nullptr;  // a row per part or face that makes heat (or a board): its power, board, copper layers
  QRadioButton* m_fansOn = nullptr;   // fans move the air
  QRadioButton* m_natural = nullptr;  // natural convection: no fans, the air rises where it is warm
  QRadioButton* m_mixed = nullptr;    // fans, and the air rising where it is warm (mixed convection)
  QLabel* m_airNote = nullptr;        // what the choice includes and what it costs
  std::vector<FanSpec> m_fanSpecs;
  int m_fan = -1;
  bool m_fanLoading = false;  // the form being filled: its edits are not the user's
  QListWidget* m_fanList = nullptr;
  QWidget* m_fanForm = nullptr;
  class QFormLayout* m_fanRows = nullptr;
  class PickBox* m_fanPick = nullptr;
  class PickBox* m_wayPick = nullptr;
  class PickBox* m_sinkPick = nullptr;
  QComboBox* m_axis = nullptr;
  QPushButton* m_flip = nullptr;
  QLabel* m_wayText = nullptr;
  QComboBox* m_model = nullptr;
  QDoubleSpinBox* m_flow = nullptr;
  QDoubleSpinBox* m_pressure = nullptr;
  QComboBox* m_sinkMaterial = nullptr;
  QWidget* m_outletRow = nullptr;   // the outlet's width and height
  QDoubleSpinBox* m_outletWidth = nullptr;
  QDoubleSpinBox* m_outletHeight = nullptr;
  std::array<double, 2> m_faceSize{0, 0};  // the face's own, for Fit
  QPushButton* m_back = nullptr;
  QPushButton* m_next = nullptr;
  QLabel* m_runNote = nullptr;  // why Run cannot run (no OpenFOAM)
  QDoubleSpinBox* m_ambient = nullptr;
  QComboBox* m_quality = nullptr;
  QLabel* m_estimate = nullptr;     // how long a run would take
  QCheckBox* m_radiation = nullptr;
  QPushButton* m_runButton = nullptr;
  QLabel* m_status = nullptr;
  QLabel* m_result = nullptr;
  QWidget* m_resultButtons = nullptr;
  std::pair<unsigned long long, unsigned long long> m_loaded{~0ull, ~0ull};  // the document's generation and revision last loaded
  class QTimer* m_syncTimer = nullptr;
  bool m_reloading = false;  // the pages being filled from the document: no change of the user's
  bool m_writing = false;    // its own writes: the document's change is no reason to reload
 public:
  bool settled() const { return true; }  // benches: loaded (nothing is measured any more: the lists start as the study left them)
  QPushButton* boxAdd() const { return m_boxAdd; }
  QPushButton* partAdd() const { return m_partAdd; }
  QPushButton* faceAdd() const { return m_faceAdd; }
};
