// OPAD_BENCH_FLY (Viewport::setFlying, case fly in tools/bench_cases/viewer.py): in perspective, W moves the eye straight
// ahead and slows down as the surface ahead comes closer, a drag turns the camera about its eye (left and right about the
// head's up, up and down without going over the top), the wheel steps forward, Esc lands; in orthographic, flying keeps
// the picture as it was, puts a clip plane at the eye, and coming closer zooms in as the distance to what it looks at
// shrinks, the clip plane following the eye; landing takes the clip plane away and keeps the zoom. Not in 2D mode. The
// window's Fly (view.fly) turns it on with a card saying the keys, and its button follows Esc.
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QKeyEvent>
#include <QThread>
#include <QTimer>
#include <QWheelEvent>

#include <AIS_AnimationCamera.hxx>
#include <Graphic3d_ClipPlane.hxx>
#include <Graphic3d_SequenceOfHClipPlane.hxx>

#include <cmath>
#include <set>

#include "BenchRegistry.hpp"
#include "MainWindow.hpp"
#include "Toast.hpp"
#include "Viewport.hpp"

bool Viewport::benchFly(const QString&) {
  bool all = true;
  auto require = [&](bool ok, const QString& what) {
    trace::log(QString("bench: fly: %1 %2").arg(what, ok ? "PASS" : "FAIL"));
    all = all && ok;
    return ok;
  };
  if (!require(m_initialised && !m_items.empty(), "a model is displayed")) return false;
  myViewAnimation->Stop();
  m_forceAnimate = false;
  auto key = [this](QEvent::Type type, int k, Qt::KeyboardModifiers mods = Qt::NoModifier) {
    QKeyEvent e(type, k, mods);
    QCoreApplication::sendEvent(this, &e);
    return e.isAccepted();
  };
  // Holding W for `frames` frames of the timer: a frame drawn first each time (the picker's depth range is the frame's).
  auto hold = [&](int k, int frames, std::vector<double>* steps = nullptr) {
    key(QEvent::ShortcutOverride, k);
    key(QEvent::KeyPress, k);
    for (int i = 0; i < frames; ++i) {
      m_view->Redraw();
      const gp_Pnt before = m_view->Camera()->Eye();
      QThread::msleep(16);
      flyTick();
      if (steps) steps->push_back(before.Distance(m_view->Camera()->Eye()));
    }
    key(QEvent::KeyRelease, k);
  };
  auto ahead = [this] {  // to the surface at the view's centre
    m_view->Redraw();
    gp_Pnt hit;
    return navigationPoint(devicePos(QPointF(width() / 2.0, height() / 2.0)), hit) ? gp_Vec(m_view->Camera()->Eye(), hit).Dot(gp_Vec(m_view->Camera()->Direction())) : -1.0;
  };
  const int planes = m_view->ClipPlanes().IsNull() ? 0 : m_view->ClipPlanes()->Size();

  // ---- perspective
  setOrthographic(false);
  standardView("iso");
  fitAll();
  m_view->Redraw();
  require(setFlying(true) && flying(), "Fly starts in perspective");
  const gp_Dir dir0 = m_view->Camera()->Direction();
  const gp_Pnt eye0 = m_view->Camera()->Eye();
  const double first = ahead();
  std::vector<double> steps;
  hold(Qt::Key_W, 80, &steps);
  const gp_Vec went(eye0, m_view->Camera()->Eye());
  const double now = ahead();
  require(went.Magnitude() > 0 && went.Dot(gp_Vec(dir0)) > 0.999 * went.Magnitude() && m_view->Camera()->Direction().IsEqual(dir0, 1e-9),
          QString("W moves the eye straight ahead (%1 mm)").arg(went.Magnitude(), 0, 'f', 2));
  auto mean = [&steps](size_t from) {  // ten frames' steps (the frame times vary)
    double sum = 0;
    for (size_t i = from; i < from + 10 && i < steps.size(); ++i) sum += steps[i];
    return sum / 10;
  };
  require(first > 0 && now > 0 && now < first && steps.size() == 80 && mean(70) < mean(0),
          QString("it slows down as the surface comes closer: %1 -> %2 mm ahead, steps %3 -> %4 mm").arg(first, 0, 'f', 2).arg(now, 0, 'f', 2)
              .arg(mean(0), 0, 'f', 3).arg(mean(70), 0, 'f', 3));
  // Looking around: about the eye, left and right about the head's up, up and down.
  const gp_Pnt eye1 = m_view->Camera()->Eye();
  const gp_Dir dir1 = m_view->Camera()->Direction();
  const gp_Dir head = flyHead();
  flyLook(100, 0);  // to the right
  const gp_Dir dir2 = m_view->Camera()->Direction();
  auto level = [&head](const gp_Dir& d) { return gp_Vec(d) - gp_Vec(head) * gp_Vec(d).Dot(gp_Vec(head)); };  // seen from above the head
  const double turned = level(dir1).Angle(level(dir2)) * 180 / M_PI;
  require(m_view->Camera()->Eye().Distance(eye1) < 1e-9 * std::max(1.0, eye1.Distance(gp::Origin())) && std::abs(turned - 25) < 0.5 &&
              std::abs(gp_Vec(dir2).Dot(gp_Vec(head)) - gp_Vec(dir1).Dot(gp_Vec(head))) < 1e-9 &&
              gp_Vec(dir1).Crossed(gp_Vec(dir2)).Dot(gp_Vec(head)) < 0,
          QString("a drag to the right turns the head right about its up, the eye in place (%1°)").arg(turned, 0, 'f', 2));
  flyLook(0, -40);  // up
  require(gp_Vec(m_view->Camera()->Direction()).Dot(gp_Vec(head)) > gp_Vec(dir2).Dot(gp_Vec(head)), "a drag upwards looks up");
  flyLook(0, -10000);  // never over the top
  require(m_view->Camera()->Direction().Angle(head) > 1.9 * M_PI / 180 && m_view->Camera()->Up().Dot(head) > 0, "it never looks over the top: the head stays upright");
  // The wheel: a step forward, a share of the distance ahead.
  const gp_Pnt eye3 = m_view->Camera()->Eye();
  const QPointF centre(width() / 2.0, height() / 2.0);
  QWheelEvent wheel(centre, mapToGlobal(centre), QPoint(), QPoint(0, 120), Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
  QCoreApplication::sendEvent(this, &wheel);
  require(gp_Vec(eye3, m_view->Camera()->Eye()).Dot(gp_Vec(m_view->Camera()->Direction())) > 0, "a wheel notch steps forward");
  key(QEvent::ShortcutOverride, Qt::Key_Escape);
  key(QEvent::KeyPress, Qt::Key_Escape);
  require(!flying(), "Esc lands");

  // ---- orthographic
  setOrthographic(true);
  standardView("front");
  fitAll();
  m_view->Redraw();
  const double scale0 = m_view->Camera()->Scale();
  require(setFlying(true) && flying() && std::abs(m_view->Camera()->Scale() / scale0 - 1) < 1e-9, "Fly starts in orthographic, the picture as it was");
  const int flyingPlanes = m_view->ClipPlanes().IsNull() ? 0 : m_view->ClipPlanes()->Size();
  require(flyingPlanes == planes + 1, "a clip plane at the eye cuts away what it passes");
  const Bnd_Box box = fitBounds();
  require(box.IsVoid() || box.IsOut(m_view->Camera()->Eye()), "the eye starts in front of the model");
  const gp_Pnt eyeA = m_view->Camera()->Eye();
  hold(Qt::Key_W, 30);
  const double moved = gp_Vec(eyeA, m_view->Camera()->Eye()).Dot(gp_Vec(m_view->Camera()->Direction()));
  const double scale1 = m_view->Camera()->Scale();
  require(moved > 0 && scale1 < scale0, QString("coming closer zooms in: %1 mm on, scale %2 -> %3").arg(moved, 0, 'f', 2).arg(scale0, 0, 'f', 2).arg(scale1, 0, 'f', 2));
  bool atEye = false;
  if (!m_view->ClipPlanes().IsNull())
    for (Graphic3d_SequenceOfHClipPlane::Iterator it(*m_view->ClipPlanes()); it.More(); it.Next()) {
      const Graphic3d_ClipPlane::Equation& q = it.Value()->GetEquation();
      const gp_Pnt eye = m_view->Camera()->Eye();
      const gp_Dir d = m_view->Camera()->Direction();
      if (std::abs(q.x() * eye.X() + q.y() * eye.Y() + q.z() * eye.Z() + q.w()) < 1e-6 * std::max(1.0, eye.Distance(gp::Origin())) && gp_Dir(q.x(), q.y(), q.z()).IsEqual(d, 1e-9))
        atEye = true;
    }
  require(atEye, "the clip plane follows the eye, facing ahead");
  hold(Qt::Key_S, 10);
  require(m_view->Camera()->Scale() > scale1, "going back zooms out again");
  key(QEvent::ShortcutOverride, Qt::Key_Escape);
  key(QEvent::KeyPress, Qt::Key_Escape);
  require(!flying() && (m_view->ClipPlanes().IsNull() ? 0 : m_view->ClipPlanes()->Size()) == planes, "landing takes the clip plane away");

  // ---- not in 2D mode
  setTwoDimensional(true);
  require(!setFlying(true) && !flying(), "Fly is refused in 2D mode");
  setTwoDimensional(false);
  standardView("iso");
  fitAll();
  return all;
}

OPAD_BENCH(OPAD_BENCH_FLY, fly) {
  Viewport* v = w.m_viewport;
  auto clock = std::make_shared<QElapsedTimer>();
  clock->start();
  auto* timer = new QTimer(&w);
  QObject::connect(timer, &QTimer::timeout, &w, [&w, v, timer, clock, value] {
    const bool ready = !v->pumpJob() && v->remainingBodies() == 0 && v->displayedCount() > 0;
    if (!ready && clock->elapsed() < 60000) return;
    timer->stop();
    timer->deleteLater();
    bool ok = ready && v->benchFly(value);
    auto require = [&ok](bool pass, const QString& what) {
      trace::log(QString("bench: fly: %1 %2").arg(what, pass ? "PASS" : "FAIL"));
      ok = ok && pass;
    };
    QAction* fly = w.action("view.fly");
    fly->trigger();
    const QList<Toast*> toasts = w.m_toasts->toasts();  // a copy: one list for both ends
    const bool card = std::any_of(toasts.begin(), toasts.end(), [v](Toast* t) { return t->text() == v->flyHint(); });
    require(fly->isChecked() && v->flying() && card, "Fly (view.fly) turns it on, with a card saying the keys");
    QKeyEvent over(QEvent::ShortcutOverride, Qt::Key_Escape, Qt::NoModifier), esc(QEvent::KeyPress, Qt::Key_Escape, Qt::NoModifier);
    QCoreApplication::sendEvent(v, &over);
    QCoreApplication::sendEvent(v, &esc);
    require(!fly->isChecked() && !v->flying(), "Esc lands, the button follows");
    QCoreApplication::exit(ok ? 0 : 2);
  });
  timer->start(50);
  return true;
}
