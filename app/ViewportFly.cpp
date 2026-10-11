// Fly (Viewport.hpp): the camera itself turns and moves, as a head would, instead of orbiting around the model, so it can
// get close to miniature features. Perspective: the speed is a share of the distance to the surface straight ahead, so
// every approach slows down by itself and a 0.1 mm detail of a 2 m assembly is reached as easily as the assembly; once
// a surface is nearer than the near plane it is passed and the next one counts. Orthographic: there is no depth in the
// picture, so coming closer is a clip plane at the eye (what it passed is cut away) and a zoom that follows the distance
// to what it was looking at, as perspective would show it.
#include <QApplication>
#include <QFocusEvent>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QTimer>
#include <QElapsedTimer>
#include <QWheelEvent>

#include <Graphic3d_ClipPlane.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Pln.hxx>

#include <algorithm>
#include <cmath>
#include <set>

#include "KeyText.hpp"
#include "Viewport.hpp"

struct Viewport::Fly {
  QTimer timer;          // 16 ms while a key is held
  QElapsedTimer clock;   // the time since the last step
  std::set<int> keys;    // held (Qt::Key)
  gp_Dir up = gp::DZ();  // the head's up: looking left and right turns about it
  double size = 100;     // the scene's diagonal
  double minimum = 0.02;  // the slowest it goes, per second (about the near plane's distance)
  double ahead = -1;      // perspective: to the surface ahead, last measured
  QElapsedTimer measured;
  bool ortho = false;
  double focus = 0, zoom = 0;  // orthographic: the distance to what it looks at, and the scale per unit of it
  Handle(Graphic3d_ClipPlane) clip;  // orthographic: at the eye, what is behind it cut away
  // The camera as fly left it: another command's move (fit, the cube, a standard view) is taken up in the next frame.
  gp_Pnt eye;
  gp_Dir dir = gp::DZ();
  double scale = 0;
  bool persp = false;
  // The left button: a drag looks around, a click is handed on as one.
  bool pressed = false, looking = false, replaying = false;
  QPointF pressAt, lastAt;
  std::unique_ptr<QMouseEvent> press;
};

namespace {
enum Axis { Forward, Side, Lift };
// What a key does while flying: the axis and the sign; false: not a fly key. Letters by their place on a Latin layout
// (Windows' virtual keys), arrows and Page Up / Page Down whatever the layout.
bool flyKey(const QKeyEvent* e, Axis& axis, int& sign) {
  int key = e->key();
  if (const quint32 vk = e->nativeVirtualKey(); (key < Qt::Key_A || key > Qt::Key_Z) && vk >= 'A' && vk <= 'Z') key = int(vk);
  switch (key) {
    case Qt::Key_W: case Qt::Key_Up: axis = Forward, sign = 1; return true;
    case Qt::Key_S: case Qt::Key_Down: axis = Forward, sign = -1; return true;
    case Qt::Key_D: case Qt::Key_Right: axis = Side, sign = 1; return true;
    case Qt::Key_A: case Qt::Key_Left: axis = Side, sign = -1; return true;
    case Qt::Key_E: case Qt::Key_PageUp: axis = Lift, sign = 1; return true;
    case Qt::Key_Q: case Qt::Key_PageDown: axis = Lift, sign = -1; return true;
    default: return false;
  }
}
int keyId(const QKeyEvent* e) {  // the same physical key for the press and the release
  Axis axis;
  int sign = 0;
  return flyKey(e, axis, sign) ? int(axis) * 2 + (sign > 0 ? 1 : 0) : -1;
}
constexpr double kLook = 0.25 * M_PI / 180;  // a pixel of drag, in radians
constexpr double kPace = 0.8;                // of the distance ahead, per second
}  // namespace

QString Viewport::flyHint() const {
  return tr("Fly: drag to look around · %1 %2 forward, back · %3 %4 sideways · %5 %6 up, down · %7 faster, %8 slower · %9 lands")
      .arg(keys::fixedText("flyForward"), keys::fixedText("flyBack"), keys::fixedText("flyLeft"), keys::fixedText("flyRight"), keys::fixedText("flyUp"),
           keys::fixedText("flyDown"), keys::fixedText("shift"), keys::fixedText("ctrl"), keys::fixedText("esc"));
}

gp_Dir Viewport::flyHead() const { return m_fly ? m_fly->up : gp::DZ(); }

bool Viewport::setFlying(bool on) {
  if (on == flying()) return true;
  if (!on) {
    Fly& f = *m_fly;
    const Handle(Graphic3d_Camera)& cam = m_view->Camera();
    // Orbit and zoom go on from what it was looking at.
    const double ahead = f.ortho ? f.focus : std::max(f.ahead, f.minimum);
    cam->SetCenter(cam->Eye().Translated(gp_Vec(cam->Direction()) * ahead));
    if (!f.clip.IsNull()) m_view->RemoveClipPlane(f.clip);
    if (f.pressed) unsetCursor();
    m_fly.reset();
    m_view->Invalidate();
    requestRedraw();
    emit hoverChanged(QString());
    emit flyingChanged(false);
    return true;
  }
  if (!m_initialised || m_sketchInput || m_twoDimensional) return false;
  if (m_zoomWindow) cancelZoomWindow();
  finishAnimation();
  m_needFit = false;
  m_fly = std::make_shared<Fly>();
  Fly& f = *m_fly;
  f.timer.setInterval(16);
  connect(&f.timer, &QTimer::timeout, this, [this] { flyTick(); });
  const Bnd_Box box = fitBounds();
  if (!box.IsVoid()) f.size = std::max(std::sqrt(box.SquareExtent()), 1e-3);
  f.minimum = f.size * 2e-4;
  const Handle(Graphic3d_Camera)& cam = m_view->Camera();
  // The head upright: world Z; looking (nearly) straight down or up, the screen's up is the head's.
  f.up = std::abs(cam->Direction().Z()) < 0.98 ? gp::DZ() : cam->Up();
  gp_Vec up(f.up);
  up -= gp_Vec(cam->Direction()) * up.Dot(gp_Vec(cam->Direction()));
  if (up.Magnitude() > 1e-6) cam->SetUp(gp_Dir(up));
  f.ortho = cam->IsOrthographic();
  if (f.ortho) flyOrthoStart();
  f.eye = cam->Eye(), f.dir = cam->Direction(), f.scale = cam->Scale(), f.persp = !f.ortho;
  setFocus();
  m_view->Invalidate();
  requestRedraw();
  emit hoverChanged(flyHint());
  emit flyingChanged(true);
  return true;
}

void Viewport::flyOrthoStart(bool redraw) {
  Fly& f = *m_fly;
  const Handle(Graphic3d_Camera)& cam = m_view->Camera();
  const gp_Dir dir = cam->Direction();
  gp_Pnt eye = cam->Eye();
  // Back out of the model: the eye in front of every corner of it (an orthographic picture does not change).
  const Bnd_Box box = fitBounds();
  if (!box.IsVoid()) {
    double x0, y0, z0, x1, y1, z1;
    box.Get(x0, y0, z0, x1, y1, z1);
    double nearest = 1e300;
    for (int i = 0; i < 8; ++i) {
      const gp_Pnt corner(i & 1 ? x1 : x0, i & 2 ? y1 : y0, i & 4 ? z1 : z0);
      nearest = std::min(nearest, gp_Vec(eye, corner).Dot(gp_Vec(dir)));
    }
    if (const double margin = f.size * 0.01; nearest < margin) {
      eye.Translate(gp_Vec(dir) * (nearest - margin));  // back along the view's axis
      cam->SetEyeAndCenter(eye, cam->Center());          // the centre was ahead of the old eye: ahead of this one too
    }
  }
  // What it looks at: the surface at the view's centre, else the middle of the model, else where the camera looks.
  double focus = -1;
  gp_Pnt hit;
  if (redraw) m_view->Redraw();  // the picker's depth range is the frame's (in a frame: the last one's)
  if (navigationPoint(devicePos(QPointF(width() / 2.0, height() / 2.0)), hit)) focus = gp_Vec(eye, hit).Dot(gp_Vec(dir));
  if (focus <= f.minimum && !box.IsVoid()) {
    double x0, y0, z0, x1, y1, z1;
    box.Get(x0, y0, z0, x1, y1, z1);
    focus = gp_Vec(eye, gp_Pnt((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)).Dot(gp_Vec(dir));
  }
  if (focus <= f.minimum) focus = std::max(eye.Distance(cam->Center()), f.size * 0.5);
  f.focus = focus;
  f.zoom = cam->Scale() / focus;
  cam->SetCenter(eye.Translated(gp_Vec(dir) * focus));
  if (f.clip.IsNull()) {
    f.clip = new Graphic3d_ClipPlane(gp_Pln(eye, dir));
    m_view->AddClipPlane(f.clip);
  } else {
    f.clip->SetEquation(gp_Pln(eye, dir));
  }
}

double Viewport::flyDistance() {
  Fly& f = *m_fly;
  if (f.ortho) return std::max(f.focus, f.minimum);
  if (f.ahead < 0 || !f.measured.isValid() || f.measured.elapsed() > 120) {
    const Handle(Graphic3d_Camera)& cam = m_view->Camera();
    gp_Pnt hit;
    double ahead = -1;
    if (navigationPoint(devicePos(QPointF(width() / 2.0, height() / 2.0)), hit)) ahead = gp_Vec(cam->Eye(), hit).Dot(gp_Vec(cam->Direction()));
    if (ahead <= 0) {  // nothing straight ahead: as far as the model's middle, or a tenth of it from inside
      const Bnd_Box box = fitBounds();
      ahead = f.size * 0.1;
      if (!box.IsVoid() && box.IsOut(cam->Eye())) {
        double x0, y0, z0, x1, y1, z1;
        box.Get(x0, y0, z0, x1, y1, z1);
        ahead = std::max(ahead, cam->Eye().Distance(gp_Pnt((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)) - f.size / 2);
      }
    }
    f.ahead = ahead;
    f.measured.start();
  }
  return std::max(f.ahead, f.minimum);
}

void Viewport::flyStep(const gp_Vec& delta) {
  Fly& f = *m_fly;
  const Handle(Graphic3d_Camera)& cam = m_view->Camera();
  const gp_Vec dir(cam->Direction());
  cam->SetEyeAndCenter(cam->Eye().Translated(delta), cam->Center().Translated(delta));
  if (f.ortho) {  // closer to what it looks at: the zoom as perspective would show it, what is passed cut away
    if (!f.clip.IsNull()) f.clip->SetEquation(gp_Pln(cam->Eye(), cam->Direction()));
    f.focus -= delta.Dot(dir);
    if (f.focus <= f.minimum) {  // reached it: the clip plane goes through, and the next surface ahead is looked at
      gp_Pnt hit;
      const double next = navigationPoint(devicePos(QPointF(width() / 2.0, height() / 2.0)), hit) ? gp_Vec(cam->Eye(), hit).Dot(dir) : -1;
      if (next > f.minimum) f.zoom = cam->Scale() / next, f.focus = next;
      else f.focus = f.minimum;
    }
    cam->SetScale(f.zoom * f.focus);
  } else if (f.ahead > 0) {
    f.ahead = std::max(f.ahead - delta.Dot(dir), 0.0);  // until it is measured again
  }
  f.eye = cam->Eye(), f.dir = cam->Direction(), f.scale = cam->Scale();
  m_needFit = false;
  m_view->Invalidate();
  requestRedraw();
}

void Viewport::flyLook(double dx, double dy) {
  Fly& f = *m_fly;
  const Handle(Graphic3d_Camera)& cam = m_view->Camera();
  gp_Vec dir(cam->Direction());
  dir.Rotate(gp_Ax1(gp::Origin(), f.up), -dx * kLook);  // right: turns to the right
  gp_Vec right = dir.Crossed(gp_Vec(f.up));
  if (right.Magnitude() > 1e-9) {
    gp_Vec pitched = dir;
    pitched.Rotate(gp_Ax1(gp::Origin(), gp_Dir(right)), -dy * kLook);  // up: looks up
    const double fromUp = pitched.Angle(gp_Vec(f.up));
    if (fromUp > 2 * M_PI / 180 && fromUp < 178 * M_PI / 180) dir = pitched;  // never over the top: the head stays upright
  }
  cam->SetDirectionFromEye(gp_Dir(dir));  // the eye stays (SetCenter ignores a centre as far as the old one)
  gp_Vec up(f.up);
  up -= gp_Vec(gp_Dir(dir)) * up.Dot(gp_Vec(gp_Dir(dir)));
  if (up.Magnitude() > 1e-9) cam->SetUp(gp_Dir(up));
  if (f.ortho && !f.clip.IsNull()) f.clip->SetEquation(gp_Pln(cam->Eye(), cam->Direction()));
  f.ahead = -1;  // something else is ahead now
  f.eye = cam->Eye(), f.dir = cam->Direction(), f.scale = cam->Scale();
  m_needFit = false;
  m_view->Invalidate();
  requestRedraw();
}

void Viewport::flyTick() {
  if (!m_fly) return;
  Fly& f = *m_fly;
  const double dt = std::clamp(f.clock.isValid() ? f.clock.restart() / 1000.0 : 0.016, 0.001, 0.1);
  if (!f.clock.isValid()) f.clock.start();
  int move[3] = {0, 0, 0};
  for (int k : f.keys) move[k / 2] += k % 2 ? 1 : -1;
  if (!move[Forward] && !move[Side] && !move[Lift]) {
    f.timer.stop();
    f.clock.invalidate();
    return;
  }
  const Qt::KeyboardModifiers mods = QGuiApplication::queryKeyboardModifiers();
  const double pace = kPace * (mods & Qt::ShiftModifier ? 4.0 : 1.0) * (mods & Qt::ControlModifier ? 0.25 : 1.0);
  const Handle(Graphic3d_Camera)& cam = m_view->Camera();
  const gp_Vec dir(cam->Direction()), up(cam->Up()), right = dir.Crossed(up);
  gp_Vec way = dir * move[Forward] + right * move[Side] + up * move[Lift];
  if (way.Magnitude() > 1) way.Normalize();
  flyStep(way * (flyDistance() * pace * dt));
}

bool Viewport::flyEvent(QEvent* e) {
  if (!m_fly || (e->type() != QEvent::ShortcutOverride && e->type() != QEvent::KeyPress && e->type() != QEvent::KeyRelease)) return false;
  auto* k = static_cast<QKeyEvent*>(e);
  if (k->modifiers() & (Qt::AltModifier | Qt::MetaModifier)) return false;
  Fly& f = *m_fly;
  if (k->key() == Qt::Key_Escape) {
    if (e->type() == QEvent::KeyPress) setFlying(false);
    e->accept();
    return true;
  }
  const int id = keyId(k);
  if (id < 0) return false;
  if (e->type() == QEvent::KeyPress && !k->isAutoRepeat()) {
    f.keys.insert(id);
    if (!f.timer.isActive()) {
      f.clock.invalidate();
      f.timer.start();
      flyTick();  // a tap moves too
    }
  } else if (e->type() == QEvent::KeyRelease && !k->isAutoRepeat()) {
    f.keys.erase(id);
  }
  e->accept();
  return true;
}

bool Viewport::flyPress(QMouseEvent* e) {
  if (!m_fly || m_fly->replaying || e->button() != Qt::LeftButton || e->modifiers() != Qt::NoModifier || cubeAt(e->position())) return false;
  Fly& f = *m_fly;
  f.pressed = true;
  f.looking = false;
  f.pressAt = f.lastAt = e->position();
  f.press = std::make_unique<QMouseEvent>(e->type(), e->position(), e->globalPosition(), e->button(), e->buttons(), e->modifiers());
  setFocus();
  e->accept();
  return true;
}

bool Viewport::flyMove(QMouseEvent* e) {
  if (!m_fly || !m_fly->pressed) return false;
  Fly& f = *m_fly;
  if (!(e->buttons() & Qt::LeftButton)) {  // released elsewhere
    f.pressed = f.looking = false;
    unsetCursor();
    return false;
  }
  if (!f.looking && (e->position() - f.pressAt).manhattanLength() >= 4) {
    f.looking = true;
    setCursor(Qt::ClosedHandCursor);
  }
  if (f.looking) {
    const QPointF d = e->position() - f.lastAt;
    flyLook(d.x(), d.y());
  }
  f.lastAt = e->position();
  e->accept();
  return true;
}

bool Viewport::flyRelease(QMouseEvent* e) {
  if (!m_fly || !m_fly->pressed || e->button() != Qt::LeftButton) return false;
  Fly& f = *m_fly;
  f.pressed = false;
  if (f.looking) {
    f.looking = false;
    unsetCursor();
    e->accept();
    return true;
  }
  // A click: the press it held back goes through as it came, then this release.
  std::unique_ptr<QMouseEvent> press = std::move(f.press);
  const std::shared_ptr<Fly> keep = m_fly;
  keep->replaying = true;
  if (press) mousePressEvent(press.get());
  keep->replaying = false;
  return false;
}

bool Viewport::flyWheel(QWheelEvent* e) {
  if (!m_fly || e->angleDelta().y() == 0) return false;
  const double notches = e->angleDelta().y() / 120.0;
  const Qt::KeyboardModifiers mods = e->modifiers();
  const double share = 0.2 * (mods & Qt::ShiftModifier ? 4.0 : 1.0) * (mods & Qt::ControlModifier ? 0.25 : 1.0);
  flyStep(gp_Vec(m_view->Camera()->Direction()) * (flyDistance() * share * notches));
  e->accept();
  return true;
}

void Viewport::flyFrame() {
  if (!m_fly) return;
  Fly& f = *m_fly;
  const Handle(Graphic3d_Camera)& cam = m_view->Camera();
  const bool moved = cam->Eye().Distance(f.eye) > 1e-9 * f.size || !cam->Direction().IsEqual(f.dir, 1e-9) || std::abs(cam->Scale() - f.scale) > 1e-9 * f.scale ||
                     f.persp != !cam->IsOrthographic();
  if (!moved) return;
  // Another command moved the camera (fit, a standard view, the cube, the projection): flying goes on from there.
  f.ortho = cam->IsOrthographic();
  f.ahead = -1;
  if (f.ortho) {
    flyOrthoStart(false);
  } else if (!f.clip.IsNull()) {
    m_view->RemoveClipPlane(f.clip);
    f.clip.Nullify();
  }
  f.eye = cam->Eye(), f.dir = cam->Direction(), f.scale = cam->Scale(), f.persp = !f.ortho;
}

void Viewport::focusOutEvent(QFocusEvent* e) {
  if (m_fly) m_fly->keys.clear();  // a key let go in another window never reaches this one
  QWidget::focusOutEvent(e);
}
