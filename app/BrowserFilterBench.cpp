// The browser's filter box clears from its clear button; case in tools/bench_cases/viewer.py.
#include <QCoreApplication>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QToolButton>

#include "BenchRegistry.hpp"
#include "BrowserPanel.hpp"
#include "MainWindow.hpp"
#include "Tracking.hpp"

// OPAD_BENCH_BROWSER_FILTER=1: "Box" typed into Filter objects shows its clear button; a click where the button is drawn (sent
// to whatever widget is there, as a mouse would) empties the box. The key hint inside the box once sat on top of the button and
// took the click, so the text had to be backspaced away.
OPAD_BENCH(OPAD_BENCH_BROWSER_FILTER, browserFilter) {
  bool all = true;
  auto require = [&all](bool ok, const QString& what) {
    trace::log(QString("bench: browser filter: %1 %2").arg(what, ok ? "PASS" : "FAIL"));
    all = all && ok;
  };
  QLineEdit* filter = nullptr;
  for (auto* e : w.m_browser->findChildren<QLineEdit*>())
    if (e->placeholderText() == BrowserPanel::tr("Filter objects")) filter = e;
  if (!filter) {
    require(false, "the Filter objects box");
    return QCoreApplication::exit(2), true;
  }
  w.m_browser->show();
  filter->setText("Box");
  QCoreApplication::processEvents();
  QToolButton* clear = nullptr;
  for (auto* b : filter->findChildren<QToolButton*>())
    if (b->isVisible() && b->x() > filter->width() / 2) clear = b;  // the trailing one (the leading one is the search icon)
  require(clear != nullptr, "typed: the clear button shows");
  if (clear) {
    const QPoint at = clear->geometry().center();
    QWidget* target = filter->childAt(at);
    require(target == clear, QString("the clear button is what is under the pointer there (%1)").arg(target ? target->metaObject()->className() : "nothing"));
    if (!target) target = filter;
    const QPoint local = target->mapFrom(filter, at);
    for (const QEvent::Type type : {QEvent::MouseButtonPress, QEvent::MouseButtonRelease}) {
      QMouseEvent e(type, local, target->mapToGlobal(local), Qt::LeftButton, type == QEvent::MouseButtonPress ? Qt::LeftButton : Qt::NoButton, Qt::NoModifier);
      QCoreApplication::sendEvent(target, &e);
    }
    QCoreApplication::processEvents();
    require(filter->text().isEmpty(), "a click on the clear button empties the box (\"" + filter->text() + "\" left)");
    QLabel* hint = nullptr;
    for (auto* l : filter->findChildren<QLabel*>())
      if (l->objectName() == "tertiary") hint = l;
    require(!hint || hint->isVisible() || hint->text().isEmpty(), "emptied: the key hint is back in the box");
  }
  QCoreApplication::exit(all ? 0 : 2);
  return true;
}
