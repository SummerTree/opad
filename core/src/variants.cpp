// Variants of one design that share no history, matched body by body (opad/variants.hpp).
#include "opad/variants.hpp"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <GProp_PrincipalProps.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>

#include <algorithm>
#include <iterator>
#include <cmath>
#include <map>
#include <unordered_map>

#include "opad/drawing_io.hpp"
#include "opad/geometry.hpp"

namespace opad {
namespace {

// A body's measures in its own frame (one per body key: instances share it).
struct Intrinsic {
  bool known = false, solid = false;
  int faces = 0, edges = 0, vertices = 0;
  double volume = 0, area = 0, size = 0;
  double moments[3] = {0, 0, 0};
  gp_Pnt centre;
  gp_Dir axes[3] = {gp::DX(), gp::DY(), gp::DZ()};
  Bnd_Box box;
};

Intrinsic measure(const Document& doc, const std::string& key) {
  Intrinsic m;
  try {
    const TopoDS_Shape shape = body_shape(doc, key);
    if (shape.IsNull()) return m;
    TopTools_IndexedMapOfShape faces, edges, vertices;
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    TopExp::MapShapes(shape, TopAbs_VERTEX, vertices);
    m.faces = faces.Extent(), m.edges = edges.Extent(), m.vertices = vertices.Extent();
    bool surfaces = true;  // a mesh has faces without surfaces: measured on its triangles
    for (int i = 1; i <= faces.Extent() && surfaces; ++i) {
      TopLoc_Location loc;
      surfaces = !BRep_Tool::Surface(TopoDS::Face(faces(i)), loc).IsNull();
    }
    m.solid = surfaces && TopExp_Explorer(shape, TopAbs_SOLID).More();
    GProp_GProps area;
    BRepGProp::SurfaceProperties(shape, area, Standard_False, !surfaces);
    m.area = std::abs(area.Mass());
    GProp_GProps mass = area;
    if (m.solid) {
      GProp_GProps volume;
      BRepGProp::VolumeProperties(shape, volume);
      m.volume = std::abs(volume.Mass());
      if (m.volume > 0) mass = volume;
    }
    if (mass.Mass() == 0 && faces.IsEmpty()) {  // a wire: its length
      GProp_GProps length;
      BRepGProp::LinearProperties(shape, length);
      mass = length;
      m.area = std::abs(length.Mass());
    }
    m.centre = mass.CentreOfMass();
    const GProp_PrincipalProps p = mass.PrincipalProperties();
    double i[3];
    p.Moments(i[0], i[1], i[2]);
    const gp_Vec a[3] = {p.FirstAxisOfInertia(), p.SecondAxisOfInertia(), p.ThirdAxisOfInertia()};
    int order[3] = {0, 1, 2};
    std::sort(order, order + 3, [&](int x, int y) { return std::abs(i[x]) < std::abs(i[y]); });
    for (int k = 0; k < 3; ++k) {
      m.moments[k] = std::abs(i[order[k]]);
      if (a[order[k]].Magnitude() > 1e-12) m.axes[k] = gp_Dir(a[order[k]]);
    }
    m.box = body_bbox(doc, key);
    if (!m.box.IsVoid()) m.size = std::sqrt(m.box.SquareExtent());
    m.known = true;
  } catch (const Standard_Failure&) {
  } catch (const std::exception&) {
  }
  return m;
}

BodyPrint place(const Intrinsic& m, const Mat4& world) {
  BodyPrint p;
  p.known = m.known, p.solid = m.solid;
  p.faces = m.faces, p.edges = m.edges, p.vertices = m.vertices;
  p.volume = m.volume, p.area = m.area, p.size = m.size;
  for (int k = 0; k < 3; ++k) p.moments[k] = m.moments[k];
  p.centre = world.apply({m.centre.X(), m.centre.Y(), m.centre.Z()});
  for (int k = 0; k < 3; ++k) {
    Vec3 d = world.apply_dir({m.axes[k].X(), m.axes[k].Y(), m.axes[k].Z()});
    const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (n > 1e-12) p.axes[k] = {d[0] / n, d[1] / n, d[2] / n};
  }
  if (!m.box.IsVoid()) {
    double x[2], y[2], z[2];
    m.box.Get(x[0], y[0], z[0], x[1], y[1], z[1]);
    p.box_min = {1e300, 1e300, 1e300}, p.box_max = {-1e300, -1e300, -1e300};
    for (double cx : x)
      for (double cy : y)
        for (double cz : z) {
          const Vec3 q = world.apply({cx, cy, cz});
          for (int k = 0; k < 3; ++k) p.box_min[size_t(k)] = std::min(p.box_min[size_t(k)], q[size_t(k)]), p.box_max[size_t(k)] = std::max(p.box_max[size_t(k)], q[size_t(k)]);
        }
  }
  return p;
}

bool close_to(double x, double y, double tolerance, double floor) { return std::abs(x - y) <= tolerance * std::max({std::abs(x), std::abs(y)}) + floor; }
double distance(const Vec3& a, const Vec3& b) { return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2])); }

// The measures of every body key met, per document, computed once.
struct Prints {
  std::map<std::pair<const Document*, std::string>, Intrinsic> by_key;
  const Intrinsic& of(const Document& doc, const std::string& key) {
    const auto k = std::make_pair(&doc, key);
    auto it = by_key.find(k);
    if (it == by_key.end()) it = by_key.emplace(k, measure(doc, key)).first;
    return it->second;
  }
};

struct Item {
  std::string id, name, path, key, source;
  std::vector<std::string> faces;  // the file's own ids of its faces (Parasolid's node_id), sorted, prefixed by the source
  Mat4 world;
  bool shown = false;
  int pair = -1;
  std::string by;
  BodyPrint print;
  bool printed = false;
};

std::vector<Item> items_of(const Document& doc, const Scene& scene) {
  std::vector<Item> out;
  for (const auto& id : scene.all_bodies()) {
    const Node* n = scene.node(id);
    if (!n || n->kind != Node::Kind::Body) continue;
    Item it;
    it.id = id;
    it.name = n->name;
    it.key = n->body_key;
    it.world = scene.world(id);
    it.shown = scene.effectively_visible(id);
    std::string path = n->name;
    for (const Node* p = n->parent.empty() ? nullptr : scene.node(n->parent); p; p = p->parent.empty() ? nullptr : scene.node(p->parent))
      if (!p->parent.empty()) path = p->name + "/" + path;  // the root is the file's own name: variants differ there
    it.path = path;
    if (const BodyEntry* e = doc.body(n->body_key); e && e->meta.contains("ids") && e->meta["ids"].is_object()) {
      const json& ids = e->meta["ids"];
      const std::string source = ids.value("source", ""), body = ids.value("body", "");
      if (!body.empty()) it.source = source + ":" + body;
      for (const auto& f : ids.value("faces", json::array()))
        if (f.is_string() && !f.get_ref<const std::string&>().empty()) it.faces.push_back(source + ":" + f.get<std::string>());
      std::sort(it.faces.begin(), it.faces.end());
      it.faces.erase(std::unique(it.faces.begin(), it.faces.end()), it.faces.end());
    }
    out.push_back(std::move(it));
  }
  return out;
}

std::vector<BodyChange> match(const Document& da, const Scene& sa, const Document& db, const Scene& sb, const Cancel& cancel, Prints& prints) {
  std::vector<Item> A = items_of(da, sa), B = items_of(db, sb);
  auto stop = [&] {
    if (cancel && cancel()) throw Error("cancelled");
  };
  auto print = [&](Item& it, const Document& d) -> const BodyPrint& {
    if (!it.printed) {
      it.print = place(prints.of(d, it.key), it.world);
      it.printed = true;
    }
    return it.print;
  };
  auto pa = [&](size_t i) -> const BodyPrint& { return print(A[i], da); };
  auto pb = [&](size_t j) -> const BodyPrint& { return print(B[j], db); };
  auto join = [&](size_t i, size_t j, const char* by) {
    A[i].pair = int(j), B[j].pair = int(i);
    A[i].by = B[j].by = by;
  };
  auto unpaired = [](const std::vector<Item>& v) {
    std::vector<size_t> out;
    for (size_t i = 0; i < v.size(); ++i)
      if (v[i].pair < 0) out.push_back(i);
    return out;
  };
  // Greedy pairing over scored candidates (lower is better), each side used once.
  struct Candidate {
    double score;
    size_t i, j;
  };
  auto take = [&](std::vector<Candidate>& c, const char* by) {
    std::stable_sort(c.begin(), c.end(), [](const Candidate& x, const Candidate& y) { return x.score < y.score; });
    for (const auto& k : c)
      if (A[k.i].pair < 0 && B[k.j].pair < 0) join(k.i, k.j, by);
  };
  // node: one document's versions
  {
    std::unordered_map<std::string, size_t> at;
    for (size_t j = 0; j < B.size(); ++j) at[B[j].id] = j;
    for (size_t i = 0; i < A.size(); ++i)
      if (const auto it = at.find(A[i].id); it != at.end()) join(i, it->second, "node");
  }
  // How much two bodies' world boxes overlap (intersection over union) and how their sizes compare (>= 1).
  auto overlap = [&](size_t i, size_t j, double& ratio) {
    const BodyPrint &p = pa(i), &q = pb(j);
    ratio = 1e300;
    if (!p.known || !q.known) return 0.0;
    double inter = 1, va = 1, vb = 1;
    for (size_t k = 0; k < 3; ++k) {
      const double lo = std::max(p.box_min[k], q.box_min[k]), hi = std::min(p.box_max[k], q.box_max[k]);
      const double pad = 1e-6 * std::max(p.size, q.size);  // flat boxes (a sheet) still have an extent
      inter *= std::max(0.0, hi - lo) + pad;
      va *= p.box_max[k] - p.box_min[k] + pad;
      vb *= q.box_max[k] - q.box_min[k] + pad;
    }
    const double x = p.solid && q.solid ? p.volume : p.area, y = p.solid && q.solid ? q.volume : q.area;
    ratio = std::max(x, y) / std::max(std::min(x, y), 1e-300);
    return inter / std::max(va + vb - inter, 1e-300);
  };
  // A name says little on its own (a reader numbers unnamed bodies in the order it meets them): a pair by name must
  // overlap, or be of a like size.
  auto alike = [&](size_t i, size_t j) {
    double ratio;
    const double iou = overlap(i, j, ratio);
    return iou > 0.01 || ratio <= 2 || !pa(i).known || !pb(j).known;
  };
  // source: the file's own ids; several instances of one part pair by nearest place
  {
    std::map<std::string, std::pair<std::vector<size_t>, std::vector<size_t>>> groups;
    for (size_t i : unpaired(A))
      if (!A[i].source.empty()) groups[A[i].source].first.push_back(i);
    for (size_t j : unpaired(B))
      if (!B[j].source.empty()) groups[B[j].source].second.push_back(j);
    std::vector<Candidate> c;
    for (const auto& [source, g] : groups) {
      stop();
      if (g.first.size() == 1 && g.second.size() == 1) {
        join(g.first.front(), g.second.front(), "source");
        continue;
      }
      for (size_t i : g.first)
        for (size_t j : g.second) c.push_back({distance(pa(i).centre, pb(j).centre), i, j});
    }
    take(c, "source");
  }
  // source, by faces: bodies that share the ids of their faces (a body keeps those of the faces an edit left alone), most
  // shared first, then the nearest (instances of a part share them all). Readers may number each part from 1, so a pair
  // must also overlap or be of a like size.
  {
    std::vector<Candidate> c;
    const auto ub = unpaired(B);
    for (size_t i : unpaired(A)) {
      if (A[i].faces.empty()) continue;
      stop();
      for (size_t j : ub) {
        if (B[j].faces.empty()) continue;
        std::vector<std::string> both;
        std::set_intersection(A[i].faces.begin(), A[i].faces.end(), B[j].faces.begin(), B[j].faces.end(), std::back_inserter(both));
        const double shared = double(both.size()) / double(A[i].faces.size() + B[j].faces.size() - both.size());
        if (shared < 0.3 || !alike(i, j)) continue;
        const double away = distance(pa(i).centre, pb(j).centre) / std::max(pa(i).size, 1e-9);
        c.push_back({(1 - shared) + std::min(away, 1e6) * 1e-7, i, j});
      }
    }
    take(c, "source");
  }
  // shape: the same geometry; in place first, then the same name, then the nearest
  {
    const auto ua = unpaired(A), ub = unpaired(B);
    std::map<std::array<int, 3>, std::vector<size_t>> bucket;  // counts -> B's unpaired
    for (size_t j : ub) {
      stop();
      const BodyPrint& p = pb(j);
      if (p.known) bucket[{p.faces, p.edges, p.vertices}].push_back(j);
    }
    std::vector<Candidate> c;
    for (size_t i : ua) {
      stop();
      const BodyPrint& p = pa(i);
      if (!p.known) continue;
      const auto it = bucket.find({p.faces, p.edges, p.vertices});
      if (it == bucket.end()) continue;
      for (size_t j : it->second) {
        const BodyPrint& q = pb(j);
        if (!same_geometry(p, q)) continue;
        const double away = distance(p.centre, q.centre) / std::max(p.size, 1e-9);
        c.push_back({same_place(p, q) ? 0.0 : (A[i].path == B[j].path ? 1.0 : A[i].name == B[j].name ? 2.0 : 3.0) + std::min(away, 1e6) * 1e-7, i, j});
      }
    }
    take(c, "shape");
  }
  // name: the same path of names, else a name unique on both sides
  {
    std::multimap<std::string, size_t> paths, names;
    std::map<std::string, int> countA, countB;
    for (size_t j : unpaired(B)) paths.emplace(B[j].path, j), names.emplace(B[j].name, j), ++countB[B[j].name];
    for (size_t i : unpaired(A)) ++countA[A[i].name];
    for (size_t i : unpaired(A)) {
      const auto range = paths.equal_range(A[i].path);
      for (auto it = range.first; it != range.second; ++it)
        if (B[it->second].pair < 0 && alike(i, it->second)) {
          join(i, it->second, "name");
          break;
        }
    }
    for (size_t i : unpaired(A))
      if (countA[A[i].name] == 1 && countB[A[i].name] == 1)
        if (const auto it = names.find(A[i].name); it != names.end() && B[it->second].pair < 0 && alike(i, it->second)) join(i, it->second, "name");
  }
  // near: overlapping in place, of a like size
  {
    std::vector<Candidate> c;
    const auto ub = unpaired(B);
    for (size_t i : unpaired(A)) {
      stop();
      for (size_t j : ub) {
        double ratio;
        const double iou = overlap(i, j, ratio);
        if (iou >= 0.2 && ratio <= 3) c.push_back({-iou, i, j});
      }
    }
    take(c, "near");
  }
  // The pairs as changes: B's bodies in its order, then what only A has.
  std::vector<BodyChange> out;
  for (size_t j = 0; j < B.size(); ++j) {
    Item& y = B[j];
    BodyChange c;
    c.id = y.id;
    c.name = y.name;
    c.key_b = y.key;
    c.world_b = y.world;
    c.shown_b = y.shown;
    if (y.pair >= 0) {
      Item& x = A[size_t(y.pair)];
      c.id_a = x.id, c.name_a = x.name, c.key_a = x.key, c.world_a = x.world, c.shown_a = x.shown, c.matched_by = y.by;
      if (&da == &db && x.key == y.key) {
        c.kind = same_placement(x.world, y.world) ? BodyChange::Kind::Unchanged : BodyChange::Kind::Moved;
      } else {
        stop();
        const BodyPrint &p = pa(size_t(y.pair)), &q = pb(j);
        c.kind = p.known && q.known && same_geometry(p, q) ? (same_place(p, q) ? BodyChange::Kind::Unchanged : BodyChange::Kind::Moved) : BodyChange::Kind::Modified;
        if (p.known && q.known) c.translation = {q.centre[0] - p.centre[0], q.centre[1] - p.centre[1], q.centre[2] - p.centre[2]};
      }
    } else {
      c.kind = BodyChange::Kind::Added;
    }
    if (c.shown_a || c.shown_b) out.push_back(std::move(c));
  }
  for (auto& x : A) {
    if (x.pair >= 0 || !x.shown) continue;
    BodyChange c;
    c.kind = BodyChange::Kind::Removed;
    c.id = c.id_a = x.id;
    c.name = c.name_a = x.name;
    c.key_a = x.key;
    c.world_a = x.world;
    c.shown_a = true;
    out.push_back(std::move(c));
  }
  return out;
}

}  // namespace

Document read_variant(const std::filesystem::path& file, const std::function<bool(double)>& progress) {
  std::string ext = file.extension().string();
  for (auto& ch : ext) ch = char(std::tolower(static_cast<unsigned char>(ch)));
  if (ext == ".opad") return Document::load_index(file);
  Document doc = Document::create();
  ImportOptions opt;
  opt.viewer = true;  // live shapes: nothing healed, written or hashed, as the file has them
  if (progress) opt.progress = [progress](double f, const std::string&) { return progress(f < 0 ? 0 : f); };
  import_file(doc, file, opt);
  doc.path = file;
  return doc;
}

BodyPrint body_print(const Document& doc, const Scene& scene, const std::string& node) {
  const Node* n = scene.node(node);
  if (!n || n->kind != Node::Kind::Body) return {};
  return place(measure(doc, n->body_key), scene.world(node));
}

bool same_geometry(const BodyPrint& a, const BodyPrint& b, double tolerance) {
  if (!a.known || !b.known || a.solid != b.solid || a.faces != b.faces || a.edges != b.edges || a.vertices != b.vertices) return false;
  const double size = std::max(a.size, b.size), floor = 1e-9 * std::max(size, 1e-3);
  if (!close_to(a.volume, b.volume, tolerance, floor * size * size) || !close_to(a.area, b.area, tolerance, floor * size)) return false;
  const double top = std::max(a.moments[2], b.moments[2]);
  for (int k = 0; k < 3; ++k)
    if (std::abs(a.moments[k] - b.moments[k]) > tolerance * top + floor * size * size * size) return false;
  return true;  // not the boxes: an axis-aligned box changes with a turn
}

bool same_place(const BodyPrint& a, const BodyPrint& b, double tolerance) {
  const double size = std::max({a.size, b.size, 1e-3});
  if (distance(a.centre, b.centre) > tolerance * 10 * size) return false;
  const double top = std::max(a.moments[2], 1e-300);
  for (int k = 0; k < 3; ++k) {
    // An axis whose moment another shares (a cylinder's, a cube's) is not fixed by the shape: not compared.
    bool distinct = true;
    for (int o = 0; o < 3; ++o)
      if (o != k && std::abs(a.moments[k] - a.moments[o]) <= 1e-3 * top) distinct = false;
    if (!distinct) continue;
    const double dot = a.axes[k][0] * b.axes[k][0] + a.axes[k][1] * b.axes[k][1] + a.axes[k][2] * b.axes[k][2];
    if (std::abs(dot) < 1 - 1e-6) return false;
  }
  return true;
}

std::vector<BodyChange> match_bodies(const Document& a, const Scene& sa, const Document& b, const Scene& sb, const Cancel& cancel) {
  Prints prints;
  return match(a, sa, b, sb, cancel, prints);
}

json variant_diff(const std::vector<BodyChange>& bodies, const Scene& sa, const Scene& sb) {
  (void)sa;
  (void)sb;
  json changes = json::array();
  json counts = {{"unchanged", 0}, {"moved", 0}, {"modified", 0}, {"added", 0}, {"removed", 0}};
  json by = json::object();
  auto entry = [](const char* change, const BodyChange& c) {
    return json{{"kind", "body"}, {"change", change}, {"id", c.id}, {"name", c.name}, {"matched_by", c.matched_by}};
  };
  for (const auto& c : bodies) {
    if (!c.matched_by.empty()) by[c.matched_by] = by.value(c.matched_by, 0) + 1;
    switch (c.kind) {
      case BodyChange::Kind::Added:
        counts["added"] = counts["added"].get<int>() + 1;
        changes.push_back(entry("added", c));
        break;
      case BodyChange::Kind::Removed:
        counts["removed"] = counts["removed"].get<int>() + 1;
        changes.push_back(entry("removed", c));
        break;
      case BodyChange::Kind::Modified:
        counts["modified"] = counts["modified"].get<int>() + 1;
        changes.push_back(entry("geometry", c));
        break;
      case BodyChange::Kind::Moved: {
        counts["moved"] = counts["moved"].get<int>() + 1;
        json m = entry("moved", c);
        m["translation"] = c.translation;
        changes.push_back(std::move(m));
        break;
      }
      default:
        counts["unchanged"] = counts["unchanged"].get<int>() + 1;
    }
    if (!c.name_a.empty() && !c.name.empty() && c.name_a != c.name && c.kind != BodyChange::Kind::Added && c.kind != BodyChange::Kind::Removed) {
      json r = entry("renamed", c);
      r["before"] = c.name_a;
      r["after"] = c.name;
      changes.push_back(std::move(r));
    }
  }
  counts["by"] = by;
  std::string summary;
  auto part = [&](const char* key, const char* word) {
    const int n = counts[key].get<int>();
    if (!n) return;
    if (!summary.empty()) summary += ", ";
    summary += std::to_string(n) + " " + word;
  };
  part("modified", "changed");
  part("moved", "moved");
  part("added", "added");
  part("removed", "removed");
  part("unchanged", "unchanged");
  return {{"relation", "variants"}, {"changes", std::move(changes)}, {"counts", std::move(counts)},
          {"summary", summary.empty() ? std::string("No bodies") : "Variant: " + summary}};
}

json variant_table(const std::vector<const Document*>& docs, const std::vector<const Scene*>& scenes, const Cancel& cancel) {
  json rows = json::array();
  if (docs.empty() || docs.size() != scenes.size()) return rows;
  Prints prints;  // the baseline is measured once for every variant
  const size_t variants = docs.size() - 1;
  std::map<std::string, size_t> row_of;  // the baseline's node -> row
  for (const auto& id : scenes[0]->all_bodies()) {
    const Node* n = scenes[0]->node(id);
    if (!n || n->kind != Node::Kind::Body || !scenes[0]->effectively_visible(id)) continue;
    row_of[id] = rows.size();
    json cells = json::array();
    for (size_t k = 0; k < variants; ++k) cells.push_back({{"id", ""}, {"change", "absent"}, {"matched_by", ""}});
    rows.push_back({{"name", n->name}, {"a", id}, {"cells", std::move(cells)}});
  }
  std::map<std::string, size_t> added_row;  // a body only variants have, one row per name
  for (size_t k = 0; k < variants; ++k) {
    const auto changes = match(*docs[0], *scenes[0], *docs[k + 1], *scenes[k + 1], cancel, prints);
    for (const auto& c : changes) {
      const char* change = c.kind == BodyChange::Kind::Unchanged ? "unchanged"
                           : c.kind == BodyChange::Kind::Moved   ? "moved"
                           : c.kind == BodyChange::Kind::Modified ? "modified"
                           : c.kind == BodyChange::Kind::Added    ? "added"
                                                                  : "removed";
      size_t r;
      if (c.kind == BodyChange::Kind::Added) {
        auto it = added_row.find(c.name);
        if (it == added_row.end() || rows[it->second]["cells"][k]["change"] != "absent") {
          json cells = json::array();
          for (size_t m = 0; m < variants; ++m) cells.push_back({{"id", ""}, {"change", "absent"}, {"matched_by", ""}});
          rows.push_back({{"name", c.name}, {"a", ""}, {"cells", std::move(cells)}});
          it = added_row.insert_or_assign(c.name, rows.size() - 1).first;
        }
        r = it->second;
      } else {
        const auto it = row_of.find(c.id_a);
        if (it == row_of.end()) continue;
        r = it->second;
      }
      rows[r]["cells"][k] = {{"id", c.kind == BodyChange::Kind::Removed ? std::string() : c.id}, {"change", change}, {"matched_by", c.matched_by}};
    }
  }
  return rows;
}

}  // namespace opad
