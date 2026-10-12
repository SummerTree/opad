// Variants that share no history matched body by body (opad/variants.hpp): the file's own ids first, then the same
// geometry (in place: unchanged; elsewhere: moved), then names, then what overlaps; the rest added or removed.
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <STEPControl_Writer.hxx>
#include <Interface_Static.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepTools.hxx>
#include <gp_Ax2.hxx>

#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <regex>
#include <sstream>

#include "check.hpp"
#include "opad/core.hpp"
#include "opad/drawing_io.hpp"
#include "opad/geometry.hpp"
#include "opad/variants.hpp"

using namespace opad;

namespace {
struct Part {
  std::string name;
  TopoDS_Shape shape;
  std::string source;  // the file's own body id ("" none)
  std::vector<std::string> faces = {};  // the file's own face ids (Parasolid's)
};

// One import of these parts under a component named after the file, as a reader leaves them.
Document make(const std::vector<Part>& parts, const std::string& file) {
  Document d = Document::create();
  json children = json::array();
  for (const auto& p : parts) {
    json meta = {{"name", p.name}};
    if (!p.source.empty()) meta["ids"] = {{"source", "parasolid"}, {"body", p.source}};
    if (!p.faces.empty()) meta["ids"] = {{"source", "parasolid"}, {"faces", p.faces}};
    const std::string key = d.add_body(brep_from_shape(p.shape), meta);
    children.push_back({{"type", "body"}, {"id", new_uuid()}, {"name", p.name}, {"key", key}});
  }
  d.append({{"op", "import"}, {"source", file}, {"nodes", json::array({{{"type", "component"}, {"id", new_uuid()}, {"name", file}, {"children", children}}})}});
  return d;
}

TopoDS_Shape box(double x, double y, double z, double dx, double dy, double dz) { return BRepPrimAPI_MakeBox(gp_Pnt(x, y, z), dx, dy, dz).Shape(); }
TopoDS_Shape rod(double x, double r, double h) { return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, 0, 0), gp::DZ()), r, h).Shape(); }
TopoDS_Shape turned(const TopoDS_Shape& s, double degrees, const gp_Pnt& about) {
  gp_Trsf t;
  t.SetRotation(gp_Ax1(about, gp::DZ()), degrees * M_PI / 180);
  return BRepBuilderAPI_Transform(s, t, true).Shape();
}

// What became of each body of b (and of a's that b has not), by name.
std::map<std::string, BodyChange> by_name(const std::vector<BodyChange>& changes) {
  std::map<std::string, BodyChange> out;
  for (const auto& c : changes) out[c.name] = c;
  return out;
}
}  // namespace

TEST(variants_match_by_geometry_then_names_then_overlap) {
  const Document a = make({{"Base", box(0, 0, 0, 40, 30, 20)},
                           {"Pin", box(60, 0, 0, 5, 5, 10)},
                           {"Lever", box(-40, 0, 0, 20, 4, 2)},
                           {"Gone", box(100, 0, 0, 3, 3, 3)},
                           {"Rod", rod(-20, 4, 30)},
                           {"Body 1", box(0, 100, 0, 2, 2, 2)}},
                          "v1.x_t");
  const Document b = make({{"Plate", box(0, 0, 0, 40, 30, 20)},                     // the same, renamed
                           {"Pin", box(70, 10, 0, 5, 5, 10)},                      // moved
                           {"Lever", turned(box(-40, 0, 0, 20, 4, 2), 30, gp_Pnt(-40, 0, 0))},  // turned about a corner
                           {"Rod", rod(-20, 5, 30)},                               // thicker
                           {"New", box(0, 50, 0, 10, 10, 10)},
                           {"Body 1", box(500, 500, 500, 50, 50, 50)}},            // a reader's numbering, not the same body
                          "v2.x_t");
  const Scene sa = resolve(a), sb = resolve(b);
  const auto changes = match_bodies(a, sa, b, sb);
  auto c = by_name(changes);
  using K = BodyChange::Kind;
  CHECK(c["Plate"].kind == K::Unchanged && c["Plate"].matched_by == "shape" && c["Plate"].name_a == "Base");
  CHECK(c["Pin"].kind == K::Moved && c["Pin"].matched_by == "shape");
  CHECK_NEAR(c["Pin"].translation[0], 10, 1e-6);
  CHECK_NEAR(c["Pin"].translation[1], 10, 1e-6);
  CHECK(c["Lever"].kind == K::Moved && c["Lever"].matched_by == "shape");
  CHECK(c["Rod"].kind == K::Modified && c["Rod"].matched_by == "name");
  CHECK(c["New"].kind == K::Added && c["New"].matched_by.empty());
  CHECK(c["Gone"].kind == K::Removed && !c["Gone"].id_a.empty());
  // "Body 1" on both sides, nowhere near each other and of another size: not paired by its name.
  int body1 = 0;
  for (const auto& x : changes)
    if (x.name == "Body 1") {
      ++body1;
      CHECK(x.kind == K::Added || x.kind == K::Removed);
    }
  CHECK_EQ(body1, 2);
  // As Compare lists them.
  const json d = variant_diff(changes, sa, sb);
  CHECK_EQ(d.value("relation", ""), "variants");
  const json& counts = d["counts"];
  CHECK(counts["unchanged"] == 1 && counts["moved"] == 2 && counts["modified"] == 1 && counts["added"] == 2 && counts["removed"] == 2);
  bool renamed = false;
  for (const auto& e : d["changes"])
    if (e.value("change", "") == "renamed") renamed = e.value("before", "") == "Base" && e.value("after", "") == "Plate";
  CHECK(renamed);
}

TEST(variants_match_by_the_files_own_ids_first) {
  // Parasolid's ids pair a body that changed shape and place; two instances of one part pair by the nearest place.
  const Document a = make({{"Body 1", box(0, 0, 0, 10, 10, 10), "17"}, {"Bolt", rod(50, 2, 10), "30"}, {"Bolt", rod(80, 2, 10), "30"}}, "a.x_t");
  const Document b = make({{"Body 1", box(300, 0, 0, 12, 10, 10), "17"}, {"Bolt", rod(81, 2, 10), "30"}, {"Bolt", rod(50, 2, 12), "30"}}, "b.x_t");
  const Scene sa = resolve(a), sb = resolve(b);
  const auto changes = match_bodies(a, sa, b, sb);
  CHECK_EQ(changes.size(), 3u);
  for (const auto& c : changes) CHECK_EQ(c.matched_by, "source");
  CHECK(changes[0].kind == BodyChange::Kind::Modified);
  CHECK(changes[1].kind == BodyChange::Kind::Moved);     // the bolt at 80 went to 81
  CHECK(changes[2].kind == BodyChange::Kind::Modified);  // the bolt at 50 got longer
  const BodyPrint moved_from = body_print(a, sa, changes[1].id_a);
  CHECK_NEAR(moved_from.centre[0], 80, 1e-6);
}

TEST(variants_match_by_the_ids_their_faces_share) {
  // Parasolid numbers faces, not bodies: the bracket lengthened and moved keeps most of its faces' ids; a far, bigger part
  // numbered the same way by another part's count is not taken for it.
  const std::vector<std::string> six = {"1", "2", "3", "4", "5", "6"};
  const Document a = make({{"Bracket", box(0, 0, 0, 10, 10, 10), "", six}, {"Housing", box(0, 0, 50, 30, 30, 5), "", {"40", "41", "42", "43", "44", "45"}}}, "a.x_t");
  const Document b = make({{"Housing", box(400, 400, 0, 90, 90, 90), "", six},  // numbered like the bracket
                           {"Bracket", box(2, 0, 0, 12, 10, 10), "", {"1", "2", "3", "4", "5", "9"}}},
                          "b.x_t");
  const Scene sa = resolve(a), sb = resolve(b);
  auto c = by_name(match_bodies(a, sa, b, sb));
  CHECK(c["Bracket"].kind == BodyChange::Kind::Modified && c["Bracket"].matched_by == "source" && c["Bracket"].name_a == "Bracket");
  CHECK(c["Housing"].matched_by != "source");
}

TEST(variants_match_by_the_faces_an_edit_kept) {
  // No ids, and the names a reader made up: a bracket drilled (four of its faces as they were), another drilled and moved;
  // a block of the same size elsewhere, with none of its faces in common, is not taken for either.
  auto drilled = [](const TopoDS_Shape& s, double x, double y) { return BRepAlgoAPI_Cut(s, BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, -1), gp::DZ()), 2, 50).Shape()).Shape(); };
  auto moved = [](const TopoDS_Shape& s, double dx) {
    gp_Trsf t;
    t.SetTranslation(gp_Vec(dx, 0, 0));
    return BRepBuilderAPI_Transform(s, t, true).Shape();
  };
  const Document a = make({{"Body 1", box(0, 0, 0, 40, 30, 20)}, {"Body 2", box(0, 100, 0, 20, 20, 10)}}, "a.step");
  const Document b = make({{"Solid 7", drilled(box(0, 0, 0, 40, 30, 20), 10, 10)},
                           {"Solid 8", moved(drilled(box(0, 100, 0, 20, 20, 10), 5, 105), 300)},
                           {"Solid 9", box(-200, 0, 0, 40, 30, 20.5)}},
                          "b.step");
  const Scene sa = resolve(a), sb = resolve(b);
  auto c = by_name(match_bodies(a, sa, b, sb));
  CHECK(c["Solid 7"].kind == BodyChange::Kind::Modified && c["Solid 7"].matched_by == "faces" && c["Solid 7"].name_a == "Body 1");
  CHECK(c["Solid 8"].kind == BodyChange::Kind::Modified && c["Solid 8"].matched_by == "faces" && c["Solid 8"].name_a == "Body 2");
  CHECK(c["Solid 9"].kind == BodyChange::Kind::Added);
}

TEST(variants_take_the_ids_a_step_exporter_wrote) {
  // Faces named by the exporter: unique names are ids; NONE everywhere is none.
  const std::filesystem::path dir = std::filesystem::temp_directory_path() / ("opad-variants-" + new_uuid());
  std::filesystem::create_directories(dir);
  STEPControl_Writer w;
  Interface_Static::SetCVal("write.step.unit", "MM");
  w.Transfer(box(0, 0, 0, 10, 20, 30), STEPControl_AsIs);
  const std::filesystem::path plain = dir / "plain.step";
  w.Write(plain.string().c_str());
  std::stringstream text;
  text << std::ifstream(plain).rdbuf();
  auto renamed = [&](const std::function<std::string(int)>& name) {
    std::string s = text.str(), out;
    std::regex face("ADVANCED_FACE\\('[^']*'");
    int n = 0;
    auto last = s.cbegin();
    for (std::sregex_iterator it(s.begin(), s.end(), face), end; it != end; ++it) {
      out.append(last, (*it)[0].first);
      out += "ADVANCED_FACE('" + name(++n) + "'";
      last = (*it)[0].second;
    }
    out.append(last, s.cend());
    return out;
  };
  std::ofstream(dir / "named.step") << renamed([](int n) { return "FACE-" + std::to_string(100 + n); });
  std::ofstream(dir / "none.step") << renamed([](int) { return std::string("NONE"); });
  for (const bool viewer : {false, true}) {
    auto ids_of = [viewer](const std::filesystem::path& file) {
      Document d = Document::create();
      ImportOptions opt;
      opt.viewer = viewer;
      import_file(d, file, opt);
      const Scene s = resolve(d);
      return d.body(s.node(s.all_bodies().front())->body_key)->meta.value("ids", json());
    };
    const json named = ids_of(dir / "named.step");
    CHECK_EQ(named.value("source", ""), "step");
    std::set<std::string> faces;
    for (const auto& f : named.value("faces", json::array())) faces.insert(f.get<std::string>());
    CHECK(faces.size() == 6 && faces.count("FACE-101") && faces.count("FACE-106"));
    CHECK(ids_of(dir / "none.step").is_null());
  }
  std::filesystem::remove_all(dir);
}

TEST(variants_table_lists_every_body_across_variants) {
  const Document base = make({{"Base", box(0, 0, 0, 40, 30, 20)}, {"Pin", box(60, 0, 0, 5, 5, 10)}}, "base.step");
  const Document v1 = make({{"Base", box(0, 0, 0, 40, 30, 20)}, {"Pin", box(65, 0, 0, 5, 5, 10)}}, "v1.step");
  const Document v2 = make({{"Base", box(0, 0, 0, 40, 30, 25)}, {"Clip", box(0, 60, 0, 4, 4, 4)}}, "v2.step");
  const Scene s0 = resolve(base), s1 = resolve(v1), s2 = resolve(v2);
  const json rows = variant_table({&base, &v1, &v2}, {&s0, &s1, &s2});
  CHECK_EQ(rows.size(), 3u);
  std::map<std::string, json> row;
  for (const auto& r : rows) row[r.value("name", "")] = r;
  CHECK(row["Base"]["cells"][0]["change"] == "unchanged" && row["Base"]["cells"][1]["change"] == "modified");
  CHECK(row["Pin"]["cells"][0]["change"] == "moved" && row["Pin"]["cells"][1]["change"] == "removed");
  CHECK(row["Clip"].value("a", "x").empty() && row["Clip"]["cells"][0]["change"] == "absent" && row["Clip"]["cells"][1]["change"] == "added");
}

TEST(variants_are_read_from_any_file) {
  const std::filesystem::path dir = std::filesystem::temp_directory_path() / ("opad-variants-" + new_uuid());
  std::filesystem::create_directories(dir);
  const std::filesystem::path brep = dir / "part.brep";
  BRepTools::Write(box(0, 0, 0, 10, 20, 30), brep.string().c_str());
  const Document d = read_variant(brep);
  const Scene s = resolve(d);
  CHECK_EQ(s.all_bodies().size(), 1u);
  CHECK(d.has_live_bodies());  // viewer mode: as the file has it, nothing written
  CHECK_NEAR(body_print(d, s, s.all_bodies().front()).volume, 6000, 1e-6);
  const std::filesystem::path opad = dir / "part.opad";
  Document saved = make({{"Box", box(0, 0, 0, 1, 1, 1)}}, "part.brep");
  saved.save_as(opad);
  CHECK_EQ(resolve(read_variant(opad)).all_bodies().size(), 1u);
  std::filesystem::remove_all(dir);
}

CHECK_MAIN()
