// Parasolid transmit files (.x_t, .x_b): the record parser (core/src/xt).
#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_Surface.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepCheck_ListOfStatus.hxx>
#include <BRepCheck_Result.hxx>
#include <BRepCheck_Status.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Quantity_Color.hxx>
#include <TCollection_AsciiString.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <TopExp_Explorer.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>

#include "check.hpp"
#include "../core/src/xt/xt_brep.hpp"
#include "../core/src/xt/xt_parse.hpp"
#include "opad/core.hpp"

using namespace opad;

namespace {
std::string read(const std::filesystem::path& p) {
  std::ifstream in(p, std::ios::binary);
  std::ostringstream s;
  s << in.rdbuf();
  return s.str();
}

// A transmit file written from the base schema (13006, no embedded layouts), as text or neutral binary: nodes give
// the fields they set, the others are written as zero, null or blank.
struct Writer {
  struct Out {
    int type = 0;
    std::map<std::string, std::vector<double>> num;
    std::map<std::string, std::vector<uint32_t>> ptr;
    std::map<std::string, std::string> text;
  };
  std::map<uint32_t, Out> nodes;
  uint32_t next = 1;

  uint32_t add(int type) {
    nodes[next].type = type;
    return next++;
  }
  Out& operator[](uint32_t i) { return nodes[i]; }

  std::string write(bool binary) const {
    const std::string modeller = ": TRANSMIT FILE created by modeller version 1300000", schema = "SCH_1300000_13006";
    std::string out = "**ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz**************************\n**END_OF_HEADER*****\n";
    auto u8 = [&](int x) { out.push_back(char(x & 0xFF)); };
    auto i16 = [&](int x) { u8(x >> 8), u8(x); };
    auto i32 = [&](long long x) { i16(int(x >> 16)), i16(int(x)); };
    auto f64 = [&](double x) {
      uint64_t b;
      std::memcpy(&b, &x, 8);
      for (int s = 56; s >= 0; s -= 8) u8(int(b >> s));
    };
    auto index = [&](uint32_t x) { i16(int(x) + 1); };  // small files: one short, the index plus one
    auto number = [&](double x) {
      char b[40];
      std::snprintf(b, sizeof b, "%.17g ", x);
      out += b;
    };
    if (binary) {
      out += std::string("PS\0\0", 4);
      i16(int(modeller.size())), out += modeller;
      i32(int(schema.size())), out += schema;
      i32(0);
    } else {
      out += "T" + std::to_string(modeller.size()) + " " + modeller + std::to_string(schema.size()) + " " + schema + "0 ";
    }
    const xt::Schema& s = xt::base_schema();
    for (const auto& [i, n] : nodes) {
      const xt::TypeDef& def = s.at(n.type);
      long long length = -1;
      for (const auto& f : def.fields) {
        if (f.n != 1) continue;
        if (auto t = n.text.find(f.name); t != n.text.end()) length = (long long)t->second.size();
        if (auto p = n.ptr.find(f.name); p != n.ptr.end()) length = (long long)p->second.size();
        if (auto v = n.num.find(f.name); v != n.num.end()) length = (long long)v->second.size();
        if (length < 0) length = 0;
      }
      binary ? i16(n.type) : number(n.type);
      if (length >= 0) binary ? i32(length) : number(double(length));
      binary ? index(i) : number(i);
      for (const auto& f : def.fields) {
        const int count = f.n == 0 ? 1 : f.n == 1 ? int(length) : f.n;
        const int k = f.type == 'v' ? 3 : f.type == 'i' ? 2 : f.type == 'b' ? 6 : 1;
        const auto p = n.ptr.find(f.name);
        const auto v = n.num.find(f.name);
        const auto t = n.text.find(f.name);
        for (int c = 0; c < count; ++c) {
          if (f.type == 'p') {
            const uint32_t x = p != n.ptr.end() && c < int(p->second.size()) ? p->second[c] : 0;
            binary ? index(x) : number(x);
          } else if (f.type == 'c') {
            u8(t != n.text.end() && c < int(t->second.size()) ? t->second[c] : ' ');
          } else if (f.type == 'l') {
            const bool b = v != n.num.end() && c < int(v->second.size()) && v->second[c] != 0;
            binary ? u8(b) : u8(b ? 'T' : 'F');
          } else {
            for (int j = 0; j < k; ++j) {
              const size_t at = size_t(c * k + j);
              const bool set = v != n.num.end() && at < v->second.size();
              const double x = set ? v->second[at] : f.type == 'f' || k > 1 ? -3.14158e13 : 0;
              if (!binary) number(x);
              else if (f.type == 'u') u8(int(x));
              else if (f.type == 'n' || f.type == 'w') i16(int(x));
              else if (f.type == 'd' || f.type == 't') i32((long long)x);
              else f64(x);
            }
          }
        }
      }
    }
    binary ? i16(1) : number(1);
    binary ? i16(0) : number(0);
    return out;
  }
};

// A 20 mm cube named "Cube", red-orange, as Parasolid writes one: a void and a solid region, one shell, six planar
// faces with their loops, 24 fins, 12 straight edges and 8 vertices; metres, like every transmit file.
Writer cube() {
  Writer w;
  const double L = 0.02;
  const uint32_t body = w.add(xt::BODY), outside = w.add(xt::REGION), inside = w.add(xt::REGION), shell = w.add(xt::SHELL);
  const uint32_t vshell = w.add(xt::SHELL);
  w[body].num["body_type"] = {1};
  w[body].ptr["region"] = {outside};
  w[body].ptr["shell"] = {shell};
  w[outside].text["type"] = "V", w[outside].ptr["body"] = {body}, w[outside].ptr["next"] = {inside}, w[outside].ptr["shell"] = {vshell};
  w[inside].text["type"] = "S", w[inside].ptr["body"] = {body}, w[inside].ptr["previous"] = {outside}, w[inside].ptr["shell"] = {shell};
  w[shell].ptr["body"] = {body}, w[shell].ptr["region"] = {inside};
  w[vshell].ptr["body"] = {body}, w[vshell].ptr["region"] = {outside};
  // name and colour attributes
  auto definition = [&](const std::string& id, int field_type) {
    const uint32_t d = w.add(xt::ATTRIB_DEF), s = w.add(xt::ATT_DEF_ID);
    w[s].text["string"] = id;
    w[d].ptr["identifier"] = {s};
    w[d].num["fields"] = {double(field_type)};
    return d;
  };
  const uint32_t name = w.add(xt::ATTRIBUTE), colour = w.add(xt::ATTRIBUTE);
  const uint32_t chars = w.add(xt::CHAR_VALUES), reals = w.add(xt::REAL_VALUES);
  w[chars].text["values"] = "Cube";
  w[reals].num["values"] = {0.8, 0.2, 0.1};
  w[name].ptr["definition"] = {definition("SDL/TYSA_NAME", 4)}, w[name].ptr["owner"] = {body}, w[name].ptr["next"] = {colour};
  w[name].ptr["fields"] = {chars};
  w[colour].ptr["definition"] = {definition("SDL/TYSA_COLOUR", 2)}, w[colour].ptr["owner"] = {body};
  w[colour].ptr["previous"] = {name}, w[colour].ptr["fields"] = {reals};
  w[body].ptr["attributes_groups"] = {name};

  uint32_t vertex[8];
  for (int i = 0; i < 8; ++i) {
    vertex[i] = w.add(xt::VERTEX);
    const uint32_t p = w.add(xt::POINT);
    w[p].num["pvec"] = {L * (i & 1), L * ((i >> 1) & 1), L * ((i >> 2) & 1)};
    w[p].ptr["owner"] = {vertex[i]};
    w[vertex[i]].ptr["point"] = {p}, w[vertex[i]].ptr["owner"] = {body};
  }
  auto corner = [&](int i, int k) { return L * ((i >> k) & 1); };
  std::map<std::pair<int, int>, uint32_t> edges;     // by (low, high) vertex: the edge runs low -> high
  std::map<std::pair<int, int>, uint32_t> fins;      // by (from, to) vertex
  uint32_t previous_face = 0;
  for (int axis = 0; axis < 3; ++axis)
    for (int side = 0; side < 2; ++side) {
      // the face's corners anticlockwise seen from outside: (u, v) with u x v = the outward normal
      const int u = (axis + 1) % 3, v = (axis + 2) % 3;
      int q[4];
      const int order[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
      for (int c = 0; c < 4; ++c) {
        const int a = order[c][side ? 0 : 1], b = order[c][side ? 1 : 0];  // the -axis side runs the other way round
        q[c] = (side << axis) | (a << u) | (b << v);
      }
      const uint32_t face = w.add(xt::FACE), loop = w.add(xt::LOOP), plane = w.add(xt::PLANE);
      std::vector<double> normal(3, 0.0), x_axis(3, 0.0);
      normal[axis] = side ? 1 : -1, x_axis[u] = 1;
      w[plane].num["pvec"] = {corner(q[0], 0), corner(q[0], 1), corner(q[0], 2)};
      w[plane].num["normal"] = normal, w[plane].num["x_axis"] = x_axis, w[plane].text["sense"] = "+";
      w[plane].ptr["owner"] = {face};
      w[face].ptr["loop"] = {loop}, w[face].ptr["shell"] = {shell}, w[face].ptr["front_shell"] = {vshell};
      w[face].ptr["surface"] = {plane}, w[face].text["sense"] = "+", w[face].num["tolerance"] = {-3.14158e13};
      if (previous_face) {
        w[previous_face].ptr["next"] = {face}, w[previous_face].ptr["next_front"] = {face};
        w[face].ptr["previous"] = {previous_face}, w[face].ptr["previous_front"] = {previous_face};
      } else {
        w[shell].ptr["face"] = {face}, w[vshell].ptr["front_face"] = {face};
      }
      previous_face = face;
      w[loop].ptr["face"] = {face};
      uint32_t f[4];
      for (int c = 0; c < 4; ++c) f[c] = w.add(xt::FIN);
      w[loop].ptr["fin"] = {f[0]};
      for (int c = 0; c < 4; ++c) {
        const int from = q[c], to = q[(c + 1) % 4];
        const auto key = std::minmax(from, to);
        uint32_t& e = edges[{key.first, key.second}];
        if (!e) {
          e = w.add(xt::EDGE);
          const uint32_t line = w.add(xt::LINE);
          w[line].num["pvec"] = {corner(key.first, 0), corner(key.first, 1), corner(key.first, 2)};
          std::vector<double> d(3);
          for (int k = 0; k < 3; ++k) d[k] = (corner(key.second, k) - corner(key.first, k)) / L;
          w[line].num["direction"] = d, w[line].text["sense"] = "+", w[line].ptr["owner"] = {e};
          w[e].ptr["curve"] = {line}, w[e].ptr["owner"] = {body};
        }
        Writer::Out& fin = w[f[c]];
        fin.ptr["loop"] = {loop}, fin.ptr["forward"] = {f[(c + 1) % 4]}, fin.ptr["backward"] = {f[(c + 3) % 4]};
        fin.ptr["vertex"] = {vertex[to]}, fin.ptr["edge"] = {e}, fin.text["sense"] = from < to ? "+" : "-";
        if (from < to) w[e].ptr["fin"] = {f[c]};
        w[vertex[to]].ptr["fin"] = {f[c]};
        fins[{from, to}] = f[c];
      }
    }
  for (const auto& [ends, fin] : fins) w[fin].ptr["other"] = {fins.at({ends.second, ends.first})};
  return w;
}

void check_cube(const xt::File& f) {
  CHECK_EQ(f.schema_version, 13006);
  const xt::Built b = xt::build(f);
  CHECK_EQ(b.bodies, 1);
  Handle(XCAFDoc_ShapeTool) st = XCAFDoc_DocumentTool::ShapeTool(b.doc->Main());
  TDF_LabelSequence roots;
  st->GetFreeShapes(roots);
  CHECK_EQ(roots.Length(), 1);
  Handle(TDataStd_Name) name;
  CHECK(roots.Value(1).FindAttribute(TDataStd_Name::GetID(), name));
  CHECK(TCollection_AsciiString(name->Get()) == "Cube");
  Quantity_Color colour;
  CHECK(XCAFDoc_DocumentTool::ColorTool(b.doc->Main())->GetColor(roots.Value(1), XCAFDoc_ColorGen, colour));
  CHECK_NEAR(colour.Red(), 0.8, 1e-6);
  const TopoDS_Shape shape = st->GetShape(roots.Value(1));
  int solids = 0;
  for (TopExp_Explorer x(shape, TopAbs_SOLID); x.More(); x.Next()) ++solids;
  CHECK_EQ(solids, 1);
  CHECK(BRepCheck_Analyzer(shape).IsValid());
  GProp_GProps g;
  BRepGProp::VolumeProperties(shape, g);
  CHECK_NEAR(g.Mass(), 8000.0, 1e-6);  // mm: 20 mm cube, outward faces (a turned-over shell measures -8000)
}
}  // namespace

TEST(cube_text) {
  const std::string text = cube().write(false);
  const xt::File f = xt::parse(text);
  CHECK(f.format == 'T');
  check_cube(f);
}

TEST(cube_neutral_binary) {
  const xt::File f = xt::parse(cube().write(true));
  CHECK(f.format == 'N');
  CHECK_EQ(f.nodes.size(), xt::parse(cube().write(false)).nodes.size());
  check_cube(f);
}

TEST(text_lexing) {
  // escapes in strings (\9 = nine spaces, \\ a backslash), a decimal comma, and lines broken anywhere
  Writer w = cube();
  std::string text = w.write(false);
  const size_t at = text.find("Cube");
  CHECK(at != std::string::npos);
  // "Cube" (4 chars) -> "C\\b\9e" = 'C', '\', 'b', nine spaces, 'e': 13 characters; the length before the index changes
  const size_t len = text.rfind(" 4 ", at);
  CHECK(len != std::string::npos);
  text.replace(at, 4, "C\\\\b\\9e");
  text.replace(len, 3, " 13 ");
  const size_t red = text.find("0.80000000000000004 ");
  CHECK(red != std::string::npos);
  text[red + 1] = ',';
  // writers drop the spaces that end a line: never break right after a separator
  std::string wrapped;
  for (size_t i = 0; i < text.size();) {
    size_t n = std::min<size_t>(80, text.size() - i);
    while (n > 1 && i + n < text.size() && text[i + n - 1] == ' ') --n;
    wrapped += text.substr(i, n) + "\r\n";
    i += n;
  }
  const xt::File f = xt::parse(wrapped);
  bool found = false;
  double r = 0;
  for (const auto& [i, n] : f.nodes) {
    if (n.type == xt::CHAR_VALUES) found = n.field("values")->text == std::string("C\\b") + std::string(9, ' ') + "e";
    if (n.type == xt::REAL_VALUES) r = n.field("values")->num.at(0);
  }
  CHECK(found);
  CHECK_NEAR(r, 0.8, 1e-12);
  CHECK_THROWS(xt::parse("not a transmit file"));
}

// The public transmit files tests/corpus/fetch.py downloads, each built when present: its solids, how many of them
// OCCT finds invalid at most, and its volume in mm^3 (assemblies counted per instance).
TEST(public_corpus) {
  struct Case {
    const char* file;
    int solids, invalid;
    double volume;
  };
  const Case cases[] = {
      {"pymapdl-CubeWithHole.x_t", 1, 0, 60858.4},
      {"pymechanical-hsec.x_t", 3, 0, 2700000.0},
      {"scorec-sphere.x_t", 1, 0, 0.469145},
      {"quickdraw-omron-assembly.x_t", 10, 0, 2251.5},
      {"quickdraw-microswitch-dock.x_t", 1, 0, 6674.9},
      {"sparkfun-display.x_t", 18, 0, 5526.8},
  };
  for (const Case& c : cases) {
    const std::filesystem::path path = std::filesystem::path(OPAD_SOURCE_DIR) / "tests" / "corpus" / c.file;
    if (!std::filesystem::exists(path)) continue;
    const xt::Built b = xt::build(xt::parse(read(path)));
    Handle(XCAFDoc_ShapeTool) st = XCAFDoc_DocumentTool::ShapeTool(b.doc->Main());
    TDF_LabelSequence roots;
    st->GetFreeShapes(roots);
    int solids = 0, invalid = 0;
    double volume = 0;
    for (int i = 1; i <= roots.Length(); ++i)
      for (TopExp_Explorer x(st->GetShape(roots.Value(i)), TopAbs_SOLID); x.More(); x.Next()) {
        ++solids;
        if (!BRepCheck_Analyzer(x.Current()).IsValid()) ++invalid;
        GProp_GProps g;
        BRepGProp::VolumeProperties(x.Current(), g);
        volume += g.Mass();
      }
    std::printf("  %-32s solids %d invalid %d volume %.1f\n", c.file, solids, invalid, volume);
    CHECK_EQ(solids, c.solids);
    CHECK(invalid <= c.invalid);
    CHECK_NEAR(volume, c.volume, c.volume * 1e-3);
  }
}

// OPAD_XT_CORPUS=<folder>: parses every file in it and lists what failed (development survey, skipped without it).
// OPAD_XT_HARVEST=<file>: also writes every layout each schema version uses, as the files embed it.
TEST(corpus_survey) {
  const char* dir = std::getenv("OPAD_XT_CORPUS");
  if (!dir || !*dir) return;
  int ok = 0, failed = 0;
  std::map<int, std::map<int, std::map<std::string, std::string>>> harvested;
  const char* harvest = std::getenv("OPAD_XT_HARVEST");
  for (const auto& e : std::filesystem::directory_iterator(dir)) {
    if (e.path().extension() == ".json") continue;
    const std::string name = e.path().filename().string();
    try {
      const xt::File f = xt::parse(read(e.path()));
      ++ok;
      std::printf("  ok   %-26s %c %6zu nodes  %s\n", f.schema.c_str(), f.format, f.nodes.size(), name.substr(0, 70).c_str());
      if (harvest && f.base_version)
        for (const auto& [type, def] : f.layouts) {
          std::string text;
          for (const auto& fd : def.fields)
            text += fd.name + ":" + fd.type + (fd.n ? std::to_string(fd.n) : "") + (fd.xmt ? "" : "!") + " ";
          harvested[f.schema_version][type][text] += name.substr(0, 40) + "; ";
        }
    } catch (const std::exception& x) {
      ++failed;
      std::printf("  FAIL %s: %s\n", name.substr(0, 70).c_str(), x.what());
    }
  }
  std::printf("corpus: %d parsed, %d failed\n", ok, failed);
  if (harvest) {
    std::ofstream out(harvest);
    for (const auto& [version, types] : harvested)
      for (const auto& [type, layouts] : types)
        for (const auto& [layout, files] : layouts) out << version << ' ' << type << " | " << layout << "| " << files.substr(0, 80) << "\n";
  }
}

// OPAD_XT_BUILD=<folder>: builds every file in it and reports per file its solids, how many are valid, their volume and
// the time taken (development survey, skipped without it).
TEST(build_survey) {
  const char* dir = std::getenv("OPAD_XT_BUILD");
  if (!dir || !*dir) return;
  int files = 0, failed = 0, solids_total = 0, invalid_total = 0;
  for (const auto& e : std::filesystem::directory_iterator(dir)) {
    if (e.path().extension() == ".json") continue;
    const std::string name = e.path().filename().string();
    const auto t0 = std::chrono::steady_clock::now();
    try {
      const xt::File f = xt::parse(read(e.path()));
      const xt::Built b = xt::build(f);
      Handle(XCAFDoc_ShapeTool) st = XCAFDoc_DocumentTool::ShapeTool(b.doc->Main());
      TDF_LabelSequence shapes;
      st->GetShapes(shapes);
      int solids = 0, invalid = 0, faces = 0;
      double volume = 0;
      for (int i = 1; i <= shapes.Length(); ++i) {
        if (st->IsAssembly(shapes.Value(i)) || st->IsComponent(shapes.Value(i))) continue;
        const TopoDS_Shape shape = st->GetShape(shapes.Value(i));
        for (TopExp_Explorer x(shape, TopAbs_SOLID); x.More(); x.Next()) {
          ++solids;
          if (!BRepCheck_Analyzer(x.Current()).IsValid()) ++invalid;
          GProp_GProps g;
          BRepGProp::VolumeProperties(x.Current(), g);
          volume += g.Mass();
        }
        for (TopExp_Explorer x(shape, TopAbs_FACE); x.More(); x.Next()) ++faces;
      }
      const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      ++files;
      solids_total += solids, invalid_total += invalid;
      std::printf("  %-44s bodies %4d solids %4d invalid %3d faces %6d vol %12.1f  %7.0f ms%s\n", name.substr(0, 44).c_str(), b.bodies,
                  solids, invalid, faces, volume, ms, b.warnings.empty() ? "" : ("  ! " + b.warnings.front()).substr(0, 90).c_str());
      std::fflush(stdout);
    } catch (const std::exception& x) {
      ++failed;
      std::printf("  FAIL %s: %s\n", name.substr(0, 60).c_str(), x.what());
    } catch (const Standard_Failure& x) {
      ++failed;
      std::printf("  FAIL %s: OCCT %s\n", name.substr(0, 60).c_str(), x.GetMessageString());
    }
  }
  std::printf("built %d files (%d failed): %d solids, %d invalid\n", files, failed, solids_total, invalid_total);
}

// OPAD_XT_ONE=<file>: builds one file and explains each invalid solid face by face (development diagnostics).
TEST(one_file) {
  const char* file = std::getenv("OPAD_XT_ONE");
  if (!file || !*file) return;
  const xt::File f = xt::parse(read(file));
  std::printf("schema %s, %zu nodes\n", f.schema.c_str(), f.nodes.size());
  for (const auto& [index, n] : f.nodes) {
    if (n.type != xt::BODY) continue;
    std::string regions;
    for (uint32_t r = n.ptr("region"), g = 0; r && g < 100; ++g) {
      const xt::Node* rn = f.get(r);
      if (!rn) break;
      regions += rn->chr("type", '?');
      r = rn->ptr("next");
    }
    std::printf("  body %u: type %d, regions %s\n", index, n.integer("body_type"), regions.c_str());
  }
  const xt::Built b = xt::build(f);
  for (const auto& w : b.warnings) std::printf("  warning: %s\n", w.c_str());
  Handle(XCAFDoc_ShapeTool) st = XCAFDoc_DocumentTool::ShapeTool(b.doc->Main());
  TDF_LabelSequence shapes;
  st->GetShapes(shapes);
  const char* kinds[] = {"plane", "cylinder", "cone", "sphere", "torus", "bezier", "bspline", "revolution", "extrusion", "offset", "other"};
  for (int i = 1; i <= shapes.Length(); ++i) {
    if (st->IsAssembly(shapes.Value(i)) || st->IsComponent(shapes.Value(i))) continue;
    const TopoDS_Shape shape = st->GetShape(shapes.Value(i));
    int solids = 0, shells = 0, faces = 0;
    for (TopExp_Explorer x(shape, TopAbs_SOLID); x.More(); x.Next()) ++solids;
    for (TopExp_Explorer x(shape, TopAbs_SHELL); x.More(); x.Next()) ++shells;
    for (TopExp_Explorer x(shape, TopAbs_FACE); x.More(); x.Next()) ++faces;
    BRepCheck_Analyzer an(shape);
    std::printf("shape %d: %d solids, %d shells, %d faces, %s\n", i, solids, shells, faces, an.IsValid() ? "valid" : "INVALID");
    if (an.IsValid()) continue;
    int shown = 0;
    for (TopExp_Explorer x(shape, TopAbs_FACE); x.More() && shown < 12; x.Next()) {
      const TopoDS_Face face = TopoDS::Face(x.Current());
      BRepCheck_Analyzer fa(face);
      if (fa.IsValid()) continue;
      ++shown;
      std::string why;
      for (TopExp_Explorer y(face, TopAbs_EDGE); y.More(); y.Next()) {
        try {  // an edge the analyser did not record throws
          const Handle(BRepCheck_Result)& r = fa.Result(y.Current());
          if (r.IsNull()) continue;
          for (BRepCheck_ListIteratorOfListOfStatus it(r->StatusOnShape(face)); it.More(); it.Next())
            if (it.Value() != BRepCheck_NoError) why += " edge:" + std::to_string(int(it.Value()));
        } catch (const Standard_Failure&) {
        }
      }
      const Handle(BRepCheck_Result)& r = fa.Result(face);
      if (!r.IsNull())
        for (BRepCheck_ListIteratorOfListOfStatus it(r->Status()); it.More(); it.Next())
          if (it.Value() != BRepCheck_NoError) why += " face:" + std::to_string(int(it.Value()));
      double worst = 0;  // how far the face's edges lie from its surface
      {
        Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
        for (TopExp_Explorer y(face, TopAbs_EDGE); y.More(); y.Next()) {
          double f0, f1;
          Handle(Geom_Curve) c = BRep_Tool::Curve(TopoDS::Edge(y.Current()), f0, f1);
          if (c.IsNull()) continue;
          for (int q = 0; q <= 4; ++q) {
            GeomAPI_ProjectPointOnSurf pr(c->Value(f0 + (f1 - f0) * q / 4), surf);
            if (pr.NbPoints()) worst = std::max(worst, pr.LowerDistance());
          }
        }
      }
      why += " off:" + std::to_string(worst);
      for (TopExp_Explorer w(face, TopAbs_WIRE); w.More(); w.Next()) {  // each loop: edges, seams, degenerate, closed
        int edges = 0, seams = 0, degenerate = 0;
        for (TopExp_Explorer ed(w.Current(), TopAbs_EDGE); ed.More(); ed.Next()) {
          ++edges;
          if (BRep_Tool::IsClosed(TopoDS::Edge(ed.Current()), face)) ++seams;
          if (BRep_Tool::Degenerated(TopoDS::Edge(ed.Current()))) ++degenerate;
        }
        why += " wire[" + std::to_string(edges) + "e " + std::to_string(seams) + "s " + std::to_string(degenerate) + "d" +
               (BRep_Tool::IsClosed(w.Current()) ? " closed]" : " open]");
      }
      {
        double u0, u1, v0, v1;
        BRepTools::UVBounds(face, u0, u1, v0, v1);
        char b[120];
        std::snprintf(b, sizeof b, " uv %.4g..%.4g x %.4g..%.4g", u0, u1, v0, v1);
        why += b;
      }
      const int k = int(BRepAdaptor_Surface(face).GetType());
      std::printf("    bad face (%s):%s\n", kinds[std::min(k, 10)], why.c_str());
    }
  }
}

// OPAD_XT_FACES=<folder>: builds every file and counts faces by the Parasolid surface they came from, and how many of
// each OCCT finds invalid (development survey of what still goes wrong).
TEST(face_survey) {
  const char* dir = std::getenv("OPAD_XT_FACES");
  if (!dir || !*dir) return;
  std::map<std::string, std::pair<int, int>> kinds;  // kind -> (faces, invalid)
  std::map<std::string, std::map<std::string, int>> where;  // kind -> file -> invalid
  for (const auto& e : std::filesystem::directory_iterator(dir)) {
    if (e.path().extension() == ".json") continue;
    try {
      const xt::Built b = xt::build(xt::parse(read(e.path())));
      Handle(XCAFDoc_ShapeTool) st = XCAFDoc_DocumentTool::ShapeTool(b.doc->Main());
      TDF_LabelSequence shapes;
      st->GetShapes(shapes);
      for (int i = 1; i <= shapes.Length(); ++i) {
        if (st->IsAssembly(shapes.Value(i)) || st->IsComponent(shapes.Value(i))) continue;
        for (TopExp_Explorer x(st->GetShape(shapes.Value(i)), TopAbs_FACE); x.More(); x.Next()) {
          const auto it = b.face_source.find(x.Current().TShape().get());
          const std::string kind = it == b.face_source.end() ? "?" : it->second;
          auto& k = kinds[kind];
          ++k.first;
          bool bad = false;
          try {
            bad = !BRepCheck_Analyzer(x.Current()).IsValid();
          } catch (const Standard_Failure&) {
            bad = true;
          }
          if (bad) ++k.second, ++where[kind][e.path().filename().string().substr(0, 40)];
        }
      }
    } catch (...) {
    }
  }
  for (const auto& [kind, n] : kinds) {
    std::printf("  %-34s faces %7d invalid %5d\n", kind.c_str(), n.first, n.second);
    int shown = 0;
    std::vector<std::pair<int, std::string>> files;
    for (const auto& [file, count] : where[kind]) files.push_back({count, file});
    std::sort(files.rbegin(), files.rend());
    for (const auto& [count, file] : files)
      if (shown++ < 3) std::printf("      %4d %s\n", count, file.c_str());
  }
}

// OPAD_XT_STATUS=<folder>: for every invalid solid, the checker's complaints by sub-shape kind and status (what makes
// solids invalid when their faces each pass), with the files that have most of each.
TEST(status_survey) {
  const char* dir = std::getenv("OPAD_XT_STATUS");
  if (!dir || !*dir) return;
  static const char* kinds[] = {"compound", "compsolid", "solid", "shell", "face", "wire", "edge", "vertex", "shape"};
  std::map<std::string, int> counts, solids_with;
  std::map<std::string, std::map<std::string, int>> where;
  int invalid = 0;
  for (const auto& e : std::filesystem::directory_iterator(dir)) {
    if (e.path().extension() == ".json") continue;
    const std::string file = e.path().filename().string().substr(0, 40);
    try {
      const xt::Built b = xt::build(xt::parse(read(e.path())));
      Handle(XCAFDoc_ShapeTool) st = XCAFDoc_DocumentTool::ShapeTool(b.doc->Main());
      TDF_LabelSequence shapes;
      st->GetShapes(shapes);
      for (int i = 1; i <= shapes.Length(); ++i) {
        if (st->IsAssembly(shapes.Value(i)) || st->IsComponent(shapes.Value(i))) continue;
        for (TopExp_Explorer x(st->GetShape(shapes.Value(i)), TopAbs_SOLID); x.More(); x.Next()) {
          BRepCheck_Analyzer an(x.Current());
          if (an.IsValid()) continue;
          ++invalid;
          std::set<std::string> seen;
          auto note = [&](const TopoDS_Shape& sub, const TopoDS_Shape& context) {
            Handle(BRepCheck_Result) r;
            try {
              r = an.Result(sub);
            } catch (const Standard_Failure&) {
              return;
            }
            if (r.IsNull()) return;
            auto add = [&](const BRepCheck_ListOfStatus& list) {
              for (BRepCheck_ListIteratorOfListOfStatus it(list); it.More(); it.Next()) {
                if (it.Value() == BRepCheck_NoError) continue;
                const std::string key = std::string(kinds[int(sub.ShapeType())]) + " " + std::to_string(int(it.Value()));
                ++counts[key], ++where[key][file], seen.insert(key);
              }
            };
            add(r->Status());
            if (!context.IsNull()) {
              try {
                add(r->StatusOnShape(context));
              } catch (const Standard_Failure&) {
              }
            }
          };
          note(x.Current(), {});
          for (TopExp_Explorer s(x.Current(), TopAbs_SHELL); s.More(); s.Next()) note(s.Current(), x.Current());
          for (TopExp_Explorer f(x.Current(), TopAbs_FACE); f.More(); f.Next()) {
            note(f.Current(), {});
            for (TopExp_Explorer w(f.Current(), TopAbs_WIRE); w.More(); w.Next()) note(w.Current(), f.Current());
            for (TopExp_Explorer ed(f.Current(), TopAbs_EDGE); ed.More(); ed.Next()) note(ed.Current(), f.Current());
          }
          for (const auto& k : seen) ++solids_with[k];
        }
      }
    } catch (...) {
    }
  }
  std::printf("invalid solids: %d (status codes: BRepCheck_Status)\n", invalid);
  for (const auto& [key, n] : counts) {
    std::printf("  %-14s %6d in %3d solids;", key.c_str(), n, solids_with[key]);
    std::vector<std::pair<int, std::string>> files;
    for (const auto& [f, c] : where[key]) files.push_back({c, f});
    std::sort(files.rbegin(), files.rend());
    for (size_t k = 0; k < files.size() && k < 2; ++k) std::printf(" %s (%d)", files[k].second.c_str(), files[k].first);
    std::printf("\n");
  }
}

// OPAD_XT_ONE=<file> OPAD_XT_NODES=<i,j,...>: prints those nodes field by field, then what they point at, two levels deep.
TEST(dump_nodes) {
  const char* file = std::getenv("OPAD_XT_ONE");
  const char* list = std::getenv("OPAD_XT_NODES");
  if (!file || !list) return;
  const xt::File f = xt::parse(read(file));
  std::function<void(uint32_t, int)> dump = [&](uint32_t i, int depth) {
    const xt::Node* n = f.get(i);
    if (!n) return;
    std::printf("%*s%u %s:", depth * 2, "", i, n->def->name.c_str());
    std::vector<uint32_t> next;
    for (size_t k = 0; k < n->def->fields.size(); ++k) {
      const auto& fd = n->def->fields[k];
      const auto& v = n->values[k];
      std::printf(" %s=", fd.name.c_str());
      if (!v.text.empty() && fd.type == 'c') std::printf("'%s'", v.text.c_str());
      else if (!v.ptr.empty()) for (auto p : v.ptr) std::printf("%u,", p);
      else for (size_t q = 0; q < v.num.size() && q < 9; ++q) std::printf("%g,", v.num[q]);
      if (fd.type == 'p' && fd.name != "next" && fd.name != "previous" && fd.name != "owner" && fd.name != "attributes_groups" &&
          fd.name != "geometric_owner")
        for (auto p : v.ptr) if (p) next.push_back(p);
    }
    std::printf("\n");
    if (depth < 2) for (auto p : next) dump(p, depth + 1);
  };
  std::string l(list);
  size_t a = 0;
  while (a < l.size()) {
    size_t b = l.find(',', a);
    if (b == std::string::npos) b = l.size();
    dump(uint32_t(std::stoul(l.substr(a, b - a))), 0);
    a = b + 1;
  }
}

CHECK_MAIN()
