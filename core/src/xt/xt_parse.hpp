#pragma once
// Parasolid transmit files (.x_t text, .x_b neutral or typed binary), read from the published "Parasolid XT Format
// Reference" (no Parasolid code): the header, the schema that lays out each node type's fields, and the nodes themselves.
// xt_brep.cpp turns the nodes into OCCT shapes. Not a public header.
//
// A file names its schema, e.g. SCH_3101290_31100_13006. Files written since Parasolid V14 with a third number carry their
// own layouts as edits of a base schema (13006) on the first node of each type; xt_schema.cpp holds that base and the
// layouts of files that carry none (older versions, and writers that use 12006 as the reference asks).
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace opad::xt {

// One field of a node type: its Parasolid type code (u c l n w d p f i v b h, t for tags), its element count (0 one value,
// 1 variable length, n an array of n) and whether it is written to the file at all.
struct FieldDef {
  std::string name;
  char type = 'd';
  int n = 0;
  bool xmt = true;
};

struct TypeDef {
  std::string name;
  std::vector<FieldDef> fields;
  bool variable() const {
    for (const auto& f : fields)
      if (f.n == 1 && f.xmt) return true;
    return false;
  }
  int find(std::string_view field) const {
    for (size_t i = 0; i < fields.size(); ++i)
      if (fields[i].name == field) return int(i);
    return -1;
  }
};

using Schema = std::map<int, TypeDef>;

constexpr double kNull = std::numeric_limits<double>::quiet_NaN();

// A field's values: numbers (characters and logicals as their codes, null as NaN), pointer indices (0 for null) and, for
// character fields, the text.
struct Value {
  std::vector<double> num;
  std::vector<uint32_t> ptr;
  std::string text;
};

struct Node {
  int type = 0;
  uint32_t index = 0;
  const TypeDef* def = nullptr;
  std::vector<Value> values;

  const Value* field(std::string_view name) const {
    const int i = def ? def->find(name) : -1;
    return i < 0 ? nullptr : &values[size_t(i)];
  }
  bool has(std::string_view name) const { return field(name) != nullptr; }
  double num(std::string_view name, size_t k = 0, double fallback = kNull) const {
    const Value* v = field(name);
    return v && k < v->num.size() ? v->num[k] : fallback;
  }
  int integer(std::string_view name, int fallback = 0) const {
    const double d = num(name);
    return std::isnan(d) ? fallback : int(d);
  }
  uint32_t ptr(std::string_view name, size_t k = 0) const {
    const Value* v = field(name);
    return v && k < v->ptr.size() ? v->ptr[k] : 0;
  }
  char chr(std::string_view name, char fallback = 0) const {
    const Value* v = field(name);
    return v && !v->text.empty() ? v->text[0] : fallback;
  }
  // A vector field (or three consecutive values of an array field from `k`); false when absent or null.
  bool vec(std::string_view name, std::array<double, 3>& out, size_t k = 0) const {
    const Value* v = field(name);
    if (!v || v->num.size() < k + 3) return false;
    for (int i = 0; i < 3; ++i) out[size_t(i)] = v->num[k + size_t(i)];
    return !std::isnan(out[0]);
  }
};

struct File {
  std::string modeller;          // "TRANSMIT FILE created by modeller version 3101290"
  std::string schema;            // SCH_3101290_31100_13006
  int schema_version = 0;        // 31100
  int base_version = 0;          // 13006 when the file carries its own layouts, else 0
  int user_fields = 0;
  char format = 'T';             // T text, N neutral binary, B typed binary
  std::string header;            // the human-readable header before the stream (PART1/2/3 keywords)
  std::unordered_map<uint32_t, Node> nodes;
  std::map<int, TypeDef> layouts;  // the layout each node type was read with
  std::vector<std::string> warnings;

  const Node* get(uint32_t index) const {
    if (index == 0) return nullptr;
    auto it = nodes.find(index);
    return it == nodes.end() ? nullptr : &it->second;
  }
  const Node* get(uint32_t index, int type) const {
    const Node* n = get(index);
    return n && n->type == type ? n : nullptr;
  }
};

// Reads a whole transmit file. Throws opad::Error naming what could not be read (an unknown schema, a damaged stream).
File parse(const std::string& bytes);

// The layout a file without embedded layouts is read with (xt_schema.cpp): the base for 13006 and older, otherwise the
// nearest known version at or below `version` adjusted by its recorded edits. Never empty.
const Schema& base_schema();
Schema schema_for(int version);
// Whether xt_schema.cpp knows `version` exactly (its layout was recorded, not guessed from the nearest older one).
bool schema_known(int version);

// Node types (XT Format Reference, "Node Types").
enum NodeType : int {
  ASSEMBLY = 10, INSTANCE = 11, BODY = 12, SHELL = 13, FACE = 14, LOOP = 15, EDGE = 16, FIN = 17, VERTEX = 18, REGION = 19,
  POINT = 29, LINE = 30, CIRCLE = 31, ELLIPSE = 32, INTERSECTION = 38, CHART = 40, LIMIT = 41, BSPLINE_VERTICES = 45,
  PLANE = 50, CYLINDER = 51, CONE = 52, SPHERE = 53, TORUS = 54, BLENDED_EDGE = 56, BLEND_BOUND = 59, OFFSET_SURF = 60,
  SWEPT_SURF = 67, SPUN_SURF = 68, LIST = 70, POINTER_LIS_BLOCK = 74, ATT_DEF_ID = 79, ATTRIB_DEF = 80, ATTRIBUTE = 81,
  INT_VALUES = 82, REAL_VALUES = 83, CHAR_VALUES = 84, POINT_VALUES = 85, VECTOR_VALUES = 86, AXIS_VALUES = 87,
  TAG_VALUES = 88, DIRECTION_VALUES = 89, GROUP = 90, MEMBER_OF_GROUP = 91, UNICODE_VALUES = 98, FIELD_NAMES = 99,
  TRANSFORM = 100, WORLD = 101, KEY = 102, PE_SURF = 120, INT_PE_DATA = 121, EXT_PE_DATA = 122, B_SURFACE = 124,
  SURFACE_DATA = 125, NURBS_SURF = 126, KNOT_MULT = 127, KNOT_SET = 128, PE_CURVE = 130, TRIMMED_CURVE = 133,
  B_CURVE = 134, CURVE_DATA = 135, NURBS_CURVE = 136, SP_CURVE = 137, GEOMETRIC_OWNER = 141, HELIX_CU_FORM = 163,
  HELIX_SU_FORM = 184,
};

}  // namespace opad::xt
