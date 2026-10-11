// Node layouts of Parasolid transmit files (see xt_parse.hpp).
//
// The base schema (13006, Parasolid V13) is what files with embedded layouts edit, and what older files and files from
// other writers (who use 12006, as the reference asks) are read with. It is written from the published "Parasolid XT
// Format Reference" (April 2008), field by field in transmit order; the layouts it shares with parasolid-kit
// (github.com/monozukuri-ai/parasolid-kit, MIT/Apache-2.0: its reviewed 13006 subset) were checked against that.
// Field names follow the reference, so the B-rep builder finds them whatever version added fields around them.
#include <algorithm>
#include <cstdlib>

#include "xt_parse.hpp"

namespace opad::xt {
namespace {

FieldDef F(const char* name, char type, int n = 0) { return FieldDef{name, type, n, true}; }

// The seven fields every curve and surface starts with.
std::vector<FieldDef> geometry(std::vector<FieldDef> more) {
  std::vector<FieldDef> f = {F("node_id", 'd'), F("attributes_groups", 'p'), F("owner", 'p'), F("next", 'p'),
                             F("previous", 'p'), F("geometric_owner", 'p'), F("sense", 'c')};
  f.insert(f.end(), more.begin(), more.end());
  return f;
}

Schema make_base() {
  Schema s;
  auto add = [&](int type, const char* name, std::vector<FieldDef> fields) { s[type] = TypeDef{name, std::move(fields)}; };
  add(ASSEMBLY, "ASSEMBLY",
      {F("highest_node_id", 'd'), F("attributes_groups", 'p'), F("attribute_chains", 'p'), F("list", 'p'), F("surface", 'p'),
       F("curve", 'p'), F("point", 'p'), F("key", 'p'), F("res_size", 'f'), F("res_linear", 'f'), F("ref_instance", 'p'),
       F("next", 'p'), F("previous", 'p'), F("state", 'u'), F("owner", 'p'), F("type", 'u'), F("sub_instance", 'p')});
  add(INSTANCE, "INSTANCE",
      {F("node_id", 'd'), F("attributes_groups", 'p'), F("type", 'u'), F("part", 'p'), F("transform", 'p'), F("assembly", 'p'),
       F("next_in_part", 'p'), F("prev_in_part", 'p'), F("next_of_part", 'p'), F("prev_of_part", 'p')});
  add(BODY, "BODY",
      {F("highest_node_id", 'd'), F("attributes_groups", 'p'), F("attribute_chains", 'p'), F("surface", 'p'), F("curve", 'p'),
       F("point", 'p'), F("key", 'p'), F("res_size", 'f'), F("res_linear", 'f'), F("ref_instance", 'p'), F("next", 'p'),
       F("previous", 'p'), F("state", 'u'), F("owner", 'p'), F("body_type", 'u'), F("nom_geom_state", 'u'), F("shell", 'p'),
       F("boundary_surface", 'p'), F("boundary_curve", 'p'), F("boundary_point", 'p'), F("region", 'p'), F("edge", 'p'),
       F("vertex", 'p')});
  add(SHELL, "SHELL",
      {F("node_id", 'd'), F("attributes_groups", 'p'), F("body", 'p'), F("next", 'p'), F("face", 'p'), F("edge", 'p'),
       F("vertex", 'p'), F("region", 'p'), F("front_face", 'p')});
  add(FACE, "FACE",
      {F("node_id", 'd'), F("attributes_groups", 'p'), F("tolerance", 'f'), F("next", 'p'), F("previous", 'p'), F("loop", 'p'),
       F("shell", 'p'), F("surface", 'p'), F("sense", 'c'), F("next_on_surface", 'p'), F("previous_on_surface", 'p'),
       F("next_front", 'p'), F("previous_front", 'p'), F("front_shell", 'p')});
  add(LOOP, "LOOP", {F("node_id", 'd'), F("attributes_groups", 'p'), F("fin", 'p'), F("face", 'p'), F("next", 'p')});
  add(EDGE, "EDGE",
      {F("node_id", 'd'), F("attributes_groups", 'p'), F("tolerance", 'f'), F("fin", 'p'), F("previous", 'p'), F("next", 'p'),
       F("curve", 'p'), F("next_on_curve", 'p'), F("previous_on_curve", 'p'), F("owner", 'p')});
  add(FIN, "FIN",
      {F("attributes_groups", 'p'), F("loop", 'p'), F("forward", 'p'), F("backward", 'p'), F("vertex", 'p'), F("other", 'p'),
       F("edge", 'p'), F("curve", 'p'), F("next_at_vx", 'p'), F("sense", 'c')});
  add(VERTEX, "VERTEX",
      {F("node_id", 'd'), F("attributes_groups", 'p'), F("fin", 'p'), F("previous", 'p'), F("next", 'p'), F("point", 'p'),
       F("tolerance", 'f'), F("owner", 'p')});
  add(REGION, "REGION",
      {F("node_id", 'd'), F("attributes_groups", 'p'), F("body", 'p'), F("next", 'p'), F("previous", 'p'), F("shell", 'p'),
       F("type", 'c')});
  add(POINT, "POINT",
      {F("node_id", 'd'), F("attributes_groups", 'p'), F("owner", 'p'), F("next", 'p'), F("previous", 'p'), F("pvec", 'v')});
  add(LINE, "LINE", geometry({F("pvec", 'v'), F("direction", 'v')}));
  add(CIRCLE, "CIRCLE", geometry({F("centre", 'v'), F("normal", 'v'), F("x_axis", 'v'), F("radius", 'f')}));
  add(ELLIPSE, "ELLIPSE",
      geometry({F("centre", 'v'), F("normal", 'v'), F("x_axis", 'v'), F("major_radius", 'f'), F("minor_radius", 'f')}));
  add(INTERSECTION, "INTERSECTION", geometry({F("surface", 'p', 2), F("chart", 'p'), F("start", 'p'), F("end", 'p')}));
  add(CHART, "CHART",
      {F("base_parameter", 'f'), F("base_scale", 'f'), F("chart_count", 'd'), F("chordal_error", 'f'), F("angular_error", 'f'),
       F("parameter_error", 'f', 2), F("hvec", 'h', 1)});
  add(LIMIT, "LIMIT", {F("type", 'c'), F("hvec", 'h', 1)});
  add(BSPLINE_VERTICES, "BSPLINE_VERTICES", {F("vertices", 'f', 1)});
  add(PLANE, "PLANE", geometry({F("pvec", 'v'), F("normal", 'v'), F("x_axis", 'v')}));
  add(CYLINDER, "CYLINDER", geometry({F("pvec", 'v'), F("axis", 'v'), F("radius", 'f'), F("x_axis", 'v')}));
  add(CONE, "CONE",
      geometry({F("pvec", 'v'), F("axis", 'v'), F("radius", 'f'), F("sin_half_angle", 'f'), F("cos_half_angle", 'f'),
                F("x_axis", 'v')}));
  add(SPHERE, "SPHERE", geometry({F("centre", 'v'), F("radius", 'f'), F("axis", 'v'), F("x_axis", 'v')}));
  add(TORUS, "TORUS",
      geometry({F("centre", 'v'), F("axis", 'v'), F("major_radius", 'f'), F("minor_radius", 'f'), F("x_axis", 'v')}));
  add(BLENDED_EDGE, "BLENDED_EDGE",
      geometry({F("blend_type", 'c'), F("surface", 'p', 2), F("spine", 'p'), F("range", 'f', 2), F("thumb_weight", 'f', 2),
                F("boundary", 'p', 2), F("start", 'p'), F("end", 'p')}));
  add(BLEND_BOUND, "BLEND_BOUND", geometry({F("boundary", 'n'), F("blend", 'p')}));
  add(OFFSET_SURF, "OFFSET_SURF",
      geometry({F("check", 'c'), F("true_offset", 'l'), F("surface", 'p'), F("offset", 'f'), F("scale", 'f')}));
  add(SWEPT_SURF, "SWEPT_SURF", geometry({F("section", 'p'), F("sweep", 'v'), F("scale", 'f')}));
  add(SPUN_SURF, "SPUN_SURF",
      geometry({F("profile", 'p'), F("base", 'v'), F("axis", 'v'), F("start", 'v'), F("end", 'v'), F("start_param", 'f'),
                F("end_param", 'f'), F("x_axis", 'v'), F("scale", 'f')}));
  add(LIST, "LIST",
      {F("node_id", 'd'), F("owner", 'p'), F("next", 'p'), F("previous", 'p'), F("list_type", 'd'), F("list_length", 'd'),
       F("block_length", 'd'), F("size_of_entry", 'd'), F("list_block", 'p'), F("finger_block", 'p'), F("finger_index", 'd'),
       F("notransmit", 'l')});
  add(POINTER_LIS_BLOCK, "POINTER_LIS_BLOCK", {F("n_entries", 'd'), F("next_block", 'p'), F("entries", 'p', 1)});
  add(ATT_DEF_ID, "ATT_DEF_ID", {F("string", 'c', 1)});
  add(ATTRIB_DEF, "ATTRIB_DEF",
      {F("next", 'p'), F("identifier", 'p'), F("type_id", 'd'), F("actions", 'u', 8), F("field_names", 'p'),
       F("legal_owners", 'l', 14), F("fields", 'u', 1)});
  add(ATTRIBUTE, "ATTRIBUTE",
      {F("node_id", 'd'), F("definition", 'p'), F("owner", 'p'), F("next", 'p'), F("previous", 'p'), F("next_of_type", 'p'),
       F("previous_of_type", 'p'), F("fields", 'p', 1)});
  add(INT_VALUES, "INT_VALUES", {F("values", 'd', 1)});
  add(REAL_VALUES, "REAL_VALUES", {F("values", 'f', 1)});
  add(CHAR_VALUES, "CHAR_VALUES", {F("values", 'c', 1)});
  add(POINT_VALUES, "POINT_VALUES", {F("values", 'v', 1)});
  add(VECTOR_VALUES, "VECTOR_VALUES", {F("values", 'v', 1)});
  add(AXIS_VALUES, "AXIS_VALUES", {F("values", 'v', 1)});
  add(TAG_VALUES, "TAG_VALUES", {F("values", 't', 1)});
  add(DIRECTION_VALUES, "DIRECTION_VALUES", {F("values", 'v', 1)});
  add(GROUP, "GROUP",
      {F("node_id", 'd'), F("attributes_groups", 'p'), F("owner", 'p'), F("next", 'p'), F("previous", 'p'), F("type", 'u'),
       F("first_member", 'p')});
  add(MEMBER_OF_GROUP, "MEMBER_OF_GROUP",
      {F("dummy_node_id", 'd'), F("owning_group", 'p'), F("owner", 'p'), F("next", 'p'), F("previous", 'p'),
       F("next_member", 'p'), F("previous_member", 'p')});
  add(UNICODE_VALUES, "UNICODE_VALUES", {F("values", 'w', 1)});
  add(FIELD_NAMES, "FIELD_NAMES", {F("names", 'p', 1)});
  add(TRANSFORM, "TRANSFORM",
      {F("node_id", 'd'), F("owner", 'p'), F("next", 'p'), F("previous", 'p'), F("rotation_matrix", 'f', 9),
       F("translation_vector", 'v'), F("scale", 'f'), F("flag", 'd'), F("perspective_vector", 'v')});
  add(WORLD, "WORLD",
      {F("assembly", 'p'), F("attribute", 'p'), F("body", 'p'), F("transform", 'p'), F("surface", 'p'), F("curve", 'p'),
       F("point", 'p'), F("alive", 'l'), F("attrib_def", 'p'), F("highest_id", 'd'), F("current_id", 'd')});
  add(KEY, "KEY", {F("string", 'c', 1)});
  add(PE_SURF, "PE_SURF", geometry({F("type", 'c'), F("data", 'p'), F("tf", 'p'), F("internal_geom", 'p', 1)}));
  add(INT_PE_DATA, "INT_PE_DATA", {F("geom_type", 'd'), F("real_array", 'p'), F("int_array", 'p')});
  add(EXT_PE_DATA, "EXT_PE_DATA", {F("key", 'p'), F("real_array", 'p'), F("int_array", 'p')});
  add(B_SURFACE, "B_SURFACE", geometry({F("nurbs", 'p'), F("data", 'p')}));
  add(SURFACE_DATA, "SURFACE_DATA",
      {F("original_uint", 'i'), F("original_vint", 'i'), F("extended_uint", 'i'), F("extended_vint", 'i'), F("self_int", 'u'),
       F("original_u_start", 'c'), F("original_u_end", 'c'), F("original_v_start", 'c'), F("original_v_end", 'c'),
       F("extended_u_start", 'c'), F("extended_u_end", 'c'), F("extended_v_start", 'c'), F("extended_v_end", 'c'),
       F("analytic_form_type", 'c'), F("swept_form_type", 'c'), F("spun_form_type", 'c'), F("blend_form_type", 'c'),
       F("analytic_form", 'p'), F("swept_form", 'p'), F("spun_form", 'p'), F("blend_form", 'p')});
  add(NURBS_SURF, "NURBS_SURF",
      {F("u_periodic", 'l'), F("v_periodic", 'l'), F("u_degree", 'n'), F("v_degree", 'n'), F("n_u_vertices", 'd'),
       F("n_v_vertices", 'd'), F("u_knot_type", 'u'), F("v_knot_type", 'u'), F("n_u_knots", 'd'), F("n_v_knots", 'd'),
       F("rational", 'l'), F("u_closed", 'l'), F("v_closed", 'l'), F("surface_form", 'u'), F("vertex_dim", 'n'),
       F("bspline_vertices", 'p'), F("u_knot_mult", 'p'), F("v_knot_mult", 'p'), F("u_knots", 'p'), F("v_knots", 'p')});
  add(KNOT_MULT, "KNOT_MULT", {F("mult", 'n', 1)});
  add(KNOT_SET, "KNOT_SET", {F("knots", 'f', 1)});
  add(PE_CURVE, "PE_CURVE", geometry({F("type", 'c'), F("data", 'p'), F("tf", 'p'), F("internal_geom", 'p', 1)}));
  add(TRIMMED_CURVE, "TRIMMED_CURVE",
      geometry({F("basis_curve", 'p'), F("point_1", 'v'), F("point_2", 'v'), F("parm_1", 'f'), F("parm_2", 'f')}));
  add(B_CURVE, "B_CURVE", geometry({F("nurbs", 'p'), F("data", 'p')}));
  add(CURVE_DATA, "CURVE_DATA", {F("self_int", 'u'), F("analytic_form", 'p')});
  add(NURBS_CURVE, "NURBS_CURVE",
      {F("degree", 'n'), F("n_vertices", 'd'), F("vertex_dim", 'n'), F("n_knots", 'd'), F("knot_type", 'u'),
       F("periodic", 'l'), F("closed", 'l'), F("rational", 'l'), F("curve_form", 'u'), F("bspline_vertices", 'p'),
       F("knot_mult", 'p'), F("knots", 'p')});
  add(SP_CURVE, "SP_CURVE",
      geometry({F("surface", 'p'), F("b_curve", 'p'), F("original", 'p'), F("tolerance_to_original", 'f')}));
  add(GEOMETRIC_OWNER, "GEOMETRIC_OWNER",
      {F("owner", 'p'), F("next", 'p'), F("previous", 'p'), F("shared_geometry", 'p')});
  // The reference swaps these two: in files the curve form has the gap and the surface form the point.
  add(HELIX_CU_FORM, "HELIX_CU_FORM",
      {F("axis_pt", 'v'), F("axis_dir", 'v'), F("hand", 'c'), F("turns", 'i'), F("pitch", 'f'), F("gap", 'f'),
       F("tol", 'f')});
  add(HELIX_SU_FORM, "HELIX_SU_FORM",
      {F("axis_pt", 'v'), F("axis_dir", 'v'), F("point", 'v'), F("hand", 'c'), F("turns", 'i'), F("pitch", 'f'),
       F("tol", 'f')});
  return s;
}

}  // namespace

const Schema& base_schema() {
  static const Schema s = make_base();
  return s;
}

namespace {
// How layouts changed after the base, as later files embed them (xt_layouts.inc, made by tools/xt_layouts.py).
struct Change {
  int type, since;
  const char* fields;
};
const Change kChanges[] = {
#include "xt_layouts.inc"
    // Seen in files without embedded layouts (Onshape, V28101): the multi-part root before mesh offsets came in.
    {176, 28101, "n_entries:d index_map_offset:d index_map:p schema_embedding_map:p entries:p1"},
    // V15 and V16 files already have the list layout the harvest first sees at 19008 (their bodies are still the base's).
    {70, 15000, "node_id:d list_type:u notransmit:l owner:p next:p previous:p list_length:d block_length:d finger_index:d finger_block:p list_block:p"},
    // V34101 and V35001 files already have the region's frame, the 16 attribute owner types and the body's boundary
    // lattice the harvest first sees at 35102.
    {80, 34101, "next:p identifier:p type_id:d actions:u8 field_names:p legal_owners:l16 fields:u1"},
    {19, 34101, "node_id:d attributes_groups:p body:p next:p previous:p shell:p frame:p type:c owner:p"},
    {12, 34101, "highest_node_id:d attributes_groups:p attribute_chains:p lattice:p surface:p curve:p point:p mesh:p polyline:p key:p res_size:f res_linear:f ref_instance:p next:p previous:p state:u owner:p body_type:u nom_geom_state:u shell:p boundary_lattice:p boundary_surface:p boundary_curve:p boundary_point:p boundary_mesh:p boundary_polyline:p region:p edge:p vertex:p index_map_offset:d index_map:p node_id_index_map:p schema_embedding_map:p child:p lowest_node_id:d mesh_offset_data:p"},
};

std::vector<FieldDef> parse_fields(const char* text) {
  std::vector<FieldDef> out;
  std::string t(text);
  size_t i = 0;
  while (i < t.size()) {
    size_t end = t.find(' ', i);
    if (end == std::string::npos) end = t.size();
    const std::string token = t.substr(i, end - i);
    i = end + 1;
    const size_t colon = token.rfind(':');
    if (token.empty() || colon == std::string::npos || colon + 1 >= token.size()) continue;
    FieldDef f;
    f.name = token.substr(0, colon);
    f.type = token[colon + 1];
    std::string rest = token.substr(colon + 2);
    if (!rest.empty() && rest.back() == '!') f.xmt = false, rest.pop_back();
    f.n = rest.empty() ? 0 : std::atoi(rest.c_str());
    out.push_back(std::move(f));
  }
  return out;
}
}  // namespace

Schema schema_for(int version) {
  Schema s = base_schema();
  if (version < 13000) {  // attribute field names came with V13; V10 knew 13 owner types, not 14
    TypeDef& def = s[ATTRIB_DEF];
    def.fields.erase(def.fields.begin() + def.find("field_names"));
    if (version < 11000) {
      def.fields[size_t(def.find("legal_owners"))].n = 13;
      // V10 fins carry no attributes, and curve data no helix form
      auto drop = [&](int type, const char* field) {
        TypeDef& t = s[type];
        if (const int k = t.find(field); k >= 0) t.fields.erase(t.fields.begin() + k);
      };
      drop(FIN, "attributes_groups");
      drop(CURVE_DATA, "analytic_form");
    }
    return s;
  }
  // Per type, the latest recorded layout at or below `version` (none: the base's); a node type the base lacks takes
  // its earliest record when none is that old.
  std::map<int, const Change*> pick;
  for (const Change& c : kChanges) {
    const auto it = pick.find(c.type);
    const bool fits = c.since <= version;
    if (it == pick.end()) {
      if (fits || !s.count(c.type)) pick[c.type] = &c;
    } else {
      const bool had = it->second->since <= version;
      if ((fits && (!had || c.since > it->second->since)) || (!fits && !had && c.since < it->second->since)) it->second = &c;
    }
  }
  for (const auto& [type, c] : pick) {
    TypeDef& def = s[type];
    if (def.name.empty()) def.name = "TYPE_" + std::to_string(type);
    def.fields = parse_fields(c->fields);
  }
  return s;
}

bool schema_known(int version) {
  if (version <= 13006) return true;
  for (const Change& c : kChanges)
    if (c.since == version) return true;
  return false;
}

}  // namespace opad::xt
