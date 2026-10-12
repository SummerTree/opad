#pragma once
// Variants of one design that share no history (exports of a model from a native CAD system, copies saved apart, a STEP
// beside an X_T): read from any file OPAD opens, and their bodies matched as Compare draws them (diff.hpp BodyChange).
// Matching goes from the surest to the loosest evidence, each step only over what is still unpaired:
//   node    the same node id (versions of one OPAD document)
//   source  the ids the file itself gave the body, or its faces (Parasolid's node_id, kept in the body meta "ids" on
//           import): the most faces shared first (an edit keeps the ids of the faces it left alone), instances of one
//           part (which share them all) by nearest place; a pair by face ids must overlap or be of a like size
//   shape   the same geometry: face, edge and vertex counts, volume, area and principal moments within a tolerance;
//           unchanged where its centre and principal axes are the same too, else moved (nearest first, its name preferred)
//   name    the same path of names (component names down to the body's), or the same name when it is unique on both
//           sides
//   near    what is left, by how much the world boxes overlap and how close the volumes are (a part reworked in place)
// A pair is unchanged or moved when its geometry is the same, else modified. Ids give no geometry: a source pair with
// other geometry is modified even where the shapes were barely touched; geometry has the last word on "unchanged".
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "diff.hpp"
#include "document.hpp"
#include "scene.hpp"

namespace opad {

// A file of any format OPAD opens as a document to compare: an .opad in index mode, anything else imported in viewer mode
// (its shapes live in the shape cache, nothing healed or hashed, ids and names as the reader left them). Throws Error.
Document read_variant(const std::filesystem::path& file, const std::function<bool(double)>& progress = {});

// What makes a body the same as another: counts, measures in its own frame, and where it is in the world.
struct BodyPrint {
  bool known = false;  // measured (false: the shape could not be read)
  bool solid = false;  // volume-based (else area-based: sheets, meshes, wires)
  int faces = 0, edges = 0, vertices = 0;
  double volume = 0, area = 0, size = 0;  // size: the box diagonal, for tolerances
  double moments[3] = {0, 0, 0};          // principal moments, ascending
  Vec3 centre{0, 0, 0};                   // world centre of mass
  Vec3 axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // world principal axes, by ascending moment
  Vec3 box_min{0, 0, 0}, box_max{0, 0, 0};           // world box
};
// The print of a body node (its shape placed in the world). Any thread.
BodyPrint body_print(const Document& doc, const Scene& scene, const std::string& node);
bool same_geometry(const BodyPrint& a, const BodyPrint& b, double tolerance = 1e-5);  // in its own frame
bool same_place(const BodyPrint& a, const BodyPrint& b, double tolerance = 1e-5);     // and where it is

using Cancel = std::function<bool()>;

// The bodies of two documents paired as above, in BodyChange form (id: b's node, or a's when removed; id_a, matched_by,
// name_a filled). Bodies hidden on both sides are left out. Throws Error("cancelled") when `cancel` says so.
std::vector<BodyChange> match_bodies(const Document& a, const Scene& sa, const Document& b, const Scene& sb, const Cancel& cancel = {});

// Those pairs as semantic_diff lists changes, so Compare's list shows them: {relation "variants", changes [{kind "body",
// change added|removed|geometry|moved|renamed, id, name, matched_by, translation (moved), before/after (renamed)}],
// counts {unchanged, moved, modified, added, removed, by: {node, source, shape, name, near}}, summary}.
json variant_diff(const std::vector<BodyChange>& bodies, const Scene& sa, const Scene& sb);

// Several variants against one (the first): per body of any of them, what each variant has of it. rows [{name, a (the
// baseline's node, "" when it has none), cells [{id (that variant's node, ""), change unchanged|moved|modified|added|
// removed|absent, matched_by}] one per variant after the baseline}]. `docs` and `scenes` in the same order.
json variant_table(const std::vector<const Document*>& docs, const std::vector<const Scene*>& scenes, const Cancel& cancel = {});

}  // namespace opad
