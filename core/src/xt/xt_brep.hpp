#pragma once
// Parasolid transmit file nodes (xt_parse.hpp) -> OCCT shapes in an XCAF document: bodies as solids, sheets or wires
// built from their topology and geometry, assemblies as XCAF components with their transforms, names and colours from
// the system attributes. Lengths go from Parasolid's metres to millimetres. Not a public header.
#include <TDocStd_Document.hxx>

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "xt_parse.hpp"

namespace opad::xt {

struct Built {
  Handle(TDocStd_Document) doc;
  int bodies = 0;
  std::vector<std::string> warnings;
  // For surveys: each face (by its TShape) -> the kind of Parasolid surface it was built on ("BLENDED_EDGE on blend", ...)
  std::unordered_map<const void*, std::string> face_source;
};

// `progress(fraction)` returns false to cancel (then opad::Error "cancelled" is thrown).
Built build(const File& file, const std::function<bool(double)>& progress = {});

}  // namespace opad::xt
