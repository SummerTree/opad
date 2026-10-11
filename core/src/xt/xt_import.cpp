// Parasolid transmit files (.x_t, .x_b, .xmt_txt, .xmt_bin) into a document: parsed (xt_parse.cpp), built into an XCAF
// document (xt_brep.cpp) and walked like a STEP file's, so names, colours, assemblies and viewer mode work the same.
#include <Standard_Failure.hxx>
#include <TCollection_AsciiString.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <fstream>
#include <sstream>

#include "../import_common.hpp"
#include "opad/core.hpp"
#include "xt_brep.hpp"
#include "xt_parse.hpp"

namespace opad::detail {

ImportResult import_parasolid(Document& doc, const std::filesystem::path& file, const ImportOptions& opt) {
  std::ifstream in(file, std::ios::binary);
  if (!in) throw Error("cannot open " + file.filename().string());
  std::ostringstream bytes;
  bytes << in.rdbuf();
  if (opt.progress && !opt.progress(-1, "parsing")) throw Error("import cancelled");
  xt::File parsed;
  try {
    parsed = xt::parse(bytes.str());
  } catch (const Error& e) {
    throw Error("cannot read " + file.filename().string() + ": " + e.what());
  }
  std::function<bool(double)> progress;
  if (opt.progress) progress = [&opt](double f) { return opt.progress(f, "building bodies"); };
  xt::Built built;
  try {
    built = xt::build(parsed, progress);
  } catch (const Error& e) {
    if (std::string(e.what()) == "cancelled") throw Error("import cancelled");
    throw;
  } catch (const Standard_Failure& e) {
    throw Error("cannot build " + file.filename().string() + ": " + e.GetMessageString());
  }
  {  // a file of one unnamed part: the part takes the file's name
    Handle(XCAFDoc_ShapeTool) st = XCAFDoc_DocumentTool::ShapeTool(built.doc->Main());
    TDF_LabelSequence roots;
    st->GetFreeShapes(roots);
    Handle(TDataStd_Name) name;
    if (roots.Length() == 1 && roots.Value(1).FindAttribute(TDataStd_Name::GetID(), name)) {
      const TCollection_AsciiString text(name->Get());
      if (text.IsEqual("Body 1") || text.IsEqual("Assembly 1"))
        TDataStd_Name::Set(roots.Value(1), TCollection_ExtendedString(file.stem().u16string().c_str()));
    }
  }
  ImportResult result = import_xcaf(doc, built.doc, file, opt, false);
  for (const auto& w : parsed.warnings) result.warnings.push_back(w);
  for (const auto& w : built.warnings) result.warnings.push_back(w);
  if (!xt::schema_known(parsed.schema_version) && !parsed.base_version)
    result.warnings.push_back("Parasolid schema " + std::to_string(parsed.schema_version) +
                              " was read with the layouts of the nearest known version");
  return result;
}

}  // namespace opad::detail
