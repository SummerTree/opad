// Parasolid nodes -> OCCT shapes and an XCAF document (see xt_brep.hpp).
//
// Geometry follows the parameterisations of the XT Format Reference, so a curve or surface here evaluates where
// Parasolid's does: elementary surfaces map onto OCCT's directly (the cone on a left-handed frame, its axis pointing
// away from the half in use); a spun surface swaps u and v, which turns its normal over (`flip`). What Parasolid keeps
// procedural is rebuilt: intersection curves through their chart points with Parasolid's own parameters, curves in a
// surface's parameter space sampled onto that surface, rolling-ball blends from their spine and supports.
// Topology is built as Parasolid shares it (one edge per EDGE, used by each fin with the fin's sense), then healed
// with ShapeFix for what OCCT needs and Parasolid does not keep (curves on the faces, seams, natural bounds).
#include "xt_brep.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepLib_FindSurface.hxx>
#include <BRepLib.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Circle.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Line.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_SurfaceOfLinearExtrusion.hxx>
#include <Geom_SurfaceOfRevolution.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Precision.hxx>
#include <Quantity_Color.hxx>
#include <ShapeAnalysis_Curve.hxx>
#include <ShapeBuild_ReShape.hxx>
#include <Approx_CurveOnSurface.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <ShapeFix_Face.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeFix_Solid.hxx>
#include <Standard_ConstructionError.hxx>
#include <Standard_Failure.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array2OfReal.hxx>
#include <TColStd_HArray1OfReal.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColgp_HArray1OfPnt.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDataStd_Name.hxx>
#include <TDF_Label.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Ax3.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <map>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include "opad/core.hpp"

namespace opad::xt {
namespace {

constexpr double S = 1000.0;  // Parasolid's metres -> mm
constexpr int kGuard = 1000000;  // longest chain walked

using V3 = std::array<double, 3>;
gp_Pnt P(const V3& v) { return gp_Pnt(v[0] * S, v[1] * S, v[2] * S); }
gp_Dir D(const V3& v) {
  if (!(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] > 1e-30)) throw Standard_ConstructionError("a null direction");
  return gp_Dir(v[0], v[1], v[2]);
}
gp_Vec Vv(const V3& v) { return gp_Vec(v[0], v[1], v[2]); }

struct Surf {
  Handle(Geom_Surface) s;
  bool flip = false;   // OCCT's natural normal is opposite Parasolid's
  bool done = false;
};

struct Crv {
  Handle(Geom_Curve) c;
  int sense = 1;       // +1: the curve runs the way its geometry's parameter does
  bool done = false;
};

class Builder {
 public:
  explicit Builder(const File& f) : f_(f) {}

  const File& f_;
  std::vector<std::string> warnings;

  std::string type_name(int type) const {
    const auto it = f_.layouts.find(type);
    return it != f_.layouts.end() ? it->second.name : std::to_string(type);
  }
  void warn(const std::string& w) {
    if (warned_.insert(w).second && warnings.size() < 50) warnings.push_back(w);
  }

  // ---- attributes (system attribute definitions: SDL/TYSA_NAME, SDL/TYSA_COLOUR)
  const Node* attribute(const Node& n, std::string_view id) const {
    uint32_t a = n.ptr("attributes_groups");
    for (int guard = 0; a && guard < 10000; ++guard) {
      const Node* an = f_.get(a);
      if (!an) break;
      if (an->type == ATTRIBUTE) {
        const Node* def = f_.get(an->ptr("definition"), ATTRIB_DEF);
        const Node* idn = def ? f_.get(def->ptr("identifier"), ATT_DEF_ID) : nullptr;
        const Value* s = idn ? idn->field("string") : nullptr;
        if (s && s->text == id) return an;
      }
      a = an->ptr("next");
    }
    return nullptr;
  }
  const Node* attribute_value(const Node& attr, size_t k) const {
    const Value* fields = attr.field("fields");
    return fields && k < fields->ptr.size() ? f_.get(fields->ptr[k]) : nullptr;
  }
  std::string name_of(const Node& n) const {
    const Node* a = attribute(n, "SDL/TYSA_NAME");
    const Node* v = a ? attribute_value(*a, 0) : nullptr;
    if (!v) return {};
    if (v->type == CHAR_VALUES) {
      const Value* t = v->field("values");
      return t ? t->text : std::string();
    }
    if (v->type == UNICODE_VALUES) {  // UTF-16 code units -> UTF-8
      const Value* t = v->field("values");
      std::string out;
      if (!t) return out;
      for (size_t i = 0; i < t->num.size(); ++i) {
        uint32_t c = uint32_t(int(t->num[i])) & 0xFFFF;
        if (c >= 0xD800 && c < 0xDC00 && i + 1 < t->num.size()) {
          const uint32_t lo = uint32_t(int(t->num[i + 1])) & 0xFFFF;
          c = 0x10000 + ((c - 0xD800) << 10) + (lo - 0xDC00), ++i;
        }
        if (c < 0x80) out += char(c);
        else if (c < 0x800) out += char(0xC0 | (c >> 6)), out += char(0x80 | (c & 0x3F));
        else if (c < 0x10000) out += char(0xE0 | (c >> 12)), out += char(0x80 | ((c >> 6) & 0x3F)), out += char(0x80 | (c & 0x3F));
        else out += char(0xF0 | (c >> 18)), out += char(0x80 | ((c >> 12) & 0x3F)), out += char(0x80 | ((c >> 6) & 0x3F)), out += char(0x80 | (c & 0x3F));
      }
      return out;
    }
    return {};
  }
  bool colour_of(const Node& n, Quantity_Color& out) const {
    const Node* a = attribute(n, "SDL/TYSA_COLOUR");
    const Node* v = a ? attribute_value(*a, 0) : nullptr;
    const Value* t = v && v->type == REAL_VALUES ? v->field("values") : nullptr;
    if (!t || t->num.size() < 3 || std::isnan(t->num[0])) return false;
    auto c = [](double x) { return std::clamp(x, 0.0, 1.0); };
    out = Quantity_Color(c(t->num[0]), c(t->num[1]), c(t->num[2]), Quantity_TOC_RGB);
    return true;
  }

  // ---- B-splines
  // Whether a knot vector is OCCT's periodic form for `poles` poles (Parasolid writes periodic B-splines that way), or
  // else the ordinary one (poles + degree + 1 knots with their multiplicities); -1 when it is neither.
  static int periodic_form(const std::vector<int>& m, int poles, int degree) {
    int sum = 0;
    for (int x : m) sum += x;
    if (sum == poles + degree + 1) return 0;
    if (m.size() >= 2 && m.front() == m.back() && sum - m.back() == poles) return 1;
    return -1;
  }
  // The knot arrays can be longer than the count the NURBS node states (padded with nulls and zero multiplicities).
  bool knots(const Node& nurbs, const char* count_field, const char* knot_field, const char* mult_field, std::vector<double>& k,
             std::vector<int>& m) const {
    const Node* ks = f_.get(nurbs.ptr(knot_field), KNOT_SET);
    const Node* ms = f_.get(nurbs.ptr(mult_field), KNOT_MULT);
    const Value* kv = ks ? ks->field("knots") : nullptr;
    const Value* mv = ms ? ms->field("mult") : nullptr;
    if (!kv || !mv) return false;
    size_t count = std::min(kv->num.size(), mv->num.size());
    if (nurbs.has(count_field) && nurbs.integer(count_field) > 0) count = std::min(count, size_t(nurbs.integer(count_field)));
    if (count < 2) return false;
    k.assign(kv->num.begin(), kv->num.begin() + count);
    m.clear();
    for (size_t i = 0; i < count; ++i) {
      if (std::isnan(k[i]) || mv->num[i] < 1) return false;
      m.push_back(int(mv->num[i]));
    }
    return true;
  }
  Handle(Geom_BSplineCurve) bcurve3(const Node& bc) {
    const Node* nurbs = f_.get(bc.ptr("nurbs"), NURBS_CURVE);
    if (!nurbs) return nullptr;
    const int degree = nurbs->integer("degree"), n = nurbs->integer("n_vertices"), dim = nurbs->integer("vertex_dim");
    const bool rational = nurbs->num("rational") > 0;
    const Node* vn = f_.get(nurbs->ptr("bspline_vertices"), BSPLINE_VERTICES);
    const Value* vv = vn ? vn->field("vertices") : nullptr;
    std::vector<double> k;
    std::vector<int> m;
    if (!vv || n < 2 || degree < 1 || int(vv->num.size()) < n * dim || !knots(*nurbs, "n_knots", "knots", "knot_mult", k, m)) return nullptr;
    if (dim != (rational ? 4 : 3)) return nullptr;
    TColgp_Array1OfPnt poles(1, n);
    TColStd_Array1OfReal weights(1, n);
    for (int i = 0; i < n; ++i) {
      const double* p = &vv->num[size_t(i * dim)];
      const double w = rational ? p[3] : 1.0;
      poles.SetValue(i + 1, gp_Pnt(p[0] / w * S, p[1] / w * S, p[2] / w * S));
      weights.SetValue(i + 1, w);
    }
    const int form = periodic_form(m, n, degree);
    if (form < 0) return nullptr;
    TColStd_Array1OfReal kk(1, int(k.size()));
    TColStd_Array1OfInteger mm(1, int(m.size()));
    for (size_t i = 0; i < k.size(); ++i) kk.SetValue(int(i) + 1, k[i]), mm.SetValue(int(i) + 1, m[i]);
    return rational ? new Geom_BSplineCurve(poles, weights, kk, mm, degree, form == 1) : new Geom_BSplineCurve(poles, kk, mm, degree, form == 1);
  }
  Handle(Geom2d_BSplineCurve) bcurve2(const Node& bc) {
    const Node* nurbs = f_.get(bc.ptr("nurbs"), NURBS_CURVE);
    if (!nurbs) return nullptr;
    const int degree = nurbs->integer("degree"), n = nurbs->integer("n_vertices"), dim = nurbs->integer("vertex_dim");
    const bool rational = nurbs->num("rational") > 0;
    const Node* vn = f_.get(nurbs->ptr("bspline_vertices"), BSPLINE_VERTICES);
    const Value* vv = vn ? vn->field("vertices") : nullptr;
    std::vector<double> k;
    std::vector<int> m;
    if (!vv || n < 2 || degree < 1 || int(vv->num.size()) < n * dim || !knots(*nurbs, "n_knots", "knots", "knot_mult", k, m)) return nullptr;
    if (dim != (rational ? 3 : 2)) return nullptr;
    TColgp_Array1OfPnt2d poles(1, n);
    TColStd_Array1OfReal weights(1, n);
    for (int i = 0; i < n; ++i) {
      const double* p = &vv->num[size_t(i * dim)];
      const double w = rational ? p[2] : 1.0;
      poles.SetValue(i + 1, gp_Pnt2d(p[0] / w, p[1] / w));
      weights.SetValue(i + 1, w);
    }
    const int form = periodic_form(m, n, degree);
    if (form < 0) return nullptr;
    TColStd_Array1OfReal kk(1, int(k.size()));
    TColStd_Array1OfInteger mm(1, int(m.size()));
    for (size_t i = 0; i < k.size(); ++i) kk.SetValue(int(i) + 1, k[i]), mm.SetValue(int(i) + 1, m[i]);
    return rational ? new Geom2d_BSplineCurve(poles, weights, kk, mm, degree, form == 1)
                    : new Geom2d_BSplineCurve(poles, kk, mm, degree, form == 1);
  }
  Handle(Geom_BSplineSurface) bsurface(const Node& bs) {
    const Node* nurbs = f_.get(bs.ptr("nurbs"), NURBS_SURF);
    if (!nurbs) return nullptr;
    const int ud = nurbs->integer("u_degree"), vd = nurbs->integer("v_degree");
    const int nu = nurbs->integer("n_u_vertices"), nv = nurbs->integer("n_v_vertices"), dim = nurbs->integer("vertex_dim");
    const bool rational = nurbs->num("rational") > 0;
    const Node* vn = f_.get(nurbs->ptr("bspline_vertices"), BSPLINE_VERTICES);
    const Value* vv = vn ? vn->field("vertices") : nullptr;
    std::vector<double> uk, vk;
    std::vector<int> um, vm;
    if (!vv || nu < 2 || nv < 2 || ud < 1 || vd < 1 || int(vv->num.size()) < nu * nv * dim) return nullptr;
    if (!knots(*nurbs, "n_u_knots", "u_knots", "u_knot_mult", uk, um) || !knots(*nurbs, "n_v_knots", "v_knots", "v_knot_mult", vk, vm)) return nullptr;
    if (dim != (rational ? 4 : 3)) return nullptr;
    TColgp_Array2OfPnt poles(1, nu, 1, nv);
    TColStd_Array2OfReal weights(1, nu, 1, nv);
    for (int i = 0; i < nu; ++i)
      for (int j = 0; j < nv; ++j) {
        const double* p = &vv->num[size_t((i * nv + j) * dim)];  // v varies fastest
        const double w = rational ? p[3] : 1.0;
        poles.SetValue(i + 1, j + 1, gp_Pnt(p[0] / w * S, p[1] / w * S, p[2] / w * S));
        weights.SetValue(i + 1, j + 1, w);
      }
    auto arr = [](const std::vector<double>& v) {
      TColStd_Array1OfReal a(1, int(v.size()));
      for (size_t i = 0; i < v.size(); ++i) a.SetValue(int(i) + 1, v[i]);
      return a;
    };
    auto iarr = [](const std::vector<int>& v) {
      TColStd_Array1OfInteger a(1, int(v.size()));
      for (size_t i = 0; i < v.size(); ++i) a.SetValue(int(i) + 1, v[i]);
      return a;
    };
    const int uform = periodic_form(um, nu, ud), vform = periodic_form(vm, nv, vd);
    if (uform < 0 || vform < 0) return nullptr;
    Handle(Geom_BSplineSurface) out =
        rational ? new Geom_BSplineSurface(poles, weights, arr(uk), arr(vk), iarr(um), iarr(vm), ud, vd, uform == 1, vform == 1)
                 : new Geom_BSplineSurface(poles, arr(uk), arr(vk), iarr(um), iarr(vm), ud, vd, uform == 1, vform == 1);
    // closed but written clamped (a tube): periodic, so that healing gives a face round it its seam (else it splits the
    // face at its two rings into slivers); the parameter range stays
    try {
      if (!out->IsUPeriodic() && out->IsUClosed()) out->SetUPeriodic();
      if (!out->IsVPeriodic() && out->IsVClosed()) out->SetVPeriodic();
    } catch (const Standard_Failure&) {
    }
    return out;
  }

  // ---- curves. The result's geometry is in mm; `sense` says which way the Parasolid curve runs along it.
  const Crv& curve(uint32_t i) {
    Crv& out = curves_[i];
    if (out.done) return out;
    out.done = true;
    const Node* n = f_.get(i);
    if (!n) return out;
    out.sense = n->chr("sense", '+') == '-' ? -1 : 1;
    try {
      V3 a, b, c;
      switch (n->type) {
        case LINE:
          if (n->vec("pvec", a) && n->vec("direction", b)) out.c = new Geom_Line(P(a), D(b));
          break;
        case CIRCLE:
          if (n->vec("centre", a) && n->vec("normal", b) && n->vec("x_axis", c))
            out.c = new Geom_Circle(gp_Ax2(P(a), D(b), D(c)), n->num("radius") * S);
          break;
        case ELLIPSE:
          if (n->vec("centre", a) && n->vec("normal", b) && n->vec("x_axis", c))
            out.c = new Geom_Ellipse(gp_Ax2(P(a), D(b), D(c)), n->num("major_radius") * S, n->num("minor_radius") * S);
          break;
        case B_CURVE:
          out.c = bcurve3(*n);
          break;
        case TRIMMED_CURVE: {  // its portion is where the edge's vertices are; it runs the way its basis does
          const Crv basis = curve(n->ptr("basis_curve"));
          out.c = basis.c;
          out.sense = basis.sense;
          break;
        }
        case INTERSECTION:
          out.c = intersection(*n);
          break;
        case SP_CURVE:
          out.c = sp_curve(*n);
          break;
        default:
          warn("curves of node type " + std::to_string(n->type) + " are not supported");
      }
    } catch (const Standard_Failure& e) {
      warn("a curve (" + type_name(n->type) + ") could not be built: " + e.GetMessageString());
      out.c.Nullify();
    }
    return out;
  }

  // An intersection curve through its chart points, at the parameters Parasolid gives them: t0 = base_parameter, each
  // chord adding its length times the scale (the reference's tangent correction is close to 1 for fine charts).
  Handle(Geom_Curve) intersection(const Node& n) {
    const Node* chart = f_.get(n.ptr("chart"), CHART);
    const Value* h = chart ? chart->field("hvec") : nullptr;
    if (!h || h->num.size() < 6) return nullptr;
    std::vector<gp_Pnt> pts;
    for (size_t k = 0; k + 2 < h->num.size(); k += 3) pts.emplace_back(h->num[k] * S, h->num[k + 1] * S, h->num[k + 2] * S);
    // A terminator's exact singular point lies just beyond the chart: the edge ends there.
    auto limit = [&](const char* field, bool at_start) {
      const Node* lim = f_.get(n.ptr(field), LIMIT);
      const Value* lh = lim ? lim->field("hvec") : nullptr;
      if (!lim || !lh || lh->num.size() < 3 || lim->chr("type") != 'T') return;
      const gp_Pnt p(lh->num[0] * S, lh->num[1] * S, lh->num[2] * S);
      if (at_start) pts.insert(pts.begin(), p);
      else pts.push_back(p);
    };
    limit("start", true);
    limit("end", false);
    const double base = chart->num("base_parameter", 0, 0), scale = chart->num("base_scale", 0, 1);
    std::vector<gp_Pnt> kept;
    std::vector<double> params;
    double t = std::isnan(base) ? 0 : base;
    const double f = std::isnan(scale) || scale == 0 ? 1.0 : scale;
    for (const gp_Pnt& p : pts) {
      if (!kept.empty()) {
        const double d = p.Distance(kept.back());
        if (d < 1e-7) continue;
        t += d / S * f;
      }
      kept.push_back(p);
      params.push_back(t);
    }
    // a closed intersection (a ring: its chart ends where it starts) is periodic, so edges may run across the junction
    if (kept.size() > 3 && kept.front().Distance(kept.back()) < 1e-6) {
      Handle(TColgp_HArray1OfPnt) hp = new TColgp_HArray1OfPnt(1, int(kept.size()) - 1);
      Handle(TColStd_HArray1OfReal) ht = new TColStd_HArray1OfReal(1, int(kept.size()));
      for (size_t i = 0; i + 1 < kept.size(); ++i) hp->SetValue(int(i) + 1, kept[i]);
      for (size_t i = 0; i < kept.size(); ++i) ht->SetValue(int(i) + 1, params[i]);
      try {
        GeomAPI_Interpolate interp(hp, ht, true, 1e-7);
        interp.Perform();
        if (interp.IsDone()) return interp.Curve();
      } catch (const Standard_Failure&) {
      }
    }
    return through(kept, params);
  }

  // A C2 curve through `pts` at `params` (cubic when there are enough points).
  static Handle(Geom_Curve) through(const std::vector<gp_Pnt>& pts, const std::vector<double>& params) {
    if (pts.size() < 2) return nullptr;
    if (pts.size() == 2) return new Geom_Line(pts[0], gp_Dir(gp_Vec(pts[0], pts[1])));
    Handle(TColgp_HArray1OfPnt) hp = new TColgp_HArray1OfPnt(1, int(pts.size()));
    Handle(TColStd_HArray1OfReal) ht = new TColStd_HArray1OfReal(1, int(pts.size()));
    for (size_t i = 0; i < pts.size(); ++i) hp->SetValue(int(i) + 1, pts[i]), ht->SetValue(int(i) + 1, params[i]);
    try {
      GeomAPI_Interpolate interp(hp, ht, false, 1e-7);
      interp.Perform();
      if (interp.IsDone()) return interp.Curve();
    } catch (const Standard_Failure&) {
    }
    try {
      TColgp_Array1OfPnt ap(1, int(pts.size()));
      for (size_t i = 0; i < pts.size(); ++i) ap.SetValue(int(i) + 1, pts[i]);
      GeomAPI_PointsToBSpline fit(ap, 1, 8, GeomAbs_C1, 1e-4);
      if (fit.IsDone()) return fit.Curve();
    } catch (const Standard_Failure&) {
    }
    return nullptr;
  }

  // A 2D curve of a surface in OCCT's parameters of that surface, when those are an affine map of Parasolid's (planes,
  // cylinders, cones, spheres, tori, B-surfaces, offsets, extrusions of lines): poles mapped, the parameter kept.
  Handle(Geom2d_BSplineCurve) occt_pcurve(uint32_t surf, const Handle(Geom2d_BSplineCurve)& c2) {
    const Node* sn = f_.get(surf);
    if (!sn || c2.IsNull()) return nullptr;
    double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
    for (int i = 1; i <= c2->NbPoles(); ++i) {
      const gp_Pnt2d p = c2->Pole(i);
      u0 = std::min(u0, p.X()), u1 = std::max(u1, p.X()), v0 = std::min(v0, p.Y()), v1 = std::max(v1, p.Y());
    }
    if (u1 - u0 < 1e-9) u1 = u0 + 1;
    if (v1 - v0 < 1e-9) v1 = v0 + 1;
    auto map = [&](double u, double v, gp_Pnt2d& out) {
      double x = u, y = v;
      if (!occt_params(*sn, x, y) || !std::isfinite(x) || !std::isfinite(y)) return false;
      out.SetCoord(x, y);
      return true;
    };
    gp_Pnt2d o, a, b;
    if (!map(u0, v0, o) || !map(u1, v0, a) || !map(u0, v1, b)) return nullptr;
    const gp_Vec2d du = gp_Vec2d(o, a) / (u1 - u0), dv = gp_Vec2d(o, b) / (v1 - v0);
    auto affine = [&](double u, double v) { return o.Translated(du * (u - u0) + dv * (v - v0)); };
    for (const auto& [u, v] : {std::pair{u1, v1}, std::pair{(u0 + u1) / 2, (v0 + 3 * v1) / 4}}) {
      gp_Pnt2d m;
      if (!map(u, v, m) || m.Distance(affine(u, v)) > 1e-9 * (1 + std::abs(m.X()) + std::abs(m.Y()))) return nullptr;
    }
    Handle(Geom2d_BSplineCurve) out = Handle(Geom2d_BSplineCurve)::DownCast(c2->Copy());
    for (int i = 1; i <= out->NbPoles(); ++i) {
      const gp_Pnt2d p = out->Pole(i);
      out->SetPole(i, affine(p.X(), p.Y()));
    }
    return out;
  }

  // The 3D curve of a 2D curve on a surface whose parameters OCCT shares (up to an affine map): approximated by OCCT
  // along the 2D curve's own parameter, split at its kinks; checked against Parasolid's evaluation of the surface.
  Handle(Geom_Curve) curve_on_surface(uint32_t surf, const Handle(Geom2d_BSplineCurve)& c2) {
    const Surf& s = surface(surf);
    if (s.s.IsNull()) return nullptr;
    Handle(Geom2d_BSplineCurve) pc = occt_pcurve(surf, c2);
    if (pc.IsNull()) return nullptr;
    try {
      const double t0 = pc->FirstParameter(), t1 = pc->LastParameter();
      Approx_CurveOnSurface ap(new Geom2dAdaptor_Curve(pc), new GeomAdaptor_Surface(s.s), t0, t1, 1e-4);
      ap.Perform(1000, 12, pc->Continuity() == GeomAbs_C0 ? GeomAbs_C0 : GeomAbs_C1, true, false);
      if (!ap.IsDone() || !ap.HasResult()) return nullptr;
      Handle(Geom_BSplineCurve) c = ap.Curve3d();
      if (c.IsNull()) return nullptr;
      for (int i = 0; i <= 16; ++i) {
        const double t = t0 + (t1 - t0) * i / 16;
        const gp_Pnt2d uv = c2->Value(t);
        gp_Pnt p;
        if (!evaluate(surf, uv.X(), uv.Y(), p) || p.Distance(c->Value(t)) > 1e-3) return nullptr;
      }
      return c;
    } catch (const Standard_Failure&) {
      return nullptr;
    }
  }

  // A curve in a surface's parameter space, sampled onto the surface at the 2D curve's own parameters.
  Handle(Geom_Curve) sp_curve(const Node& n) {
    const Node* bc = f_.get(n.ptr("b_curve"), B_CURVE);
    if (!bc) return nullptr;
    Handle(Geom2d_BSplineCurve) c2 = bcurve2(*bc);
    if (c2.IsNull()) return nullptr;
    const uint32_t surf = n.ptr("surface");
    if (Handle(Geom_Curve) c = curve_on_surface(surf, c2); !c.IsNull()) return c;
    // the 2D curve says little about the 3D one (a straight line on a helical surface winds many turns): sample more
    // until the curve through the samples meets the surface between them too
    const double t0 = c2->FirstParameter(), t1 = c2->LastParameter();
    Handle(Geom_Curve) best;
    for (int samples = std::clamp((c2->NbPoles() + c2->Degree()) * 4, 16, 400); samples <= 6400; samples *= 2) {
      std::vector<gp_Pnt> pts;
      std::vector<double> params;
      for (int i = 0; i <= samples; ++i) {
        const double t = t0 + (t1 - t0) * i / samples;
        const gp_Pnt2d uv = c2->Value(t);
        gp_Pnt p;
        if (!evaluate(surf, uv.X(), uv.Y(), p)) return best;
        if (!pts.empty() && p.Distance(pts.back()) < 1e-9) continue;
        pts.push_back(p);
        params.push_back(t);
      }
      Handle(Geom_Curve) c = through(pts, params);
      if (c.IsNull()) return best;
      best = c;
      double worst = 0;
      for (int i = 0; i < samples && worst <= 1e-3; i += std::max(1, samples / 64)) {
        const double t = t0 + (t1 - t0) * (i + 0.5) / samples;
        const gp_Pnt2d uv = c2->Value(t);
        gp_Pnt p;
        if (evaluate(surf, uv.X(), uv.Y(), p)) worst = std::max(worst, p.Distance(c->Value(t)));
      }
      if (worst <= 1e-3) break;
    }
    return best;
  }

  // ---- surfaces
  const Surf& surface(uint32_t i) {
    Surf& out = surfaces_[i];
    if (out.done) return out;
    out.done = true;
    const Node* n = f_.get(i);
    if (!n) return out;
    if (!building_.insert(i).second) return out;  // a cycle: leave it unbuilt
    try {
      V3 a, b, c;
      switch (n->type) {
        case PLANE:
          if (n->vec("pvec", a) && n->vec("normal", b) && n->vec("x_axis", c)) out.s = new Geom_Plane(gp_Ax3(P(a), D(b), D(c)));
          break;
        case CYLINDER:
          if (n->vec("pvec", a) && n->vec("axis", b) && n->vec("x_axis", c))
            out.s = new Geom_CylindricalSurface(gp_Ax3(P(a), D(b), D(c)), n->num("radius") * S);
          break;
        case CONE:
          if (n->vec("pvec", a) && n->vec("axis", b) && n->vec("x_axis", c)) {
            // R(u,v) = P + vA + (X cos u + Y sin u)(r + v tan a): files open the cone along +A (the reference writes -vA,
            // which puts every cone's edges off its surface); which way its normal points is measured below
            const double half = std::atan2(n->num("sin_half_angle"), n->num("cos_half_angle"));
            out.s = new Geom_ConicalSurface(gp_Ax3(P(a), D(b), D(c)), half, n->num("radius") * S);
          }
          break;
        case SPHERE:
          if (n->vec("centre", a) && n->vec("axis", b) && n->vec("x_axis", c))
            out.s = new Geom_SphericalSurface(gp_Ax3(P(a), D(b), D(c)), n->num("radius") * S);
          break;
        case TORUS:
          if (n->vec("centre", a) && n->vec("axis", b) && n->vec("x_axis", c)) {
            const double major = n->num("major_radius") * S, minor = n->num("minor_radius") * S;
            if (major > minor) {
              out.s = new Geom_ToroidalSurface(gp_Ax3(P(a), D(b), D(c)), major, minor);
            } else {  // apple or lemon: the meridian circle turned about the axis, with the torus's own parameters
              const gp_Dir A = D(b), X = D(c);
              const gp_Pnt centre = P(a).Translated(gp_Vec(X) * major);
              Handle(Geom_Circle) meridian = new Geom_Circle(gp_Ax2(centre, X.Crossed(A), X), minor);
              out.s = new Geom_SurfaceOfRevolution(meridian, gp_Ax1(P(a), A));
            }
          }
          break;
        case B_SURFACE:
          out.s = bsurface(*n);
          break;
        case OFFSET_SURF: {
          const Surf& basis = surface(n->ptr("surface"));
          if (!basis.s.IsNull()) {
            // along the natural normal times the offset, negated for a negative sense (the reference); OCCT offsets along
            // its own normal, which a flipped basis turns over
            double d = n->num("offset") * S;
            if (n->chr("sense", '+') == '-') d = -d;
            if (basis.flip) d = -d;
            out.s = new Geom_OffsetSurface(basis.s, d);
            out.flip = basis.flip;
          }
          break;
        }
        case SWEPT_SURF: {
          const Crv& section = curve(n->ptr("section"));
          if (!section.c.IsNull() && n->vec("sweep", a)) out.s = new Geom_SurfaceOfLinearExtrusion(section.c, D(a));
          break;
        }
        case SPUN_SURF: {
          const Crv& profile = curve(n->ptr("profile"));
          if (!profile.c.IsNull() && n->vec("base", a) && n->vec("axis", b)) {
            // a circle in a plane through the axis spins into a sphere or a ring torus: OCCT handles those (their poles,
            // their seams) far better than a revolution of a periodic profile; the normal is then measured
            out.s = spun_analytic(n->ptr("profile"), gp_Ax1(P(a), D(b)));
            if (!out.s.IsNull()) {
              out.flip = !same_normal(i, *n, out.s);
            } else {
              out.s = new Geom_SurfaceOfRevolution(profile.c, gp_Ax1(P(a), D(b)));
              out.flip = true;  // OCCT turns about the axis in u and runs the profile in v: Parasolid the other way round
            }
          }
          break;
        }
        case BLENDED_EDGE:
          out.s = blend(*n, i);
          break;
        default:
          warn("surfaces of node type " + std::to_string(n->type) + " are not supported");
      }
      if (!out.s.IsNull() && (n->type == PLANE || n->type == CYLINDER || n->type == CONE || n->type == SPHERE || n->type == TORUS))
        out.flip = !same_normal(i, *n, out.s);
    } catch (const Standard_Failure& e) {
      warn("a surface (" + type_name(n->type) + ") could not be built: " + e.GetMessageString());
      out.s.Nullify();
    }
    building_.erase(i);
    return out;
  }

  // Whether OCCT's natural normal of `s` points the way Parasolid's does (dR/du x dR/dv of the reference's formula),
  // measured at one point of an elementary surface.
  bool same_normal(uint32_t i, const Node& n, const Handle(Geom_Surface)& s) {
    double u = 0.37, v = 0;
    if (n.type == PLANE) v = 0.21;
    if (n.type == SPHERE || n.type == TORUS) v = 0.29;
    if (n.type == CONE && std::abs(n.num("radius")) < 1e-9) v = 0.01;  // not on the apex
    const double h = 1e-6;
    gp_Pnt p, pu0, pu1, pv0, pv1;
    if (!evaluate(i, u, v, p) || !evaluate(i, u - h, v, pu0) || !evaluate(i, u + h, v, pu1) || !evaluate(i, u, v - h, pv0) ||
        !evaluate(i, u, v + h, pv1))
      return true;
    const gp_Vec ps = gp_Vec(pu0, pu1).Crossed(gp_Vec(pv0, pv1));
    GeomAPI_ProjectPointOnSurf proj(p, s);
    if (!proj.IsDone() || proj.NbPoints() == 0) return true;
    double U, V;
    proj.LowerDistanceParameters(U, V);
    gp_Pnt q;
    gp_Vec du, dv;
    s->D1(U, V, q, du, dv);
    return ps.Dot(du.Crossed(dv)) >= 0;
  }

  // The sphere or ring torus a spun circle makes, when its plane holds the axis; null otherwise.
  Handle(Geom_Surface) spun_analytic(uint32_t profile, const gp_Ax1& axis) {
    const Node* pn = f_.get(profile);
    if (pn && pn->type == TRIMMED_CURVE) pn = f_.get(pn->ptr("basis_curve"));
    if (!pn || pn->type != CIRCLE) return nullptr;
    V3 a, b;
    if (!pn->vec("centre", a) || !pn->vec("normal", b)) return nullptr;
    const gp_Pnt c = P(a);
    const gp_Dir normal = D(b), along = axis.Direction();
    const double r = pn->num("radius") * S;
    if (!(r > 0) || std::abs(normal.Dot(along)) > 1e-9) return nullptr;  // the circle's plane must hold the axis' direction
    if (std::abs(gp_Vec(axis.Location(), c).Dot(gp_Vec(normal))) > 1e-6) return nullptr;  // ... and the axis itself
    const gp_Vec rel(axis.Location(), c);
    const gp_Pnt foot = axis.Location().Translated(gp_Vec(along) * rel.Dot(gp_Vec(along)));
    const gp_Vec out(foot, c);
    const double R = out.Magnitude();
    if (R < 1e-6) {  // centred on the axis: a sphere
      const gp_Dir x = std::abs(along.X()) < 0.9 ? along.Crossed(gp::DX()) : along.Crossed(gp::DY());
      return new Geom_SphericalSurface(gp_Ax3(foot, along, x), r);
    }
    if (R > r + 1e-6) return new Geom_ToroidalSurface(gp_Ax3(foot, along, gp_Dir(out)), R, r);
    return nullptr;  // a spindle or horn torus stays a revolution
  }

  // A point of a surface at Parasolid's parameters (u, v) (SP-curves live there).
  bool evaluate(uint32_t i, double u, double v, gp_Pnt& p) {
    const Node* n = f_.get(i);
    if (!n) return false;
    V3 a, b, c;
    switch (n->type) {
      case PLANE:
        if (!n->vec("pvec", a) || !n->vec("normal", b) || !n->vec("x_axis", c)) return false;
        {
          const gp_Vec X = Vv(c), Y = Vv(b).Crossed(X);
          p = P(a).Translated((X * u + Y * v) * S);
        }
        return true;
      case CYLINDER:
        if (!n->vec("pvec", a) || !n->vec("axis", b) || !n->vec("x_axis", c)) return false;
        {
          const gp_Vec A = Vv(b), X = Vv(c), Y = A.Crossed(X);
          const double r = n->num("radius");
          p = P(a).Translated(((X * std::cos(u) + Y * std::sin(u)) * r + A * v) * S);
        }
        return true;
      case CONE:
        if (!n->vec("pvec", a) || !n->vec("axis", b) || !n->vec("x_axis", c)) return false;
        {
          const gp_Vec A = Vv(b), X = Vv(c), Y = A.Crossed(X);
          const double r = n->num("radius"), t = n->num("sin_half_angle") / n->num("cos_half_angle");
          p = P(a).Translated(((X * std::cos(u) + Y * std::sin(u)) * (r + v * t) + A * v) * S);
        }
        return true;
      case SPHERE:
        if (!n->vec("centre", a) || !n->vec("axis", b) || !n->vec("x_axis", c)) return false;
        {
          const gp_Vec A = Vv(b), X = Vv(c), Y = A.Crossed(X);
          const double r = n->num("radius");
          p = P(a).Translated(((X * std::cos(u) + Y * std::sin(u)) * (r * std::cos(v)) + A * (r * std::sin(v))) * S);
        }
        return true;
      case TORUS:
        if (!n->vec("centre", a) || !n->vec("axis", b) || !n->vec("x_axis", c)) return false;
        {
          const gp_Vec A = Vv(b), X = Vv(c), Y = A.Crossed(X);
          const double R = n->num("major_radius"), r = n->num("minor_radius");
          p = P(a).Translated(((X * std::cos(u) + Y * std::sin(u)) * (R + r * std::cos(v)) + A * (r * std::sin(v))) * S);
        }
        return true;
      case OFFSET_SURF:
      case B_SURFACE:
      case SWEPT_SURF: {
        const Surf& s = surface(i);
        if (s.s.IsNull()) return false;
        double su = u, sv = v;
        if (!occt_params(*n, su, sv)) return false;
        p = s.s->Value(su, sv);
        return true;
      }
      case SPUN_SURF: {  // the profile at u turned about the axis by v
        const Crv& profile = curve(n->ptr("profile"));
        if (profile.c.IsNull() || !n->vec("base", a) || !n->vec("axis", b)) return false;
        gp_Trsf turn;
        turn.SetRotation(gp_Ax1(P(a), D(b)), v);
        p = profile.c->Value(curve_param(n->ptr("profile"), u)).Transformed(turn);
        return true;
      }
      case BLENDED_EDGE:
        return blend_point(*n, u, v, p);
      default:
        return false;
    }
  }
  // Parasolid parameters -> those of the OCCT surface built for `n` (where they differ).
  bool occt_params(const Node& n, double& u, double& v) {
    switch (n.type) {
      case B_SURFACE:
        return true;
      case OFFSET_SURF: {
        const Node* basis = f_.get(n.ptr("surface"));
        return basis && occt_params(*basis, u, v);
      }
      case PLANE:
        u *= S, v *= S;
        return true;
      case CYLINDER:
        v *= S;
        return true;
      case CONE:  // OCCT's v runs along the slant
        v *= S / std::cos(std::atan2(n.num("sin_half_angle"), n.num("cos_half_angle")));
        return true;
      case SPHERE:
      case TORUS:
        return true;
      case SWEPT_SURF:
        u = curve_param(n.ptr("section"), u), v *= S;
        return true;
      case SPUN_SURF: {
        if (const auto it = surfaces_.find(n.index);
            it != surfaces_.end() && !it->second.s.IsNull() && !it->second.s->IsKind(STANDARD_TYPE(Geom_SurfaceOfRevolution)))
          return false;  // made a sphere or a torus: not an affine map of Parasolid's parameters
        const double profile = curve_param(n.ptr("profile"), u);
        u = v, v = profile;
        return true;
      }
      default:
        return false;
    }
  }
  double curve_param(uint32_t i, double t) {
    const Node* n = f_.get(i);
    if (!n) return t;
    if (n->type == LINE) return t * S;
    if (n->type == TRIMMED_CURVE) return curve_param(n->ptr("basis_curve"), t);
    return t;  // circles, ellipses, B-curves, our intersection and SP curves: Parasolid's own parameter
  }

  // ---- rolling-ball blends: at each spine parameter the ball's centre is on the spine and touches each support where
  // the centre projects onto it; the surface sweeps the arc between the two contacts (v: 0 on the first support, 1 on
  // the second, swapped when the spine runs backwards).
  bool contact(uint32_t support, const gp_Pnt& centre, gp_Pnt& at) {
    const Node* sn = f_.get(support);
    if (!sn) return false;
    if (sn->type == BLENDED_EDGE && std::abs(sn->num("range", 0, 0)) < 1e-12 && std::abs(sn->num("range", 1, 0)) < 1e-12) {
      const Crv& cliff = curve(sn->ptr("spine"));  // a cliff edge: the ball rests on the edge's curve
      if (cliff.c.IsNull()) return false;
      GeomAPI_ProjectPointOnCurve pc(centre, cliff.c);
      if (pc.NbPoints() == 0) return false;
      at = pc.NearestPoint();
      return true;
    }
    const Surf& s = surface(support);
    if (s.s.IsNull()) return false;
    GeomAPI_ProjectPointOnSurf ps(centre, s.s);
    if (!ps.IsDone() || ps.NbPoints() == 0) return false;
    at = ps.NearestPoint();
    return true;
  }
  // Where the ball centred at `centre` touches support k of blend `n`: on the blend's own contact curve with that support
  // when the file has one (the intersection of a BLEND_BOUND of the blend with the support, charted exactly), else by
  // projecting onto the support. The curves matter when a support is itself a blend: projecting onto its fitted
  // approximation adds that fit's error to this one, and blends on blends on blends drifted by millimetres.
  bool blend_contact(const Node& n, int k, const gp_Pnt& centre, gp_Pnt& at) {
    const double r = std::abs(n.num("range", size_t(k), 0)) * S;
    const Node* support = f_.get(n.ptr("surface", size_t(k)));
    const bool on_blend = support && support->type == BLENDED_EDGE;  // elsewhere the projection is exact (and was better)
    if (const auto it = contact_curves_.find(n.index); on_blend && it != contact_curves_.end() && r > 0) {
      double best = 1e300;
      gp_Pnt found;
      for (uint32_t ci : it->second[k]) {
        const Crv& c = curve(ci);
        if (c.c.IsNull()) continue;
        try {
          GeomAPI_ProjectPointOnCurve pc(centre, c.c);
          if (pc.NbPoints() > 0 && pc.LowerDistance() < best) best = pc.LowerDistance(), found = pc.NearestPoint();
        } catch (const Standard_Failure&) {
        }
      }
      // the ball touches its contact curve at its radius; past the curve's ends (the margin of the range) it does not
      if (best < 1e299 && std::abs(best - r) < 0.05 * r + 1e-4) {
        at = found;
        return true;
      }
    }
    return contact(n.ptr("surface", size_t(k)), centre, at);
  }
  // Each blend's contact curves with its two supports, from the intersections that name one of its BLEND_BOUNDs.
  void index_contact_curves() {
    for (const auto& [index, node] : f_.nodes) {
      if (node.type != INTERSECTION) continue;
      for (int side = 0; side < 2; ++side) {
        const Node* bound = f_.get(node.ptr("surface", size_t(side)), BLEND_BOUND);
        const Node* blend = bound ? f_.get(bound->ptr("blend"), BLENDED_EDGE) : nullptr;
        if (!blend) continue;
        const uint32_t other = node.ptr("surface", size_t(1 - side));
        for (int k = 0; k < 2; ++k)
          if (other && blend->ptr("surface", size_t(k)) == other) contact_curves_[blend->index][k].push_back(index);
      }
    }
  }
  bool blend_point(const Node& n, double u, double v, gp_Pnt& p) {
    const Crv& spine = curve(n.ptr("spine"));
    if (spine.c.IsNull()) return false;
    const gp_Pnt c = spine.c->Value(u);
    gp_Pnt q0, q1;
    if (!blend_contact(n, 0, c, q0) || !blend_contact(n, 1, c, q1)) return false;
    if (spine.sense < 0) std::swap(q0, q1);
    p = arc_point(c, q0, q1, v);
    return true;
  }
  static gp_Pnt arc_point(const gp_Pnt& c, const gp_Pnt& q0, const gp_Pnt& q1, double v) {
    gp_Vec a(c, q0), b(c, q1);
    const double r = a.Magnitude();
    if (r < 1e-12 || b.Magnitude() < 1e-12) return q0;
    const gp_Vec x = a / r;
    gp_Vec y = b - x * b.Dot(x);
    if (y.Magnitude() < 1e-12) return q0;
    y.Normalize();
    const double angle = std::atan2(b.Dot(y), b.Dot(x));
    return c.Translated((x * std::cos(angle * v) + y * std::sin(angle * v)) * r);
  }
  Handle(Geom_Surface) blend(const Node& n, uint32_t index) {
    const Crv& spine = curve(n.ptr("spine"));
    if (spine.c.IsNull()) {
      warn("a blend surface without a spine curve was approximated from its edges");
      return nullptr;
    }
    // the stretch of spine the blend's faces need: their vertices projected onto it, with a margin
    double u0 = spine.c->FirstParameter(), u1 = spine.c->LastParameter();
    // a ring (a closed spine) blended all round, its face's vertices at one place on it or at both ends: the whole spine
    const bool ring = spine.c->IsPeriodic() || (!Precision::IsInfinite(u0) && !Precision::IsInfinite(u1) &&
                                                spine.c->Value(u0).Distance(spine.c->Value(u1)) < 1e-6);
    const auto it = blend_range_.find(index);
    const double span = it != blend_range_.end() ? it->second.second - it->second.first : 0;
    const bool all_round = ring && it != blend_range_.end() && (span < (u1 - u0) * 0.01 || span > (u1 - u0) * 0.95);
    if (it != blend_range_.end() && !all_round) {
      const double pad = (it->second.second - it->second.first) * 0.02 + 1e-9;  // more reaches past the blend: worse
      double a = it->second.first - pad, b = it->second.second + pad;
      if (!spine.c->IsPeriodic()) a = std::max(a, u0), b = std::min(b, u1);
      if (b > a) u0 = a, u1 = b;
    }
    if (Precision::IsInfinite(u0) || Precision::IsInfinite(u1)) return nullptr;
    const int nu = 24, nv = 9;
    TColgp_Array2OfPnt grid(1, nu, 1, nv);
    for (int i = 0; i < nu; ++i) {
      // all round: the closing row is the first one again, which the periodic fit adds itself
      const double u = u0 + (u1 - u0) * i / (all_round ? nu : nu - 1);
      const gp_Pnt c = spine.c->Value(u);
      gp_Pnt q0, q1;
      if (!blend_contact(n, 0, c, q0) || !blend_contact(n, 1, c, q1)) return nullptr;
      if (spine.sense < 0) std::swap(q0, q1);
      for (int j = 0; j < nv; ++j) grid.SetValue(i + 1, j + 1, arc_point(c, q0, q1, double(j) / (nv - 1)));
    }
    // a blend pinching to a point (at a terminator) makes rows of equal points: a looser fit takes those; a ring is
    // periodic along the spine, so that its faces get the seam OCCT needs
    for (const auto& [degree, continuity, tol] : {std::tuple{3, GeomAbs_C2, 1e-3}, std::tuple{1, GeomAbs_C0, 1e-2}}) {
      try {
        GeomAPI_PointsToBSplineSurface fit;
        fit.Init(grid, Approx_ChordLength, degree, 8, continuity, tol, all_round);
        if (fit.IsDone()) return fit.Surface();
      } catch (const Standard_Failure&) {
      }
    }
    return nullptr;
  }
  // Spine parameters a blend's faces reach, from their vertices (before the blend surface is built).
  void blend_ranges() {
    for (const auto& [index, node] : f_.nodes) {
      if (node.type != FACE) continue;
      const Node* s = f_.get(node.ptr("surface"), BLENDED_EDGE);
      if (!s) continue;
      try {
        blend_range(node, *s);
      } catch (const Standard_Failure&) {
      }
    }
  }
  void blend_range(const Node& node, const Node& sref) {
    const Node* s = &sref;
    {
      const Crv& spine = curve(s->ptr("spine"));
      if (spine.c.IsNull()) return;
      auto& range = blend_range_[node.ptr("surface")];
      bool any = range.first < range.second;
      for (uint32_t lp = node.ptr("loop"), guard = 0; lp && guard < kGuard; ++guard) {
        const Node* loop = f_.get(lp, LOOP);
        if (!loop) break;
        uint32_t start = loop->ptr("fin"), fi = start;
        for (int g = 0; fi && g < kGuard; ++g) {
          const Node* fin = f_.get(fi, FIN);
          if (!fin) break;
          auto reach = [&](const gp_Pnt& p) {
            GeomAPI_ProjectPointOnCurve pc(p, spine.c);
            if (pc.NbPoints() == 0) return;
            const double t = pc.LowerDistanceParameter();
            if (!any) range = {t, t}, any = true;
            range.first = std::min(range.first, t), range.second = std::max(range.second, t);
          };
          auto point_of = [&](uint32_t vi, gp_Pnt& p) {
            const Node* vx = f_.get(vi, VERTEX);
            const Node* pt = vx ? f_.get(vx->ptr("point"), POINT) : nullptr;
            V3 a;
            if (!pt || !pt->vec("pvec", a)) return false;
            p = P(a);
            return true;
          };
          gp_Pnt end;
          if (point_of(fin->ptr("vertex"), end)) reach(end);
          // the edge between its vertices, which can reach further along the spine than they do (a long side)
          const Node* en = f_.get(fin->ptr("edge"), EDGE);
          const Node* cn = en ? f_.get(en->ptr("curve")) : nullptr;
          const Node* other = f_.get(fin->ptr("other"), FIN);
          gp_Pnt begin;
          if (cn && cn->type != SP_CURVE && cn->type != TRIMMED_CURVE && other && point_of(other->ptr("vertex"), begin) &&
              point_of(fin->ptr("vertex"), end)) {
            const Crv& ec = curve(en->ptr("curve"));
            if (!ec.c.IsNull()) {
              ShapeAnalysis_Curve sac;
              gp_Pnt proj;
              double t0 = 0, t1 = 0;
              sac.Project(ec.c, begin, 1e-7, proj, t0, false);
              sac.Project(ec.c, end, 1e-7, proj, t1, false);
              if (ec.c->IsPeriodic() && std::abs(t1 - t0) > ec.c->Period() / 2) t1 += t1 < t0 ? ec.c->Period() : -ec.c->Period();
              for (int k = 1; k < 8; ++k) reach(ec.c->Value(t0 + (t1 - t0) * k / 8));
            }
          }
          fi = fin->ptr("forward");
          if (fi == start) break;
        }
        lp = loop->ptr("next");
      }
    }
  }

  // ---- topology
  TopoDS_Vertex vertex(uint32_t i) {
    if (auto it = vertices_.find(i); it != vertices_.end()) return it->second;
    TopoDS_Vertex v;
    const Node* n = f_.get(i, VERTEX);
    const Node* pt = n ? f_.get(n->ptr("point"), POINT) : nullptr;
    V3 a;
    if (pt && pt->vec("pvec", a) && !std::isnan(a[1]) && !std::isnan(a[2])) {
      const double tol = n->num("tolerance");
      BRep_Builder().MakeVertex(v, P(a), std::isnan(tol) ? Precision::Confusion() : std::max(tol * S, Precision::Confusion()));
    }
    return vertices_[i] = v;
  }

  TopoDS_Edge edge(uint32_t i) {
    if (auto it = edges_.find(i); it != edges_.end()) return it->second;
    TopoDS_Edge e;
    try {
      e = make_edge(i);
    } catch (const Standard_Failure& x) {
      warn(std::string("an edge could not be built: ") + x.GetMessageString());
      e.Nullify();
    }
    return edges_[i] = e;
  }
  TopoDS_Edge make_edge(uint32_t i) {
    TopoDS_Edge result;
    const Node* n = f_.get(i, EDGE);
    if (!n) return result;
    BRep_Builder bb;
    // the positive fin's vertex ends the edge, the negative one's starts it (the reference, "Dummy Fins")
    const Node* fin = f_.get(n->ptr("fin"), FIN);
    const Node* other = fin ? f_.get(fin->ptr("other"), FIN) : nullptr;
    TopoDS_Vertex vs = other ? vertex(other->ptr("vertex")) : TopoDS_Vertex(), ve = fin ? vertex(fin->ptr("vertex")) : TopoDS_Vertex();
    const bool same_vertex = fin && other && fin->ptr("vertex") == other->ptr("vertex");
    uint32_t cptr = n->ptr("curve");
    if (!cptr && fin) {  // a tolerant edge: its fins carry (trimmed) SP-curves
      // preferably one on an exact surface: a blend is our approximation, and a curve mapped through it inherits its error
      auto on_blend = [&](uint32_t ci) {
        const Node* cn = f_.get(ci);
        if (cn && cn->type == TRIMMED_CURVE) cn = f_.get(cn->ptr("basis_curve"));
        const Node* on = cn && cn->type == SP_CURVE ? f_.get(cn->ptr("surface")) : nullptr;
        return on && on->type == BLENDED_EDGE;
      };
      uint32_t fi = fin->index;
      for (int g = 0; fi && g < 64; ++g) {
        const Node* fn = f_.get(fi, FIN);
        if (!fn) break;
        if (const uint32_t ci = fn->ptr("curve")) {
          if (!cptr || on_blend(cptr)) cptr = ci;
          if (!on_blend(ci)) break;
        }
        fi = fn->ptr("other");
        if (fi == fin->index) break;
      }
    }
    Crv c = curve(cptr);
    if (const Node* cn = f_.get(cptr); cn && !c.c.IsNull()) {  // a fin's SP-curve: the face on its surface gets it as pcurve
      const Node* sp = cn->type == TRIMMED_CURVE ? f_.get(cn->ptr("basis_curve"), SP_CURVE) : cn->type == SP_CURVE ? cn : nullptr;
      if (sp) edge_sp_[i] = {sp->index, c.sense < 0};
    }
    const double etol = std::isnan(n->num("tolerance")) ? Precision::Confusion() : std::max(n->num("tolerance") * S, Precision::Confusion());
    if (c.c.IsNull()) {
      if (vs.IsNull() || ve.IsNull() || same_vertex) {
        warn("an edge without a usable curve was left out");
        return result;
      }
      const gp_Pnt a = BRep_Tool::Pnt(vs), b = BRep_Tool::Pnt(ve);
      if (a.Distance(b) < 1e-9) return result;
      c.c = new Geom_Line(a, gp_Dir(gp_Vec(a, b)));
      c.sense = 1;
      const Node* cn = f_.get(cptr);
      std::string kind = cn ? type_name(cn->type) : "none";
      const Node* sp = cn && cn->type == TRIMMED_CURVE ? f_.get(cn->ptr("basis_curve")) : cn;
      if (sp && sp->type == SP_CURVE)
        if (const Node* on = f_.get(sp->ptr("surface"))) kind += " on " + type_name(on->type);
      warn("an edge's curve (" + kind + ") could not be built and was replaced by a straight line");
    }
    Handle(Geom_Curve) g = c.sense < 0 ? c.c->Reversed() : c.c;
    const bool periodic = g->IsPeriodic();
    double p0 = g->FirstParameter(), p1 = g->LastParameter();
    ShapeAnalysis_Curve sac;
    auto param = [&](const TopoDS_Vertex& v, double& t, double& dist) {
      gp_Pnt proj;
      dist = sac.Project(g, BRep_Tool::Pnt(v), 1e-7, proj, t, false);
    };
    double d0 = 0, d1 = 0;
    if (vs.IsNull() || ve.IsNull()) {  // a ring edge: the whole closed curve, a vertex where it starts
      if (Precision::IsInfinite(p0) || Precision::IsInfinite(p1)) return result;
      bb.MakeVertex(vs, g->Value(p0), Precision::Confusion());
      ve = vs;
      if (periodic) p1 = p0 + g->Period();
    } else {
      param(vs, p0, d0);
      param(ve, p1, d1);
      if (periodic) {
        const double T = g->Period();
        while (p1 <= p0 + 1e-9) p1 += T;
        while (p1 > p0 + T + 1e-9) p1 -= T;
        if (same_vertex) p1 = p0 + T;
      } else if (p1 < p0) {
        // a closed curve: a vertex at its closure was placed at the wrong end
        const double f = g->FirstParameter(), l = g->LastParameter();
        if (!Precision::IsInfinite(f) && !Precision::IsInfinite(l) && g->Value(f).Distance(g->Value(l)) < 1e-6) {
          if (std::abs(p0 - l) < 1e-9 * (1 + std::abs(l))) p0 = f;
          else if (std::abs(p1 - f) < 1e-9 * (1 + std::abs(f))) p1 = l;
        }
        if (p1 < p0) {  // the curve runs the other way: turned round, so the edge keeps the direction its fins refer to
          p0 = g->ReversedParameter(p0), p1 = g->ReversedParameter(p1);
          g = g->Reversed();
          if (auto it = edge_sp_.find(i); it != edge_sp_.end()) it->second.second = !it->second.second;
        }
      }
      if (!periodic) p0 = std::max(p0, g->FirstParameter()), p1 = std::min(p1, g->LastParameter());
      if (p1 - p0 < 1e-12) {
        warn("an edge of no length on its curve was left out");
        return result;
      }
    }
    TopoDS_Edge e;
    bb.MakeEdge(e, g, etol);
    bb.Add(e, vs.Oriented(TopAbs_FORWARD));
    bb.Add(e, ve.Oriented(TopAbs_REVERSED));
    bb.Range(e, p0, p1);
    const double vtol0 = std::max(BRep_Tool::Tolerance(vs), d0 * 1.05), vtol1 = std::max(BRep_Tool::Tolerance(ve), d1 * 1.05);
    bb.UpdateVertex(vs, vtol0);
    bb.UpdateVertex(ve, vtol1);
    bb.UpdateEdge(e, std::max(etol, std::max(d0, d1)));
    result = e;
    return result;
  }

  // A face oriented so that its normal is Parasolid's face normal.
  TopoDS_Face face(uint32_t i) {
    if (auto it = faces_.find(i); it != faces_.end()) return it->second;
    TopoDS_Face f;
    try {
      f = make_face(i);
    } catch (const Standard_Failure& x) {
      warn(std::string("a face could not be built: ") + x.GetMessageString());
      f.Nullify();
    }
    return faces_[i] = f;
  }
  TopoDS_Face make_face(uint32_t i) {
    TopoDS_Face result;
    const Node* n = f_.get(i, FACE);
    if (!n) return result;
    const uint32_t sp = n->ptr("surface");
    const Node* sn = f_.get(sp);
    const Surf& s = surface(sp);
    BRep_Builder bb;
    std::vector<TopoDS_Wire> wires;
    for (uint32_t lp = n->ptr("loop"), guard = 0; lp && guard < kGuard; ++guard) {
      const Node* loop = f_.get(lp, LOOP);
      if (!loop) break;
      TopoDS_Wire w;
      bb.MakeWire(w);
      int count = 0;
      const uint32_t start = loop->ptr("fin");
      for (uint32_t fi = start, g = 0; fi && g < kGuard; ++g) {
        const Node* fin = f_.get(fi, FIN);
        if (!fin) break;
        if (fin->ptr("edge")) {
          const TopoDS_Edge e = edge(fin->ptr("edge"));
          if (!e.IsNull()) bb.Add(w, fin->chr("sense", '+') == '-' ? TopoDS::Edge(e.Reversed()) : e), ++count;
        }
        fi = fin->ptr("forward");
        if (fi == start) break;
      }
      if (count) wires.push_back(w);
      lp = loop->ptr("next");
    }
    if (s.s.IsNull()) {  // no usable surface: the plane of a flat boundary, else the face is left out
      // (BRepFill_Filling, the general patch, crashes on some such boundaries and cannot be guarded)
      if (wires.empty()) return result;
      BRepLib_FindSurface plane(wires.front(), 1e-3, true);
      if (plane.Found()) {
        TopoDS_Face f;
        bb.MakeFace(f, plane.Surface(), Precision::Confusion());
        for (const auto& w : wires) bb.Add(f, w);
        result = f;
        warn("a face whose surface could not be built was made flat from its boundary");
      } else {
        warn("a face whose surface (" + (sn ? type_name(sn->type) : std::string("none")) + ") could not be built was left out");
      }
      return result;
    }
    TopoDS_Face f;
    bb.MakeFace(f, s.s, Precision::Confusion());
    for (const auto& w : wires) bb.Add(f, w);
    // the face normal is the surface's natural normal when the face and surface senses agree (the reference)
    const bool natural = (n->chr("sense", '+') == '-') == (sn && sn->chr("sense", '+') == '-');
    if (natural == s.flip) f.Reverse();
    attach_pcurves(*n, sp, f);
    result = f;
    return result;
  }

  // An edge whose 3D curve was made from an SP-curve on this face's surface shares that curve's parameter: the 2D curve
  // is its pcurve here, which spares healing a projection of the 3D curve (slow on long ones, and less exact).
  void attach_pcurves(const Node& face, uint32_t sp, const TopoDS_Face& f) {
    BRep_Builder bb;
    std::unordered_map<uint32_t, int> uses;  // an edge twice in the face (a seam) needs two pcurves: left to healing
    std::vector<uint32_t> fins;
    for (uint32_t lp = face.ptr("loop"), guard = 0; lp && guard < kGuard; ++guard) {
      const Node* loop = f_.get(lp, LOOP);
      if (!loop) break;
      const uint32_t start = loop->ptr("fin");
      for (uint32_t fi = start, g = 0; fi && g < kGuard; ++g) {
        const Node* fin = f_.get(fi, FIN);
        if (!fin) break;
        if (fin->ptr("edge")) ++uses[fin->ptr("edge")], fins.push_back(fi);
        fi = fin->ptr("forward");
        if (fi == start) break;
      }
      lp = loop->ptr("next");
    }
    for (uint32_t fi : fins) {
      const Node* fin = f_.get(fi, FIN);
      const uint32_t ep = fin->ptr("edge");
      const auto it = edge_sp_.find(ep);
      if (uses[ep] != 1 || it == edge_sp_.end()) continue;
      const Node* spn = f_.get(it->second.first, SP_CURVE);
      if (!spn || spn->ptr("surface") != sp) continue;
      const auto e = edges_.find(ep);
      if (e == edges_.end() || e->second.IsNull()) continue;
      try {
        const Node* bc = f_.get(spn->ptr("b_curve"), B_CURVE);
        Handle(Geom2d_BSplineCurve) pc = bc ? occt_pcurve(sp, bcurve2(*bc)) : nullptr;
        double a, b;
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(e->second, a, b);
        if (pc.IsNull() || c3.IsNull() || std::abs(c3->FirstParameter() - pc->FirstParameter()) > 1e-9 ||
            std::abs(c3->LastParameter() - pc->LastParameter()) > 1e-9)
          continue;  // not the same parameter (a 3D curve sampled with points dropped, a straight stand-in)
        Handle(Geom2d_Curve) use = it->second.second ? Handle(Geom2d_Curve)(pc->Reversed()) : Handle(Geom2d_Curve)(pc);
        bb.UpdateEdge(e->second, use, f, BRep_Tool::Tolerance(e->second));
      } catch (const Standard_Failure&) {
      }
    }
  }

  // ---- bodies
  TopoDS_Shape body(const Node& b) {
    BRep_Builder bb;
    TopoDS_Compound all;
    bb.MakeCompound(all);
    int parts = 0;
    body_faces_.clear();
    std::unordered_set<uint32_t> used_faces;
    std::vector<uint32_t> loose_faces, wire_edges, acorns;
    for (uint32_t r = b.ptr("region"), guard = 0; r && guard < kGuard; ++guard) {
      const Node* region = f_.get(r, REGION);
      if (!region) break;
      const bool solid = region->chr("type") == 'S';
      TopoDS_Solid so;
      int shells = 0;
      if (solid) bb.MakeSolid(so);
      for (uint32_t sh = region->ptr("shell"), g = 0; sh && g < kGuard; ++g) {
        const Node* shell = f_.get(sh, SHELL);
        if (!shell) break;
        TopoDS_Shell s;
        bb.MakeShell(s);
        int faces = 0;
        // back faces (normal out of the region) as they are, front faces turned over: a solid's shells face outwards
        auto take = [&](uint32_t head, const char* next, bool back) {
          for (uint32_t fi = head, h = 0; fi && h < kGuard; ++h) {
            const Node* fn = f_.get(fi, FACE);
            if (!fn) break;
            if (solid) {
              const TopoDS_Face f = face(fi);
              if (!f.IsNull()) bb.Add(s, back ? f : TopoDS::Face(f.Reversed())), ++faces, used_faces.insert(fi), body_faces_.push_back(fi);
            } else {
              loose_faces.push_back(fi);
            }
            fi = fn->ptr(next);
          }
        };
        take(shell->ptr("face"), "next", true);
        take(shell->ptr("front_face"), "next_front", false);
        for (uint32_t e = shell->ptr("edge"), h = 0; e && h < kGuard; ++h) {
          const Node* en = f_.get(e, EDGE);
          if (!en) break;
          wire_edges.push_back(e);
          e = en->ptr("next");
        }
        if (shell->ptr("vertex")) acorns.push_back(shell->ptr("vertex"));
        if (solid && faces) bb.Add(so, s), ++shells;
        sh = shell->ptr("next");
      }
      if (solid && shells) bb.Add(all, so), ++parts;
      r = region->ptr("next");
    }
    // sheets: each face once, whichever side its shells use it from
    TopoDS_Shell sheet;
    bb.MakeShell(sheet);
    int sheet_faces = 0;
    for (uint32_t fi : loose_faces) {
      if (!used_faces.insert(fi).second) continue;
      const TopoDS_Face f = face(fi);
      if (!f.IsNull()) bb.Add(sheet, f), ++sheet_faces, body_faces_.push_back(fi);
    }
    if (sheet_faces) bb.Add(all, sheet), ++parts;
    std::unordered_set<uint32_t> seen_edges;
    for (uint32_t e : wire_edges)
      if (seen_edges.insert(e).second)
        if (const TopoDS_Edge ed = edge(e); !ed.IsNull()) bb.Add(all, ed), ++parts;
    for (uint32_t v : acorns)
      if (const TopoDS_Vertex vx = vertex(v); !vx.IsNull()) bb.Add(all, vx), ++parts;
    if (!parts) return {};
    return all;
  }

  // Faces healing could not trim (a blend's edges projected off its fitted patch, where the spline runs away) reach far
  // outside the body: left out, so that the body's box, fitting the view and thumbnails stay right.
  TopoDS_Shape drop_runaway_faces(const TopoDS_Shape& s, const Bnd_Box& extent, const std::function<uint32_t(const TopoDS_Shape&)>& built_from) {
    if (extent.IsVoid()) return s;
    double x0, y0, z0, x1, y1, z1;
    extent.Get(x0, y0, z0, x1, y1, z1);
    const double margin = std::max(1.0, gp_Pnt(x0, y0, z0).Distance(gp_Pnt(x1, y1, z1)));
    Bnd_Box allowed = extent;
    allowed.Enlarge(margin);
    Handle(ShapeBuild_ReShape) drop = new ShapeBuild_ReShape();
    int dropped = 0;
    for (TopExp_Explorer x(s, TopAbs_FACE); x.More(); x.Next()) {
      Bnd_Box b;
      try {
        BRepBndLib::Add(x.Current(), b, false);
      } catch (const Standard_Failure&) {
        continue;
      }
      if (b.IsVoid() || allowed.IsOut(b.CornerMin()) || allowed.IsOut(b.CornerMax())) {
        drop->Remove(x.Current()), ++dropped;
        const uint32_t fi = built_from(x.Current());
        ++dropped_kinds_[fi ? surface_kind(fi) : std::string("?")];
      }
    }
    if (!dropped) return s;
    dropped_faces_ += dropped;  // one warning with the total, at the end
    try {
      return drop->Apply(s);
    } catch (const Standard_Failure&) {
      return s;
    }
  }

  // ShapeFix for what OCCT needs: curves on the faces, seams of periodic faces, natural bounds of closed ones. Faces
  // replaced on the way are reported through `fixed` (for their colours).
  TopoDS_Shape heal(const TopoDS_Shape& s, Handle(ShapeBuild_ReShape)& context) {
    try {
      ShapeFix_Shape fix(s);
      fix.SetPrecision(1e-6);
      fix.SetMaxTolerance(0.5);
      fix.FixSolidTool()->CreateOpenSolidMode() = true;  // a solid with a gap stays a solid, not a loose shell
      fix.Perform();
      context = fix.Context();
      return fix.Shape();
    } catch (const Standard_Failure&) {
    }
    // one face at a time, so that one face ShapeFix cannot handle does not leave the whole body unfixed
    context = new ShapeBuild_ReShape();
    int failed = 0;
    for (TopExp_Explorer x(s, TopAbs_FACE); x.More(); x.Next()) {
      const TopoDS_Face f = TopoDS::Face(x.Current());
      try {
        ShapeFix_Face fix(f);
        fix.SetPrecision(1e-6);
        fix.SetMaxTolerance(0.5);
        fix.Perform();
        if (!fix.Face().IsSame(f)) context->Replace(f, fix.Face());
      } catch (const Standard_Failure&) {
        ++failed;
      }
    }
    if (failed) warn("healing left " + std::to_string(failed) + " faces of a body as they are");
    try {
      return context->Apply(s);
    } catch (const Standard_Failure&) {
      return s;
    }
  }

  // ---- assemblies and the XCAF document
  Built run(const std::function<bool(double)>& progress) {
    Built out;
    XCAFApp_Application::GetApplication()->NewDocument("MDTV-XCAF", out.doc);
    st_ = XCAFDoc_DocumentTool::ShapeTool(out.doc->Main());
    ct_ = XCAFDoc_DocumentTool::ColorTool(out.doc->Main());
    index_contact_curves();
    blend_ranges();
    for (const auto& [index, node] : f_.nodes)
      if (node.type == BODY) ++total_;
    std::vector<uint32_t> roots;
    const Node* root = f_.get(1);
    if (!root) throw Error("the Parasolid file has no root node");
    if (root->type == BODY || root->type == ASSEMBLY) {
      roots.push_back(1);
    } else if (root->type == WORLD) {  // a partition: its parts not instanced elsewhere
      for (const char* chain : {"assembly", "body"})
        for (uint32_t p = root->ptr(chain), g = 0; p && g < kGuard; ++g) {
          const Node* pn = f_.get(p);
          if (!pn) break;
          if (!pn->ptr("ref_instance")) roots.push_back(p);
          p = pn->ptr("next");
        }
    } else if (root->has("entries")) {  // an array of parts (POINTER_LIS_BLOCK and its later forms)
      for (const Node* blk = root; blk;) {
        const Value* ent = blk->field("entries");
        const int count = blk->integer("n_entries", ent ? int(ent->ptr.size()) : 0);
        if (ent)
          for (int k = 0; k < count && k < int(ent->ptr.size()); ++k)
            if (ent->ptr[size_t(k)]) roots.push_back(ent->ptr[size_t(k)]);
        blk = f_.get(blk->ptr("next_block"));
        if (blk == root) break;
      }
    } else {
      throw Error("the Parasolid file's root is a node of type " + std::to_string(root->type) + ", not a part");
    }
    for (uint32_t r : roots) {
      const TDF_Label l = part(r, progress);
      (void)l;
    }
    st_->UpdateAssemblies();
    out.bodies = built_;
    out.face_source = std::move(face_source_);
    out.face_node = std::move(face_node_);
    if (dropped_faces_) {
      std::string kinds;
      for (const auto& [kind, count] : dropped_kinds_) kinds += (kinds.empty() ? "" : ", ") + kind + " " + std::to_string(count);
      warnings.push_back(std::to_string(dropped_faces_) +
                         (dropped_faces_ == 1 ? " face that could not be fitted to its edges was left out ("
                                              : " faces that could not be fitted to their edges were left out (") +
                         kinds + ")");
    }
    out.warnings = warnings;
    return out;
  }

  TDF_Label part(uint32_t index, const std::function<bool(double)>& progress) {
    if (auto it = labels_.find(index); it != labels_.end()) return it->second;
    TDF_Label& label = labels_[index];
    const Node* n = f_.get(index);
    if (!n) return label;
    if (n->type == BODY) {
      if (progress && !progress(double(done_) / std::max(1, total_))) throw Error("cancelled");
      ++done_;
      TopoDS_Shape shape;
      try {
        shape = body(*n);
      } catch (const Standard_Failure& x) {
        warn(std::string("a body could not be built: ") + x.GetMessageString());
      }
      if (shape.IsNull()) return label;
      Handle(ShapeBuild_ReShape) context;
      Bnd_Box extent;  // the body's own extent, from its edges (exact curves, whatever the faces became)
      for (TopExp_Explorer x(shape, TopAbs_EDGE); x.More(); x.Next()) {
        try {
          BRepBndLib::Add(x.Current(), extent, false);
        } catch (const Standard_Failure&) {
        }
      }
      shape = heal(shape, context);
      std::unordered_map<const Geom_Surface*, uint32_t> by_surface;  // faces healing left -> the faces built (see below)
      for (uint32_t fi : body_faces_) {
        TopLoc_Location loc;
        if (const Handle(Geom_Surface) g = BRep_Tool::Surface(faces_[fi], loc); !g.IsNull()) by_surface.emplace(g.get(), fi);
      }
      auto built_from = [&](const TopoDS_Shape& f) -> uint32_t {
        TopLoc_Location loc;
        const Handle(Geom_Surface) g = BRep_Tool::Surface(TopoDS::Face(f), loc);
        const auto it = g.IsNull() ? by_surface.end() : by_surface.find(g.get());
        return it == by_surface.end() ? 0 : it->second;
      };
      shape = drop_runaway_faces(shape, extent, built_from);
      label = st_->AddShape(shape, false);
      ++built_;
      std::string name = name_of(*n);
      if (name.empty()) name = "Body " + std::to_string(++unnamed_bodies_);  // not XCAF's shape type ("COMPOUND")
      TDataStd_Name::Set(label, TCollection_ExtendedString(name.c_str(), true));
      Quantity_Color col;
      if (colour_of(*n, col)) ct_->SetColor(label, col, XCAFDoc_ColorGen);
      // the faces healing left, matched to the faces built by their surface: healing rebuilds a face whose edges it
      // replaced (a new TShape, not in its record) but keeps the surface
      for (TopExp_Explorer x(shape, TopAbs_FACE); x.More(); x.Next()) {
        const uint32_t fi = built_from(x.Current());
        if (!fi) continue;
        face_source_[x.Current().TShape().get()] = surface_kind(fi);
        face_node_[x.Current().TShape().get()] = fi;
        const Node* fn = f_.get(fi);
        if (fn && colour_of(*fn, col)) {
          const TDF_Label sub = st_->AddSubShape(label, x.Current());
          if (!sub.IsNull()) ct_->SetColor(sub, col, XCAFDoc_ColorSurf);
        }
      }
      return label;
    }
    if (n->type == ASSEMBLY) {
      label = st_->NewShape();
      std::string name = name_of(*n);
      if (name.empty()) name = "Assembly " + std::to_string(++unnamed_assemblies_);
      TDataStd_Name::Set(label, TCollection_ExtendedString(name.c_str(), true));
      for (uint32_t in = n->ptr("sub_instance"), g = 0; in && g < kGuard; ++g) {
        const Node* inst = f_.get(in, INSTANCE);
        if (!inst) break;
        const TDF_Label child = part(inst->ptr("part"), progress);
        if (!child.IsNull()) {
          const TDF_Label comp = st_->AddComponent(label, child, location(inst->ptr("transform")));
          const std::string iname = name_of(*inst);
          if (!comp.IsNull() && !iname.empty()) TDataStd_Name::Set(comp, TCollection_ExtendedString(iname.c_str(), true));
        }
        in = inst->ptr("next_in_part");
      }
      return label;
    }
    warn("a part of node type " + std::to_string(n->type) + " was left out");
    return label;
  }

  // The kind of surface a face is on, for surveys: its type, and for a blend what it rests on.
  std::string surface_kind(uint32_t face) const {
    const Node* fn = f_.get(face);
    const Node* sn = fn ? f_.get(fn->ptr("surface")) : nullptr;
    if (!sn) return "none";
    std::string kind = type_name(sn->type);
    if (sn->type == BLENDED_EDGE) {
      for (int k = 0; k < 2; ++k) {
        const Node* support = f_.get(sn->ptr("surface", k));
        if (support && support->type == BLENDED_EDGE)
          kind += std::abs(support->num("range", 0, 0)) < 1e-12 ? " on cliff" : " on blend";
      }
    }
    return kind;
  }

  // x' = (R x + t) * scale (the reference); a scale other than 1 cannot be a placement and is dropped.
  TopLoc_Location location(uint32_t t) {
    const Node* n = f_.get(t, TRANSFORM);
    if (!n) return {};
    const Value* r = n->field("rotation_matrix");
    V3 tr;
    if (!r || r->num.size() < 9 || !n->vec("translation_vector", tr)) return {};
    const double sc = n->num("scale", 0, 1);
    if (!std::isnan(sc) && std::abs(sc - 1) > 1e-9) warn("an instance's scale was ignored");
    gp_Trsf T;
    try {
      // rotation_matrix[3][3] row by row: x'_i = sum_j R[i][j] x_j (assembled parts then touch; transposed they do not)
      T.SetValues(r->num[0], r->num[1], r->num[2], tr[0] * S, r->num[3], r->num[4], r->num[5], tr[1] * S, r->num[6], r->num[7],
                  r->num[8], tr[2] * S);
    } catch (const Standard_Failure&) {
      warn("an instance's transform is not a rigid placement and was ignored");
      return {};
    }
    return TopLoc_Location(T);
  }

 private:
  std::unordered_map<uint32_t, Surf> surfaces_;
  std::unordered_map<uint32_t, Crv> curves_;
  std::unordered_map<uint32_t, TopoDS_Vertex> vertices_;
  std::unordered_map<uint32_t, TopoDS_Edge> edges_;
  std::unordered_map<uint32_t, std::pair<uint32_t, bool>> edge_sp_;  // edge -> the SP-curve its curve is made from, reversed
  std::unordered_map<uint32_t, TopoDS_Face> faces_;
  std::unordered_map<uint32_t, std::pair<double, double>> blend_range_;
  std::unordered_map<uint32_t, std::array<std::vector<uint32_t>, 2>> contact_curves_;  // blend -> per support
  std::unordered_map<uint32_t, TDF_Label> labels_;
  std::vector<uint32_t> body_faces_;  // the faces of the body being built
  int unnamed_bodies_ = 0, unnamed_assemblies_ = 0, dropped_faces_ = 0;
  std::map<std::string, int> dropped_kinds_;
  std::unordered_map<const void*, std::string> face_source_;
  std::unordered_map<const void*, uint32_t> face_node_;
  std::set<uint32_t> building_;
  std::set<std::string> warned_;
  Handle(XCAFDoc_ShapeTool) st_;
  Handle(XCAFDoc_ColorTool) ct_;
  int total_ = 0, done_ = 0, built_ = 0;
};

}  // namespace

Built build(const File& file, const std::function<bool(double)>& progress) {
  Builder b(file);
  return b.run(progress);
}

}  // namespace opad::xt
