#!/usr/bin/env python3
"""Download public STEP and Parasolid files for the robustness corpus (opt-in; see README.md).

Sources are (url, filename) pairs from repositories that publish sample STEP files under permissive terms.
Files land next to this script and are git-ignored.
"""
import os
import sys
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))

# Open CASCADE's own sample data and a few well-known open-hardware parts. Extend freely.
SOURCES = [
    # Open CASCADE sample data (LGPL with exception)
    ("https://raw.githubusercontent.com/Open-Cascade-SAS/OCCT/master/data/step/screw.step", "occt-screw.step"),
    ("https://raw.githubusercontent.com/Open-Cascade-SAS/OCCT/master/data/step/linkrods.step", "occt-linkrods.step"),
    # STEPcode test data (BSD): AP214 assemblies from the classic CAx-IF test cases
    ("https://raw.githubusercontent.com/stepcode/stepcode/develop/data/ap214e3/as1-oc-214.stp", "stepcode-as1-oc-214.stp"),
    ("https://raw.githubusercontent.com/stepcode/stepcode/develop/data/ap214e3/dm1-id-214.stp", "stepcode-dm1-id-214.stp"),
    ("https://raw.githubusercontent.com/stepcode/stepcode/develop/data/ap214e3/io1-cm-214.stp", "stepcode-io1-cm-214.stp"),
    ("https://raw.githubusercontent.com/stepcode/stepcode/develop/data/ap214e3/sg1-c5-214.stp", "stepcode-sg1-c5-214.stp"),
    ("https://raw.githubusercontent.com/stepcode/stepcode/develop/data/ap214e3/s1-c5-214/s1-c5-214.stp", "stepcode-s1-c5-214.stp"),
    # Parasolid transmit files (test_xt builds those present): test assets of PyMAPDL / PyMechanical (MIT) and SCOREC
    # core (BSD), and open-hardware parts: an assembly, a threaded dock with rolling-ball blends, a display module
    ("https://raw.githubusercontent.com/ansys/pymapdl/32234e5cf35b9aaa4dff22da8badf782c1e5f60c/tests/test_files/CAD_formats/CubeWithHole.x_t",
     "pymapdl-CubeWithHole.x_t"),
    ("https://raw.githubusercontent.com/ansys/pymechanical/b90aea8746c15f62fe0929ede5465559d3e97c9f/tests/assets/hsec.x_t",
     "pymechanical-hsec.x_t"),
    ("https://raw.githubusercontent.com/SCOREC/core/7166244ff601fb3b3847543d725762a2e330feef/python_wrappers/input/sphere.x_t",
     "scorec-sphere.x_t"),
    ("https://raw.githubusercontent.com/Annex-Engineering/Quickdraw_Probe/90ffc6ad86e9b21b09139dd89395dd152c9ed2ea/CAD/Release_1/Omron_Mag_Probe_Assembly.x_t",
     "quickdraw-omron-assembly.x_t"),
    ("https://raw.githubusercontent.com/Annex-Engineering/Quickdraw_Probe/90ffc6ad86e9b21b09139dd89395dd152c9ed2ea/CAD/Release_1/Microswitch%20dock%20-%20short.x_t",
     "quickdraw-microswitch-dock.x_t"),
    ("https://raw.githubusercontent.com/sparkfun/3D_Models/53d298c3b8202cf8fce9ab61885855720c84c012/products/12923/Sparkfun_display.x_t",
     "sparkfun-display.x_t"),
]


def main() -> int:
    ok = 0
    for url, name in SOURCES:
        dest = os.path.join(HERE, name)
        if os.path.exists(dest):
            ok += 1
            continue
        try:
            print(f"fetching {name} ...", end=" ", flush=True)
            urllib.request.urlretrieve(url, dest)
            print(f"{os.path.getsize(dest)} bytes")
            ok += 1
        except Exception as e:  # noqa: BLE001
            print(f"FAILED: {e}")
    print(f"{ok}/{len(SOURCES)} files available in {HERE}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
