# Slipstream

An aircraft design and aerodynamics workstation in the browser. One project aircraft is shared by five modules.

**Run it:** open `index.html` through any web server (for example `python3 -m http.server`), or publish this folder with GitHub Pages. Opening the file directly from disk will not work, because browsers block Web Workers and file loading on `file://` pages.

The 3D tunnel needs WebGPU: Chrome or Edge 113+, or Safari 26. The other two views work in any modern browser.

## Modules
- **Geometry:** wing planform, airfoil, flaps, winglets, tail, fuselage, mass, CG and propulsion; tail volume, loading and Reynolds number.
- **Airfoil (2D LBM):** D2Q9 lattice Boltzmann with Smagorinsky LES around the project's NACA section; momentum-exchange forces; Lagrangian supercooled droplets (Schiller–Naumann drag) and cell-based rime/glaze ice accretion with runback.
- **Wing and tail (VLM):** horseshoe vortex lattice on the cambered surfaces, Kutta–Joukowski forces, Richardson-extrapolated Trefftz-plane induced drag, skin-friction drag build-up, critical-section stall, neutral point, static margin and trim. Validated: rectangular AR 6 wing gives CLα = 4.29/rad, e = 0.96.
- **Flight and icing:** ISA atmosphere, point-mass longitudinal dynamics with an autopilot, aircraft aerodynamics taken from the VLM solve, ice accretion with Langmuir–Blodgett collection efficiency and empirical iced-wing penalties.
- **3D flow (WebGPU):** D3Q19 lattice Boltzmann compute shaders with a symmetry plane, voxelised project mesh or reference models, GPU streamlines, flow slices. Load your own STL, OBJ or GLB.

Colour maps: viridis (magnitude) and cool-warm (signed quantities).

## Model sources
| File | Source |
|---|---|
| `models/nasa-crm.slm` | NASA Common Research Model (NASA Langley), DPW-4 wing-body-tail surface distributed with the DAFoam tutorials; decimated |
| `models/onera-m6.slm` | ONERA M6 wing surface from the SU2 project validation mesh |
| `models/nasa-x57.slm` | NASA X-57 Maxwell, NASA 3D Resources (public domain); decimated |
| `models/nasa-global-hawk.slm` | NASA Global Hawk, NASA 3D Resources (public domain) |

NASA models are used under NASA's media usage guidelines; their use implies no endorsement by NASA.

`.slm` format: `'SLM1'`, uint32 vertex count, uint32 triangle count, float32 xyz positions, uint32 triangle indices (little endian). Axes: x downstream (nose at −x), y up, z span.
