# Slipstream

Fly a real aircraft and watch what the air and the weather do to it. `index.html` is the live explore view. Pick a Cessna 172, Spitfire Mk IX, Boeing 737-800 or Airbus A320 (full 3D airframes), set the speed, altitude and wing angle, and fly, hold it in a wind tunnel, or park it on a runway. Weather can be clear, supercooled icing cloud, freezing rain, snow or frost:

- **Ice** builds on the real surfaces wherever the airflow strikes them (wing and tail leading edges, nose, struts, windscreen), using local impingement × Langmuir–Blodgett collection efficiency. Cold, small-drop cloud gives milky **rime** on the leading edge; freezing rain gives clear **glaze** that runs back over the surface. Ice lowers maximum lift and raises drag (empirical iced-wing penalties), so the stall speed rises.
- **Snow and frost** settle on the upward-facing surfaces of a parked aircraft and change the take-off (FAA AC 120-58 contamination penalties, up to 30% lift loss).
- **Airflow:** blue wingtip vortex trails, smoke streams that follow the vortex-lattice velocity field, a lift map on the wing, and a live 2D lattice-Boltzmann cross-section of the selected aircraft's own wing section (NACA 2412, NACA 2213, or a supercritical-style approximation for the airliners) coloured by simulated pressure or speed, with the lift, stagnation point and separation point marked.
- Six guided experiments: lift, stall, wingtip vortices, flying into icing cloud, freezing rain, and snow and frost before take-off.

`lab.html` is the engineering workstation with the full numbers. One project aircraft is shared by five modules.

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
| `vendor/three-0.149.0.min.js` | three.js r149 (MIT licence, see `vendor/three-LICENSE.txt`) |
| `models/nasa-crm.slm` | NASA Common Research Model (NASA Langley), DPW-4 wing-body-tail surface distributed with the DAFoam tutorials; decimated |
| `models/onera-m6.slm` | ONERA M6 wing surface from the SU2 project validation mesh |
| `models/nasa-x57.slm` | NASA X-57 Maxwell, NASA 3D Resources (public domain); decimated |
| `models/nasa-global-hawk.slm` | NASA Global Hawk, NASA 3D Resources (public domain) |

| `models/c172.slm3` | FlightGear Cessna 172P by the c172p team, GPL-2.0 |
| `models/spitfire.slm3` | FlightGear Spitfire IX (FGMEMBERS), GPL-2.0 |
| `models/b738.slm3` | FlightGear Boeing 737-800 (FGMEMBERS), GPL-2.0 |
| `models/a320.slm3` | FlightGear Airbus A320 family by legoboyvdlp and contributors, GPL-2.0 |

The four aircraft meshes were converted from the FlightGear AC3D models: interior, cockpit, lights and animation-only parts were removed, and the meshes were moved into the aerodynamic body frame. They stay under the GNU General Public License version 2 (full text in `models/LICENSE-GPL-2.0.txt`); the original models and their full source are at https://github.com/c172p-team/c172p, https://github.com/FGMEMBERS/SpitfireIX, https://github.com/FGMEMBERS/737-800 and https://github.com/legoboyvdlp/A320-family. Aircraft names are used only to identify the types; no manufacturer endorses this project. The paint schemes are generic.

NASA models are used under NASA's media usage guidelines; their use implies no endorsement by NASA.

`.slm` format: `'SLM1'`, uint32 vertex count, uint32 triangle count, float32 xyz positions, uint32 triangle indices (little endian). Axes: x downstream (nose at −x), y up, z span.
