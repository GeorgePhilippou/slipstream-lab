# Slipstream Lab

A browser aerodynamics lab: a 2D wing and icing simulation, an aircraft designer with a vortex-lattice solver, flight analysis, and a 3D wind tunnel that runs a lattice Boltzmann solver on the GPU.

**Live:** https://georgephilippou.github.io/slipstream-lab/

Static site: open `index.html` through any web server. The 3D tunnel needs a browser with WebGPU.

These are teaching models, not engineering tools. The limits of each solver are explained in the app.

## Model sources

- NASA Common Research Model: NASA Langley, from the 4th AIAA CFD Drag Prediction Workshop, distributed with the DAFoam tutorials
- ONERA M6 wing (Schmitt & Charpin, 1979): surface from the SU2 project's validation mesh
- NASA X-57 Maxwell and Global Hawk: NASA 3D Resources (public domain)

NASA models are used under NASA's media guidelines and imply no endorsement by NASA.
