# Digital Twin — Data Generation

This folder contains the code that produced the dataset in [`../data/`](../data/). It is the
Python end of the digital-twin pipeline:

```
DEM (Copernicus COP30)
  → MATLAB / RoadRunner R2024a   (HD map from the GU-186 terrain)
  → CARLA Sim 0.9.16             (autonomous traversal, 20 Hz kinematic logging)
  → energy model in Python       (this folder)
```

CARLA's Traffic Manager drives one vehicle over the truncated GU-186 segment while a Python
client logs the kinematic trace; the energy variables (SOC, power, current, voltage,
regeneration, battery temperature, …) are then computed tick-by-tick by the model here and
written to a CSV.

## Files

| File | Purpose |
|---|---|
| `ev_model.py` | Longitudinal EV energy model with weather coupling, plus `EV_LIBRARY`, a dictionary of parameterized vehicles (Tesla Model 3 RWD, Audi e-tron 55 quattro, and a Cybertruck entry). |
| `carla_acquisition.py` | CARLA acquisition script. Holds the 12-scenario factorial catalog (`SCENARIOS`); running a scenario is a matter of setting `ACTIVE_SCENARIO`. Writes `datos_grid_<ID>.csv`. |

## `ev_model.py`

`ElectricVehicleModel` integrates the longitudinal force balance (grade, rolling resistance,
aerodynamic drag, inertia) and maps wheel power to battery power through motor/regeneration
efficiencies, discharge/regen limits, an internal-resistance thermal model, and SOC-dependent
pack voltage. Optional weather coupling applies three effects:

1. HVAC auxiliary power vs ambient temperature (`hvac_aux_power`),
2. a battery-capacity factor vs ambient temperature (`battery_capacity_factor`), and
3. an effective rolling-resistance coefficient vs pavement condition (`rolling_coefficient`).

The module has no CARLA dependency and can be imported on its own (it is also reused by the
federated experiments' physical baseline). The full vehicle- and weather-parameter tables, and
their literature sources, are documented in the [dataset README](../data/README.md#2-experimental-design).

## `carla_acquisition.py`

The script defines all 12 scenarios of the 2 × 2 × 3 grid (vehicle × initial SOC × weather) in
the `SCENARIOS` constant. To produce one log:

1. set `ACTIVE_SCENARIO` to the desired identifier (e.g. `'S05'`);
2. start CARLA 0.9.16 with the GU-186 HD map loaded;
3. run the script.

A run is one-way and ends when the vehicle has accumulated ≥ 10 km of physical travel (a stuck
detector and a safety timeout are the fallback stop conditions). The output file is named
`datos_grid_<ID>.csv` and matches the schema documented in the
[dataset README](../data/README.md#32-column-schema).

**Note on road grade.** The OpenDRIVE network exported from RoadRunner has flat lane elevation
(`z = 0`), while the visual mesh and physical colliders keep the DEM elevation. The model
therefore reads the grade from the vehicle's instantaneous pitch angle, not from the waypoint
network — see the [dataset README](../data/README.md#1-geographic-and-topographic-context) for
details.

## Requirements

- CARLA Sim **0.9.16** and its Python API (only for `carla_acquisition.py`).
- Python 3.12 with `numpy` and `opencv-python` (`cv2`, used for the live HUD).

`ev_model.py` on its own needs only the Python standard library.
