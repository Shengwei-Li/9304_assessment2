# 3D view

A browser view of `hydrofoil_sim.slx`: heave and pitch of the boat, the actual
foil angles, the wave surface, the ultrasonic measurement at the bow, and where
the controller believes the boat is (orange outline). The panel on the right
shows the current values and synchronised plots.

Everything that moves comes from the simulation; the page only draws it.

| On screen | Source |
|---|---|
| hull position and attitude | true state `x` of the nonlinear model |
| orange outline | the controller's Kalman estimate `xh` |
| foil angles | actuator output `delta` |
| water surface | the model's wave components, evaluated with the formula of `wave_eval.m` in the boat's frame |
| grey dots in the plots | sensor readings, with noise and (ultrasonic) the wave surface |
| controller timing | computing and round-trip time reported in the software-in-the-loop mode |

## Running

1. Start the bridge (Python standard library only):
   ```
   python viz/sil_bridge.py
   ```
2. Open http://localhost:8000. three.js is loaded from jsdelivr, so the browser
   needs internet access.
3. In MATLAB, with `matlab/` as current folder:
   ```matlab
   viz_on = 1;                                          % send telemetry
   load_system('hydrofoil_sim');
   set_param('hydrofoil_sim', 'EnablePacing', 'on');    % run in real time
   sim('hydrofoil_sim')
   ```
   For the controller in a separate process add `ctrl_ext = 1; sil_lockstep = 0;`
   and start `ctrl_server` first. `start_sil.bat` in the repository root starts
   the controller process, the bridge and the browser; `matlab/run_sil_demo.m`
   is a 30 s run in an irregular sea.

Without pacing the simulation runs much faster than real time; use the replay
in that case.

## Replay, video and still images

Each run is recorded to `viz/recordings/run_<date>_<time>.bin`. Choose one in
the list at the bottom of the page to

- play it at 0.1× to 1× and scrub through it; in slow motion the command (thin
  line) changes every 20 ms and the servo (thick line) follows it;
- switch the view: side, 3/4, chase, bow (sensor and front foil), underwater
  (both foils); drag to rotate, scroll to zoom;
- draw the motion 3× or 10× larger: the real heave is a few centimetres; the
  hull keeps its shape, only the motion and the wave surface are scaled.

Press S to save the 3D view as a 2400 × 1500 PNG in `viz/snapshots/`. Record
video with any screen recorder.

## Files

| File | Purpose |
|---|---|
| `sil_bridge.py` | receives telemetry on port 5502, serves the page and a WebSocket on port 8000, records runs, saves snapshots |
| `index.html` | page layout |
| `app.js` | data, plots, replay and controls |
| `scene.js` | 3D scene (three.js): sky, water shader, boat |

## Notes

- The large-scale water surface is the model's wave field, evaluated per vertex
  on the GPU, so the ultrasonic beam ends on the surface that is drawn. Short
  ripples and the strut wakes change only the shading and are not part of the
  model. Waves fade out beyond 40 m from the boat, where the mesh is too coarse
  for short waves.
- The hull shape is schematic and the foil chord is drawn as 0.16 m (0.073 m in
  the model) to keep it visible. Foil positions, submergence (0.30 m at trim)
  and the sensor position (3 m ahead of the CM) come from the model parameters.
- Telemetry format: see the comments in `matlab/SilTelemetry.m` (big-endian
  doubles, each packet preceded by its length).
