# MATLAB and Simulink code

Modelling, controller design and simulation of the ride-height and pitch control
of a fully submerged hydrofoil boat (parameters of the Técnico Solar Boat SR02).
The derivation of the model is in `../model_derivation.pdf`; the design notes are
in `../control_design.pdf`; the Simulink model is explained block by block in
`simulink_guide.html`.

Run everything with this folder as the current folder. Some MATLAB installations
contain another file called `params.m`, which would otherwise be called instead
of ours.

## 1. Files

| File | Type | Purpose | Run it directly? |
|---|---|---|---|
| `params.m` | function | all physical parameters, as a struct `p` | no |
| `hydrofoil_nl.m` | function | nonlinear model: state derivative from state, foil angles and waves | no |
| `hydrofoil_trim.m` | function | trim: foil angles for level flight | no |
| `hydrofoil_lin.m` | function | linearisation: `A`, `B`, `E`, `C`, `D` | no |
| `wave_params.m` | function | regular head waves (one component) | no |
| `wave_irregular.m` | function | irregular head waves (JONSWAP spectrum) | no |
| `wave_eval.m` | function | water velocity at the foils and wave surface under the sensor; used by every simulation and by Simulink | no |
| `design_lqr.m` | function | continuous LQR with two integrators | no |
| `design_kf.m` | function | continuous Kalman filter with a force-bias state | no |
| `sim_closed_loop.m` | function | nonlinear closed loop with the continuous controller | no |
| `design_digital.m` | function | 50 Hz design: discrete LQR and Kalman filter, optionally with a one-sample delay | no |
| `sim_digital.m` | function | continuous boat with the controller run every 20 ms | no |
| `digital_controller.m` | function | one controller sample; called by the Simulink model and by `ctrl_server` | no |
| `ctrl_server.m` | function | software in the loop: the controller as a separate MATLAB process | in a second MATLAB |
| `SilLink.m` | class | Simulink side of the TCP link (MATLAB System block) | no |
| `SilTelemetry.m` | class | sends the simulation state to the 3D view (MATLAB System block) | no |
| `run_openloop.m` | script | open-loop analysis; fig1–fig5 | yes |
| `run_lqr.m` | script | LQR with the true state; fig7–fig10 | yes |
| `run_observer.m` | script | Kalman filter and LQR from the sensors only; fig11–fig13 | yes |
| `run_digital.m` | script | 50 Hz implementation, delay, sample rate, robustness; fig15–fig17 | yes |
| `run_irregular.m` | script | three irregular sea states; fig18–fig19 | yes |
| `run_observer_compare.m` | script | pole-placement observers against the Kalman filter; saves `figs/observer_compare.mat` | yes |
| `hydrofoil_sim.slx` | Simulink model | the complete closed loop | open and run |
| `hydrofoil_sim_init.m` | script | default workspace variables for the model | no, the model calls it |
| `run_simulink.m` | script | runs the model and checks it against MATLAB; fig6, fig14 | yes |
| `run_sil.m` | script | software-in-the-loop tests; fig20, saves `figs/sil_results.mat` | yes, with `ctrl_server` running |
| `run_sil_demo.m` | script | 30 s real-time run for the 3D view | for the demo |
| `make_paper_figs.m` | script | all figures of the paper, written to `../paper/figs/` | yes |
| `build_hydrofoil_sim.m` | function | rebuilds `hydrofoil_sim.slx` from code | only to change the model |
| `export_slx_figs.m` | function | numbered diagrams for `simulink_guide.html` | only after model changes |
| `figs/` | folder | output figures and saved results | — |

Call structure:

```
run_openloop.m ─── params, hydrofoil_lin (→ hydrofoil_trim), hydrofoil_nl

run_lqr.m ──────┬─ params, hydrofoil_lin, hydrofoil_trim, wave_params
                ├─ design_lqr
                └─ sim_closed_loop (→ hydrofoil_nl)           observer off

run_observer.m ─┬─ params, hydrofoil_lin, hydrofoil_trim, wave_params
                ├─ design_lqr, design_kf
                └─ sim_closed_loop (→ hydrofoil_nl)           observer on

run_digital.m ──┬─ params, hydrofoil_lin, hydrofoil_trim, wave_params
                ├─ design_lqr, design_kf, design_digital (→ design_lqr, design_kf)
                └─ sim_closed_loop, sim_digital (→ hydrofoil_nl)

run_irregular.m ┬─ params, hydrofoil_lin, wave_irregular
                └─ design_digital, sim_digital

run_simulink.m ─┬─ params, hydrofoil_lin, design_digital, wave_params
                ├─ build_hydrofoil_sim                         (only if the .slx is missing)
                ├─ sim_digital                                 (reference for the check)
                └─ hydrofoil_sim.slx ─┬─ hydrofoil_sim_init    (on open and before each run)
                                      ├─ hydrofoil_nl          (boat block)
                                      ├─ digital_controller    (controller, Onboard choice)
                                      ├─ SilLink ──TCP──→ ctrl_server (→ digital_controller)
                                      │                       (controller, External choice)
                                      └─ SilTelemetry ──TCP──→ ../viz/sil_bridge.py → browser

run_sil.m ─────── hydrofoil_sim.slx with ctrl_ext = 1; needs ctrl_server running
run_sil_demo.m ── the same, plus ../viz/sil_bridge.py for the 3D view
```

## 2. Running

1. `run_openloop`: the plant alone. Prints the matrices, eigenvalues, ranks and the trim against speed.
2. `run_lqr`: the controller with the true state.
3. `run_observer`: the controller with the Kalman filter, from the sensors only.
4. `run_digital`: the 50 Hz implementation. A few minutes.
5. `run_irregular`: irregular seas. A few minutes.
6. `run_simulink`: checks of the Simulink model.
7. `run_observer_compare`: the observer comparison of the paper. A few minutes.
8. `run_sil`: software in the loop. Start `ctrl_server` first (section 6.4). About 3 minutes, in real time.
9. `make_paper_figs`: the paper figures.

Each `run_` script starts with `clear; close all; clc;`, so results do not depend
on what is in the workspace. Runs with sensor noise use fixed seeds.

## 3. Symbols and units

All angles in the code are in **radians**; plots show degrees.

| Symbol | Meaning | Unit |
|---|---|---|
| `x = [h; v; theta; q]` | heave, heave rate, pitch, pitch rate; deviations from trim, `h` up and `theta` bow-up positive | m, m/s, rad, rad/s |
| `u` or `delta` | **absolute** foil incidence `[front, rear]` | rad |
| `u0` | trim incidence (3.53° and 3.11° at 7 m/s) | rad |
| `u - u0` | deviation from trim, used by the linear model and the controller | rad |
| `w = [w_f, w_r]` | vertical water velocity at the foils (waves) | m/s |
| `y = [bow height; pitch; pitch rate]` | ultrasonic sensor, AHRS, rate gyro | m, rad, rad/s |
| `r` | height command relative to trim | m |
| `xe = [h; v; theta; q; b]` | observer state; `b` is the estimated constant vertical force | as above, m/s² |

## 4. Model

### 4.1 `params.m`

All parameters in one struct: `p = params();`. Each line is tagged `[B]` (from
Belime 2020) or `[est]` (estimated or assumed). Foil quantities are two-element
row vectors `[front, rear]`: area `S`, position ahead of the CM `x` (negative for
the rear foil), depth below the CM `z`, lift coefficients `CL0` and `CLa`, chord
`c`, trim submergence `d0`, stall angle `alpha_stall`. Also the speed `U`, the
sensor position `ls`, the noise levels `sigma_y`, `sigma_th`, `sigma_q`, the
sample times, and the actuator limits `u_min`, `u_max`, `rate_max`, `tau_act`.
The chord, submergence, stall angle, noise levels, sample times, rate limit and
servo time constant are assumed.

### 4.2 `hydrofoil_nl.m`

`[xdot, out] = hydrofoil_nl(x, u, w, p)` returns the state derivative; `out`
holds the lift, angle of attack and submergence of each foil. For each foil it
computes the position relative to the CM, the velocity relative to the water and
the inflow angle, the angle of attack (limited at stall), the submergence and the
lift with the free-surface factor `Kfs`, and adds the vertical force and the
pitching moment. `Kfs` is zero once a foil leaves the water.

### 4.3 `hydrofoil_trim.m`

`[u0, L0] = hydrofoil_trim(p)`: the force and moment balance gives the lift of
each foil, and the lift equation is inverted for the incidence. It depends on
`p.U`.

### 4.4 `hydrofoil_lin.m`

`[A, B, E, C, D, u0] = hydrofoil_lin(p)` gives the analytic linear model
`dx/dt = A x + B (u - u0) + E w`, `y = C x`. It computes four coefficient pairs:
`k` (lift per radian of angle of attack), `kap` (lift per metre of submergence),
`sig` (lift from pitch rate) and `Wz` (pendulum moment), and fills `A`, `B`, `E`
row by row. `C` has rows `[1 0 ls 0]`, `[0 0 1 0]`, `[0 0 0 1]`. `run_openloop`
checks the matrices against a numerical Jacobian.

### 4.5 Waves: `wave_params.m`, `wave_irregular.m`, `wave_eval.m`

Waves are a sum of components with amplitude `a`, frequency `om`, wave number
`k` and phase `ph`.

- `wave_params(p, amplitude, wavelength)`: one component; default 5 cm, 8 m.
- `wave_irregular(p, Hs, Tp, seed)`: 80 components of a JONSWAP spectrum with
  random phases; the same seed gives the same sea.
- `[w, zeta] = wave_eval(wv, t, p)`: water velocity at the foils,
  `w = -a ω exp(-k d0) sin(...)`, and the surface under the sensor,
  `zeta = a cos(...)`, at the encounter frequency `ω + k U`.

Every simulation and the Simulink model take their waves from `wave_eval` only.
An early version had the sign of `w` wrong, which shifted the wave force against
the measured surface by 180°; it was found while writing `wave_eval` and all
affected results were recomputed (`control_design.pdf`, D12).

### 4.6 `run_openloop.m`

| Section | Content | Output |
|---|---|---|
| 1. Checks | numerical vs analytic Jacobian; eigenvalues; reachability and observability ranks; trim against speed | console |
| 2. Linear vs nonlinear | step, pulse and sine inputs | `fig1` |
| 3. Range of validity | front-foil steps from 0.25° to 3° | `fig2` |
| 4. Actuator limits | ideal and limited actuators | `fig3` |
| 5. Model uncertainty | trim inputs with wrong speed or mass | `fig4` |
| 6. Waves | regular head waves and noisy sensor readings | `fig5` |

## 5. Control

### 5.1 `design_lqr.m`

`ctrl = design_lqr(p)` returns `K` (2×4), `Ki` (2×2), `Nx`, `Nu`, `Cz`, the
prefilter time constant `tf` (0.3 s) and the closed-loop poles. The control law,
in deviations from trim:

```
u - u0 = Nu*rf - K*(x - Nx*rf) - Ki*xi
dxi/dt = [h - rf; theta]        integrators on height and pitch
drf/dt = (r - rf)/tf            reference prefilter
```

With the observer, `h` in the integrator is the estimate and `theta` the AHRS
reading. Integrating height alone leaves a pitch offset under model errors;
integrating both brings both to their targets. The weights follow Bryson's rule
(2 cm of heave, 2° of pitch, 3° of foil deflection, ...); the integral weights
were tuned (factor 10, normalised to 2 cm and 1°).

### 5.2 `design_kf.m`

`obs = design_kf(p)` returns the augmented model `Ae`, `Be`, `Ce`, the gain `L`
(5×3), the noise intensities `Qn`, `Rn`, the poles, and `Ao = Ae - L*Ce`,
`Bo = [Be L]`.

- **Bias state `b`**: a speed or mass error changes both lifts in proportion,
  which is a vertical force without a moment. Without this state the filter
  would explain it as a pitch offset and its estimates would be biased.
- **Process noise**: wave velocities through `E`, and a random walk on `b` with
  `b_rate = 0.5`. A larger value recovers faster from model errors but follows
  long waves, which look like a slowly varying bias.
- **Measurement noise**: the ultrasonic reading contains the wave surface, whose
  power lies in a band of about 0.25 Hz around the encounter frequency. Its
  density there is about 100 times that of white noise of the same variance;
  using it keeps the filter from following the water surface.

### 5.3 `sim_closed_loop.m`

`out = sim_closed_loop(ptrue, pdes, ctrl, obs, ref, tgrid, opt)`: `ptrue` is the
simulated boat, `pdes` the boat the controller was designed for; `ref` is a
function `r(t)`; `opt` sets `observer`, `antiwindup`, `wave`, `noise`, `seed`.
Each step: measurements with wave surface and noise (held for 20 ms), observer,
control law, position limit, servo lag and rate limit, boat. The integrators
stop while a command saturates.

### 5.4 `run_lqr.m`

| Section | Content | Output |
|---|---|---|
| 1. Design | gains and poles | console |
| 2. 5 cm step | linear and nonlinear | `fig7` |
| 3. Waves | open and closed loop | `fig8` |
| 4. Robustness | pole grid over speed 5.9–9 m/s and mass 190–240 kg; steps with errors | console, `fig9` |
| 5. Saturation | 20 cm step without prefilter, with and without anti-windup | `fig10` |

### 5.5 `run_observer.m`

| Section | Content | Output |
|---|---|---|
| 1. Observer design | gain, poles, separation-principle check | console |
| 2. Estimation | true, estimated and measured states in waves | `fig11` |
| 3. Output vs full state | the same test with estimate or true state | `fig12` |
| 4. Robustness | pole grid and steps with speed and mass errors; bias estimate | console, `fig13` |

### 5.6 `design_digital.m`

`dc = design_digital(p, Ts, delay)`, e.g. `design_digital(p, 0.02, 1)`: 50 Hz
with a one-sample computation delay. The continuous filter cannot be run with a
20 ms Euler step: its pole at −287 rad/s needs `Ts < 2/287 = 7 ms`. Both designs
are therefore repeated in discrete time:

1. zero-order-hold discretisation (`c2d`);
2. `dlqr` with the same weights times `Ts`; integrators `xi = xi + Ts*error`;
   with `delay = 1` the previous command is an extra state, with gain `Kd`;
3. `dlqe` with the process noise discretised exactly (Van Loan) and measurement
   covariance equal to the continuous intensity divided by `Ts`;
4. prefilter time constant 0.6 s.

### 5.7 `sim_digital.m` and `digital_controller.m`

`out = sim_digital(ptrue, pdes, dc, ref, Tend, opt)` integrates the boat with
`ode45` and runs the controller every `Ts`; `opt.delay` may differ from the
design, `opt.x0` sets the initial state. `digital_controller.m` is the same
algorithm as a function with persistent state, written as it would run on the
boat (`clear digital_controller` resets it). Each call:

1. predict the estimate one sample ahead, with the mean encoder reading;
2. correct it with the new measurements;
3. control law, including `-Kd*(du_prev - Nu*rf)` for the delay;
4. update the integrators unless the command saturates;
5. update the prefilter.

### 5.8 `run_digital.m`

| Section | Content | Output |
|---|---|---|
| 1. Emulation | why the continuous observer diverges at 50 Hz | console |
| 2. Digital design | equivalent poles and gains | console |
| 3. Continuous vs digital | with and without delay and compensation | console, `fig15` |
| 4. Sample rate | 100 to 10 Hz | console, `fig16` |
| 5. Robustness | pole grid and steps with speed and mass errors | console, `fig17` |

### 5.9 `run_irregular.m` and `run_observer_compare.m`

`run_irregular` tests the 50 Hz controller in three sea states (S1 to S3,
Hs 0.10, 0.15, 0.25 m), five random seas each (`fig18`, `fig19`).
`run_observer_compare` keeps the controller and replaces the Kalman gain by
pole-placement gains (3× and 1× the controller poles, and the Kalman filter's
own poles), in regular waves, sea S2 and with a speed error.

## 6. Simulink

### 6.1 `hydrofoil_sim_init.m`

Creates the workspace variables the model needs if they do not exist yet:
50 Hz controller with delay, 5 cm step at 1 s, waves, noise and actuator limits
on, 10 s, controller inside the model (`ctrl_ext = 0`), no telemetry
(`viz_on = 0`). Software-in-the-loop variables: `ctrl_ext`, `sil_lockstep`,
`sil_extra_ms`, `sil_jitter_ms`, `viz_on`. Called when the model is opened and
before each run.

### 6.2 `run_simulink.m`

1. Check 1: open loop, ideal actuators, front-foil step; Simulink against `ode45`.
2. Check 2: open loop in waves; nonlinear part against `ode45`, linear part against `lsim`.
3. Open-loop demo with limits, waves and noise (`fig6`).
4. Check 3: closed loop in waves without noise; Simulink against `sim_digital` (0.3 mm).
5. Closed-loop demo (`fig14`).

Results are in `out.x_nl`, `out.x_lin`, `out.delta`, `out.y_meas`, `out.xhat`
and `out.link`.

### 6.3 `build_hydrofoil_sim.m`

Deletes and rebuilds `hydrofoil_sim.slx` (manual layout changes are lost).
Solver `ode45` with a 5 ms maximum step and zero-crossing detection off (the
actuator limits switch too often for it). The controller is a variant subsystem
with two choices, both atomic with a 20 ms sample time: Onboard (a MATLAB
Function calling `digital_controller.m`) and External (MATLAB System block
`SilLink`). An `Encoder` memory block gives the controller the foil angle from
before its new command and avoids an algebraic loop.

### 6.4 Software in the loop: `ctrl_server.m`, `SilLink.m`, `run_sil.m`

The controller runs in a second MATLAB process. Every 20 ms the model sends the
three sensor readings, the encoder readings and the height command over TCP; the
controller replies with the command, its estimate and its computing time. It
never sees the true state.

- `ctrl_server.m`: start it in a second MATLAB with this folder as current
  folder, or with `../start_sil.bat`. Port 5501. If computing takes longer than
  a sample, the older readings only update the filter and only the newest is
  answered.
- `SilLink.m`: outputs the reply to the previous sample, then sends the current
  readings, so a command computed from `y[k]` acts at `k+1`, the delay the
  controller was designed for.

| Mode | Setting | Behaviour |
|---|---|---|
| lockstep | `sil_lockstep = 1` | waits for each reply; deterministic |
| real time | `sil_lockstep = 0` and `EnablePacing` on | holds the last command when a reply is late |

MATLAB's `tcpclient` waits for the 15.6 ms Windows timer on every read and write
(31 ms round trip) and `tcpserver` needs the Instrument Control Toolbox, so both
ends use Java sockets (0.2–0.4 ms round trip).

`run_sil.m` results: lockstep identical to the in-model controller; real time
unchanged up to 18 ms of computing time, one sample late from 22 to 35 ms with
the same height error, 2–3 cm at 40 ms and about 7 cm at 50 ms; random
computing times of 10–30 ms make 80% of replies late with no change in error;
the linear check gives a damping ratio of 0.62, 0.32, 0.11 and −0.03 for 0 to 3
extra samples.

### 6.5 3D view: `SilTelemetry.m` and `../viz/`

With `viz_on = 1` the model sends its signals every 20 ms to
`../viz/sil_bridge.py`, which serves the viewer at http://localhost:8000 and
records each run. See `../viz/README.md`.

### 6.6 `export_slx_figs.m`

Adds temporary numbered labels to the model, prints the diagrams for
`simulink_guide.html` and closes the model without saving.

## 7. Common changes

| To | Do |
|---|---|
| change the speed | `p = params(); p.U = 8;` then call `hydrofoil_lin`, `design_lqr`, `design_kf` again |
| change a parameter permanently | edit `params.m` |
| switch the controller off in Simulink | `ctrl_on = 0` |
| change the controller rate | `dc = design_digital(p, 0.01, 1)` (100 Hz) |
| switch waves, noise or limits | `wave_on`, `noise_on`, `act_on` to 1 or 0 |
| change the run length | `Tstop = 20` |
| change the height command | `r_ts = timeseries([0;0;0.1;0.1], [0;2;2;20])` |
| change the waves | `wv = wave_params(p, 0.08, 12)` or `wv = wave_irregular(p, 0.15, 3.0, 1)` |
| return to the defaults | `clear`, then run the model |
| controller in a separate process | `ctrl_server` in a second MATLAB, then `ctrl_ext = 1` |
| real time | `sil_lockstep = 0; load_system('hydrofoil_sim'); set_param('hydrofoil_sim', 'EnablePacing', 'on')` |
| 3D view | `python ../viz/sil_bridge.py`, then `viz_on = 1` |

## 8. Scope and limitations

- Hull contact is not modelled; results below a heave of about −30 cm are not physical.
- The forward speed is constant; speed changes are treated as parameter errors.
- The controller is designed at 7 m/s without gain scheduling. A speed 1 m/s off
  during a 5 cm step gives a transient of up to about 15–19 cm (6 m/s) and
  10 cm (8 m/s).
- The front foil has 3.5° of upward travel; it saturates briefly in sea state S3.
- Sensors and controller are assumed synchronous at 50 Hz.
- `b_rate` trades recovery from model errors against motion in long waves.
- In the software-in-the-loop test both processes run on one computer, and the
  controller is triggered by the arrival of a reading, so bus delays and clock
  drift are not represented.
