function build_hydrofoil_sim()
%BUILD_HYDROFOIL_SIM  Generate the Simulink model hydrofoil_sim.slx.
%   The model contains the 50 Hz digital controller (Kalman filter + LQR), the actuators,
%   the nonlinear boat, the linear model (run in parallel for comparison),
%   the wave disturbance and the three sensors.
%
%   Workspace variables the model needs (defaults come from hydrofoil_sim_init.m,
%   run_simulink.m overrides them):
%     p, A, B, E, u0        parameters and linear model
%     dc                    digital controller (design_digital), run at p.Ts_ctrl
%     wv                    wave data: aw, we, kw, zeta_a
%     r_ts                  timeseries, requested height change [m]
%     u_cmd_ts              timeseries, extra open-loop foil command [rad], 2 columns
%     ctrl_on, act_on, wave_on, noise_on   1/0 switches
%     Tstop                 simulation time [s]

mdl = 'hydrofoil_sim';
here = fileparts(mfilename('fullpath'));
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(fullfile(here, [mdl '.slx']), 'file'), delete(fullfile(here, [mdl '.slx'])); end
new_system(mdl);
set_param(mdl, 'Solver', 'ode45', 'MaxStep', '0.005', 'RelTol', '1e-6', ...
               'StopTime', 'Tstop', 'SaveOutput', 'off', 'SaveTime', 'off', ...
               'ReturnWorkspaceOutputs', 'on', ...
               'ZeroCrossControl', 'DisableAll');   % actuator limits switch often; MaxStep handles them
% make the model usable by itself: put its folder on the path and create any
% missing workspace variables when it is opened and before every run
set_param(mdl, 'PostLoadFcn', ...
    'addpath(fileparts(get_param(''hydrofoil_sim'', ''FileName''))); hydrofoil_sim_init;');
set_param(mdl, 'InitFcn', 'hydrofoil_sim_init;');

%% ---------------- top level blocks ----------------
% Row 1: the loop, signal flows left to right
blk('simulink/Sources/Constant',       'Trim u0',      [60 120 140 150], 'Value', 'u0(:)');
blk('simulink/Sources/From Workspace', 'Foil command', [40 185 140 215], ...
    'VariableName', 'u_cmd_ts', 'OutputAfterFinalValue', 'Holding final value');
blk('simulink/Signal Routing/From',    'From y_meas',  [20 252 80 276], 'GotoTag', 'y_meas');
blk('simulink/Signal Routing/From',    'From delta ctrl', [10 287 60 311], 'GotoTag', 'delta');
blk('simulink/Discrete/Memory',        'Encoder',         [75 287 105 311], 'InitialCondition', 'u0(:)');   % reading taken before the new command
blk('simulink/Sources/From Workspace', 'Height reference', [10 322 90 348], ...
    'VariableName', 'r_ts', 'OutputAfterFinalValue', 'Holding final value');
sub('Controller',                      [130 245 230 345]);
blk('simulink/Math Operations/Sum',    'Sum cmd',      [270 170 300 230], 'Inputs', '+++', 'IconShape', 'rectangular');
sub('Actuators',                       [370 175 480 225]);
sub('Nonlinear boat',                  [590 165 730 235]);
sub('Sensor',                          [860 170 980 230]);
sub('Waves',                           [370 290 480 350]);
blk('simulink/Sinks/To Workspace',     'log y_meas',   [1050 185 1140 215], 'VariableName', 'y_meas', 'SaveFormat', 'Timeseries');
blk('simulink/Signal Routing/Goto',    'Goto delta',   [520 115 580 141],   'GotoTag', 'delta');
blk('simulink/Signal Routing/Goto',    'Goto x',       [770 115 830 141],   'GotoTag', 'x_nl');
blk('simulink/Signal Routing/Goto',    'Goto w',       [620 292 680 318],   'GotoTag', 'w');
blk('simulink/Signal Routing/Goto',    'Goto y_meas',  [1050 245 1110 271], 'GotoTag', 'y_meas');

% Row 2: linear model, driven by the same foil angles and the same waves
blk('simulink/Signal Routing/From',    'From delta',   [60 477 120 503], 'GotoTag', 'delta');
blk('simulink/Sources/Constant',       'Trim u0 lin',  [60 515 120 541], 'Value', 'u0(:)');
blk('simulink/Math Operations/Sum',    'Sum dev',      [200 480 230 520], 'Inputs', '+-', 'IconShape', 'rectangular');
blk('simulink/Signal Routing/From',    'From w',       [200 537 260 563], 'GotoTag', 'w');
blk('simulink/Signal Routing/Mux',     'Mux lin',      [300 475 305 575], 'Inputs', '2');
blk('simulink/Continuous/State-Space', 'Linear model', [380 500 500 550], ...
    'A', 'A', 'B', '[B E]', 'C', 'eye(4)', 'D', 'zeros(4,4)', 'X0', 'zeros(4,1)');
blk('simulink/Signal Routing/From',    'From x',       [600 485 660 511], 'GotoTag', 'x_nl');
blk('simulink/Sinks/Scope',            'Scope states', [720 480 760 550], 'NumInputPorts', '2');
blk('simulink/Sinks/To Workspace',     'log x_lin',    [600 570 690 600], 'VariableName', 'x_lin', 'SaveFormat', 'Timeseries');

% Row 3: logging
blk('simulink/Signal Routing/From',    'From delta log', [60 667 120 693],  'GotoTag', 'delta');
blk('simulink/Sinks/To Workspace',     'log delta',      [170 665 260 695], 'VariableName', 'delta', 'SaveFormat', 'Timeseries');
blk('simulink/Signal Routing/From',    'From x log',     [340 667 400 693], 'GotoTag', 'x_nl');
blk('simulink/Sinks/To Workspace',     'log x_nl',       [450 665 540 695], 'VariableName', 'x_nl',  'SaveFormat', 'Timeseries');

% colours and hidden names for the small routing blocks
for n = {'Actuators', 'Nonlinear boat', 'Sensor', 'Waves'}
    set_param([mdl '/' n{1}], 'BackgroundColor', 'lightBlue');
end
set_param([mdl '/Controller'], 'BackgroundColor', 'yellow');
set_param([mdl '/Linear model'], 'BackgroundColor', 'green');
for n = {'From y_meas', 'From delta ctrl', 'Goto delta', 'Goto x', 'Goto w', 'Goto y_meas', 'From delta', ...
         'From w', 'From x', 'From delta log', 'From x log', 'Sum cmd', 'Sum dev', 'Mux lin'}
    set_param([mdl '/' n{1}], 'ShowName', 'off');
end

%% ---------------- Controller: digital (50 Hz) Kalman filter + LQR ----------------
% An atomic subsystem with sample time p.Ts_ctrl: its inputs are sampled every
% Ts and its output is held in between (zero-order hold). The algorithm itself
% is in digital_controller.m, called from a MATLAB Function block.
s = [mdl '/Controller'];
set_param(s, 'TreatAsAtomicUnit', 'on', 'SystemSampleTime', 'p.Ts_ctrl');
add_block('simulink/Sources/In1', [s '/y_meas'], 'Position', [30 43 60 57]);
add_block('simulink/Sources/In1', [s '/delta'],  'Position', [30 93 60 107]);
add_block('simulink/Sources/In1', [s '/r'],      'Position', [30 143 60 157]);
add_block('simulink/User-Defined Functions/MATLAB Function', [s '/Digital controller'], ...
          'Position', [130 30 300 170]);
code = sprintf(['function u_fb = digital_controller_block(y, delta, r, dc, p, u0)\n' ...
                '%% One sample of the 50 Hz controller, see digital_controller.m\n' ...
                'u_fb = digital_controller(y, delta, r, dc, p, u0);\n']);
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', [s '/Digital controller']);
ch.Script = code;
for nm = {'dc', 'p', 'u0'}
    d = find(ch, '-isa', 'Stateflow.Data', 'Name', nm{1});
    d.Scope = 'Parameter';  d.Tunable = false;
end
add_block('simulink/Math Operations/Gain', [s '/ctrl_on'], 'Position', [350 85 400 115], 'Gain', 'ctrl_on');
add_block('simulink/Math Operations/Reshape', [s '/to vector'], 'Position', [430 85 470 115], ...
          'OutputDimensionality', '1-D array');      % 2x1 matrix -> 2-element vector
add_block('simulink/Sinks/Out1', [s '/u_fb'], 'Position', [510 93 540 107]);
add_line(s, 'y_meas/1', 'Digital controller/1', 'autorouting', 'smart');
add_line(s, 'delta/1', 'Digital controller/2', 'autorouting', 'smart');
add_line(s, 'r/1', 'Digital controller/3', 'autorouting', 'smart');
add_line(s, 'Digital controller/1', 'ctrl_on/1', 'autorouting', 'smart');
add_line(s, 'ctrl_on/1', 'to vector/1', 'autorouting', 'smart');
add_line(s, 'to vector/1', 'u_fb/1', 'autorouting', 'smart');

%% ---------------- Actuators ----------------
s = [mdl '/Actuators'];
add_block('simulink/Sources/In1',               [s '/u_cmd'],      'Position', [20 93 50 107]);
add_block('simulink/Discontinuities/Saturation',[s '/Position limit'], 'Position', [90 85 130 115], ...
          'UpperLimit', 'p.u_max', 'LowerLimit', 'p.u_min');
add_block('simulink/Math Operations/Sum',       [s '/Servo error'], 'Position', [170 85 200 115], 'Inputs', '+-', 'IconShape', 'rectangular');
add_block('simulink/Math Operations/Gain',      [s '/1 over tau'],  'Position', [230 85 280 115], 'Gain', '1/p.tau_act');
add_block('simulink/Discontinuities/Saturation',[s '/Rate limit'],  'Position', [310 85 350 115], ...
          'UpperLimit', 'p.rate_max', 'LowerLimit', '-p.rate_max');
add_block('simulink/Continuous/Integrator',     [s '/Foil angle'],  'Position', [380 85 420 115], 'InitialCondition', 'u0(:)');
add_block('simulink/Sources/Constant',          [s '/act_on'],      'Position', [380 150 420 170], 'Value', 'act_on');
add_block('simulink/Signal Routing/Switch',     [s '/Ideal or real'], 'Position', [480 135 520 185], ...
          'Criteria', 'u2 ~= 0');
add_block('simulink/Sinks/Out1',                [s '/delta'],       'Position', [560 153 590 167]);
add_line(s, 'u_cmd/1', 'Position limit/1');
add_line(s, 'Position limit/1', 'Servo error/1');
add_line(s, 'Servo error/1', '1 over tau/1');
add_line(s, '1 over tau/1', 'Rate limit/1');
add_line(s, 'Rate limit/1', 'Foil angle/1');
add_line(s, 'Foil angle/1', 'Servo error/2', 'autorouting', 'smart');
add_line(s, 'Foil angle/1', 'Ideal or real/1', 'autorouting', 'smart');
add_line(s, 'act_on/1', 'Ideal or real/2', 'autorouting', 'smart');
add_line(s, 'u_cmd/1', 'Ideal or real/3', 'autorouting', 'smart');
add_line(s, 'Ideal or real/1', 'delta/1');

%% ---------------- Nonlinear boat ----------------
s = [mdl '/Nonlinear boat'];
add_block('simulink/Sources/In1', [s '/delta'], 'Position', [20 78 50 92]);
add_block('simulink/Sources/In1', [s '/w'],     'Position', [20 118 50 132]);
add_block('simulink/User-Defined Functions/MATLAB Function', [s '/Boat dynamics'], 'Position', [120 30 260 140]);
add_block('simulink/Continuous/Integrator', [s '/States'], 'Position', [310 70 350 100], 'InitialCondition', 'zeros(4,1)');
add_block('simulink/Sinks/Out1', [s '/x'], 'Position', [410 78 440 92]);
code = sprintf(['function xdot = boat_dynamics(x, delta, w, p)\n' ...
                '%% Nonlinear heave/pitch dynamics, see hydrofoil_nl.m\n' ...
                '[xdot, ~] = hydrofoil_nl(x, delta, w, p);\n']);
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', [s '/Boat dynamics']);
ch.Script = code;
d = find(ch, '-isa', 'Stateflow.Data', 'Name', 'p');
d.Scope = 'Parameter';  d.Tunable = false;
add_line(s, 'Boat dynamics/1', 'States/1');
add_line(s, 'States/1', 'x/1');
add_line(s, 'States/1', 'Boat dynamics/1', 'autorouting', 'smart');
add_line(s, 'delta/1', 'Boat dynamics/2', 'autorouting', 'smart');
add_line(s, 'w/1', 'Boat dynamics/3', 'autorouting', 'smart');

%% ---------------- Waves ----------------
s = [mdl '/Waves'];
add_block('simulink/Sources/Sine Wave', [s '/w front'], 'Position', [30 30 70 60], ...
          'Amplitude', 'wv.aw', 'Frequency', 'wv.we', 'Phase', 'wv.kw*p.x(1)');
add_block('simulink/Sources/Sine Wave', [s '/w rear'],  'Position', [30 90 70 120], ...
          'Amplitude', 'wv.aw', 'Frequency', 'wv.we', 'Phase', 'wv.kw*p.x(2)');
add_block('simulink/Sources/Sine Wave', [s '/elevation at sensor'], 'Position', [30 170 70 200], ...
          'Amplitude', 'wv.zeta_a', 'Frequency', 'wv.we', 'Phase', 'wv.kw*p.ls + pi/2');
add_block('simulink/Signal Routing/Mux', [s '/Mux'], 'Position', [120 30 125 120], 'Inputs', '2');
add_block('simulink/Math Operations/Gain', [s '/wave_on w'],    'Position', [170 60 220 90],   'Gain', 'wave_on');
add_block('simulink/Math Operations/Gain', [s '/wave_on zeta'], 'Position', [170 170 220 200], 'Gain', 'wave_on');
add_block('simulink/Sinks/Out1', [s '/w'],      'Position', [270 68 300 82]);
add_block('simulink/Sinks/Out1', [s '/zeta_s'], 'Position', [270 178 300 192]);
add_line(s, 'w front/1', 'Mux/1');
add_line(s, 'w rear/1', 'Mux/2');
add_line(s, 'Mux/1', 'wave_on w/1');
add_line(s, 'wave_on w/1', 'w/1');
add_line(s, 'elevation at sensor/1', 'wave_on zeta/1');
add_line(s, 'wave_on zeta/1', 'zeta_s/1');

%% ---------------- Sensor ----------------
% y_meas = [bow height; pitch angle; pitch rate]
s = [mdl '/Sensor'];
add_block('simulink/Sources/In1', [s '/x'],      'Position', [20 73 50 87]);
add_block('simulink/Sources/In1', [s '/zeta_s'], 'Position', [20 173 50 187]);
add_block('simulink/Signal Routing/Demux', [s '/Demux'], 'Position', [90 30 95 130], 'Outputs', '4');
add_block('simulink/Sinks/Terminator', [s '/v unused'], 'Position', [130 57 150 77], 'ShowName', 'off');
add_block('simulink/Math Operations/Trigonometric Function', [s '/sin'], 'Position', [130 82 170 102], 'Operator', 'sin', 'ShowName', 'off');
add_block('simulink/Math Operations/Gain', [s '/ls'], 'Position', [200 77 240 107], 'Gain', 'p.ls');
add_block('simulink/Sources/Random Number', [s '/Height noise'], 'Position', [130 220 170 250], ...
          'Mean', '0', 'Variance', 'p.sigma_y^2', 'Seed', '1', 'SampleTime', 'p.Ts_sens');
add_block('simulink/Math Operations/Gain', [s '/noise_on h'], 'Position', [200 220 250 250], 'Gain', 'noise_on');
add_block('simulink/Math Operations/Sum', [s '/Ultrasonic height'], 'Position', [310 30 340 200], 'Inputs', '++-+', 'IconShape', 'rectangular');
add_block('simulink/Sources/Random Number', [s '/Angle noise'], 'Position', [130 300 170 330], ...
          'Mean', '0', 'Variance', 'p.sigma_th^2', 'Seed', '3', 'SampleTime', 'p.Ts_sens');
add_block('simulink/Math Operations/Gain', [s '/noise_on th'], 'Position', [200 300 250 330], 'Gain', 'noise_on');
add_block('simulink/Math Operations/Sum', [s '/AHRS pitch angle'], 'Position', [310 270 340 330], 'Inputs', '++', 'IconShape', 'rectangular');
add_block('simulink/Sources/Random Number', [s '/Gyro noise'], 'Position', [130 400 170 430], ...
          'Mean', '0', 'Variance', 'p.sigma_q^2', 'Seed', '2', 'SampleTime', 'p.Ts_sens');
add_block('simulink/Math Operations/Gain', [s '/noise_on q'], 'Position', [200 400 250 430], 'Gain', 'noise_on');
add_block('simulink/Math Operations/Sum', [s '/Pitch-rate gyro'], 'Position', [310 370 340 430], 'Inputs', '++', 'IconShape', 'rectangular');
add_block('simulink/Signal Routing/Mux', [s '/Mux'], 'Position', [410 100 415 400], 'Inputs', '3');
add_block('simulink/Sinks/Out1', [s '/y_meas'], 'Position', [460 243 490 257]);
r2 = {'autorouting', 'smart'};
add_line(s, 'x/1', 'Demux/1');
add_line(s, 'Demux/1', 'Ultrasonic height/1', r2{:});
add_line(s, 'Demux/2', 'v unused/1');
add_line(s, 'Demux/3', 'sin/1');
add_line(s, 'sin/1', 'ls/1');
add_line(s, 'ls/1', 'Ultrasonic height/2', r2{:});
add_line(s, 'zeta_s/1', 'Ultrasonic height/3', r2{:});
add_line(s, 'Height noise/1', 'noise_on h/1');
add_line(s, 'noise_on h/1', 'Ultrasonic height/4', r2{:});
add_line(s, 'Demux/3', 'AHRS pitch angle/1', r2{:});
add_line(s, 'Angle noise/1', 'noise_on th/1');
add_line(s, 'noise_on th/1', 'AHRS pitch angle/2', r2{:});
add_line(s, 'Demux/4', 'Pitch-rate gyro/1', r2{:});
add_line(s, 'Gyro noise/1', 'noise_on q/1');
add_line(s, 'noise_on q/1', 'Pitch-rate gyro/2', r2{:});
add_line(s, 'Ultrasonic height/1', 'Mux/1', r2{:});
add_line(s, 'AHRS pitch angle/1', 'Mux/2', r2{:});
add_line(s, 'Pitch-rate gyro/1', 'Mux/3', r2{:});
add_line(s, 'Mux/1', 'y_meas/1');

%% ---------------- top level wiring ----------------
r = {'autorouting', 'smart'};
% row 1
add_line(mdl, 'Trim u0/1', 'Sum cmd/1', r{:});
add_line(mdl, 'Foil command/1', 'Sum cmd/2', r{:});
add_line(mdl, 'From y_meas/1', 'Controller/1', r{:});
add_line(mdl, 'From delta ctrl/1', 'Encoder/1', r{:});
add_line(mdl, 'Encoder/1', 'Controller/2', r{:});
add_line(mdl, 'Height reference/1', 'Controller/3', r{:});
named(add_line(mdl, 'Controller/1', 'Sum cmd/3', r{:}), 'u_fb');
named(add_line(mdl, 'Sum cmd/1', 'Actuators/1', r{:}), 'u_cmd');
named(add_line(mdl, 'Actuators/1', 'Nonlinear boat/1', r{:}), 'delta');
add_line(mdl, 'Actuators/1', 'Goto delta/1', r{:});
named(add_line(mdl, 'Waves/1', 'Goto w/1', r{:}), 'w');
add_line(mdl, 'Waves/1', 'Nonlinear boat/2', r{:});
named(add_line(mdl, 'Waves/2', 'Sensor/2', r{:}), 'zeta_s');
named(add_line(mdl, 'Nonlinear boat/1', 'Sensor/1', r{:}), 'x');
add_line(mdl, 'Nonlinear boat/1', 'Goto x/1', r{:});
named(add_line(mdl, 'Sensor/1', 'log y_meas/1', r{:}), 'y_meas');
add_line(mdl, 'Sensor/1', 'Goto y_meas/1', r{:});
% row 2
add_line(mdl, 'From delta/1', 'Sum dev/1', r{:});
add_line(mdl, 'Trim u0 lin/1', 'Sum dev/2', r{:});
add_line(mdl, 'Sum dev/1', 'Mux lin/1', r{:});
add_line(mdl, 'From w/1', 'Mux lin/2', r{:});
add_line(mdl, 'Mux lin/1', 'Linear model/1', r{:});
named(add_line(mdl, 'Linear model/1', 'Scope states/2', r{:}), 'x_lin');
add_line(mdl, 'Linear model/1', 'log x_lin/1', r{:});
add_line(mdl, 'From x/1', 'Scope states/1', r{:});
% row 3
add_line(mdl, 'From delta log/1', 'log delta/1', r{:});
add_line(mdl, 'From x log/1', 'log x_nl/1', r{:});

% titled areas
area('1. Closed loop: controller -> actuators -> nonlinear boat -> sensor', [10 70 1160 395]);
area('2. Linear model for comparison (same foil angles, same waves)', [10 440 800 615]);
area('3. Logging to the workspace', [10 640 800 715]);

save_system(mdl, fullfile(here, [mdl '.slx']));
fprintf('Built %s\n', fullfile(here, [mdl '.slx']));

    function blk(lib, name, pos, varargin)
        add_block(lib, [mdl '/' name], 'Position', pos, varargin{:});
    end
    function named(h, name)
        set_param(h, 'Name', name);
    end
    function area(txt, pos)
        add_block('built-in/Area', [mdl '/' txt], 'Position', pos, 'FontSize', 12);
    end
    function sub(name, pos)
        add_block('built-in/Subsystem', [mdl '/' name], 'Position', pos);
    end
end
