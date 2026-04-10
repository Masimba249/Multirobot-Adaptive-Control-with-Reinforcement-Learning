%% Solution B: Lead Compensator & PID Controller Design
%  This script designs two proper compensators for the DC motor plant:
%    Part 1 — Lead compensator (manual design) with DC precompensator
%    Part 2 — PID controller (auto-tuned via pidtune)
%  Both guarantee adequate gain/phase margins AND unity DC gain.

clc; clear; close all;

% ========== Motor Parameters ==========
J  = 1.2e-5;          % Rotor inertia [kg*m^2]
b  = 0;               % Viscous friction [N*m*s/rad]
R  = 1.3275;          % Armature resistance [Ohm]
L  = 0.00075;         % Armature inductance [H]
Kt = 0.065;           % Torque constant [N*m/A]
Ke = 0.05949;         % Back-EMF constant [V*s/rad]

s = tf('s');

% ========== Open-Loop Transfer Function (voltage -> speed) ==========
G = Kt / (J*L*s^2 + (b*L + J*R)*s + (b*R + Kt*Ke));

fprintf('========== PLANT DIAGNOSTICS ==========\n');
fprintf('Poles of G(s): ');  disp(pole(G).');
fprintf('DC gain = %.4f (%.2f dB)\n\n', dcgain(G), 20*log10(dcgain(G)));

% Show instability of raw unity feedback
T_raw = feedback(G, 1);
p_raw = pole(T_raw);
fprintf('Uncompensated closed-loop poles:\n');  disp(p_raw.');
if any(real(p_raw) > 0)
    fprintf('>> UNSTABLE closed-loop.\n\n');
else
    fprintf('>> Stable closed-loop.\n\n');
end


%                    PART 1: LEAD COMPENSATOR

fprintf('============================================================\n');
fprintf('          PART 1: LEAD COMPENSATOR DESIGN\n');
fprintf('============================================================\n\n');

% Design target:
%   - Crossover frequency wc ~ 1500 rad/s (between the two plant poles
%     at 290 and 1480 rad/s — this is where lead is genuinely needed)
%   - Phase margin > 45 degrees
%   - Gain margin > 6 dB
%   - Unity DC gain via precompensator

wc_target = 1500;  % desired gain-crossover frequency [rad/s]

% Step 1: Evaluate plant magnitude and phase at wc_target
[mag_wc, ph_wc] = bode(G, wc_target);
mag_wc = squeeze(mag_wc);
ph_wc  = squeeze(ph_wc);

% MATLAB may return phase wrapped for all-pole systems.
% For a 2nd-order system with two real negative poles and positive DC gain,
% the true phase runs from 0 to -180 degrees.
% Force-unwrap: if phase is positive, subtract 360.
if ph_wc > 0
    ph_wc = ph_wc - 360;
end

fprintf('At wc = %d rad/s:\n', wc_target);
fprintf('  |G(jwc)| = %.4f (%.2f dB)\n', mag_wc, 20*log10(mag_wc));
fprintf('  arg(G(jwc)) = %.2f deg (corrected)\n\n', ph_wc);

% Step 2: Determine how much phase lead is needed
% At gain crossover: phase(C) + phase(G) = -180 + PM_desired
% So: phase(C) = -180 + PM_desired - phase(G)
PM_desired = 50;        % desired phase margin [deg]
phi_lead = -180 + PM_desired - ph_wc + 5;  % +5 deg safety margin

% Clamp to practical bounds
phi_lead = max(phi_lead, 10);    % at least 10 deg of lead
phi_lead = min(phi_lead, 70);    % practical limit for single-stage lead
fprintf('Phase lead needed from compensator: %.1f deg\n', phi_lead);

% Step 3: Compute lead compensator parameters
%   C_lead(s) = K * (T*s + 1) / (alpha*T*s + 1),  with alpha < 1
%   Maximum phase lead at w_m = 1/(T*sqrt(alpha))
%   sin(phi_max) = (1 - alpha) / (1 + alpha)
%   => alpha = (1 - sin(phi)) / (1 + sin(phi))

alpha = (1 - sind(phi_lead)) / (1 + sind(phi_lead));
T_lead = 1 / (wc_target * sqrt(alpha));

fprintf('alpha = %.6f  (must be < 1 for lead)\n', alpha);
fprintf('T     = %.6f s\n', T_lead);
fprintf('Zero  at s = -%.1f rad/s\n', 1/T_lead);
fprintf('Pole  at s = -%.1f rad/s\n\n', 1/(alpha*T_lead));

% Step 4: Build the lead compensator (without gain)
C_lead_noK = (T_lead*s + 1) / (alpha*T_lead*s + 1);

% Compute gain K so that |K * C_lead * G| = 1 at wc_target
[mag_CG, ~] = bode(C_lead_noK * G, wc_target);
mag_CG = squeeze(mag_CG);
K_lead = 1 / mag_CG;
fprintf('Gain K = %.6f\n', K_lead);

% Step 5: Assemble full compensator
C_lead = K_lead * (T_lead*s + 1) / (alpha*T_lead*s + 1);
L_lead = C_lead * G;

% Step 6: Add DC precompensator for unity closed-loop DC gain
% T(0) = C(0)*G(0) / (1 + C(0)*G(0))
% To make T(0) = 1, we scale the reference by 1/T(0)
T_lead_cl_raw = feedback(L_lead, 1);
dc_cl_raw = dcgain(T_lead_cl_raw);
K_pre = 1 / dc_cl_raw;
fprintf('DC precompensator K_pre = %.6f\n\n', K_pre);

% Apply precompensator (scales the reference, not the loop)
T_lead_cl = K_pre * T_lead_cl_raw;

% Verify margins (margins are properties of the LOOP, not affected by K_pre)
[GM_lead, PM_lead, WG_lead, WP_lead] = margin(L_lead);
fprintf('LEAD COMPENSATOR RESULTS:\n');
if isinf(GM_lead)
    fprintf('  Gain Margin  = Inf dB (phase never reaches -180)\n');
else
    fprintf('  Gain Margin  = %.2f dB at %.1f rad/s\n', 20*log10(GM_lead), WG_lead);
end
fprintf('  Phase Margin = %.2f deg at %.1f rad/s\n', PM_lead, WP_lead);

p_lead = pole(T_lead_cl_raw);
fprintf('  Closed-loop poles:\n');  disp(p_lead.');
if all(real(p_lead) < 0)
    fprintf('  >> STABLE\n\n');
else
    fprintf('  >> UNSTABLE - adjust parameters\n\n');
end

S_lead = stepinfo(T_lead_cl);
fprintf('  Rise Time     = %.4f s\n', S_lead.RiseTime);
fprintf('  Settling Time = %.4f s\n', S_lead.SettlingTime);
fprintf('  Overshoot     = %.2f %%\n', S_lead.Overshoot);
fprintf('  DC gain (T)   = %.4f (with precompensator)\n\n', dcgain(T_lead_cl));

% Display compensator transfer function
fprintf('Lead Compensator C(s):\n');
disp(C_lead);

% ---------- Lead Compensator Figures ----------
figure('Name','Lead - Loop Bode','NumberTitle','off');
margin(L_lead);
grid on;
title('Lead Compensator - Open-Loop L(s) = C_{lead}(s) G(s)');

figure('Name','Lead - Nyquist','NumberTitle','off');
nyquist(L_lead);
grid on;
title('Lead Compensator - Nyquist of L(s)');

figure('Name','Lead - Closed-Loop Step','NumberTitle','off');
step(T_lead_cl);
grid on;
title('Lead Compensator - Closed-Loop Step Response (with precompensator)');
ylabel('Angular Velocity (rad/s)');

figure('Name','Lead - Bode Comparison','NumberTitle','off');
bode(G, 'b--', L_lead, 'r-');
grid on;
legend('Uncompensated G(s)', 'L(s) = C_{lead} G');
title('Bode Comparison: Plant vs Compensated Loop');

%                    PART 2: PID CONTROLLER

fprintf('============================================================\n');
fprintf('          PART 2: PID CONTROLLER (AUTO-TUNED)\n');
fprintf('============================================================\n\n');

% Use MATLAB pidtune to design a PID controller
% Target bandwidth ~ 500 rad/s
wc_pid = 500;

C_pid = pidtune(G, 'PID', wc_pid);

fprintf('PID Controller (parallel form):\n');
fprintf('  Kp = %.6f\n', C_pid.Kp);
fprintf('  Ki = %.6f\n', C_pid.Ki);
fprintf('  Kd = %.8f\n', C_pid.Kd);
fprintf('  Tf = %.8f (derivative filter)\n\n', C_pid.Tf);
disp(C_pid);

L_pid = C_pid * G;

% Verify margins
[GM_pid, PM_pid, WG_pid, WP_pid] = margin(L_pid);
fprintf('PID CONTROLLER RESULTS:\n');
if isinf(GM_pid)
    fprintf('  Gain Margin  = Inf dB\n');
else
    fprintf('  Gain Margin  = %.2f dB at %.1f rad/s\n', 20*log10(GM_pid), WG_pid);
end
fprintf('  Phase Margin = %.2f deg at %.1f rad/s\n', PM_pid, WP_pid);

T_pid_cl = feedback(L_pid, 1);
p_pid = pole(T_pid_cl);
fprintf('  Closed-loop poles:\n');  disp(p_pid.');
if all(real(p_pid) < 0)
    fprintf('  >> STABLE\n\n');
else
    fprintf('  >> UNSTABLE\n\n');
end

S_pid = stepinfo(T_pid_cl);
fprintf('  Rise Time     = %.4f s\n', S_pid.RiseTime);
fprintf('  Settling Time = %.4f s\n', S_pid.SettlingTime);
fprintf('  Overshoot     = %.2f %%\n', S_pid.Overshoot);
fprintf('  DC gain (T)   = %.4f\n\n', dcgain(T_pid_cl));

% ---------- PID Figures ----------
figure('Name','PID - Loop Bode','NumberTitle','off');
margin(L_pid);
grid on;
title('PID Controller - Open-Loop L(s) = C_{PID}(s) G(s)');

figure('Name','PID - Nyquist','NumberTitle','off');
nyquist(L_pid);
grid on;
title('PID Controller - Nyquist of L(s)');

figure('Name','PID - Closed-Loop Step','NumberTitle','off');
step(T_pid_cl);
grid on;
title('PID Controller - Closed-Loop Step Response');
ylabel('Angular Velocity (rad/s)');

%           PART 3: SIDE-BY-SIDE COMPARISON OF ALL SOLUTIONS
fprintf('============================================================\n');
fprintf('          COMPARISON: ALL SOLUTIONS\n');
fprintf('============================================================\n\n');

% Also compute the simple gain-reduction for comparison
K_simple = 0.05;
G_simple = K_simple * G;
T_simple_raw = feedback(G_simple, 1);
% Add precompensator for gain reduction too
K_pre_simple = 1 / dcgain(T_simple_raw);
T_simple = K_pre_simple * T_simple_raw;

fprintf('%-25s %10s %10s %10s %10s %10s\n', ...
    'Method', 'PM [deg]', 'GM [dB]', 'Rise [ms]', 'OS [%%]', 'DC gain');
fprintf('%s\n', repmat('-', 1, 80));

% Gain reduction
[gm1, pm1, ~, ~] = margin(G_simple);
si1 = stepinfo(T_simple);
fprintf('%-25s %10.1f %10s %10.2f %10.1f %10.4f\n', ...
    'Gain Reduction (K=0.05)', pm1, 'Inf', si1.RiseTime*1000, ...
    si1.Overshoot, dcgain(T_simple));

% Lead compensator
gm_lead_str = 'Inf';
if ~isinf(GM_lead)
    gm_lead_str = sprintf('%.1f', 20*log10(GM_lead));
end
fprintf('%-25s %10.1f %10s %10.2f %10.1f %10.4f\n', ...
    'Lead Compensator', PM_lead, gm_lead_str, S_lead.RiseTime*1000, ...
    S_lead.Overshoot, dcgain(T_lead_cl));

% PID
gm_pid_str = 'Inf';
if ~isinf(GM_pid)
    gm_pid_str = sprintf('%.1f', 20*log10(GM_pid));
end
fprintf('%-25s %10.1f %10s %10.2f %10.1f %10.4f\n', ...
    'PID Controller', PM_pid, gm_pid_str, S_pid.RiseTime*1000, ...
    S_pid.Overshoot, dcgain(T_pid_cl));

fprintf('\n');

% ---------- Final comparison figures ----------
figure('Name','All Solutions - Step Comparison','NumberTitle','off');
step(T_simple, 'b-', T_lead_cl, 'r-', T_pid_cl, 'm-');
grid on;
legend(sprintf('Gain K=%.2f (+ precomp)', K_simple), ...
    'Lead Compensator (+ precomp)', 'PID Controller', ...
    'Location','southeast');
title('Closed-Loop Step Response - All Solutions');
ylabel('Angular Velocity (rad/s)');
xlabel('Time (s)');

figure('Name','All Solutions - Bode Comparison','NumberTitle','off');
bode(G_simple, 'b-', L_lead, 'r-', L_pid, 'm-');
grid on;
legend(sprintf('K=%.2f * G', K_simple), 'C_{lead} * G', 'C_{PID} * G', ...
    'Location','southwest');
title('Open-Loop Bode - All Solutions');

fprintf('Recommendation: Use the Lead or PID compensator for best\n');
fprintf('performance. Avoid relying on transport delay for stability.\n');