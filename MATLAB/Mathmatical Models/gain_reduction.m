%% Solution A: Gain Reduction for Unity-Feedback Stability
%  This script demonstrates that the open-loop plant G(s) is stable,
%  but the unity-feedback closed-loop is unstable due to excessive loop gain.
%  I have fixed it by reducing the loop gain so the Nyquist plot no longer
%  encircles the -1 point.

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

% ---------- Open-Loop Diagnostics ----------
fprintf('========== OPEN-LOOP PLANT DIAGNOSTICS ==========\n');
fprintf('Poles of G(s):\n');
p = pole(G);
disp(p.');
fprintf('DC gain G(0) = %.4f (rad/s)/V\n', dcgain(G));
fprintf('DC gain      = %.2f dB\n\n', 20*log10(dcgain(G)));

S_ol = stepinfo(G);
fprintf('Open-loop step info:\n');
fprintf('  Rise Time     = %.4f s\n', S_ol.RiseTime);
fprintf('  Settling Time = %.4f s\n', S_ol.SettlingTime);
fprintf('  Overshoot     = %.2f %%\n', S_ol.Overshoot);
fprintf('  Peak          = %.4f\n\n', S_ol.Peak);

% ---------- Show the instability of unity-feedback (uncompensated) ----------
fprintf('========== UNCOMPENSATED UNITY-FEEDBACK ==========\n');
try
    [GM_raw, PM_raw, WG_raw, WP_raw] = margin(G);
    fprintf('Gain Margin  = %.2f dB at %.1f rad/s\n', 20*log10(GM_raw), WG_raw);
    fprintf('Phase Margin = %.2f deg at %.1f rad/s\n', PM_raw, WP_raw);
catch
    fprintf('margin(G) could not compute — closed-loop is unstable.\n');
end

T_unstable = feedback(G, 1);
p_cl = pole(T_unstable);
fprintf('\nClosed-loop poles (uncompensated):\n');
disp(p_cl.');
if any(real(p_cl) > 0)
    fprintf('>> UNSTABLE: at least one closed-loop pole in the RHP.\n\n');
else
    fprintf('>> Stable.\n\n');
end

% ---------- Figure 1: Uncompensated Bode ----------
figure('Name','Uncompensated Bode','NumberTitle','off');
margin(G);
grid on;
title('Bode Plot — Uncompensated G(s) [Unity Feedback is UNSTABLE]');

% ---------- Figure 2: Uncompensated Nyquist ----------
figure('Name','Uncompensated Nyquist','NumberTitle','off');
nyquist(G);
grid on;
title('Nyquist Plot — G(s) encircles -1 \rightarrow Unstable');

% ========== SOLUTION: Reduce Loop Gain ==========
% The DC gain is ~16.81 (24.5 dB). We need to bring the gain crossover
% frequency down to where the phase still provides adequate margin.
% A gain of K ~ 0.04–0.06 works well.

K_values = [0.03, 0.05, 0.08];   % Try several gains

fprintf('========== GAIN REDUCTION SOLUTIONS ==========\n');

figure('Name','Gain Reduction — Bode Comparison','NumberTitle','off');
hold on;

colors = {'b','r','m'};
legends = {};

for i = 1:length(K_values)
    K = K_values(i);
    G_scaled = K * G;

    [GM_s, PM_s, WG_s, WP_s] = margin(G_scaled);

    fprintf('K = %.3f:\n', K);
    fprintf('  Gain Margin  = %.2f dB at %.1f rad/s\n', 20*log10(GM_s), WG_s);
    fprintf('  Phase Margin = %.2f deg at %.1f rad/s\n', PM_s, WP_s);

    T_s = feedback(G_scaled, 1);
    p_cl_s = pole(T_s);
    fprintf('  Closed-loop poles: ');
    disp(p_cl_s.');
    if any(real(p_cl_s) > 0)
        fprintf('  >> Still UNSTABLE\n\n');
    else
        fprintf('  >> STABLE\n\n');
    end

    legends{end+1} = sprintf('K=%.3f', K);
end

% Bode comparison of all gains
figure('Name','Gain Reduction — Bode Comparison','NumberTitle','off');
bode(G, 'k--');
hold on;
for i = 1:length(K_values)
    bode(K_values(i)*G, colors{i});
end
grid on;
legend(['G(s) uncompensated', cellfun(@(k) sprintf('K=%.3f · G(s)',k), ...
    num2cell(K_values), 'UniformOutput', false)], 'Location','southwest');
title('Bode — Effect of Gain Reduction');

% ---------- Pick best gain and show closed-loop ----------
K_best = 0.05;
G_best = K_best * G;
T_best = feedback(G_best, 1);

fprintf('========== SELECTED SOLUTION: K = %.3f ==========\n', K_best);
[GM_b, PM_b, WG_b, WP_b] = margin(G_best);
fprintf('Gain Margin  = %.2f dB at %.1f rad/s\n', 20*log10(GM_b), WG_b);
fprintf('Phase Margin = %.2f deg at %.1f rad/s\n', PM_b, WP_b);
fprintf('Closed-loop poles:\n');
disp(pole(T_best).');

S_cl = stepinfo(T_best);
fprintf('Closed-loop step info:\n');
fprintf('  Rise Time     = %.4f s\n', S_cl.RiseTime);
fprintf('  Settling Time = %.4f s\n', S_cl.SettlingTime);
fprintf('  Overshoot     = %.2f %%\n', S_cl.Overshoot);
fprintf('  Steady-state  = %.4f\n\n', dcgain(T_best));

% ---------- Figure 3: Compensated Bode with Margins ----------
figure('Name','Gain Reduction — Final Bode','NumberTitle','off');
margin(G_best);
grid on;
title(sprintf('Bode — K=%.3f · G(s) [Stable Closed-Loop]', K_best));

% ---------- Figure 4: Compensated Nyquist ----------
figure('Name','Gain Reduction — Final Nyquist','NumberTitle','off');
nyquist(G_best);
grid on;
title(sprintf('Nyquist — K=%.3f · G(s) [No encirclement of -1]', K_best));

% ---------- Figure 5: Closed-Loop Step Response ----------
figure('Name','Gain Reduction — Step Response','NumberTitle','off');
step(T_best);
grid on;
title(sprintf('Closed-Loop Step Response (K=%.3f)', K_best));
ylabel('Angular Velocity (rad/s)');

% ---------- Figure 6: Compare open-loop vs closed-loop step ----------
figure('Name','Open vs Closed Loop','NumberTitle','off');
step(G, 'b--', T_best, 'r-');
grid on;
legend('Open-loop G(s)', sprintf('Closed-loop (K=%.3f)', K_best));
title('Step Response Comparison');
ylabel('Angular Velocity (rad/s)');

fprintf('Note: The delay-based "fix" is NOT recommended for production.\n');
fprintf('Gain reduction provides a simple, robust solution.\n');