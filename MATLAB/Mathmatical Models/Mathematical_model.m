clc; clear; close all
% ---------- Parameters ----------
J  = 1.2e-5;          % kg*m^2
b  = 0;               % N*m*s/rad (можете поставить 1e-4)
R  = 1.3275;          % Ohm
L  = 0.00075;         % H
Kt = 0.065;           % N*m/A
Ke = 0.05949;         % V*s/rad (6.23 V/kRPM -> SI)

s = tf('s');

% ---------- Transfer Function (voltage -> speed) ----------
G = Kt / (J*L*s^2 + (b*L + J*R)*s + (b*R + Kt*Ke));

% Numeric diagnostics
disp('Poles of G:');  disp(pole(G).')
disp(['DC gain G(0)= ', num2str(dcgain(G)),' (rad/s)/V'])
S = stepinfo(G);      disp(S)

% ---------- State-Space (x=[ia; omega], y=omega) ----------
A = [-R/L    -Ke/L;
      Kt/J   -b/J ];
B = [1/L; 0];
C = [0 1];           % measure speed
D = 0;
sys = ss(A,B,C,D);

 % ---------- State-Space (x=[ia; omega], y=omega) ----------
A = [-R/L    -Ke/L;
      Kt/J   -b/J ];
B = [1/L; 0];
C = [0 1];           % measure speed
D = 0;
sys = ss(A,B,C,D);

% Controllability / Observability
Co = ctrb(A,B);  rankCo = rank(Co);
Ob = obsv(A,C);  rankOb = rank(Ob);
disp(['rank(Co)=',num2str(rankCo),'  rank(Ob)=',num2str(rankOb)])
 
figure(2); bode(G); grid on; title('Bode Plot')
margin(G)


figure(3); nyquist(G); grid on; title('Nyquist Plot')


% Numeric diagnostics
disp('Poles of G:');  disp(pole(G).')
disp(['DC gain G(0)= ', num2str(dcgain(G)),' (rad/s)/V'])
S = stepinfo(G);      disp(S)

% (Optional) driver delay ~1 ms -> finite margins
Td = 1e-3; [numd,dend] = pade(Td,1); Gd = G*tf(numd,dend);
figure(4); bode(G,Gd); grid on; legend('ideal','with 1 ms delay')
[GM,PM,WG,WP] = margin(Gd);
disp(['Margins with delay: GM=',num2str(GM),' PM=',num2str(PM),...
      ' at wg=',num2str(WG),' wp=',num2str(WP)])
margin(Gd)