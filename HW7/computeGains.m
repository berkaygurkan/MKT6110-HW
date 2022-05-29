% select gains for roll loop
% get transfer function data for delta_a to phi
[num,den]=tfdata(T_phi_delta_a,'v');
a_phi2 = num(3);
a_phi1 = den(2);
% maximum possible aileron command
delta_a_max = 45*pi/180;
% Roll command when delta_a_max is achieved
phi_max = 15*pi/180;
% pick natural frequency to achieve delta_a_max for step of phi_max
zeta_roll = 0.707;
%wn_roll = sqrt(a_phi2*delta_a_max/phi_max);
wn_roll = 1.1*sqrt(a_phi2*delta_a_max*sqrt(1-zeta_roll^2)/phi_max);

% set control gains based on zeta and wn
P.roll_kp = wn_roll^2/a_phi2;
P.roll_kd = (2*zeta_roll*wn_roll - a_phi1)/a_phi2;
P.roll_ki = 0.1;


% select gains for heading loop
zeta_heading = 0.707;
wn_heading = wn_roll/10;
P.heading_kp = 2*zeta_heading*wn_heading*P.Va/P.gravity;
P.heading_ki = wn_heading^2*P.Va/P.gravity;
P.heading_kd = 0;

% select gains for sideslip hold
% get transfer function data for delta_r to v_r
[num,den]=tfdata(T_v_delta_r,'v');
a_beta2 = num(2);
a_beta1 = den(2);
% maximum possible aileron command
delta_r_max = 20*pi/180;
% Roll command when delta_a_max is achieved
vr_max = 3;
% pick natural frequency to achieve delta_a_max for step of phi_max
zeta_beta = 0.707;
P.beta_kp = delta_r_max/vr_max;
wn_beta = (a_beta2*P.beta_kp+a_beta1)/2/zeta_beta;
P.beta_ki = 0;%wn_beta^2/a_beta2;
P.beta_kd = 0;


% select gains for the pitch loop
% get transfer function delta_e to theta
[num,den]=tfdata(T_theta_delta_e,'v');
a_theta1 = den(2);
a_theta2 = den(3);
a_theta3 = num(3)

% maximum possible elevator command
delta_e_max = 45*pi/180;
% Pitch command when delta_e_max is achieved
theta_max = 15*pi/180;

% TODO
%% BG
%wn_pitch = 1.3974;
zeta_pitch =0.7156;
wn_pitch = sqrt((abs(a_theta3)*delta_e_max*sqrt(1-zeta_pitch^2))/(theta_max))
%wn_pitch = 15;


kp_theta = (wn_pitch^2 - a_theta2)/(a_theta3)
kd_theta = (2*zeta_pitch*wn_pitch-a_theta1)/(a_theta3)
K_theta_DC = (kp_theta*a_theta3)/(a_theta2+kp_theta*a_theta3);

%% TEST



% set control gains based on zeta and wn
P.pitch_kp = kp_theta;
P.pitch_kd = kd_theta;
P.pitch_ki = 0.0;
P.K_theta_DC = K_theta_DC;

% wn_pitch with saturation constraints
%

%%

% select gains for altitude loop
zeta_altitude = 0.707;
wn_altitude = wn_pitch/30;
P.altitude_kp = 2*zeta_altitude*wn_altitude/P.K_theta_DC/P.Va;
P.altitude_ki = wn_altitude^2/P.K_theta_DC/P.Va;
P.altitude_kd = 0;

% airspeed hold using pitch
[num,den]=tfdata(T_Va_theta,'v');
a_V1 = den(2);
zeta_airspeed_pitch = 0.707;
wn_airspeed_pitch = wn_pitch/10;
P.airspeed_pitch_kp = (a_V1-2*zeta_airspeed_pitch*wn_airspeed_pitch)/P.K_theta_DC/P.gravity;
P.airspeed_pitch_ki = -wn_airspeed_pitch^2/P.K_theta_DC/P.gravity;
P.airspeed_pitch_kd = 0;

% airspeed hold using throttle
[num,den]=tfdata(T_Va_delta_t,'v');
a_Vt1 = den(2);
a_Vt2 = num(2);
zeta_airspeed_throttle = 0.707;
wn_airspeed_throttle = 5;
P.airspeed_throttle_kp = (2*zeta_airspeed_throttle*wn_airspeed_throttle-a_Vt1)/a_Vt1;
P.airspeed_throttle_ki = wn_airspeed_throttle^2/a_Vt2;
P.airspeed_throttle_kd = 0;

% gains for slideslip
P.sideslip_kp = .1;
P.sideslip_kd = -.5;
P.sideslip_ki = 0;


