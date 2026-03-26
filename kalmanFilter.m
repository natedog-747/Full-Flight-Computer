function [stateNext, PNext, prevGPSPos, prevGPSVel, pos, vel, q, ab, wb, g, omegaCorrected] = ...
    kalmanFilter(state, am, wm, P, dt, Q, V, Hx, GPSPos, GPSVel, prevGPSPos, prevGPSVel)
% Error-State Kalman Filter with geodetic (LLA) position states
% Based on Sola 2017 "Quaternion kinematics for the error-state KF" (arXiv 1711.02508)
%
% NOMINAL STATE (19x1):
%   state = [lat(rad); lon(rad); alt(m); vel_NED(3); q(4); ab(3); wb(3); g_NED(3)]
%   q = [qw; qx; qy; qz]  (Hamilton convention, body->NED)
%
% ERROR STATE (18x1):
%   dx = [dlat; dlon; dalt; dv_NED(3); dtheta(3); dab(3); dwb(3); dg_NED(3)]
%
% INPUTS:
%   state       - 19x1 nominal state
%   am          - 3x1 accelerometer measurement (body frame, m/s^2)
%   wm          - 3x1 gyroscope measurement (body frame, rad/s)
%   P           - 18x18 error state covariance
%   dt          - time step (s)
%   Q           - 12x12 process noise covariance
%                 [vel_noise(3); angle_noise(3); ab_noise(3); wb_noise(3)]
%   V           - 6x6 GPS measurement noise covariance
%                 [lat_noise; lon_noise; alt_noise; vN; vE; vD] units: [rad^2, rad^2, m^2, (m/s)^2 x3]
%   Hx          - 6x19 measurement matrix (typically [eye(6), zeros(6,13)])
%   GPSPos      - 3x1 GPS position [lat(rad); lon(rad); alt(m)]
%   GPSVel      - 3x1 GPS velocity NED [vN; vE; vD] (m/s)
%   prevGPSPos  - 3x1 previous GPS position (for update detection)
%   prevGPSVel  - 3x1 previous GPS velocity (for update detection)
%
% OUTPUTS:
%   stateNext       - 19x1 updated nominal state
%   PNext           - 18x18 updated covariance
%   prevGPSPos/Vel  - updated for next call
%   pos             - 3x1 LLA position [lat;lon;alt]
%   vel             - 3x1 NED velocity
%   q               - 4x1 quaternion [qw;qx;qy;qz]
%   ab              - 3x1 accel bias
%   wb              - 3x1 gyro bias
%   g               - 3x1 gravity NED
%   omegaCorrected  - 3x1 bias-corrected angular rate

%% Detect GPS update
correction = (GPSPos(1) ~= prevGPSPos(1)) || (GPSVel(1) ~= prevGPSVel(1));
prevGPSPos = GPSPos;
prevGPSVel = GPSVel;

%% Unpack nominal state
lat = state(1);  lon = state(2);  alt = state(3);
vel = state(4:6);
q   = state(7:10);   % [qw; qx; qy; qz]
ab  = state(11:13);
wb  = state(14:16);
g   = state(17:19);

%% WGS84 Earth radii at current latitude
a_e = 6378137.0;          % semi-major axis (m)
e2  = 0.00669437999014;   % first eccentricity squared
sinlat = sin(lat);
N_e = a_e / sqrt(1 - e2*sinlat^2);              % transverse (normal) radius
M_e = a_e*(1 - e2) / (1 - e2*sinlat^2)^1.5;    % meridional radius

Rn = M_e + alt;                 % effective N-S radius
Re = (N_e + alt) * cos(lat);   % effective E-W radius (arc length denominator)

% LLA coupling matrix: maps NED velocity to [lat_dot; lon_dot; alt_dot]
%   lat_dot = vN / Rn
%   lon_dot = vE / Re
%   alt_dot = -vD  (NED: vD positive down, alt positive up)
T_gps = diag([1/Rn, 1/Re, -1]);

%% Body-to-NED DCM from quaternion (Hamilton: q = [qw; qx; qy; qz])
qw = q(1); qx = q(2); qy = q(3); qz = q(4);
R = [qw^2+qx^2-qy^2-qz^2,   2*(qx*qy - qw*qz),   2*(qx*qz + qw*qy);
     2*(qx*qy + qw*qz),   qw^2-qx^2+qy^2-qz^2,   2*(qy*qz - qw*qx);
     2*(qx*qz - qw*qy),   2*(qy*qz + qw*qx),   qw^2-qx^2-qy^2+qz^2];

%% Bias-corrected IMU
ac = am - ab;   % corrected acceleration (body frame)
wc = wm - wb;   % corrected angular rate (body frame)
omegaCorrected = wc;

%% Skew-symmetric matrices
askew = [  0,    -ac(3),  ac(2);
          ac(3),   0,    -ac(1);
         -ac(2),  ac(1),   0   ];

wskew = [  0,    -wc(3),  wc(2);
          wc(3),   0,    -wc(1);
         -wc(2),  wc(1),   0   ];

%% Quaternion kinematics matrix  (qdot = 0.5 * Omega(wc) * q)
A_q = [  0,    -wc(1), -wc(2), -wc(3);
        wc(1),   0,     wc(3), -wc(2);
        wc(2), -wc(3),   0,    wc(1);
        wc(3),  wc(2), -wc(1),   0  ];
qdot = 0.5 * A_q * q;

%% ---- NOMINAL STATE PROPAGATION ----
lat_n = lat + vel(1)/Rn * dt;
lon_n = lon + vel(2)/Re * dt;
alt_n = alt - vel(3)    * dt;   % vD positive down

vel_n = vel + (R*ac + g) * dt;

q_n = q + qdot * dt;
q_n = q_n / norm(q_n);   % renormalize

ab_n = ab;
wb_n = wb;
g_n  = g;

stateNext = [lat_n; lon_n; alt_n; vel_n; q_n; ab_n; wb_n; g_n];

%% ---- ERROR-STATE COVARIANCE PROPAGATION ----
% Fx is 18x18 (error-state transition).
% Layout: [dpos(3), dvel(3), dtheta(3), dab(3), dwb(3), dg(3)]
%
%   Row 1-3  (dpos):    dpos_next  = dpos + T_gps*dvel*dt
%   Row 4-6  (dvel):    dvel_next  = dvel - R*[ac]x*dtheta*dt - R*dab*dt + dg*dt
%   Row 7-9  (dtheta):  dtheta_next = (I - [wc]x*dt)*dtheta - dwb*dt
%   Row 10-12 (dab):    dab_next   = dab
%   Row 13-15 (dwb):    dwb_next   = dwb
%   Row 16-18 (dg):     dg_next    = dg

Fx = [eye(3),      T_gps*dt,         zeros(3,3),       zeros(3,3),  zeros(3,3),  zeros(3,3);
      zeros(3,3),  eye(3),           -R*askew*dt,       -R*dt,       zeros(3,3),  eye(3)*dt;
      zeros(3,3),  zeros(3,3),  eye(3)-wskew*dt,   zeros(3,3),      -eye(3)*dt,  zeros(3,3);
      zeros(3,3),  zeros(3,3),       zeros(3,3),        eye(3),      zeros(3,3),  zeros(3,3);
      zeros(3,3),  zeros(3,3),       zeros(3,3),       zeros(3,3),   eye(3),      zeros(3,3);
      zeros(3,3),  zeros(3,3),       zeros(3,3),       zeros(3,3),  zeros(3,3),  eye(3)];

% Fi maps 12 process noise inputs [na; ntheta; nab; nwb] into 18-dim error state
% noise enters dvel, dtheta, dab, dwb rows (rows 4-15)
Fi = [zeros(3,12);
      eye(12);
      zeros(3,12)];

PNext = Fx * P * Fx' + Fi * Q * Fi';

%% ---- GPS CORRECTION (when new measurement available) ----
if correction
    % Work with propagated state
    state  = stateNext;
    P      = PNext;

    lat = state(1);  lon = state(2);  alt = state(3);
    vel = state(4:6);
    q   = state(7:10);
    ab  = state(11:13);  wb = state(14:16);  g = state(17:19);

    qw = q(1); qx = q(2); qy = q(3); qz = q(4);

    % Innovation: y - h(x)
    % h(x) = state(1:6)  since GPS directly observes lat,lon,alt,vN,vE,vD
    y    = [GPSPos; GPSVel];   % 6x1: [lat;lon;alt;vN;vE;vD]
    xhat = state(1:6);         % 6x1 predicted measurement

    % Xdx: 19x18 Jacobian of nominal state w.r.t. error state
    % Quaternion block: dq/d(dtheta) = 0.5 * Xi(q)  (4x3)
    Qdth = 0.5 * [-qx, -qy, -qz;
                   qw, -qz,  qy;
                   qz,  qw, -qx;
                  -qy,  qx,  qw];

    Xdx = [eye(6),      zeros(6,12);
           zeros(4,6),  Qdth,        zeros(4,9);
           zeros(9,9),  eye(9)];

    % H = Hx * Xdx  (6x18)
    H = Hx * Xdx;

    % Kalman gain
    S = H * P * H' + V;
    K = P * H' / S;       % 18x6

    % Error state correction
    dx = K * (y - xhat);  % 18x1

    dp   = dx(1:3);
    dv   = dx(4:6);
    dth  = dx(7:9);
    dab  = dx(10:12);
    dwb  = dx(13:15);
    dg   = dx(16:18);

    % Inject correction into nominal state
    lat_c = lat + dp(1);
    lon_c = lon + dp(2);
    alt_c = alt + dp(3);

    vel_c = vel + dv;

    % Quaternion correction via small rotation
    dq  = [1; 0.5*dth];   % approximate: [1; dtheta/2]
    q_c = quatmultiply(q', dq')';
    q_c = q_c / norm(q_c);

    ab_c = ab + dab;
    wb_c = wb + dwb;
    g_c  = g  + dg;

    stateNext = [lat_c; lon_c; alt_c; vel_c; q_c; ab_c; wb_c; g_c];

    % Joseph form for numerical stability
    PNext = (eye(18) - K*H) * P * (eye(18) - K*H)' + K*V*K';
end

%% Unpack outputs
pos           = stateNext(1:3);
vel           = stateNext(4:6);
q             = stateNext(7:10);
ab            = stateNext(11:13);
wb            = stateNext(14:16);
g             = stateNext(17:19);

end
