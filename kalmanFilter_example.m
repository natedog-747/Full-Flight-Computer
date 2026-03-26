% kalmanFilter_example.m
% Example initialization and usage of the geodetic ESKF (kalmanFilter.m)
%
% State: [lat(rad); lon(rad); alt(m); vN; vE; vD; qw; qx; qy; qz; abx; aby; abz; wbx; wby; wbz; gN; gE; gD]
% Error: 18x1 [dlat; dlon; dalt; dvNED(3); dtheta(3); dab(3); dwb(3); dg(3)]

%% ---- Initial conditions ----
lat0_deg = 38.9717;   % degrees
lon0_deg = -76.9223;  % degrees
alt0_m   = 100.0;     % meters above ellipsoid

lat0 = deg2rad(lat0_deg);
lon0 = deg2rad(lon0_deg);

vel0  = [0; 0; 0];        % NED velocity (m/s)
q0    = [1; 0; 0; 0];     % identity quaternion (body = NED at start)
ab0   = zeros(3,1);       % accel bias (m/s^2)
wb0   = zeros(3,1);       % gyro bias (rad/s)
g0    = [0; 0; 9.81];     % gravity NED (m/s^2), gD positive = down

state = [lat0; lon0; alt0_m; vel0; q0; ab0; wb0; g0];  % 19x1

%% ---- Initial covariance ----
% Diagonal P0 for each error state component
sig_pos   = [1e-4; 1e-4; 1.0];   % [rad, rad, m]  (~10 m level at equator for lat/lon)
sig_vel   = [0.1; 0.1; 0.1];     % m/s
sig_theta = [0.01; 0.01; 0.01];  % rad (~0.6 deg)
sig_ab    = [0.05; 0.05; 0.05];  % m/s^2
sig_wb    = [0.001; 0.001; 0.001]; % rad/s
sig_g     = [0.1; 0.1; 0.1];     % m/s^2

P = diag([sig_pos; sig_vel; sig_theta; sig_ab; sig_wb; sig_g].^2);  % 18x18

%% ---- Process noise Q (12x12) ----
% Noise inputs: [accel_noise(3); angle_noise(3); ab_walk(3); wb_walk(3)]
sig_na   = 0.1  * ones(3,1);   % accel noise (m/s^2 / sqrt(Hz))
sig_nth  = 0.01 * ones(3,1);   % angle noise (rad / sqrt(Hz))
sig_nab  = 1e-4 * ones(3,1);   % accel bias random walk
sig_nwb  = 1e-5 * ones(3,1);   % gyro bias random walk

Q = diag([sig_na; sig_nth; sig_nab; sig_nwb].^2);  % 12x12

%% ---- GPS measurement noise V (6x6) ----
% Measurement: [lat(rad); lon(rad); alt(m); vN(m/s); vE(m/s); vD(m/s)]
% Convert GPS position accuracy from meters to radians:
%   dlat = dpos_N / Rn,  dlon = dpos_E / Re
%   At mid-latitudes Rn ~ Re ~ 6.37e6 m
GPS_pos_sigma_m   = 3.0;   % horizontal GPS accuracy (m)
GPS_alt_sigma_m   = 5.0;   % vertical GPS accuracy (m)
GPS_vel_sigma_mps = 0.1;   % GPS velocity accuracy (m/s)

Rearth = 6.37e6;  % approximate, good enough for V initialization
sig_gps_lat = GPS_pos_sigma_m / Rearth;
sig_gps_lon = GPS_pos_sigma_m / Rearth;
sig_gps_alt = GPS_alt_sigma_m;
sig_gps_vel = GPS_vel_sigma_mps * ones(3,1);

V = diag([sig_gps_lat; sig_gps_lon; sig_gps_alt; sig_gps_vel].^2);  % 6x6

%% ---- Measurement matrix Hx (6x19) ----
% GPS directly observes [lat; lon; alt; vN; vE; vD] = state(1:6)
Hx = [eye(6), zeros(6, 13)];  % 6x19

%% ---- Simulation loop ----
dt      = 0.01;   % IMU rate: 100 Hz
dt_gps  = 0.2;    % GPS rate:   5 Hz
t_end   = 10.0;   % seconds
N       = round(t_end / dt);

prevGPSPos = state(1:3);
prevGPSVel = state(4:6);

% Storage
pos_hist = zeros(3, N);
vel_hist = zeros(3, N);
q_hist   = zeros(4, N);

% Simulated constant IMU (hovering, no motion)
am_true = [0; 0; -9.81];   % body frame accel (gravity up = -z body when level)
wm_true = [0; 0; 0];       % no rotation

rng(42);
accel_noise_std = sig_na(1);
gyro_noise_std  = sig_nth(1);

for k = 1:N
    t = k * dt;

    % Simulated IMU measurement
    am = am_true + accel_noise_std * randn(3,1);
    wm = wm_true + gyro_noise_std  * randn(3,1);

    % GPS update every dt_gps seconds
    if mod(t, dt_gps) < dt   % rising edge
        % Simulated GPS (true position + noise)
        gps_pos_noise = [GPS_pos_sigma_m/Rearth * randn(2,1); GPS_alt_sigma_m * randn];
        gps_vel_noise = GPS_vel_sigma_mps * randn(3,1);
        GPSPos = state(1:3) + gps_pos_noise;
        GPSVel = state(4:6) + gps_vel_noise;
    else
        GPSPos = prevGPSPos;
        GPSVel = prevGPSVel;
    end

    [state, P, prevGPSPos, prevGPSVel, pos, vel, q, ab, wb, g, wc] = ...
        kalmanFilter(state, am, wm, P, dt, Q, V, Hx, GPSPos, GPSVel, prevGPSPos, prevGPSVel);

    pos_hist(:,k) = pos;
    vel_hist(:,k) = vel;
    q_hist(:,k)   = q;
end

%% ---- Plot results ----
t_vec = (1:N)*dt;

figure(1); clf;
subplot(3,1,1);
plot(t_vec, rad2deg(pos_hist(1,:))); ylabel('Lat (deg)'); grid on; title('Position (LLA)');
subplot(3,1,2);
plot(t_vec, rad2deg(pos_hist(2,:))); ylabel('Lon (deg)'); grid on;
subplot(3,1,3);
plot(t_vec, pos_hist(3,:)); ylabel('Alt (m)'); xlabel('Time (s)'); grid on;

figure(2); clf;
subplot(3,1,1); plot(t_vec, vel_hist(1,:)); ylabel('vN (m/s)'); grid on; title('NED Velocity');
subplot(3,1,2); plot(t_vec, vel_hist(2,:)); ylabel('vE (m/s)'); grid on;
subplot(3,1,3); plot(t_vec, vel_hist(3,:)); ylabel('vD (m/s)'); xlabel('Time (s)'); grid on;

fprintf('Final position: lat=%.6f deg, lon=%.6f deg, alt=%.2f m\n', ...
    rad2deg(state(1)), rad2deg(state(2)), state(3));
fprintf('Final velocity NED: [%.3f, %.3f, %.3f] m/s\n', state(4), state(5), state(6));
