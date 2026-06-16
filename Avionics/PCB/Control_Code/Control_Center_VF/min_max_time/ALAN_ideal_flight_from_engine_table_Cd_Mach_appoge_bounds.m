%% ALAN IDEAL FLIGHT TRAJECTORY FROM ENGINE THRUST TABLE
% Goal:
%   1) Read engine data table.
%   2) Use column 1 = time [s], column 3 = thrust [N].
%   3) Simulate an ideal rocket flight.
%   4) Find the ideal engine shutdown time that reaches desired apogee.
%   5) Export a 2-column table for GUI upload:
%          column 1 = time_s
%          column 2 = altitude_m
%
% Units: SI / metric.

clear; clc; close all;

%% ===================== USER PARAMETERS / CHANGE HERE =====================

% Engine file. It must have at least 3 columns:
% col 1 = time [s], col 2 = pressure or unused data, col 3 = thrust [N]
engine_csv_file = "engine_data_curves.csv";

% Output file for the GUI perfect trajectory upload.
output_gui_csv_file = "ideal_flight_from_engine_1.csv";

% Rocket / launch conditions
initial_altitude_m      = 0.0;     % launch site altitude above sea level [m]
desired_apogee_m        = 3048.0;    % desired apogee ABOVE LAUNCH SITE [m], 10000 ft = 3048 m
margin_m                = 300.0;
appoge_low_m            = desired_apogee_m - margin_m;   % lower apogee target ABOVE LAUNCH SITE [m], 100 ft below target
appoge_high_m           = desired_apogee_m + margin_m;   % higher apogee target ABOVE LAUNCH SITE [m], 100 ft above target
initial_vertical_vel_mps = 0.0;      % usually 0 at launch [m/s]
initial_horizontal_vel_mps = 0.0;    % usually 0 [m/s]
launch_tilt_angle_rad   = 0.0524;       % 0 = perfectly vertical; positive gives horizontal component

% Mass model
initial_wet_mass_kg     = 40.1656044 + 4.32 + 8.527;      % rocket mass at ignition [kg]
propellant_mass_kg      = 4.32 / 2 + 8.527;    % propellant consumed over full engine curve [kg]
minimum_mass_kg         = initial_wet_mass_kg - propellant_mass_kg;

% Aerodynamics
cross_sectional_area_m2 = 0.0182;    % rocket reference area [m^2]

% Cd model:
%   true  = use Cd vs Mach table from RASAero / flight simulation CSV
%   false = use one constant Cd value
use_cd_mach_table       = true;
cd_mach_csv_file        = "Flight Test 85.52lb.CSV";  % must contain columns: Mach Number, CD
Cd_constant             = 0.47;      % backup only; used if use_cd_mach_table = false

% Physics / numerical settings
g_mps2                  = 9.80665;   % gravity [m/s^2]
dt_s                    = 0.005;     % integration step [s]
max_sim_time_s          = 1800.0;     % safety stop [s]
shutdown_tolerance_m    = 0.25;      % binary-search apogee tolerance [m]
max_binary_iterations   = 60;

% Thrust cleanup
thrust_start_threshold_N = 1.0;      % first thrust above this is treated as engine start
negative_thrust_to_zero  = true;     % removes small negative load-cell offset

% GUI output options
include_prelaunch_flat_line = true;
prelaunch_time_s            = 8.0;   % GUI graph starts at -8 seconds
output_altitude_above_launch = false; % true: altitude_m starts at 0; false: absolute altitude

% Plotting
make_plot = false;

%% ===================== READ AND PREPARE ENGINE DATA =====================

raw = readmatrix(engine_csv_file);

engine_time_raw_s = raw(:, 1);
engine_thrust_raw_N = raw(:, 3);

if negative_thrust_to_zero
    engine_thrust_raw_N(engine_thrust_raw_N < 0) = 0;
end

% Remove all rows before real thrust appears.
first_thrust_index = find(engine_thrust_raw_N > thrust_start_threshold_N, 1, "first");

if isempty(first_thrust_index)
    error("No engine thrust above threshold was found. Lower thrust_start_threshold_N or check the CSV.");
end

engine_time_s = engine_time_raw_s(first_thrust_index:end);
engine_thrust_N = engine_thrust_raw_N(first_thrust_index:end);

% Shift engine time so real ignition/thrust start is t = 0.
engine_time_s = engine_time_s - engine_time_s(1);

% Make sure time is monotonic and unique for interpolation.
[engine_time_s, unique_idx] = unique(engine_time_s, "stable");
engine_thrust_N = engine_thrust_N(unique_idx);

engine_end_time_s = engine_time_s(end);

% Total impulse from the cleaned engine curve.
total_impulse_Ns = trapz(engine_time_s, engine_thrust_N);

if total_impulse_Ns <= 0
    error("Total impulse is zero or negative. Check thrust data.");
end

fprintf("Engine start removed before raw time %.4f s\n", engine_time_raw_s(first_thrust_index));
fprintf("Cleaned engine duration = %.4f s\n", engine_end_time_s);
fprintf("Total impulse = %.2f N*s\n", total_impulse_Ns);

%% ===================== READ AND PREPARE CD VS MACH DATA =====================

if use_cd_mach_table
    cd_raw = readtable(cd_mach_csv_file, "VariableNamingRule", "preserve");

    cd_mach_raw = cd_raw.("Mach Number");
    cd_value_raw = cd_raw.("CD");

    valid_cd_rows = isfinite(cd_mach_raw) & isfinite(cd_value_raw) & ...
                    cd_mach_raw > 0 & cd_value_raw > 0.05;

    cd_mach_raw = cd_mach_raw(valid_cd_rows);
    cd_value_raw = cd_value_raw(valid_cd_rows);

    [cd_mach_table, sort_idx] = sort(cd_mach_raw);
    cd_value_table = cd_value_raw(sort_idx);

    % Remove duplicate Mach values. If several rows have the same Mach,
    % use the average Cd at that Mach.
    [cd_mach_table, ~, group_idx] = unique(cd_mach_table);
    cd_value_table = accumarray(group_idx, cd_value_table, [], @mean);

    if numel(cd_mach_table) < 2
        error("Cd table does not have enough valid Mach/CD points.");
    end

    fprintf("Cd table loaded: %d points, Mach %.4f to %.4f\n", ...
        numel(cd_mach_table), cd_mach_table(1), cd_mach_table(end));
else
    cd_mach_table = [];
    cd_value_table = [];
    fprintf("Using constant Cd = %.4f\n", Cd_constant);
end

%% ===================== FIND IDEAL SHUTDOWN TIMES =====================

ideal_shutdown_time_s = find_ideal_shutdown_time_for_apogee( ...
    desired_apogee_m, engine_time_s, engine_thrust_N, total_impulse_Ns, ...
    engine_end_time_s, initial_altitude_m, initial_vertical_vel_mps, initial_horizontal_vel_mps, ...
    launch_tilt_angle_rad, initial_wet_mass_kg, propellant_mass_kg, minimum_mass_kg, ...
    cross_sectional_area_m2, Cd_constant, use_cd_mach_table, cd_mach_table, cd_value_table, ...
    g_mps2, dt_s, max_sim_time_s, shutdown_tolerance_m, max_binary_iterations, ...
    "desired_apogee_m");

ideal_shutdown_time_appoge_low_s = find_ideal_shutdown_time_for_apogee( ...
    appoge_low_m, engine_time_s, engine_thrust_N, total_impulse_Ns, ...
    engine_end_time_s, initial_altitude_m, initial_vertical_vel_mps, initial_horizontal_vel_mps, ...
    launch_tilt_angle_rad, initial_wet_mass_kg, propellant_mass_kg, minimum_mass_kg, ...
    cross_sectional_area_m2, Cd_constant, use_cd_mach_table, cd_mach_table, cd_value_table, ...
    g_mps2, dt_s, max_sim_time_s, shutdown_tolerance_m, max_binary_iterations, ...
    "appoge_low_m");

ideal_shutdown_time_appoge_high_s = find_ideal_shutdown_time_for_apogee( ...
    appoge_high_m, engine_time_s, engine_thrust_N, total_impulse_Ns, ...
    engine_end_time_s, initial_altitude_m, initial_vertical_vel_mps, initial_horizontal_vel_mps, ...
    launch_tilt_angle_rad, initial_wet_mass_kg, propellant_mass_kg, minimum_mass_kg, ...
    cross_sectional_area_m2, Cd_constant, use_cd_mach_table, cd_mach_table, cd_value_table, ...
    g_mps2, dt_s, max_sim_time_s, shutdown_tolerance_m, max_binary_iterations, ...
    "appoge_high_m");

%% ===================== FINAL IDEAL SIMULATION =====================

[final_apogee_m, flight_log] = simulate_flight_to_apogee( ...
    ideal_shutdown_time_s, engine_time_s, engine_thrust_N, total_impulse_Ns, ...
    initial_altitude_m, initial_vertical_vel_mps, initial_horizontal_vel_mps, ...
    launch_tilt_angle_rad, initial_wet_mass_kg, propellant_mass_kg, minimum_mass_kg, ...
    cross_sectional_area_m2, Cd_constant, use_cd_mach_table, cd_mach_table, cd_value_table, g_mps2, dt_s, max_sim_time_s);

final_apogee_above_launch_m = final_apogee_m - initial_altitude_m;

fprintf("\n=========== IDEAL RESULT ===========\n");
fprintf("Desired apogee above launch = %.3f m = %.3f ft\n", desired_apogee_m, desired_apogee_m * 3.28084);
fprintf("Ideal shutdown time = %.4f s after thrust start\n", ideal_shutdown_time_s);
fprintf("Appoge low target = %.3f m = %.3f ft | shutdown time = %.4f s after thrust start\n", ...
    appoge_low_m, appoge_low_m * 3.28084, ideal_shutdown_time_appoge_low_s);
fprintf("Appoge high target = %.3f m = %.3f ft | shutdown time = %.4f s after thrust start\n", ...
    appoge_high_m, appoge_high_m * 3.28084, ideal_shutdown_time_appoge_high_s);
fprintf("Final apogee above launch = %.3f m = %.3f ft\n", final_apogee_above_launch_m, final_apogee_above_launch_m * 3.28084);
fprintf("Final apogee absolute = %.3f m\n", final_apogee_m);

%% ===================== BUILD GUI OUTPUT TABLE =====================

time_s = flight_log.time_s;

if output_altitude_above_launch
    altitude_m = flight_log.altitude_m - initial_altitude_m;
else
    altitude_m = flight_log.altitude_m;
end

% Add flat line from -8 to 0 seconds for GUI if desired.
if include_prelaunch_flat_line
    pre_t = (-prelaunch_time_s:dt_s:-dt_s).';
    if output_altitude_above_launch
        pre_alt = zeros(size(pre_t));
    else
        pre_alt = initial_altitude_m * ones(size(pre_t));
    end

    time_s = [pre_t; time_s];
    altitude_m = [pre_alt; altitude_m];
end

gui_table = table(time_s, altitude_m);
writetable(gui_table, output_gui_csv_file);

fprintf("GUI trajectory saved to: %s\n", output_gui_csv_file);

%% ===================== PLOT =====================

if make_plot
    figure;
    plot(time_s, altitude_m, "LineWidth", 1.5);
    hold on;
    yline(desired_apogee_m, "--", "Desired apogee");
    xline(ideal_shutdown_time_s, "--", "Engine shutdown");
    xlabel("Time from thrust start (s)");
    ylabel("Altitude above launch (m)");
    title("Ideal Flight Trajectory for GUI Upload");
    grid on;
end

%% ===================== LOCAL FUNCTIONS =====================

function ideal_shutdown_time_s = find_ideal_shutdown_time_for_apogee( ...
    target_apogee_above_launch_m, engine_time_s, engine_thrust_N, total_impulse_Ns, ...
    engine_end_time_s, initial_altitude_m, initial_vertical_vel_mps, initial_horizontal_vel_mps, ...
    launch_tilt_angle_rad, initial_wet_mass_kg, propellant_mass_kg, minimum_mass_kg, ...
    cross_sectional_area_m2, Cd_constant, use_cd_mach_table, cd_mach_table, cd_value_table, ...
    g_mps2, dt_s, max_sim_time_s, shutdown_tolerance_m, max_binary_iterations, target_name)

    target_apogee_absolute_m = initial_altitude_m + target_apogee_above_launch_m;

    % Apogee if engine is shut down immediately.
    [coast_only_apogee_m, ~] = simulate_flight_to_apogee( ...
        0.0, engine_time_s, engine_thrust_N, total_impulse_Ns, ...
        initial_altitude_m, initial_vertical_vel_mps, initial_horizontal_vel_mps, ...
        launch_tilt_angle_rad, initial_wet_mass_kg, propellant_mass_kg, minimum_mass_kg, ...
        cross_sectional_area_m2, Cd_constant, use_cd_mach_table, cd_mach_table, cd_value_table, g_mps2, dt_s, max_sim_time_s);

    % Apogee if full engine curve is used.
    [full_burn_apogee_m, ~] = simulate_flight_to_apogee( ...
        engine_end_time_s, engine_time_s, engine_thrust_N, total_impulse_Ns, ...
        initial_altitude_m, initial_vertical_vel_mps, initial_horizontal_vel_mps, ...
        launch_tilt_angle_rad, initial_wet_mass_kg, propellant_mass_kg, minimum_mass_kg, ...
        cross_sectional_area_m2, Cd_constant, use_cd_mach_table, cd_mach_table, cd_value_table, g_mps2, dt_s, max_sim_time_s);

    if coast_only_apogee_m >= target_apogee_absolute_m
        ideal_shutdown_time_s = 0.0;
        warning("%s target apogee is lower than/no higher than coast-only apogee. Shutdown time set to 0.", target_name);
    elseif full_burn_apogee_m < target_apogee_absolute_m
        ideal_shutdown_time_s = engine_end_time_s;
        warning("%s target apogee is unreachable with this engine table and rocket setup. Full burn used.", target_name);
    else
        lo = 0.0;
        hi = engine_end_time_s;

        for iter = 1:max_binary_iterations %#ok<NASGU>
            mid = 0.5 * (lo + hi);

            [apogee_mid_m, ~] = simulate_flight_to_apogee( ...
                mid, engine_time_s, engine_thrust_N, total_impulse_Ns, ...
                initial_altitude_m, initial_vertical_vel_mps, initial_horizontal_vel_mps, ...
                launch_tilt_angle_rad, initial_wet_mass_kg, propellant_mass_kg, minimum_mass_kg, ...
                cross_sectional_area_m2, Cd_constant, use_cd_mach_table, cd_mach_table, cd_value_table, g_mps2, dt_s, max_sim_time_s);

            if abs(apogee_mid_m - target_apogee_absolute_m) <= shutdown_tolerance_m
                break;
            end

            if apogee_mid_m < target_apogee_absolute_m
                lo = mid;
            else
                hi = mid;
            end
        end

        ideal_shutdown_time_s = 0.5 * (lo + hi);
    end
end

function [apogee_m, log] = simulate_flight_to_apogee( ...
    shutdown_time_s, engine_time_s, engine_thrust_N, total_impulse_Ns, ...
    initial_altitude_m, initial_vertical_vel_mps, initial_horizontal_vel_mps, ...
    launch_tilt_angle_rad, initial_wet_mass_kg, propellant_mass_kg, minimum_mass_kg, ...
    cross_sectional_area_m2, Cd_constant, use_cd_mach_table, cd_mach_table, cd_value_table, g_mps2, dt_s, max_sim_time_s)

    max_steps = ceil(max_sim_time_s / dt_s) + 10;

    time_log = zeros(max_steps, 1);
    altitude_log = zeros(max_steps, 1);
    velocity_log = zeros(max_steps, 1);
    mass_log = zeros(max_steps, 1);
    thrust_log = zeros(max_steps, 1);

    t = 0.0;
    x = 0.0;
    y = initial_altitude_m;

    vx = initial_horizontal_vel_mps + initial_vertical_vel_mps * sin(launch_tilt_angle_rad);
    vy = initial_vertical_vel_mps * cos(launch_tilt_angle_rad);

    mass_kg = initial_wet_mass_kg;
    prop_used_kg = 0.0;

    k = 1;
    time_log(k) = t;
    altitude_log(k) = y;
    velocity_log(k) = hypot(vx, vy);
    mass_log(k) = mass_kg;
    thrust_log(k) = 0.0;

    previous_y = y;
    previous_vy = vy;
    previous_t = t;

    while t < max_sim_time_s && k < max_steps

        previous_y = y;
        previous_vy = vy;
        previous_t = t;

        % Engine thrust. Shutdown cuts thrust instantly.
        if t <= shutdown_time_s && t <= engine_time_s(end)
            thrust_N = interp1(engine_time_s, engine_thrust_N, t, "linear", 0.0);
        else
            thrust_N = 0.0;
        end

        % Propellant mass flow estimate from thrust fraction of total impulse.
        if thrust_N > 0 && prop_used_kg < propellant_mass_kg
            mdot_kg_s = propellant_mass_kg * thrust_N / total_impulse_Ns;
            prop_used_kg = prop_used_kg + mdot_kg_s * dt_s;
            if prop_used_kg > propellant_mass_kg
                prop_used_kg = propellant_mass_kg;
            end
        end

        mass_kg = max(minimum_mass_kg, initial_wet_mass_kg - prop_used_kg);

        speed = hypot(vx, vy);

        rho = isa_density_simple(y);

        if use_cd_mach_table
            speed_of_sound_mps = isa_speed_of_sound_simple(y);
            mach = speed / speed_of_sound_mps;
            Cd = interpolate_cd_from_mach(mach, cd_mach_table, cd_value_table);
        else
            Cd = Cd_constant;
        end

        if speed > 1e-9
            drag_N = 0.5 * rho * Cd * cross_sectional_area_m2 * speed * speed;
            drag_ax = -drag_N * vx / speed / mass_kg;
            drag_ay = -drag_N * vy / speed / mass_kg;
        else
            drag_ax = 0.0;
            drag_ay = 0.0;
        end

        thrust_ax = thrust_N * sin(launch_tilt_angle_rad) / mass_kg;
        thrust_ay = thrust_N * cos(launch_tilt_angle_rad) / mass_kg;

        ax = thrust_ax + drag_ax;
        ay = thrust_ay + drag_ay - g_mps2;

        % Semi-implicit Euler integration.
        vx = vx + ax * dt_s;
        vy = vy + ay * dt_s;
        x = x + vx * dt_s; %#ok<NASGU>
        y = y + vy * dt_s;
        t = t + dt_s;

        k = k + 1;
        time_log(k) = t;
        altitude_log(k) = y;
        velocity_log(k) = hypot(vx, vy);
        mass_log(k) = mass_kg;
        thrust_log(k) = thrust_N;

        % Apogee reached after vertical velocity crosses from positive to non-positive.
        if t > 0.05 && previous_vy > 0 && vy <= 0
            break;
        end

        % Safety: if it crashes before apogee for bad parameters.
        if y < initial_altitude_m - 10 && t > 1.0
            break;
        end
    end

    time_log = time_log(1:k);
    altitude_log = altitude_log(1:k);
    velocity_log = velocity_log(1:k);
    mass_log = mass_log(1:k);
    thrust_log = thrust_log(1:k);

    % Interpolate apogee using vertical velocity zero crossing.
    if previous_vy > 0 && vy <= 0
        f = previous_vy / (previous_vy - vy);
        apogee_m = previous_y + f * (y - previous_y);
    else
        apogee_m = max(altitude_log);
    end

    log = table(time_log, altitude_log, velocity_log, mass_log, thrust_log, ...
        'VariableNames', {'time_s', 'altitude_m', 'speed_mps', 'mass_kg', 'thrust_N'});
end

function Cd = interpolate_cd_from_mach(mach, cd_mach_table, cd_value_table)
    % Manual/fast linear interpolation for Cd(Mach).
    % Below the first table Mach, clamp to first Cd.
    % Above the last table Mach, clamp to last Cd.

    if mach <= cd_mach_table(1)
        Cd = cd_value_table(1);
    elseif mach >= cd_mach_table(end)
        Cd = cd_value_table(end);
    else
        i_hi = find(cd_mach_table >= mach, 1, "first");
        i_lo = i_hi - 1;

        m0 = cd_mach_table(i_lo);
        m1 = cd_mach_table(i_hi);
        c0 = cd_value_table(i_lo);
        c1 = cd_value_table(i_hi);

        f = (mach - m0) / (m1 - m0);
        Cd = c0 * (1 - f) + c1 * f;
    end
end

function a = isa_speed_of_sound_simple(altitude_m)
    % Simple ISA speed of sound model.
    % Used only to convert speed to Mach for Cd lookup.

    if altitude_m < 0
        altitude_m = 0;
    end

    T0 = 288.15;     % K
    L = 0.0065;      % K/m
    gamma = 1.4;
    R = 287.05287;   % J/(kg*K)

    if altitude_m <= 11000
        T = T0 - L * altitude_m;
    else
        T = T0 - L * 11000;
    end

    a = sqrt(gamma * R * T);
end

function rho = isa_density_simple(altitude_m)
    % Simple ISA density model for troposphere.
    % Valid enough for low-altitude rocket sim.
    if altitude_m < 0
        altitude_m = 0;
    end

    T0 = 288.15;        % K
    p0 = 101325.0;      % Pa
    L = 0.0065;         % K/m
    R = 287.05287;      % J/(kg*K)
    g0 = 9.80665;       % m/s^2

    if altitude_m <= 11000
        T = T0 - L * altitude_m;
        p = p0 * (T / T0)^(g0 / (R * L));
        rho = p / (R * T);
    else
        % Clamp above 11 km for this simple version.
        T = T0 - L * 11000;
        p = p0 * (T / T0)^(g0 / (R * L));
        rho = p / (R * T);
    end
end
