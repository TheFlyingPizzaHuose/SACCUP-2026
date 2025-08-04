clc
%====Thermo-physical properties====
N2O_molar_mass = 0.044013; %kg/mol
N2_molar_mass = 0.0280134; %kg/mol
R_u = 8.314; %J/mol*K universal gas constant
Cp_N2O = 38.6; %J/mol*K specific heat at constant pressure
Cp_N2 = 29.1; 
R_N2O = R_u/N2O_molar_mass;
R_N2 = R_u/N2_molar_mass;
Cv_N2O = Cp_N2O - R_u; %J/mol*K
Cv_N2 = Cp_N2 - R_u; %J/mol*K
N2O_viscosity = 0.0003237; %dynamic viscosity Pa*s

%====Fuel Properties====
fuel_rho = 1180; %kg/m3
fuel_a = 0.284; %PMMA1: 0.284, PMMA2: 0.087
fuel_n = 0.335; %PMMA1: 0.335, PMMA2: 0.615 

%GSE dimensions
inlet_pipe_length = 1; %meters
inlet_pipe_diam = 0.5*0.0254; %meters
inlet_pipe_area = pi * (inlet_pipe_diam /2)^2; % m^2
supply_tank_volume = 0.045;

%====Tank dimensions====
oriffice_diam = 0.06*0.0254; %meters
orifice_area = pi * (oriffice_diam / 2)^2; % m^2

tank_internal_diam = 3.5*0.0254; %meters
tank_length = 8*0.3048; %meters
tank_volume = tank_length*pi*(tank_internal_diam / 2)^2; %m^3

%====injector dimensions====
injector_diam = 0.04*0.0254; %meters
num_injectors = 61;
injector_area = num_injectors * pi * (injector_diam / 2)^2; % m^2

%====Chamber dimensions====
port_diam = 2*0.0254; %meters
chamber_length = 2*0.3048; %meters
chamber_OD = 3.5*0.0254; %meters
chamber_volume = pi * (port_diam/2)^2 * chamber_length;

%====Nozzle dimensions====
throat_diam = 1.330*0.0254; %meters
exit_diam = 2.61*0.0254; %meters
throat_area = pi * (throat_diam / 2)^2; % m^2
exit_area = pi * (exit_diam / 2)^2; % m^2

%====CEA INPUT====
rawData = readlines('PMMA_CEA.txt');  % read each line as a string

% --- Get headers and clean whitespace ---
firstLine = strtrim(rawData(3));                 % remove leading/trailing spaces
firstLine = regexprep(firstLine, '\s+', ' ');   % replace multiple spaces with a single space
headers = split(firstLine, ' ');                 % split by space
t_idx = strcmp(headers, 't');
gam_idx = strcmp(headers, 'gam');
m_idx = strcmp(headers, 'm');
rho_idx = strcmp(headers, 'rho');

data = str2double(split(strtrim(rawData(4:2:end))));%Read numeric data (skipping header, skips every 2nd element)

% --- Assign columns ---
t_col   = data(:, t_idx);
gam_col = data(:, gam_idx);
m_col   = data(:, m_idx);
rho_col   = data(:, rho_idx);

% --- Define axis arrays ---
p_psi = str2double(split(strtrim(rawData(1)),','));
of_ratios = str2double(split(strtrim(rawData(2)),','));

% --- Make axes row 1xn instead of nx1 ---
p_psi = reshape(p_psi, 1, []);
of_ratios = reshape(of_ratios, 1, []);

n_p = numel(p_psi);
n_of = numel(of_ratios);

% --- Reshape to 2D matrices (rows: O/F, cols: P) ---
T_matrix   = reshape(t_col,   n_p, n_of)';
GAM_matrix = reshape(gam_col, n_p, n_of)';
M_matrix   = reshape(m_col,   n_p, n_of)';
RHO_matrix   = reshape(rho_col,   n_p, n_of)';
% --- OPTIONAL: visualize dimensions ---
%imagesc(p_psi, of_ratios, T_matrix); colorbar; xlabel('P [psia]'); ylabel('O/F'); title('Temperature');
%====END CEA INPUT====
[P_grid, O_grid] = ndgrid(of_ratios, p_psi);

F_T     = griddedInterpolant(P_grid, O_grid, T_matrix);
F_M     = griddedInterpolant(P_grid, O_grid, M_matrix);
F_GAM   = griddedInterpolant(P_grid, O_grid, GAM_matrix);
F_RHO   = griddedInterpolant(P_grid, O_grid, RHO_matrix);

combustion_props_input = {F_T, F_GAM, F_M, F_RHO};
function [T, R, gamma, rho, c_star] = combustion_props(pres_Pa, o_f_ratio, input_data)
    F_T = input_data{1};
    F_GAM = input_data{2};
    F_M = input_data{3};
    F_RHO = input_data{4};
    pres_psi = 14.7*pres_Pa/101325;
    T = F_T(o_f_ratio, pres_psi);
    R = 8314/F_M(o_f_ratio, pres_psi);
    gamma = F_GAM(o_f_ratio, pres_psi);
    rho = F_RHO(o_f_ratio, pres_psi);
    term1 = R*T/gamma;
    term2 = (gamma+1)/2;
    term3 = (gamma+1)/(gamma-1);
    c_star = sqrt(term1*(term2^term3));
end

%{
test = 0.1;
temps600 = [];
temps500 = [];
temps400 = [];
temps300 = [];
temps200 = [];
temps100 = [];
ratios = [];
while test < 5
    ratios = [ratios, test];
    temps600 = [temps600, combustion_props(101325*600/14.7, test, combustion_props_input)];
    temps500 = [temps500, combustion_props(101325*500/14.7, test, combustion_props_input)];
    temps400 = [temps400, combustion_props(101325*400/14.7, test, combustion_props_input)];
    temps300 = [temps300, combustion_props(101325*300/14.7, test, combustion_props_input)];
    temps200 = [temps200, combustion_props(101325*200/14.7, test, combustion_props_input)];
    temps100 = [temps100, combustion_props(101325*100/14.7, test, combustion_props_input)];
    test = test + 0.1;
end
figure;
plot(ratios, temps600, 'r', 'LineWidth', 0.5);
hold on;
plot(ratios, temps500, 'g', 'LineWidth', 0.5);
plot(ratios, temps400, 'b', 'LineWidth', 0.5);
plot(ratios, temps300, 'y', 'LineWidth', 0.5);
plot(ratios, temps200, 'w', 'LineWidth', 0.5);
plot(ratios, temps100, 'm', 'LineWidth', 0.5);
hold off;
legend('600', '500', '400', '300', '200', '100');
%}

function cham_pres = calc_cham_pres1(prev_pres, exit_pres, gam, rho, R, temp, m_dot, in_diam, throat_diam)
    choked_pres_ratio = (2/(gam+1))^(gam/(gam-1)); %exit pressure/inlet pressure
    mach_throat = 0;
    mach_in = 0;
    throat_area = pi*(throat_diam/2)^2; %m2
    if(exit_pres/prev_pres > choked_pres_ratio)%not choked flow
        inlet_area = pi*(in_diam/2)^2; %m2
        v = m_dot/(rho*inlet_area);
        mach_in = v/sqrt(gam*R*temp);
        disp([mach_in, rho]);
        mach_throat = subsonic_mach_from_area_ratio(inlet_area, throat_area, gam, mach_in); %can return m=1
    else%choked flow
        mach_throat = 1;
    end
    if(mach_throat > 1)
        disp('Throat Mach Error')
        mach_throat = 1;
    end
    term0 = (1+(mach_throat^2)*(gam-1)/2);
    % --- choked mass flow --- https://www.grc.nasa.gov/www/k-12/airplane/astar.html
    term1 = throat_area*sqrt(gam/(temp*R));
    term2 = -(gam+1)/(2*gam-2);
    term3 = mach_throat*term0^term2;
    total_pres = m_dot/(term1*term3);

    term0 = (1+(mach_in^2)*(gam-1)/2);
    cham_pres = total_pres*term0^(-gam/(gam-1));%Pa https://www.grc.nasa.gov/www/k-12/airplane/isentrop.html  
end

function cham_pres = calc_cham_pres2(m_dot, c_star, throat_A)
    cham_pres = c_star*m_dot/throat_A;
end

function [y_mix, R_mix] = mix_N2O_N2(mass1, mass2, molar1, molar2, Cp1, Cp2, Cv1, Cv2)
    mol_N2O = mass1/molar1;
    mol_N2 = mass2/molar2;
    R1 = 8.314/molar1;
    R2 = 8.314/molar2;
    R_mix = (R1*mol_N2O + R2*mol_N2)/(mol_N2O+mol_N2); %J/kg*K
    Cp_mix = (Cp1*mol_N2O + Cp2*mol_N2)/(mol_N2O+mol_N2); %J/mol*K
    Cv_mix = (Cv1*mol_N2O + Cv2*mol_N2)/(mol_N2O+mol_N2); %J/mol*K
    y_mix = Cp_mix/Cv_mix;
end

function m_flow = vent_mass_out(y_mix, R_mix, P_tank, rho_mix, T_tank, A)
    min_P_ratio = ((y_mix+1)/2)^(y_mix/(y_mix-1)); %minimum pressure ratio for choked flow
    if P_tank < 101325
        m_flow = 0;
        return
    end
    if(P_tank/101325 > min_P_ratio)
        term = (2/(y_mix+1))^((y_mix+1)/(y_mix-1));
        m_flow = A*sqrt(y_mix*rho_mix*P_tank*term);
    else %https://ntrs.nasa.gov/api/citations/19690028201/downloads/19690028201.pdf
        term1 = A*P_tank/sqrt(T_tank);
        term2 = 2*9.81*y_mix/(R_mix*(y_mix-1));
        term3 = (101325/P_tank)^(2/y_mix) - (101325/P_tank)^((y_mix+1)/y_mix);
        m_flow = term1*sqrt(term2*term3);
    end
end

function m_flow = mass_in(rho, P_1, P_2, A, k) %https://en.wikipedia.org/wiki/Hagen%E2%80%93Poiseuille_equation?
    %m_flow = (A^2)*rho*(P_1-P_2)/(8*pi*L*u);
    m_flow = A*sqrt(2*rho*(P_1-P_2)/k);
end
    
function [rho_liquid, rho_gas, vap_pres] = N2O_properties(temp)  
    %====N2O liquid and gas properties at different temps====
    T = [-20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 36.42]; %Celcius
    P_vapor = [18.01, 20.83, 23.97, 27.44, 31.27, 35.47, 40.07, 45.1, 50.6, 56.6, 63.15, 70.33, 72.51]; %Bar
    rhos_liquid = [995.4, 975.2, 953.9, 931.4, 907.4, 881.6, 853.5, 822.2, 786.6, 743.9, 688.0, 589.4, 452.0]; %kg/m3
    rhos_vapour  = [46.82, 54.47, 63.21, 73.26, 84.86, 98.41, 114.5, 133.9, 158.1, 190.0, 236.7, 330.4, 452.0]; %kg/m3
    rho_gas = interp1(T, rhos_vapour, temp-273.15);
    rho_liquid = interp1(T, rhos_liquid, temp-273.15);
    vap_pres = interp1(T, P_vapor, temp-273.15)*101325;
end

function latent_heat = N2O_latent_heat(temp)
    %https://airgasspecialtyproducts.com/wp-content/uploads/2016/02/Properties-Nitrous-Oxide.pdf
    T = [32, 40, 50, 60, 70, 80, 97]; %F
    heats = [106.8, 101.5, 93.7, 86.2, 77.7, 69, 0]; %BTU/lb
    latent_heat = interp1(T, heats, ((temp-273.15)*9/5)+32)*2326; %J/kg
end

function pressure = real_gas_law(mass, molar, temp, volume, prev_pressure)
    pres_atm = [0, 10, 20, 30, 40, 50, 60, 70, 80];
    Zs = [1, 0.85, 0.78, 0.72, 0.65, 0.56, 0.49, 0.27, 0.2];
    Z = interp1(pres_atm, Zs, prev_pressure/101325);
    pressure = Z*temp*8.314*mass/(molar*volume);
end

function [liq_mass_new, gas_mass_new] = balance_rho(liq_mass, gas_mass, temp, volume)
    if(liq_mass+gas_mass <= 0)
        liq_mass_new = liq_mass;
        gas_mass_new = gas_mass;
        return
    end
    [rho_liq, rho_gas] = N2O_properties(temp);
    mass = liq_mass+gas_mass;
    gas_ratio = (volume*rho_liq*rho_gas-mass*rho_gas)/(mass*rho_liq-mass*rho_gas);
    if(gas_ratio > 1) %indicates there's not enough mass to fill the tank with the given gas density
        gas_ratio = 1;
    end
    gas_mass_new = mass*gas_ratio;
    liq_mass_new = mass-gas_mass_new;
end

function [d_temp, new_vaporized] = vaporize(old_gas_mass, new_gas_mass, liquid_mass, dt, temp, reset)
    persistent delay_vaporized;
    if isempty(delay_vaporized) || reset
        delay_vaporized = 0;
    end
    if(false && new_gas_mass < old_gas_mass) %if amount of gas is being reduced send end sim signal
        d_temp = 1000;
        new_vaporized = 0;
        return
    end
    time_constant = dt/0.15;
    Cp = 1720; %J/kg*K https://rocketprops.readthedocs.io/en/latest/n2o_prop.html?
    latent_heat = N2O_latent_heat(temp);
    A = (new_gas_mass-old_gas_mass);
    delay_vaporized = delay_vaporized + time_constant*(A-delay_vaporized);
    new_vaporized = delay_vaporized;
    heat = delay_vaporized*latent_heat;
    d_temp = -heat/(liquid_mass*Cp);
end

function mach=subsonic_mach_from_area_ratio(inlet_area, throat_area, gamma, inlet_mach)
    term1 = (gamma+1)/(2*gamma-2);
    term2 = ((gamma+1)/2)^(-term1);
    term3 = (gamma-1)/2;
    f = @(x) term2*((1+term3*x^2)^term1)/x; %https://www.grc.nasa.gov/www/k-12/airplane/astar.html
    ratio1 = f(inlet_mach); %inlet_area/A* inlet mach number assumed to be 0.4
    a_star = inlet_area/ratio1;
    if a_star > throat_area %If A* > throat_area, means flow is choked and M=1 at throat
        mach = 1;
        return
    end
    ratio2 = throat_area/a_star;%throat_area/A*
    f2 = @(x) term2*((1+term3*x^2)^term1)/x - ratio2; %the zero of this function is the mach no at throat
    mach = fzero(f2, 0.99);
end

function mach=exit_mach(throat_area, exit_area, gamma)
    term1 = (gamma+1)/(2*gamma-2);
    term2 = ((gamma+1)/2)^(-term1);
    term3 = (gamma-1)/2;
    ratio2 = exit_area/throat_area;%throat_area/A*
    f2 = @(x) term2*((1+term3*x^2)^term1)/x - ratio2; %the zero of this function is the mach no at throat
    mach = fzero(f2, 5);
end

function [thrust, Pe]=calc_thrust(Pc, Pa, Tc, gamma, Rc, A_throat, A_exit, m_dot)
    mach_num = exit_mach(A_throat, A_exit, gamma);
    term0 = (1+(mach_num^2)*(gamma-1)/2);
    Te = Tc/term0;
    term1 = -gamma/(gamma-1);
    Pe = Pc*term0^(term1);
    V_exit = mach_num*sqrt(gamma*Rc*Te);
    thrust = m_dot*V_exit + (Pe-Pa)*A_exit;
end

%====sim loop vars====
liquid_mass = 7.514; % Kg
vapor_mass = 0; % Kg
prop_used = 0; %Kg

outside_temp = 273.15+10; % K
tank_temp = outside_temp; % K

N20_pres = 0; %Pa
N2_pres = 0; %Pa
N2_mols = 0; %mol

outside_pres = 101325; %Pa
tank_pres = 612*101325/14.7; % Pa
chamber_pres = 101325*14.7/14.7; %Pa

chamber_mass = chamber_volume*chamber_pres/(287*300); %Kg Ideal gas law with air

impulse = 0; %Ns

time = 0; % s
dt = 10^-3; % s

times = [];
pressures = [];
cham_presses = [];
exit_presses = [];
tank_liq_masses = [];
tank_vap_masses = [];
tank_temps = [];
masses = [];
of_ratios_record = [];
thrusts = [];

%reset vaporize_condense internal variable
vaporize(0, 0, 0, 0, 0, 1);

%determine liquid vs vapor mass ratios in tank
[x,y] = balance_rho(liquid_mass, vapor_mass, tank_temp, tank_volume);
liquid_mass = x;
vapor_mass = y;
[x,y,z] = N2O_properties(tank_temp);
N20_pres = z;
N2_pres = tank_pres - N20_pres;
if N2_pres < 0
    disp('Error: N2_pres is negative, try reducing initial tank temperature');
    return;
end

vapor_volume = vapor_mass/y; %m3
N2_mols = (N2_pres*vapor_volume)/(R_u*tank_temp);

last = liquid_mass;


%while chekc is "has liq N2O and tank pressure higher than chamber pressure
%and has fuel grain left
while liquid_mass > 0.015 && vapor_mass > 0 && port_diam < chamber_OD && tank_pres > chamber_pres
    [rho_liquid, ~, ~] = N2O_properties(tank_temp); %calculate new nitrous properties

    %transfer mass from tank to chamber
    d_mass = dt*mass_in(rho_liquid, tank_pres, chamber_pres, injector_area, 2);%k=2 is based on aspire space experience
    liquid_mass = liquid_mass - d_mass; 

    %vaporize some nitrous
    [liq_mass_new, gas_mass_new] = balance_rho(liquid_mass, vapor_mass, tank_temp, tank_volume); %calculate new mass fractions of liquid vs gas in tank
    [d_temp, new_vaporized] = vaporize(vapor_mass, gas_mass_new, liquid_mass, dt, tank_temp, 0); %calculate temp reduced from vaporization of liquid into gas
    if d_temp == 1000 %if end sim signal, break while loop
        break
    end
    tank_temp = tank_temp + d_temp; %cool tank (liquid technically)

    %convert some liquid into vapor based on the output of vaporize function
    liquid_mass = liquid_mass - new_vaporized;
    vapor_mass = vapor_mass + new_vaporized;

    %calculate new nitrogen volume, pressure and tank pressure
    [~, rho_gas, vap_pres] = N2O_properties(tank_temp); %calculate new nitrous properties
    vapor_volume = vapor_mass/rho_gas; %m3
    N2_pres = (N2_mols*R_u*tank_temp)/vapor_volume;
    tank_pres = N2_pres + vap_pres;

    %calculate fuel regression and fuel mass input
    port_area = pi * (port_diam/2)^2; %m2
    G_ox = (1000*d_mass/dt)/(port_area*10000); %g/cm2*s
    r_dot = 0.001*fuel_a*(G_ox^fuel_n)*chamber_length; %m/s Zilliac G.,Karabeyoglu A. "Hybrid Rocket Fuel Regression Rate Data and Modeling"
    
    d_volume = pi*port_diam*chamber_length*r_dot;

    fuel_input = d_volume*fuel_rho*dt;
    port_diam = port_diam + 2*r_dot*dt;

    %calculate mass flow
    mass_flow = (fuel_input+d_mass)/dt;
    oxidizer_fuel_ratio = (d_mass)/fuel_input;
    [chamber_temp, burn_R, burn_gamma, burn_rho, c_Star] = combustion_props(chamber_pres, oxidizer_fuel_ratio, combustion_props_input);
    
    %calculate mass flow out and chamber pressure
    flow_out = chamber_pres * throat_area / (c_Star);
    chamber_pres = chamber_pres + dt*chamber_pres*((mass_flow-flow_out)/chamber_mass - (d_volume/chamber_volume));
    chamber_mass = chamber_mass + mass_flow*dt - flow_out*dt;
    chamber_volume = chamber_volume + d_volume*dt;
    %chamber_pres = 0.9*chamber_pres + 0.1*calc_cham_pres1(chamber_pres, outside_pres, burn_gamma, burn_rho, burn_R, chamber_temp, mass_flow, port_diam, throat_diam);
    %chamber_pres = calc_cham_pres2(mass_flow, c_Star, throat_area);
    %disp([14.7*chamber_pres/101325, c_Star])

    %calculate thrust
    [thrust, Pe] = calc_thrust(chamber_pres, outside_pres, chamber_temp, burn_gamma, burn_R, throat_area, exit_area, flow_out);

    time = time+dt;

    if(last - liquid_mass > 0.01)
        disp([time, d_mass/dt, 14.7*chamber_pres/101325, 14.7*tank_pres/101325, oxidizer_fuel_ratio])
        last = liquid_mass;
    end

    %record data
    times = [times, time];
    masses = [masses, liquid_mass+vapor_mass];
    tank_liq_masses = [tank_liq_masses, liquid_mass];
    tank_vap_masses = [tank_vap_masses, vapor_mass];
    tank_temps = [tank_temps, tank_temp];
    pressures = [pressures, 14.7*tank_pres/101325];
    cham_presses = [cham_presses, 14.7*chamber_pres/101325];
    of_ratios_record = [of_ratios_record, oxidizer_fuel_ratio];
    thrusts = [thrusts, thrust/4.448];
    exit_presses = [exit_presses, 14.7*Pe/101325];

    impulse = impulse+thrust*dt;
    prop_used = prop_used + fuel_input+d_mass;
end
disp(['Impulse (Ns): ', sprintf('%.2f',impulse)])
disp(['ISP:', sprintf('%.2f',impulse/(prop_used*9.81))])
disp(['Propellant mass used (Kg): ', sprintf('%.3f',prop_used)])
disp(['Final grain ID (in): ', sprintf('%.3f',port_diam/0.0254)])

figure;
subplot(2, 3, 1);
plot(times, masses, 'r', 'LineWidth', 0.5);
hold on
plot(times, tank_liq_masses, 'g', 'LineWidth', 0.5);
plot(times, tank_vap_masses, 'b', 'LineWidth', 0.5);
hold off
legend('Total', 'Liquid', 'Vapor');
xlabel('Time (s)');
ylabel('Tank mass (kg)');
title('Tank mass vs time');
grid on;

subplot(2, 3, 2);
plot(times, pressures, 'b', 'LineWidth', 0.5);
hold on 
plot(times, cham_presses, 'r', 'LineWidth', 0.5);
hold off
legend('Tank', 'Chamber');
xlabel('Time (s)');
ylabel('Pressure (psi)');
title('Pressures vs time');
grid on;

subplot(2,3,3);
plot(times, thrusts, 'g', 'LineWidth', 0.5);
xlabel('Time (s)');
ylabel('Thrust (lbf)');
title('Thrust vs time');
grid on;

subplot(2, 3, 4);
plot(times, tank_temps, 'g', 'LineWidth', 0.5);
xlabel('Time (s)');
ylabel('Temperature (K)');
title('Tank Temperature vs time');
grid on;

subplot(2, 3, 5);
plot(times, of_ratios_record, 'r', 'LineWidth', 0.5);
xlabel('Time (s)');
ylabel('O/F Ratio');
title('Oxidizer Fuel Ratio vs time');
grid on;

subplot(2, 3, 6);
plot(times, exit_presses, 'g', 'LineWidth', 0.5);
xlabel('Time (s)');
ylabel('Pressure (psi)');
title('Exit Pressure vs time');
grid on;
