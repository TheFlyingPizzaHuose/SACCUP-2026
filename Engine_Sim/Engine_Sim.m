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
oriffice_diam = 0.03*0.0254; %meters
orifice_area = pi * (oriffice_diam / 2)^2; % m^2

tank_internal_diam = 3.5*0.0254; %meters
tank_length = 8*0.3048; %meters
tank_volume = tank_length*pi*(tank_internal_diam / 2)^2; %m^3

%====Chamber dimensions====
port_diam = 2*0.0254; %meters
chamber_length = 2*0.3048; %meters
chamber_OD = 3.5*0.0254; %meters
chamber_volume = pi * (port_diam/2)^2 * chamber_length;

%====Injector dimensions====
injector_diam = 0.07*0.0254; %meters
num_injectors = 61;
injector_area = num_injectors * pi * (injector_diam / 2)^2; % m^2

%====Nozzle dimensions====
throat_diam = 1.33*0.0254; %meters
exit_diam = 2.61*0.0254; %meters
throat_area = pi * (throat_diam / 2)^2; % m^2
exit_area = pi * (exit_diam / 2)^2; % m^2

%====Leak dimensions -ugh.====
leak_diam = 0.12;
leak_area = pi*(0.0254*leak_diam/2)^2;

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
while test < 30
    ratios = [ratios, test];
    [~,~,~,~, cat] = combustion_props(101325*600/14.7, test, combustion_props_input);
    temps600 = [temps600, cat];
    [~,~,~,~, cat] = combustion_props(101325*500/14.7, test, combustion_props_input);
    temps500 = [temps500, cat];
    [~,~,~,~, cat] = combustion_props(101325*400/14.7, test, combustion_props_input);
    temps400 = [temps400, cat];
    [~,~,~,~, cat] = combustion_props(101325*300/14.7, test, combustion_props_input);
    temps300 = [temps300, cat];
    [~,~,~,~, cat] = combustion_props(101325*200/14.7, test, combustion_props_input);
    temps200 = [temps200, cat];
    [~,~,~,~, cat] = combustion_props(101325*100/14.7, test, combustion_props_input);
    temps100 = [temps100, cat];
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

function m_flow = mass_in(rho, P_1, P_2, A, k, T) %https://en.wikipedia.org/wiki/Hagen%E2%80%93Poiseuille_equation?
    R_N2O = 188.8987;
    %L = 0.25;
    %D = 2*sqrt(A/pi);
    %N2O_viscosity = 0.0000149; %dynamic viscosity Pa*s
    %choke_P_ratio = (2/2.3)^(1.3/0.3);
    %m_flow = (A^2)*rho*(P_1-P_2)/(8*pi*L*u);
    %[~,~,P_vap] = N2O_properties(T);
    %if(P_1 > 1.05 * P_vap)
    m_flow = A*sqrt(2*rho*(P_1-P_2)/k);
    term1 = 2.3/2;
    term2 = -2.3/0.6;
    a_flow = A*P_1*sqrt(1.3/(T*R_N2O))*(term1^term2);
    if(P_2 > P_1)
        m_flow = 10e-6;
        disp('Error Backflow')
        return
    end
    %{
    else%if(P_2/P_1 < choke_P_ratio)
    else
        Z_factor = N2O_Z_fac(P_1);
        term1 = log(P_1/P_2);
        term2 = -16*N2O_viscosity*L/(term1*D^2);
        term3 = (256*N2O_viscosity^2*L^2)/(term1^2*D^4);
        term4 = (P_1^2 - P_2^2)/(2*R_N2O*Z_factor*T*term1);
        m_flow = A*(term2+sqrt(term3+term4));
    end
    %}
end
    
function P_loss = pipe_loss(rho, m_dot, A, L)
    if(m_dot == 0)
        P_loss = 0;
        return
    end
    D = 2*sqrt(A/pi);
    N2O_viscosity = 0.0000149; %dynamic viscosity Pa*s
    v = m_dot/(rho*A);
    Re = rho*v*L/N2O_viscosity;
    e = 3*10^-6; %PTFE Roughness
    term1 = (e/D)/3.7;
    term2 = 5.74/Re^0.9;
    f = 0.25/log10(term1+term2)^2;
    P_loss = L*f*rho*v^2/(D*2);
end

function [rho_liquid, rho_gas, vap_pres] = N2O_properties(temp)  
    %====N2O liquid and gas properties at different temps====
    T = [-30, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 36.42]; %Celcius
    P_vapor = [16, 18.01, 20.83, 23.97, 27.44, 31.27, 35.47, 40.07, 45.1, 50.6, 56.6, 63.15, 70.33, 72.51]; %Bar
    rhos_liquid = [1000, 995.4, 975.2, 953.9, 931.4, 907.4, 881.6, 853.5, 822.2, 786.6, 743.9, 688.0, 589.4, 452.0]; %kg/m3
    rhos_vapour  = [30, 46.82, 54.47, 63.21, 73.26, 84.86, 98.41, 114.5, 133.9, 158.1, 190.0, 236.7, 330.4, 452.0]; %kg/m3
    rho_gas = interp1(T, rhos_vapour, temp-273.15);
    rho_liquid = interp1(T, rhos_liquid, temp-273.15);
    vap_pres = interp1(T, P_vapor, temp-273.15)*101325;
end

function [heat_capacity, latent_heat] = N2O_latent_heat(temp)
    %https://airgasspecialtyproducts.com/wp-content/uploads/2016/02/Properties-Nitrous-Oxide.pdf
    T = [-30, -20, -10, 0, 10, 20, 32, 40, 50, 60, 70, 80, 97]; %F
    heats = [135.9, 131.9, 127.7, 123.0, 118.5, 113.4, 106.8, 101.5, 93.7, 86.2, 77.7, 69, 0]; %BTU/lb
    R = [350, 400, 450, 500, 525, 550]; %Rankines
    caps = [0.42, 0.45, 0.49, 0.57, 0.8, 1.3]; %BTU/lbm*F
    heat_capacity = interp1(R, caps, 1.8*temp)*4186.8; %K/kg*k
    latent_heat = interp1(T, heats, ((temp-273.15)*9/5)+32)*2326; %J/kg
    %https://www.unitconverters.net/specific-heat-capacity/btu-it-pound-176-f-to-joule-kilogram-k.htm
end

function Z = N2O_Z_fac(prev_pressure)
    pres_atm = [0, 10, 20, 30, 40, 50, 60, 70, 80];
    Zs = [1, 0.85, 0.78, 0.72, 0.65, 0.56, 0.49, 0.27, 0.2];
    Z = interp1(pres_atm, Zs, prev_pressure/101325);
end

function [liq_mass_new, gas_mass_new] = balance_rho(liq_mass, gas_mass, temp, volume, mol_N2, tank_pres)
    R_N2O = 188.8987;
    if(liq_mass+gas_mass <= 0)
        liq_mass_new = liq_mass;
        gas_mass_new = gas_mass;
        return
    end
    total_mass = liq_mass + gas_mass;
    [rho_liq, ~] = N2O_properties(temp);
    Z_factor = N2O_Z_fac(tank_pres);
    A = temp*mol_N2*8.314;
    B = temp*Z_factor*total_mass*R_N2O;
    C = total_mass/rho_liq;
    gas_ratio = (A+tank_pres*C-tank_pres*volume)/(tank_pres*C-B);
    gas_mass_new = gas_ratio*total_mass;
    liq_mass_new = total_mass-gas_mass_new;
    if(gas_ratio < 0)
        disp('Error')
    end
    if(gas_ratio > 1)
        disp('Error')
    end
end

function [d_temp, new_vaporized] = vaporize(old_gas_mass, new_gas_mass, liquid_mass, dt, temp, reset)
    persistent delay_vaporized;
    if isempty(delay_vaporized) || reset
        delay_vaporized = 0;
    end
    time_constant = dt/0.15;
    [Cp, latent_heat] = N2O_latent_heat(temp);
    %Cp = 1720; %J/kg*K https://rocketprops.readthedocs.io/en/latest/n2o_prop.html?
    A = (new_gas_mass-old_gas_mass);
    delay_vaporized = delay_vaporized + time_constant*(A-delay_vaporized);
    new_vaporized = 0.01*delay_vaporized;
    heat = delay_vaporized*latent_heat;
    mew = liquid_mass-new_vaporized; %losing my mind here
    d_temp = -heat/(mew*Cp);
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

function [thrust, Pe]=calc_thrust(Pc, Pa, Tc, g, Rc, A_throat, A_exit, m_dot, cee_star)
    mach_num = exit_mach(A_throat, A_exit, g);
    term0 = (1+(mach_num^2)*(g-1)/2);
    Te = Tc/term0;
    term1 = -g/(g-1);
    Pe = Pc*term0^(term1);
    V_exit = mach_num*sqrt(g*Rc*Te);
    %term1 = 2*g^2/(g-1);
    %term2 = (2/(g+1))^((g+1)/(g-1));
    %term3 = 1-(Pe/Pc)^((g-1)/g);
    %term4 = ((Pe-Pa)/Pc)*A_exit/A_throat;
    %Cf = sqrt(term1*term2*term3) + term4;
    %thrust = m_dot*cee_star*Cf;
    thrust = m_dot*V_exit + A_throat*(Pe - Pa);
    if(thrust < 0)
        thrust = 0;
        return
    end
end

%====sim loop vars====
total = 7.514; %kg
liquid_mass = 7.014; % Kg
vapor_mass = total-liquid_mass; % Kg
prop_used = 0; %Kg
prop_leak = 0; %Kg

outside_temp = 273.15+5; % K
tank_temp = outside_temp; % K

N20_pres = 0; %Pa
N2_pres = 0; %Pa
N2_mols = 0; %mol

outside_pres = 101325; %Pa
tank_pres = 612*101325/14.7; % Pa
chamber_pres = 101325; %Pa

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
pipe_losses = [];

%determine amount of gas space and therfore how much nitrous
[rho_liquid,~,vap_pres] = N2O_properties(tank_temp);
[liquid_mass, vapor_mass] = balance_rho(liquid_mass, vapor_mass, tank_temp, tank_volume, 0, vap_pres); %calculate starting mass fractions of liquid vs gas in tank      
vapor_volume = tank_volume-liquid_mass/rho_liquid; %m3
N2_pres = tank_pres - vap_pres;
if(N2_pres < 0)
    disp('Error: N2_pres negative, N2O temp too high')
    return;
end

N2_mols = (N2_pres*vapor_volume)/(R_u*tank_temp);

last = time;
P_loss = 0;

%reset vaporize_condense internal variable
vaporize(0, 0, 0, 0, 0, 1);

Z2 = 0;

steps = 0;
%while "has liq N2O and tank pressure higher than chamber pressure
%and has fuel grain left"
while vapor_mass >= 0 && port_diam < chamber_OD && tank_pres > chamber_pres
    steps = steps+1;
    [rho_liquid, ~, vap_pres] = N2O_properties(tank_temp);

    %transfer mass from tank to chamber (check if liquid or vapor)
    if(liquid_mass <= 0.015)
        rho_out = (N2_mols*N2_molar_mass+vapor_mass)/tank_volume;
        rho_vapor = vapor_mass/tank_volume;

        d_mass = dt*mass_in(rho_out, tank_pres, chamber_pres, injector_area, 2, tank_temp);%k=2 is based on aspire space experience
        
        N20_N2_ratio = vapor_mass/(N2_mols*N2_molar_mass+vapor_mass);
        vapor_mass = vapor_mass - N20_N2_ratio*d_mass; 
        if(vapor_mass < 0.05)
            break
        end
        N2_mols = N2_mols - (1-N20_N2_ratio)*d_mass/N2_molar_mass;
        d_mass = N20_N2_ratio*d_mass; %get only the N2O out for the combustion calcs
    else
        rho_vapor = vapor_mass/tank_volume;
        inject_pres = tank_pres - P_loss;
        d_mass = dt*mass_in(rho_liquid, inject_pres, chamber_pres, injector_area, 2, tank_temp);%k=2 is based on aspire space experience
        P_loss = 0.5*P_loss + 0.5*pipe_loss(rho_liquid, d_mass/dt, inlet_pipe_area, 2);
        liquid_mass = liquid_mass - d_mass; 

        %leak tank
        leak_mass = dt*mass_in(rho_liquid, tank_pres, 101325, leak_area, 2, tank_temp);%k=2 is based on aspire space experience
        liquid_mass = liquid_mass - leak_mass; 

        prop_leak = prop_leak + leak_mass; %Kg
    end
    
    %Here be treasure (not really)
    %{
    if(false && N2_mols > 0)%vent hole
        vapor_volume = tank_volume-liquid_mass/rho_liquid; %m3
        rho_out = (N2_mols*N2_molar_mass+vapor_mass)/vapor_volume;
        vent_mass = dt*mass_in(rho_out, tank_pres, 101325, orifice_area, 2, tank_temp);%k=2 is based on aspire space experience
        N20_N2_ratio = vapor_mass/(N2_mols*N2_molar_mass+vapor_mass);
        if(vapor_mass > 0.05)
            vapor_mass = vapor_mass - N20_N2_ratio*vent_mass;
        end
        N2_mols = N2_mols - (1-N20_N2_ratio)*vent_mass/N2_molar_mass;
        vent_mass = N20_N2_ratio*vent_mass; %get only the N2O out for the combustion calcs
    end
    %}


    %check if N2 pressure is still high enough to keep N2O in liquid phase
    if(liquid_mass <= 0.015)%check if in vapor phase emptying
        rho_vapor_new = vapor_mass/tank_volume;
        Z_ratio = (Z2/Z1);
        rho_ratio = rho_vapor_new/rho_vapor;
        N2O_pres = N2O_pres*Z_ratio*rho_ratio^1.3;
        N2_pres = (N2_mols*R_u*tank_temp)/vapor_volume;
        tank_pres = N2_pres+N2O_pres;
        tank_temp = (tank_temp/Z_ratio)*rho_ratio^(1.3-1);
    elseif tank_pres > vap_pres %if the gas pres is higher than vap pres, don't boil any nitrous
        vapor_volume = tank_volume-liquid_mass/rho_liquid; %m3
        N2_pres = (N2_mols*R_u*tank_temp)/vapor_volume;
        N2O_pres = N2O_Z_fac(tank_pres)*R_N2O*tank_temp*vapor_mass/vapor_volume;
        tank_pres = N2_pres+N2O_pres;
    else %if gas pres is lower than vap pres, then boil nitrous to regain vap pressure
        [liq_mass_new, gas_mass_new] = balance_rho(liquid_mass, vapor_mass, tank_temp, tank_volume, N2_mols, vap_pres); %calculate new mass fractions of liquid vs gas in tank
        [d_temp, new_vaporized] = vaporize(vapor_mass, gas_mass_new, liquid_mass, dt, tank_temp, 0); %calculate temp reduced from vaporization of liquid into gas
    
        tank_temp = tank_temp + d_temp; %cool the tank (the liquid technically)

        %convert some liquid into vapor based on the output of vaporize function
        liquid_mass = liquid_mass - new_vaporized;
        vapor_mass = vapor_mass + new_vaporized;

        vapor_volume = tank_volume-liquid_mass/rho_liquid; %m3
        N2_pres = (N2_mols*R_u*tank_temp)/vapor_volume;
        Z_factor = N2O_Z_fac(tank_pres);
        N2O_pres = Z_factor*vapor_mass*R_N2O*tank_temp/vapor_volume;
        tank_pres = N2_pres+N2O_pres;
    end

    %Pressure loss error prevention (unclean fix)
    if(P_loss > tank_pres)
        P_loss = 0.9*tank_pres;
    end
    
    %running compressibility factors
    Z1 = Z2;
    Z2 = N2O_Z_fac(tank_pres);

    %calculate fuel regression and fuel mass input
    port_area = pi * (port_diam/2)^2; %m2
    G_ox = (1000*d_mass/dt)/(port_area*10000); %g/cm2*s
    r_dot = 0.001*fuel_a*(G_ox^fuel_n)*chamber_length; %m/s Zilliac G.,Karabeyoglu A. "Hybrid Rocket Fuel Regression Rate Data and Modeling"
    
    d_volume = pi*port_diam*chamber_length*r_dot;

    fuel_input = d_volume*fuel_rho*dt;
    port_diam = port_diam + 2*r_dot*dt;

    %calculate mass flow
    mass_flow = (fuel_input+d_mass)/dt;
    oxidizer_fuel_ratio = d_mass/fuel_input;
    [chamber_temp, burn_R, burn_gamma, burn_rho, c_Star] = combustion_props(chamber_pres, oxidizer_fuel_ratio, combustion_props_input);
    
    %calculate mass flow out and new chamber pressure
    flow_out = 0.9*chamber_pres * throat_area / c_Star; %0.9 is the discharge coefficient for conical nozzle
    chamber_pres = chamber_pres + dt*chamber_pres*((mass_flow-flow_out)/chamber_mass - (d_volume/chamber_volume));
    chamber_mass = chamber_mass + mass_flow*dt - flow_out*dt;
    chamber_volume = chamber_volume + d_volume*dt;
    %chamber_pres = 0.9*chamber_pres + 0.1*calc_cham_pres1(chamber_pres, outside_pres, burn_gamma, burn_rho, burn_R, chamber_temp, mass_flow, port_diam, throat_diam);
    %chamber_pres = calc_cham_pres2(mass_flow, c_Star, throat_area);
    %disp([14.7*chamber_pres/101325, c_Star])

    %calculate thrust
    [thrust, Pe] = calc_thrust(chamber_pres, outside_pres, chamber_temp, burn_gamma, burn_R, throat_area, exit_area, flow_out, c_Star);
    thrust = 0.7*thrust; %divergence losses
    
    time = time+dt;

    if(time - last > 0.1)
        disp([time, d_mass/dt, 14.7*chamber_pres/101325, G_ox, chamber_mass])
        last = time;
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
    pipe_losses = [pipe_losses, 14.7*P_loss/101325];

    impulse = impulse+thrust*dt;
    prop_used = prop_used + fuel_input + d_mass;
end
if(vapor_mass < 0.05)
    disp('End Condition: ran out of vapor N2O')
elseif(liquid_mass < 0.015)
    disp('End Condition: ran out of liquid N2O')
elseif(vapor_mass < 0)
    disp('End Condition: N2O vapor negative')
elseif(port_diam >= chamber_OD)
    disp('End Condition: ran out of fuel')
elseif(tank_pres <= chamber_pres)
    disp('End Condition: backflow into tank')
end
disp(['Impulse (Ns): ', sprintf('%.2f',impulse), ' || ISP: ', ...
      sprintf('%.2f',impulse/(prop_used*9.81)), ' || Sim Steps: ', ...
      sprintf('%d',steps)])
disp(['Propellant mass used (Kg): ', sprintf('%.3f',prop_used), ...
      ' || Final grain ID (in): ', sprintf('%.3f',port_diam/0.0254), ...
      ' || Propellant mass leaked (Kg): ', sprintf('%.3f',prop_leak)])

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
plot(times, pipe_losses, 'g', 'LineWidth', 0.5);
hold off
legend('Tank', 'Chamber', 'Pipe Loss');
xlabel('Time (s)');
ylabel('Pressure (psi)');
title('Pressures vs time');
grid on;

% Extract arrays
time2 = [-2.72, -2.68, -2.18, -1.37, -0.432, -0.372, -0.271, 0.0454, 0.484, 0.909, 1.39, 1.89];
thrust2 = [-0.194, 0.668, 0.299, -0.0252, -0.265, -0.403, -0.488, -0.645, -0.805, -0.901, -0.969, -1];
time2 = time2+2.74174;
time2 = time2.*(8/(2.14402+2.74174));
thrust2 = thrust2+1;
thrust2 = thrust2.*(800/2);

subplot(2,3,3);
plot(time2, thrust2, 'r', 'LineWidth', 1.5);
hold on;
plot(times, thrusts, 'g', 'LineWidth', 0.5);
hold off;
legend('Experiment', 'Simulation');
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
