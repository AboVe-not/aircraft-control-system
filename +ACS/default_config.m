function P = default_config()
% User-adjustable parameters for the aircraft control system.
% Default flight condition used by this project.
P.configuration_id = 6;
P.H_km = 4;
P.V_kmh = 800;
P.Mach = 0.81;
P.V_ms = 264;
P.c1 = 0.60;
P.c2 = 8.80;
P.c3 = 15.80;
P.c4 = 1.10;
P.c5 = 0.220;
P.c6_over_g = 0.470;
P.c9 = 0.180;

% Стандартные полиномы для синтеза с идеальным приводом.
% Частоты ниже являются явными настройками синтеза, не оптимумом.
P.design.static_frequency = 5;
P.design.astatic_frequency = 3;
P.design.static_pitch_outer_frequency = 0.25;
P.design.astatic_pitch_outer_frequency = 0.6;
P.design.static_ny = -P.design.static_frequency*[1 1];
P.design.astatic_ny = -P.design.astatic_frequency*[1 1 1];
P.design.static_pitch = [-P.design.static_frequency*[1 1], ...
    -P.design.static_pitch_outer_frequency];
P.design.astatic_pitch = [-P.design.astatic_frequency*[1 1 1], ...
    -P.design.astatic_pitch_outer_frequency];
[P.static,P.design.result.static_ny] = ACS.standard_coefficients(P,'static_ny',P.design.static_ny);
[P.astatic,P.design.result.astatic_ny] = ACS.standard_coefficients(P,'astatic_ny',P.design.astatic_ny);
[P.pitch_static,P.design.result.static_pitch] = ACS.standard_coefficients(P,'static_pitch',P.design.static_pitch);
[P.pitch_astatic,P.design.result.astatic_pitch] = ACS.standard_coefficients(P,'astatic_pitch',P.design.astatic_pitch);

P.f1 = P.c1 + P.c4 + P.c5;
P.f0 = P.c2 + P.c1 * P.c4;
P.omega_aircraft = sqrt(P.f0);
P.zeta_aircraft = P.f1 / (2 * P.omega_aircraft);

% User-selectable actuator settings for the full model.
P.actuator.dynamic = true;
% Независимое предварительное значение для проверки полной модели.
% Метод стандартных коэффициентов ниже не определяет частоту привода.
P.actuator.Omega = 30;
P.actuator.zeta = 0.80;
P.actuator.options = 5:5:30;

% Справочные коэффициенты используются только для сравнения, не для синтеза/оптимизации.
P.reference.static_ny = struct('Kn',1.11,'Kw',0.32,'Ki',0,'Ktheta',0);
P.reference.astatic_ny = struct('Kn',1.16,'Kw',0.44,'Ki',5.25,'Ktheta',0);
P.reference.static_pitch = struct('Kn',1.11,'Kw',0.32,'Ki',0,'Ktheta',1);
P.reference.astatic_pitch = struct('Kn',1.16,'Kw',0.44,'Ki',5.25,'Ktheta',0.3);
P.refinement.max_iterations = 200;
P.refinement.max_evaluations = 1000;
P.refinement.max_seconds = 60;
P.refinement.max_overshoot = 10;
P.refinement.target_time = 1;
P.refinement.band = 0.05;
P.refinement.dt = 0.01;
P.refinement.validation_dt = 0.002;
P.refinement.horizon = 30;
% Ограничения задачи, их можно расширить и повторить поиск.
P.refinement.lower = [0.001 0.001 0.001 0.001];
P.refinement.upper = [15 8 40 8]; % Kn Kw Ki Ktheta

P.t_end = 10;
P.dt = 0.01;
P.command.ny = 1;       % g
P.command.theta = 1;    % deg
P.command.gust = 1;     % deg of effective incidence
P.command.moment = 1;   % equivalent angular acceleration, deg/s^2

% One-factor-at-a-time comparisons. Edit these without changing core code.
P.sweep.c2 = [0.5 1 1.5];
P.sweep.c1 = [0.5 1 1.5];
P.sweep.c9 = [0 1 2];
P.sweep.Omega = [0.5 1 2];
P.sweep.Kw = [0.5 1 1.5];
P.sweep.Kn = [0.5 1 1.5];
P.sweep.Ktheta = [0.5 1 1.5];
end
