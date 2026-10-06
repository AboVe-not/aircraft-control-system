function paths = build_simulink(P, output_dir, kinds, extension, save_models, suffix)
% Four editable component-level Simulink models corresponding to Figs. 1-4.
if ~exist(output_dir,'dir'), mkdir(output_dir); end
[A_aircraft,B_aircraft,C_aircraft,D_aircraft] = ACS.plant_matrices(P);
if nargin<3
    kinds = {'static_ny','astatic_ny','static_pitch','astatic_pitch'};
elseif ischar(kinds) || isstring(kinds)
    kinds = cellstr(kinds);
end
if nargin<4, extension='.slx'; end
if nargin<5, save_models=true; end
if nargin<6, suffix=''; end
paths = cell(size(kinds));
for j=1:numel(kinds)
    kind = kinds{j};
    mdl = ['mode_v6_' kind suffix];
    if bdIsLoaded(mdl)
        assert(strcmp(get_param(mdl,'Dirty'),'off'), ...
            'В модели %s есть несохранённые изменения. Сначала сохраните её.',mdl);
        close_system(mdl,0);
    end
    new_system(mdl);
    set_param(mdl,'StopTime',num2str(P.t_end),'Solver','ode45');
    ws = get_param(mdl,'ModelWorkspace');
    assignin(ws,'P',P);
    assignin(ws,'A_aircraft',A_aircraft);
    assignin(ws,'B_aircraft',B_aircraft);
    assignin(ws,'C_aircraft',C_aircraft);
    assignin(ws,'D_aircraft',D_aircraft);

    add_block('simulink/Sources/Step',[mdl '/Reference'], ...
        'Position',[35 95 65 125], 'Time','0', 'Before','0', ...
        'After',local_reference(P,kind));
    add_block('simulink/Sources/Step',[mdl '/Gust'], ...
        'Position',[450 270 480 300], 'Time','0', 'Before','0', ...
        'After','0');
    add_block('simulink/Sources/Step',[mdl '/Moment'], ...
        'Position',[450 330 480 360], 'Time','0', 'Before','0', ...
        'After','0');
    add_block('simulink/Signal Routing/Mux',[mdl '/Plant_inputs'], ...
        'Inputs','3','Position',[520 177 525 260]);
    add_block('simulink/Continuous/State-Space',[mdl '/Aircraft'], ...
        'A','A_aircraft','B','B_aircraft','C','C_aircraft','D','D_aircraft', ...
        'Position',[550 190 665 245]);
    add_block('simulink/Signal Routing/Demux',[mdl '/Aircraft_outputs'], ...
        'Outputs','3','Position',[705 185 710 250]);
    add_block('simulink/Signal Routing/Mux',[mdl '/To_scope'], ...
        'Inputs','3','Position',[770 255 775 325]);
    add_block('simulink/Sinks/Scope',[mdl '/Scope_ny_omega_theta'], ...
        'Position',[810 272 850 312]);

    is_pitch = endsWith(kind,'_pitch');
    is_astatic = startsWith(kind,'astatic_');
    if is_pitch
        add_block('simulink/Math Operations/Sum',[mdl '/Theta_error'], ...
            'Inputs','+-','Position',[100 95 125 125]);
        if is_astatic, t_gain='P.pitch_astatic.Ktheta';
        else, t_gain='P.pitch_static.Ktheta'; end
        add_block('simulink/Math Operations/Gain',[mdl '/Ktheta'], ...
            'Gain',t_gain,'Position',[155 95 210 125]);
        add_line(mdl,'Reference/1','Theta_error/1','autorouting','on');
        add_line(mdl,'Theta_error/1','Ktheta/1','autorouting','on');
    end

    add_block('simulink/Math Operations/Sum',[mdl '/Ny_error'], ...
        'Inputs','+-','Position',[230 95 255 125]);
    if is_pitch
        add_line(mdl,'Ktheta/1','Ny_error/2','autorouting','on');
    else
        add_line(mdl,'Reference/1','Ny_error/2','autorouting','on');
    end
    if is_pitch && is_astatic, gain_group='P.pitch_astatic';
    elseif is_pitch, gain_group='P.pitch_static';
    elseif is_astatic, gain_group='P.astatic';
    else, gain_group='P.static'; end
    kn=[gain_group '.Kn']; kw=[gain_group '.Kw'];
    add_block('simulink/Math Operations/Gain',[mdl '/Kny'], ...
        'Gain',kn,'Position',[290 55 345 85]);
    add_block('simulink/Math Operations/Gain',[mdl '/Kwz'], ...
        'Gain',kw,'Position',[290 165 345 195]);
    if is_astatic
        add_block('simulink/Continuous/Integrator',[mdl '/Integral_ny_error'], ...
            'Position',[285 100 315 130]);
        add_block('simulink/Math Operations/Gain',[mdl '/Kiny'], ...
            'Gain',[gain_group '.Ki'],'Position',[345 100 395 130]);
        add_block('simulink/Math Operations/Sum',[mdl '/Command_sum'], ...
            'Inputs','+++','Position',[425 95 450 155]);
        add_line(mdl,'Ny_error/1','Integral_ny_error/1','autorouting','on');
        add_line(mdl,'Integral_ny_error/1','Kiny/1','autorouting','on');
        add_line(mdl,'Kiny/1','Command_sum/2','autorouting','on');
        add_line(mdl,'Aircraft_outputs/1','Kny/1','autorouting','on');
    else
        add_block('simulink/Math Operations/Sum',[mdl '/Command_sum'], ...
            'Inputs','++','Position',[425 100 450 145]);
        add_line(mdl,'Ny_error/1','Kny/1','autorouting','on');
    end
    add_line(mdl,'Aircraft_outputs/1','Ny_error/1','autorouting','on');
    add_line(mdl,'Aircraft_outputs/2','Kwz/1','autorouting','on');
    add_line(mdl,'Kny/1','Command_sum/1','autorouting','on');
    if is_astatic, rate_port='3'; else, rate_port='2'; end
    add_line(mdl,'Kwz/1',['Command_sum/' rate_port],'autorouting','on');
    if is_pitch
        add_line(mdl,'Aircraft_outputs/3','Theta_error/2','autorouting','on');
    end

    if P.actuator.dynamic
        add_block('simulink/Continuous/Transfer Fcn',[mdl '/Actuator'], ...
            'Numerator','P.actuator.Omega^2', ...
            'Denominator','[1 2*P.actuator.zeta*P.actuator.Omega P.actuator.Omega^2]', ...
            'Position',[480 100 575 145]);
    else
        add_block('simulink/Math Operations/Gain',[mdl '/Actuator'], ...
            'Gain','1','Position',[480 100 575 145]);
    end
    add_line(mdl,'Command_sum/1','Actuator/1','autorouting','on');
    add_line(mdl,'Actuator/1','Plant_inputs/1','autorouting','on');
    add_line(mdl,'Gust/1','Plant_inputs/2','autorouting','on');
    add_line(mdl,'Moment/1','Plant_inputs/3','autorouting','on');
    add_line(mdl,'Plant_inputs/1','Aircraft/1','autorouting','on');
    add_line(mdl,'Aircraft/1','Aircraft_outputs/1','autorouting','on');
    add_line(mdl,'Aircraft_outputs/1','To_scope/1','autorouting','on');
    add_line(mdl,'Aircraft_outputs/2','To_scope/2','autorouting','on');
    add_line(mdl,'Aircraft_outputs/3','To_scope/3','autorouting','on');
    add_line(mdl,'To_scope/1','Scope_ny_omega_theta/1','autorouting','on');

    paths{j}=fullfile(output_dir,[mdl extension]);
    if save_models
        save_system(mdl,paths{j});
        close_system(mdl);
    else
        set_param(mdl,'SimulationCommand','update');
        close_system(mdl,0);
    end
end
end

function s=local_reference(P,kind)
if endsWith(kind,'_pitch'), s=num2str(P.command.theta);
else, s=num2str(P.command.ny); end
end
