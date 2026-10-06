function check_standard_integration()
root=fileparts(mfilename('fullpath')); addpath(root);
check_standard_design();
f=interactive_mode('off');
cleanup=onCleanup(@()delete(f));
dropdowns=findall(f,'Type','uidropdown');
for j=1:numel(dropdowns)
    if numel(dropdowns(j).Items)==4, modes=dropdowns(j); break; end
end
for j=1:4
    modes.Value=modes.ItemsData{j};
    callback=modes.ValueChangedFcn; callback(modes,[]);
    drawnow;
end
P=ACS.default_config();
paths=ACS.build_simulink(P,fullfile(root,'models_standard'));
kinds={'static_ny','astatic_ny','static_pitch','astatic_pitch'};
for j=1:4
    load_system(paths{j});
    [~,mdl]=fileparts(paths{j});
    model_cleanup=onCleanup(@()close_system(mdl,0));
    set_param(mdl,'SimulationCommand','update');
    [A,~,~,~]=linmod(mdl);
    sys=ACS.closed_loop(P,kinds{j});
    actual=real(poly(eig(A))); expected=real(poly(pole(sys)));
    assert(numel(actual)==numel(expected));
    assert(norm(actual-expected,inf)/max(1,norm(expected,inf))<1e-7, ...
        'Модель Simulink не совпала с closed_loop.');
    clear model_cleanup;
end
fprintf('Окно и четыре модели Simulink проверены.\n');
end
