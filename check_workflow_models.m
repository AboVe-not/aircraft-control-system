function check_workflow_models()
root=fileparts(mfilename('fullpath')); addpath(root);
kinds={'static_ny','astatic_ny','static_pitch','astatic_pitch'};
for i=1:4
    kind=kinds{i}; run_mode_workflow(kind,30);
    folder=fullfile(root,'results_workflow',kind);
    S=load(fullfile(folder,'calculation.mat'));
    labels={'initial','refined','reference'};
    for j=1:3
        mdl=['mode_v6_' kind '_' labels{j}];
        path=fullfile(folder,'models',[mdl '.slx']);
        load_system(path); cleanup=onCleanup(@()close_system(mdl,0));
        set_param(mdl,'SimulationCommand','update');
        [A,~,~,~]=linmod(mdl);
        Q=ACS.apply_settings(S.P,kind,S.gains{j}); sys=ACS.closed_loop(Q,kind);
        actual=real(poly(eig(A))); expected=real(poly(pole(sys)));
        assert(numel(actual)==numel(expected));
        assert(norm(actual-expected,inf)/max(1,norm(expected,inf))<1e-7);
        clear cleanup;
    end
end
fprintf('Все 12 моделей Simulink совпадают с блочным расчётом.\n');
end
