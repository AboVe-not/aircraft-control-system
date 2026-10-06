function R=run_mode_workflow(kind,Omega,output_dir)
% Пакетный расчёт только выбранной системы и сохранение трёх моделей.
% Пример  run_mode_workflow('astatic_pitch',30)
root=fileparts(mfilename('fullpath')); addpath(root);
if nargin<1, kind='static_ny'; end
P=lr1.default_config();
if nargin>=2
    assert(ismember(Omega,5:5:30),'Выберите частоту из 5, 10, 15, 20, 25, 30 рад/с.');
    P.actuator.Omega=Omega;
end
if nargin<3, output_dir=fullfile(root,'results_workflow',kind); end
if ~exist(output_dir,'dir'), mkdir(output_dir); end
R=lr1.refine_design(P,kind);
fprintf('Система %s. Частота привода %.1f рад/с, демпфирование %.2f\n',kind,P.actuator.Omega,P.actuator.zeta);
fprintf('%s. Итераций %d, вычислений %d\n',R.reason,R.iterations,R.evaluations);
gains={R.initial,R.refined,P.reference.(kind)};
names={'Начальная СК','Уточнённая','Методичка'};
suffixes={'initial','refined','reference'};
metrics=cell(3,7);
for j=1:3
    M=lr1.reference_metrics(P,kind,gains{j},P.refinement.validation_dt);
    H=lr1.hurwitz_test(lr1.characteristic(P,kind,gains{j}));
    fprintf('%s. %s. Перерегулирование %.3f%%, время %.3f с, y∞ %.4f\n', ...
        names{j},H.status,M.overshoot,M.settling,M.steady);
    disp(gains{j});
    metrics(j,:)={names{j},H.status,M.overshoot,M.settling,M.steady,M.tracking_error,M.met};
    Q=lr1.apply_settings(P,kind,gains{j});
    lr1.build_simulink(Q,fullfile(output_dir,'models'),{kind},'.slx',true,['_' suffixes{j}]);
end
save(fullfile(output_dir,'calculation.mat'),'P','kind','R','gains','names','metrics');
fprintf('Параметры и модели сохранены в %s\n',output_dir);
end
