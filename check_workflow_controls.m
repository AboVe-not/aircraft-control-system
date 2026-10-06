function check_workflow_controls()
% Проверка обработчиков UI без ручного ввода и изменения пользовательских MAT.
root=fileparts(mfilename('fullpath')); addpath(root);
f=interactive_mode('off'); cleanup=onCleanup(@()delete(f));
mode=findall(f,'Tag','mode');
for i=1:4
    mode.Value=mode.ItemsData{i}; mode.ValueChangedFcn(mode,[]);
    S=getappdata(f,'workflow'); assert(~isempty(S.result));
    assert(size(S.metrics,1)==4);
    slider=findall(f,'Tag','Kn'); slider.Value=slider.Value*1.01;
    slider.ValueChangedFcn(slider,struct('Value',slider.Value));
    S=getappdata(f,'workflow'); assert(abs(S.candidate.Kn-slider.Value)<1e-12);
    assert(numel(findall(f,'Tag','corridor'))==3,'Накопление коридоров.');
    fprintf('Окно %s проверено.\n',S.kind);
end
exportapp(f,'/private/tmp/lr1_workflow_pitch.png');
frequency=findall(f,'Tag','frequency'); frequency.Value=5; frequency.ValueChangedFcn(frequency,[]);
S=getappdata(f,'workflow'); assert(S.P.actuator.Omega==5);
damping=findall(f,'Tag','damping'); damping.Value=.9; damping.ValueChangedFcn(damping,[]);
S=getappdata(f,'workflow'); assert(S.P.actuator.zeta==.9);
fprintf('Ползунки, частота, демпфирование, четыре режима и коридоры проверены.\n');
end
