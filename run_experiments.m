% Прежние пакетные факторные эксперименты, без численного уточнения K.
clear; clc;
root = fileparts(mfilename('fullpath'));
addpath(root);
P = lr1.default_config();
out = fullfile(root,'results');
models = fullfile(root,'models_standard');
if ~exist(out,'dir'), mkdir(out); end

fprintf('Variant %d, H=%.1f km, V=%.0f km/h\n',P.variant,P.H_km,P.V_kmh);
fprintf('Aircraft omega_n=%.4f rad/s, zeta=%.4f\n', ...
    P.omega_aircraft,P.zeta_aircraft);
fprintf('Стандартные коэффициенты контура ny, Kn=%.4f, Kw=%.4f. Для тангажа Ktheta=%.4f\n', ...
    P.static.Kn,P.static.Kw,P.pitch_static.Ktheta);
fprintf('Assumed actuator Omega=%.4f rad/s, zeta=%.4f\n', ...
    P.actuator.Omega,P.actuator.zeta);

fid=fopen(fullfile(out,'parameters.txt'),'w');
fprintf(fid,'Variant %d. H=%.1f km, V=%.0f km/h.\n', ...
    P.variant,P.H_km,P.V_kmh);
fprintf(fid,'Aircraft natural frequency %.6f rad/s, damping %.6f.\n', ...
    P.omega_aircraft,P.zeta_aircraft);
fprintf(fid,'Синтез с идеальным приводом. Контур ny, Kn %.6f, Kw %.6f. Для тангажа Ktheta %.6f.\n', ...
    P.static.Kn,P.static.Kw,P.pitch_static.Ktheta);
fprintf(fid,'Actuator values are assumptions, not source data. Omega %.6f rad/s, zeta %.6f.\n', ...
    P.actuator.Omega,P.actuator.zeta);
for design_group = {'static','astatic','pitch_static','pitch_astatic'}
    group_name = design_group{1};
    fprintf(fid,'\n%s\n',group_name);
    group_fields = fieldnames(P.(group_name));
    for design_index = 1:numel(group_fields)
        field_name = group_fields{design_index};
        fprintf(fid,'%s = %.9g\n',field_name,P.(group_name).(field_name));
    end
end
fclose(fid);

% Control System Toolbox models preserve the exact elevator lift term c9.
ny_sys = lr1.closed_loop(P,'static_ny');
pitch_sys = lr1.closed_loop(P,'static_pitch');
% Theta is a free kinematic integrator in an ny-only loop. Judge the ny
% dynamics after removing that unobservable/uncontrolled pole at zero.
assert(isstable(minreal(ny_sys(1,:),1e-7)), ...
    'The nominal static ny dynamics are unstable.');
assert(isstable(pitch_sys),'The nominal pitch system is unstable.');
fprintf('Static ny poles\n'); disp(pole(ny_sys));
fprintf('Static pitch poles\n'); disp(pole(pitch_sys));

t=(0:P.dt:P.t_end)';
local_case(ny_sys,t,[P.command.ny,0,0],1, ...
    'Static loop, ny command',fullfile(out,'static_ny_reference.png'));
local_case(ny_sys,t,[0,0,P.command.moment],1, ...
    'Static loop, constant moment',fullfile(out,'static_ny_moment.png'));
local_case(ny_sys,t,[0,P.command.gust,0],1, ...
    'Static loop, vertical gust',fullfile(out,'static_ny_gust.png'));
local_case(pitch_sys,t,[P.command.theta,0,0],3, ...
    'Pitch loop, theta command',fullfile(out,'pitch_reference.png'));
local_case(pitch_sys,t,[0,0,P.command.moment],3, ...
    'Pitch loop, constant moment',fullfile(out,'pitch_moment.png'));
local_case(pitch_sys,t,[0,P.command.gust,0],3, ...
    'Pitch loop, vertical gust',fullfile(out,'pitch_gust.png'));

% Compare the ideal and dynamic actuators with every other parameter fixed.
P_ideal=P; P_ideal.actuator.dynamic=false;
sys_ideal=lr1.closed_loop(P_ideal,'static_ny');
u=[ones(size(t))*P.command.ny,zeros(numel(t),2)];
y1=lsim(sys_ideal,u,t); y2=lsim(ny_sys,u,t);
f=figure('Visible','off');
plot(t,y1(:,1),'LineWidth',1.4); hold on;
plot(t,y2(:,1),'LineWidth',1.4); grid on;
xlabel('t, s'); ylabel('n_y, g');
legend('Ideal actuator','Second-order actuator','Location','best');
title('Actuator dynamics');
exportgraphics(f,fullfile(out,'actuator_comparison.png')); close(f);

local_gain_sweep(P,t,out,'Kw',P.sweep.Kw);
local_gain_sweep(P,t,out,'Kn',P.sweep.Kn);
local_gain_sweep(P,t,out,'Ktheta',P.sweep.Ktheta);

% Stability regions in (Kn,Kw) with one parameter varied at a time.
local_regions(P,out,'c2',P.sweep.c2);
local_regions(P,out,'c1',P.sweep.c1);
local_regions(P,out,'c9',P.sweep.c9);
local_regions(P,out,'Omega',P.sweep.Omega);
local_nominal_region(P,out);
local_metrics(P,ny_sys,pitch_sys,t,out);

if license('test','Simulink')
    created=lr1.build_simulink(P,models);
    fprintf('Simulink models created\n'); disp(created(:));
else
    warning('Simulink is not licensed; .slx models were not created.');
end
fprintf('Results saved in %s\n',out);

function local_case(sys,t,amplitude,output,title_text,filename)
u=ones(numel(t),1)*amplitude;
y=lsim(sys,u,t);
f=figure('Visible','off');
plot(t,y(:,output),'LineWidth',1.5); grid on;
xlabel('t, s');
if output==1, ylabel('n_y, g'); else, ylabel('theta, deg'); end
title(title_text,'Interpreter','none');
exportgraphics(f,filename); close(f);
fprintf('%s: final=%.5f, peak=%.5f\n',title_text,y(end,output),max(abs(y(:,output))));
end

function local_gain_sweep(P,t,out,name,multipliers)
f=figure('Visible','off'); hold on;
labels=cell(numel(multipliers),1);
for i=1:numel(multipliers)
    Q=P;
    if strcmp(name,'Ktheta')
        Q.pitch_static.Ktheta=P.pitch_static.Ktheta*multipliers(i);
        kind='static_pitch'; output=3; amplitude=P.command.theta;
    else
        Q.static.(name)=P.static.(name)*multipliers(i);
        kind='static_ny'; output=1; amplitude=P.command.ny;
    end
    sys=lr1.closed_loop(Q,kind);
    u=[ones(numel(t),1)*amplitude,zeros(numel(t),2)];
    y=lsim(sys,u,t);
    plot(t,y(:,output),'LineWidth',1.2);
    labels{i}=sprintf('%s x %.2g',name,multipliers(i));
end
grid on; xlabel('t, s');
if output==1, ylabel('n_y, g'); else, ylabel('theta, deg'); end
legend(labels,'Location','best'); title([name ' gain variation']);
exportgraphics(f,fullfile(out,['sweep_' name '.png'])); close(f);
end

function local_regions(P,out,field,multipliers)
Kn=linspace(0,4*P.static.Kn,55);
Kw=linspace(0,4*P.static.Kw,55);
f=figure('Visible','off'); hold on;
colors=lines(numel(multipliers));
labels=cell(numel(multipliers),1);
for k=1:numel(multipliers)
    Q=P;
    if strcmp(field,'Omega')
        Q.actuator.Omega=P.actuator.Omega*multipliers(k);
    else
        Q.(field)=P.(field)*multipliers(k);
    end
    R=zeros(numel(Kw),numel(Kn));
    for i=1:numel(Kn)
        for j=1:numel(Kw)
            R(j,i)=max(real(roots(lr1.static_characteristic(Q,Kn(i),Kw(j)))));
        end
    end
    contour(Kn,Kw,R,[0 0],'LineWidth',1.6,'LineColor',colors(k,:));
    labels{k}=sprintf('%s x %.2g',field,multipliers(k));
end
plot(P.static.Kn,P.static.Kw,'ko','MarkerFaceColor','k');
grid on; xlabel('K_{ny}'); ylabel('K_{omega_z}');
title(['Stability boundaries, variation of ' field]);
legend([labels;{'Nominal gains'}],'Location','best');
exportgraphics(f,fullfile(out,['region_' field '.png'])); close(f);
end

function local_nominal_region(P,out)
Kn=linspace(0,4*P.static.Kn,90);
Kw=linspace(0,4*P.static.Kw,90);
R=zeros(numel(Kw),numel(Kn));
for i=1:numel(Kn)
    for j=1:numel(Kw)
        R(j,i)=max(real(roots(lr1.static_characteristic(P,Kn(i),Kw(j)))));
    end
end
f=figure('Visible','off');
imagesc(Kn,Kw,double(R<0)); axis xy; hold on;
contour(Kn,Kw,R,[0 0],'k','LineWidth',1.5);
plot(P.static.Kn,P.static.Kw,'ko','MarkerFaceColor','w');
colormap([0.96 0.74 0.72;0.72 0.88 0.75]);
caxis([0 1]); colorbar('Ticks',[0.25 0.75], ...
    'TickLabels',{'Unstable','Stable'});
xlabel('K_{ny}'); ylabel('K_{omega_z}');
title('Static ny loop stability, nominal aircraft and actuator');
exportgraphics(f,fullfile(out,'region_nominal.png')); close(f);
end

function local_metrics(P,ny_sys,pitch_sys,t,out)
f=fopen(fullfile(out,'metrics.txt'),'w');
fprintf(f,'Коэффициенты рассчитаны равенством стандартных полиномов при идеальном приводе.\n');
fprintf(f,'Для тангажа использованы отдельные внутренние коэффициенты. Проверка ниже с динамическим приводом.\n');
fprintf(f,'Actuator Omega and zeta are assumptions.\n\n');
cases={ny_sys,1,P.command.ny,'ny reference'; ...
       ny_sys,3,P.command.moment,'ny moment'; ...
       ny_sys,2,P.command.gust,'ny gust'; ...
       pitch_sys,1,P.command.theta,'theta reference'; ...
       pitch_sys,3,P.command.moment,'theta moment'; ...
       pitch_sys,2,P.command.gust,'theta gust'};
for k=1:size(cases,1)
    sys=cases{k,1}; input=cases{k,2}; amp=cases{k,3};
    u=zeros(numel(t),3); u(:,input)=amp;
    y=lsim(sys,u,t);
    if k<=3, column=1; else, column=3; end
    z=y(:,column);
    fprintf(f,'%s: final %.6f, min %.6f, max %.6f.\n', ...
        cases{k,4},z(end),min(z),max(z));
    if k==1 || k==4
        s=stepinfo(z,t,z(end));
        fprintf(f,'  2%% settling time %.6f s; overshoot %.6f%%.\n', ...
            s.SettlingTime,s.Overshoot);
        fprintf(f,'  finite-horizon tracking error %.6f.\n',amp-z(end));
    end
end
fclose(f);
end
