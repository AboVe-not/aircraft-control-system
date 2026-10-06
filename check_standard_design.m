function check_standard_design()
% Проверка синтеза и независимая проверка модели через closed_loop.
root = fileparts(mfilename('fullpath')); addpath(root);
P = ACS.default_config();
kinds = {'static_ny','astatic_ny','static_pitch','astatic_pitch'};
groups = {'static','astatic','pitch_static','pitch_astatic'};
for j=1:4
    kind = kinds{j}; info = P.design.result.(kind);
    fprintf('\n%s\n',kind); disp(P.(groups{j}));
    fprintf('Заданный полином\n'); disp(info.target);
    fprintf('Полученный полином\n'); disp(info.actual);
    Q=P; Q.actuator.dynamic=false;
    sys=ACS.closed_loop(Q,kind);
    % В контуре ny исключаем свободный интегратор theta, не входящий в ОС.
    if endsWith(kind,'_ny'), sys=minreal(sys(1:2,:),1e-9); end
    actual=real(poly(pole(sys)));
    assert(numel(actual)==numel(info.target),'Не совпал порядок модели.');
    err=norm(actual-info.target,inf)/max(1,norm(info.target,inf));
    assert(err<1e-8,'Полином модели не совпал со стандартным.');
    assert(isstable(sys),'Идеальная система неустойчива.');
    Q.actuator.dynamic=true;
    sys=ACS.closed_loop(Q,kind);
    if endsWith(kind,'_ny'), sys=minreal(sys(1:2,:),1e-9); end
    assert(isstable(sys),'Динамический привод нарушил устойчивость.');
    fprintf('Ошибка равенства %.3g. С приводом max Re(pole) = %.6g\n', ...
        err,max(real(pole(sys))));
end
% Проверка известного статического характеристического полинома.
Q=P; Q.actuator.dynamic=false;
p=ACS.static_characteristic(Q,P.static.Kn,P.static.Kw);
assert(norm(p/p(1)-P.design.result.static_ny.target,inf)<1e-8);
% Недопустимый запрос не должен тихо возвращать отрицательные усиления.
rejected=false;
try
    ACS.standard_coefficients(P,'static_ny',[-1 -1]);
catch
    rejected=true;
end
assert(rejected,'Не отклонены недопустимые коэффициенты.');
fprintf('\nВсе проверки пройдены.\n');
end
