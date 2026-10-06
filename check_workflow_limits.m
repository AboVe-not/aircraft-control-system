function check_workflow_limits()
P=lr1.default_config(); kinds={'static_ny','astatic_ny','static_pitch','astatic_pitch'};
for om=5:5:30
    P.actuator.Omega=om;
    for j=1:4
        R=lr1.refine_design(P,kinds{j});
        assert(R.iterations<=P.refinement.max_iterations && R.evaluations<=P.refinement.max_evaluations);
        if R.metrics.stable
            assert(all(real(roots(lr1.characteristic(P,kinds{j},R.refined)))<0));
        end
        fprintf('Omega=%g %s stable=%d target=%d OS=%.2f Ts=%.3f iter=%d\n', ...
            om,kinds{j},R.metrics.stable,R.metrics.met,R.metrics.overshoot,R.metrics.settling,R.iterations);
    end
end
P.refinement.max_iterations=1; P.refinement.max_evaluations=5;
A=lr1.refine_design(P,'static_pitch'); assert(A.iterations<=1 && A.evaluations<=5);
P.reference=struct(); B=lr1.refine_design(P,'static_pitch');
assert(isequal(A.refined,B.refined),'Поиск зависит от эталона.');
P.refinement.max_iterations=200; P.refinement.max_evaluations=1000;
C=lr1.refine_design(P,'static_pitch',@(~)true);
assert(contains(C.reason,'пользователем'),'Не работает остановка.');
fprintf('Лимиты, независимость от эталона и остановка проверены.\n');
end
