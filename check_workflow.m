function results=check_workflow()
% Численные проверки полиномов, критерия и ограниченного синтеза.
P=lr1.default_config(); rng(6);
kinds={'static_ny','astatic_ny','static_pitch','astatic_pitch'};
assert(lr1.hurwitz_test(poly([-1 -2 -3 -4])).stable);
assert(~lr1.hurwitz_test(poly([-1 -2 1])).stable);
assert(~lr1.hurwitz_test([1 0 1]).stable);
for i=1:4
    kind=kinds{i}; g=lr1.gain_settings(P,kind);
    for om=5:5:30
        P.actuator.Omega=om;
        [p,n]=lr1.characteristic(P,kind,g);
        Q=lr1.apply_settings(P,kind,g); sys=lr1.closed_loop(Q,kind);
        out=1; if endsWith(kind,'pitch'), out=3; end
        w=logspace(-2,2,41);
        a=squeeze(freqresp(sys(out,1),w)); b=squeeze(freqresp(tf(n,p),w));
        assert(max(abs(a-b))<1e-7,'Несовпадение с блочной моделью.');
        for j=1:15
            q=g;
            for key={'Kn','Kw','Ki','Ktheta'}, q.(key{1})=q.(key{1})*(.1+4*rand); end
            pp=lr1.characteristic(P,kind,q); r=roots(pp);
            if min(abs(real(r)))>1e-6
                assert(lr1.hurwitz_test(pp).stable==all(real(r)<0),'Гурвиц и корни расходятся.');
            end
        end
    end
end
P.actuator.Omega=30;
results=struct();
for i=1:4
    kind=kinds{i}; R=lr1.refine_design(P,kind);
    assert(R.iterations<=P.refinement.max_iterations);
    assert(R.evaluations<=P.refinement.max_evaluations);
    assert(R.metrics.stable,'Не получена устойчивая настройка.');
    fprintf('%s  OS=%.3f%%  Ts=%.3f s  iter=%d eval=%d time=%.1f s\n', ...
        kind,R.metrics.overshoot,R.metrics.settling,R.iterations,R.evaluations,R.seconds);
    disp(R.refined); disp(R.reason);
    results.(kind)=R;
end
fprintf('Полиномы, блочные модели, Гурвиц и ограничения проверены.\n');
end
