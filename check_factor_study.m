function check_factor_study()
P=lr1.default_config(); original=P; S=lr1.static_factor_study(P);
assert(isequal(P,original));
assert(isequal(S.panels(4).values,[10 15 20]));
assert(S.config.actuator.Omega==20 && S.config.actuator.zeta==.8);
rng(6);
for i=1:4
    panel=S.panels(i);
    for j=1:3
        Q=S.config;
        if strcmp(panel.field,'Omega'), Q.actuator.Omega=panel.values(j);
        else, Q.(panel.field)=panel.values(j); end
        m=panel.maps{j};
        for k=1:100
            ix=randi(numel(m.x)); iy=randi(numel(m.y));
            g=S.gains; g.Kn=m.x(ix); g.Kw=m.y(iy);
            p=lr1.characteristic(Q,'static_ny',g);
            if abs(m.margin(iy,ix))>1e-8
                H=lr1.hurwitz_test(p);
                assert(m.stable(iy,ix)==H.stable);
                assert(m.stable(iy,ix)==all(real(roots(p))<0));
            end
        end
    end
end
fprintf('12 карт проверены по Гурвицу и корням, 1200 контрольных точек.\n');
end
