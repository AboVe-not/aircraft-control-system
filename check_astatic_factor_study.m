function check_astatic_factor_study()
P=lr1.default_config(); original=P; rng(6);
for Ki=[P.reference.astatic_ny.Ki P.astatic.Ki]
    S=lr1.astatic_factor_study(P,Ki);
    assert(isequal(P,original)); assert(numel(S.panels)==5);
    assert(isequal(S.panels(5).values,Ki*[.5 1 1.5]));
    for i=1:5
        panel=S.panels(i);
        for j=1:3
            Q=S.config; g=S.gains;
            if strcmp(panel.field,'Omega'), Q.actuator.Omega=panel.values(j);
            elseif strcmp(panel.field,'Ki'), g.Ki=panel.values(j);
            else, Q.(panel.field)=panel.values(j); end
            m=panel.maps{j};
            for k=1:100
                ix=randi(numel(m.x)); iy=randi(numel(m.y));
                g.Kn=m.x(ix); g.Kw=m.y(iy);
                p=lr1.characteristic(Q,'astatic_ny',g);
                if abs(m.margin(iy,ix))>1e-8
                    H=lr1.hurwitz_test(p);
                    assert(m.stable(iy,ix)==H.stable);
                    assert(m.stable(iy,ix)==all(real(roots(p))<0));
                end
            end
        end
    end
end
fprintf('Астатический контур, 3000 контрольных точек, проверка пройдена.\n');
end
