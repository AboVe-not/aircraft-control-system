function study=static_factor_study(P)
% Изолированные однофакторные эксперименты статического контура ny.
% Параметры P на выходе не изменяются, повторного синтеза K нет.
P.actuator.dynamic=true; P.actuator.Omega=20; P.actuator.zeta=.8;
g=ACS.gain_settings(P,'static_ny'); ref=P.reference.static_ny;
fields={'c2','c1','c9','Omega'};
values={P.c2*[.5 1 1.5],P.c1*[.5 1 1.5],P.c9*[0 1 2],[10 15 20]};
study=struct('config',P,'gains',g,'reference',ref,'panels',[]);
for i=1:4
    models=cell(1,3);
    for j=1:3
        Q=P;
        if strcmp(fields{i},'Omega'), Q.actuator.Omega=values{i}(j);
        else, Q.(fields{i})=values{i}(j); end
        models{j}=Q;
    end
    bounds=[-25 15 -3 12];
    for attempt=1:5
        x=linspace(bounds(1),bounds(2),161); y=linspace(bounds(3),bounds(4),141);
        edges=false(1,4);
        for j=1:3
            S=local_map(models{j},g,x,y);
            m=S.stable;
            edges=edges | [any(m(:,1)),any(m(:,end)),any(m(1,:)),any(m(end,:))];
        end
        if ~any(edges), break; end
        if attempt<5
            span=[diff(bounds(1:2)) diff(bounds(3:4))];
            bounds=bounds+.5*[-span(1) span(1) -span(2) span(2)].*edges;
        end
    end
    panel=struct('field',fields{i},'values',values{i},'bounds',bounds,'maps',{{}}, ...
        'initial_stable',false(1,3),'reference_stable',false(1,3));
    x=linspace(bounds(1),bounds(2),401); y=linspace(bounds(3),bounds(4),321);
    for j=1:3
        panel.maps{j}=local_map(models{j},g,x,y);
        panel.initial_stable(j)=ACS.hurwitz_test(ACS.characteristic(models{j},'static_ny',g)).stable;
        panel.reference_stable(j)=ACS.hurwitz_test(ACS.characteristic(models{j},'static_ny',ref)).stable;
    end
    study.panels=[study.panels panel]; %#ok<AGROW>
end
end

function S=local_map(P,g,x,y)
% Векторизованный критерий Гурвица для полинома четвёртого порядка.
% p=s^4+a1*s^3+a2*s^2+a3*s+a4. При положительных коэффициентах
% достаточно H2=a1*a2-a3>0 и H3=H2*a3-a1^2*a4>0.
[X,Y]=meshgrid(x,y); q=g; q.Kn=0; q.Kw=0;
p0=ACS.characteristic(P,'static_ny',q);
q.Kn=1; pn=ACS.characteristic(P,'static_ny',q)-p0;
q.Kn=0; q.Kw=1; pw=ACS.characteristic(P,'static_ny',q)-p0;
a=cell(1,4);
for k=1:4, a{k}=p0(k+1)+pn(k+1)*X+pw(k+1)*Y; end
h2=a{1}.*a{2}-a{3}; h3=h2.*a{3}-a{1}.^2.*a{4};
Z=ones(size(X));
for k=1:4, Z=min(Z,a{k}./(1+abs(a{k}))); end
Z=min(Z,h2./(1+abs(a{1}.*a{2})+abs(a{3})));
Z=min(Z,h3./(1+abs(h2.*a{3})+abs(a{1}.^2.*a{4})));
m=Z>1e-12;
S=struct('x',x,'y',y,'margin',Z,'stable',m, ...
    'touches_edge',any([m(1,:) m(end,:) m(:,1).' m(:,end).']));
end
