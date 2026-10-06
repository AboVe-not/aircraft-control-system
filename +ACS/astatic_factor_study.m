function study=astatic_factor_study(P,Ki)
% Пять однофакторных серий. Общий Ki для всех сравнений Kn, Kw.
if nargin<2, Ki=P.reference.astatic_ny.Ki; end
validateattributes(Ki,{'numeric'},{'scalar','real','finite','positive'});
P.actuator.dynamic=true; P.actuator.Omega=20; P.actuator.zeta=.8;
g=ACS.gain_settings(P,'astatic_ny'); ref=P.reference.astatic_ny;
g.Ki=Ki; ref.Ki=Ki;
fields={'c2','c1','c9','Omega','Ki'};
values={P.c2*[.5 1 1.5],P.c1*[.5 1 1.5],P.c9*[0 1 2],[10 15 20],Ki*[.5 1 1.5]};
study=struct('config',P,'gains',g,'reference',ref,'fixed_Ki',Ki,'panels',[]);
for i=1:5
    models=cell(1,3); settings=cell(1,3);
    for j=1:3
        Q=P; q=g;
        if strcmp(fields{i},'Omega'), Q.actuator.Omega=values{i}(j);
        elseif strcmp(fields{i},'Ki'), q.Ki=values{i}(j);
        else, Q.(fields{i})=values{i}(j); end
        models{j}=Q; settings{j}=q;
    end
    bounds=[-25 15 -3 12];
    for attempt=1:5
        x=linspace(bounds(1),bounds(2),161); y=linspace(bounds(3),bounds(4),141);
        edges=false(1,4);
        for j=1:3
            S=local_map(models{j},settings{j},x,y); m=S.stable;
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
        q=settings{j}; r=ref; r.Ki=q.Ki;
        panel.maps{j}=local_map(models{j},q,x,y);
        panel.initial_stable(j)=ACS.hurwitz_test(ACS.characteristic(models{j},'astatic_ny',q)).stable;
        panel.reference_stable(j)=ACS.hurwitz_test(ACS.characteristic(models{j},'astatic_ny',r)).stable;
    end
    study.panels=[study.panels panel]; %#ok<AGROW>
end
end

function S=local_map(P,g,x,y)
% Полный полином пятого порядка, включая интегратор и динамический привод.
[X,Y]=meshgrid(x,y); q=g; q.Kn=0; q.Kw=0;
p0=ACS.characteristic(P,'astatic_ny',q);
q.Kn=1; pn=ACS.characteristic(P,'astatic_ny',q)-p0;
q.Kn=0; q.Kw=1; pw=ACS.characteristic(P,'astatic_ny',q)-p0;
a=cell(1,5);
scale=max(1,abs(p0(end))^(1/5));
for k=1:5, a{k}=(p0(k+1)+pn(k+1)*X+pw(k+1)*Y)/scale^k; end
h2=a{1}.*a{2}-a{3};
h3=a{3}.*h2-a{1}.*(a{1}.*a{4}-a{5});
h4=h2.*(a{3}.*a{4}-a{2}.*a{5})-(a{1}.*a{4}-a{5}).^2;
Z=ones(size(X));
for k=1:5, Z=min(Z,a{k}./(1+abs(a{k}))); end
Z=min(Z,h2./(1+abs(a{1}.*a{2})+abs(a{3})));
Z=min(Z,h3./(1+abs(a{3}.*h2)+abs(a{1}.*(a{1}.*a{4}-a{5}))));
Z=min(Z,h4./(1+abs(h2.*(a{3}.*a{4}-a{2}.*a{5}))+(a{1}.*a{4}-a{5}).^2));
% H1=a1, H5=a5*H4. Положительные коэффициенты и H2,H3,H4 достаточны.
m=Z>1e-12;
S=struct('x',x,'y',y,'margin',Z,'stable',m, ...
    'touches_edge',any([m(1,:) m(end,:) m(:,1).' m(:,end).']));
end
