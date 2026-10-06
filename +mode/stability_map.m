function S = stability_map(P,kind,g,x,y)
% Сечение пространства коэффициентов при фиксированных Ki и Ktheta.
% Ноль запаса Гурвица отделяет устойчивые точки от остальных.
[X,Y]=meshgrid(x,y); Z=zeros(size(X));
for j=1:numel(X)
    q=g; q.Kn=X(j); q.Kw=Y(j);
    H=lr1.hurwitz_test(lr1.characteristic(P,kind,q)); Z(j)=H.margin;
end
S=struct('x',x,'y',y,'margin',Z,'stable',Z>1e-12, ...
    'touches_edge',any([Z(1,:) Z(end,:) Z(:,1).' Z(:,end).']>1e-12));
end
