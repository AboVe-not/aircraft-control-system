function bounds = map_domain(P,kind,gains)
% Расширяем область обзора до внешних границ, не замыкая их искусственно.
% На карте допустимы отрицательные K. Поиск настроек остаётся положительным.
bounds=[-20 16 -3 12]; stableX=[]; stableY=[];
gx=cellfun(@(g)g.Kn,gains); gy=cellfun(@(g)g.Kw,gains);
for attempt=1:5
    x=unique([linspace(bounds(1),bounds(2),35),gx]);
    y=unique([linspace(bounds(3),bounds(4),35),gy]);
    [X,Y]=meshgrid(x,y); mask=false(size(X));
    for j=1:numel(gains)
        S=lr1.stability_map(P,kind,gains{j},x,y); mask=mask | S.stable;
    end
    stableX=X(mask); stableY=Y(mask);
    touches=[any(mask(:,1)),any(mask(:,end)),any(mask(1,:)),any(mask(end,:))];
    if ~any(touches), break; end
    span=[bounds(2)-bounds(1),bounds(4)-bounds(3)];
    bounds=bounds+[-span(1) span(1) -span(2) span(2)].*touches;
end
% Сохраняем запас вокруг найденной области и все точки сравнения.
if ~isempty(stableX) && ~any(touches)
    bx=[min([stableX(:);gx(:)]),max([stableX(:);gx(:)])];
    by=[min([stableY(:);gy(:)]),max([stableY(:);gy(:)])];
    sx=max(2,diff(bx)); sy=max(1,diff(by));
    bounds=[bx+[-.25 .25]*sx,by+[-.25 .25]*sy];
end
% Повторная проверка после сужения обзора. Узкий выступ области мог
% не попасть в грубую сетку и не должен обрезаться рамкой.
for attempt=1:4
    x=linspace(bounds(1),bounds(2),101); y=linspace(bounds(3),bounds(4),81);
    mask=false(numel(y),numel(x));
    for j=1:numel(gains)
        S=lr1.stability_map(P,kind,gains{j},x,y); mask=mask | S.stable;
    end
    touches=[any(mask(:,1)),any(mask(:,end)),any(mask(1,:)),any(mask(end,:))];
    if ~any(touches), break; end
    span=[bounds(2)-bounds(1),bounds(4)-bounds(3)];
    bounds=bounds+.4*[-span(1) span(1) -span(2) span(2)].*touches;
end
end
