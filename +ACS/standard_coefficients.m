function [G, info] = standard_coefficients(P, kind, desired_poles)
% Подбор по равенству коэффициентов полиномов, идеальный привод.
% desired_poles задаёт стандартный полином, а не частоту привода.
% Для тангажа подбираются также внутренние Kn и Kw данного контура.
validateattributes(desired_poles,{'numeric'},{'vector','real','finite','nonempty'});
assert(all(desired_poles<0),'Заданные вещественные полюса должны быть отрицательными.');
D = [1 P.c1+P.c4+P.c5 P.c2+P.c1*P.c4];
N = P.c6_over_g*[P.c9 P.c9*(P.c1+P.c5) -P.c3*P.c4+P.c2*P.c9];
W = [-(P.c3-P.c5*P.c9) -(P.c3*P.c4-P.c2*P.c9)];
switch kind
    case 'static_ny'
        base = D; terms = {N,W};
    case 'astatic_ny'
        base = [D 0]; terms = {[N 0],[W 0],N};
    case 'static_pitch'
        base = [D 0]; terms = {[N 0],[W 0],W};
        % Последняя неизвестная L = Kn*Ktheta.
    case 'astatic_pitch'
        base = [D 0 0]; terms = {[N 0 0],[W 0 0],[N 0],W};
        % Последняя неизвестная L = Ki*Ktheta.
    otherwise
        error('mode:Kind','Неизвестная система %s.',kind);
end
n = numel(base)-1;
assert(numel(desired_poles)==n,'Число заданных полюсов должно равняться порядку модели.');
target = poly(desired_poles);
F = zeros(n+1,numel(terms));
for j = 1:numel(terms)
    F(end-numel(terms{j})+1:end,j) = -terms{j}(:);
end
% p = base + F*z. Старший коэффициент зависит от Kn из-за c9.
% Условия p(j) = p(1)*target(j) дают линейную систему H*z = rhs.
H = F(2:end,:) - target(2:end).'*F(1,:);
rhs = base(1)*target(2:end).' - base(2:end).';
assert(rcond(H)>1e-12,'Условия подбора вырождены. Измените заданные полюса.');
z = H\rhs;
G = struct('Kn',z(1),'Kw',z(2));
if startsWith(kind,'astatic_'), G.Ki = z(3); end
if strcmp(kind,'static_pitch')
    assert(abs(G.Kn)>1e-10,'Нельзя определить Ktheta при Kn=0.');
    G.Ktheta = z(3)/G.Kn;
elseif strcmp(kind,'astatic_pitch')
    assert(abs(G.Ki)>1e-10,'Нельзя определить Ktheta при Ki=0.');
    G.Ktheta = z(4)/G.Ki;
end
assert(all(structfun(@(v)isfinite(v)&&v>0,G)), ...
    'Заданный полином не реализуется положительными коэффициентами. Измените заданные полюса.');
p = base + (F*z).';
assert(abs(p(1))>1e-8,'Вырожденный старший коэффициент.');
actual = p/p(1);
info.target = target;
info.actual = actual;
info.desired_poles = desired_poles;
info.relative_error = norm(actual-target,inf)/max(1,norm(target,inf));
assert(info.relative_error<1e-10,'Не выполнено равенство коэффициентов.');
info.model = 'Идеальный привод. Полная модель проверяется отдельно.';
end
