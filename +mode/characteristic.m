function [p,num] = characteristic(P,kind,g)
% Полином полного контура и числитель канала задания.
% Не содержит свободный интегратор theta в системах только по ny.
D=[1 P.c1+P.c4+P.c5 P.c2+P.c1*P.c4];
N=P.c6_over_g*[P.c9 P.c9*(P.c1+P.c5) -P.c3*P.c4+P.c2*P.c9];
W=[-(P.c3-P.c5*P.c9) -(P.c3*P.c4-P.c2*P.c9)];
if P.actuator.dynamic
    A=[1 2*P.actuator.zeta*P.actuator.Omega P.actuator.Omega^2];
    b=P.actuator.Omega^2;
else
    A=1; b=1;
end
B=conv(D,A);
switch kind
    case 'static_ny'
        p=B; terms={g.Kn*b*N,g.Kw*b*W}; num=-g.Kn*b*N;
    case 'astatic_ny'
        p=[B 0]; terms={g.Kn*b*[N 0],g.Kw*b*[W 0],g.Ki*b*N}; num=-g.Ki*b*N;
    case 'static_pitch'
        p=[B 0]; terms={g.Kn*b*[N 0],g.Kw*b*[W 0],g.Kn*g.Ktheta*b*W};
        num=-g.Kn*g.Ktheta*b*W;
    case 'astatic_pitch'
        p=[B 0 0]; terms={g.Kn*b*[N 0 0],g.Kw*b*[W 0 0], ...
            g.Ki*b*[N 0],g.Ki*g.Ktheta*b*W}; num=-g.Ki*g.Ktheta*b*W;
    otherwise, error('Неизвестная система.');
end
for j=1:numel(terms)
    t=terms{j}; p(end-numel(t)+1:end)=p(end-numel(t)+1:end)-t;
end
end
