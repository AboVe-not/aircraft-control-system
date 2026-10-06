function [M,t,y] = reference_metrics(P,kind,g,dt)
if nargin<4, dt=P.refinement.dt; end
[p,num]=ACS.characteristic(P,kind,g);
H=ACS.hurwitz_test(p);
M=struct('stable',H.stable,'overshoot',Inf,'settling',Inf,'steady',NaN, ...
    'tracking_error',NaN,'ise',Inf,'horizon',P.refinement.horizon,'met',false);
t=[]; y=[];
if ~H.stable, return; end
T=P.refinement.horizon;
t=(0:dt:T).';
% Оценка всегда по реальному DC-значению, не по последней точке графика.
M.steady=num(end)/p(end);
if ~isfinite(M.steady) || M.steady<=1e-8, return; end
y=step(tf(num,p),t); y=y(:);
if any(~isfinite(y)), return; end
z=y/M.steady;
M.overshoot=100*max(0,max(z)-1);
last=find(abs(z-1)>P.refinement.band,1,'last');
if isempty(last), M.settling=0;
elseif last<numel(t), M.settling=t(last+1); end
M.tracking_error=1-M.steady;
M.ise=trapz(t,(z-1).^2);
M.met=M.overshoot<=P.refinement.max_overshoot && M.settling<=P.refinement.target_time;
end
