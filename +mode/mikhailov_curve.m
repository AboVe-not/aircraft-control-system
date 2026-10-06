function [z,w] = mikhailov_curve(p)
% Положительное радиальное сжатие сохраняет аргумент годографа.
% Это не исходный масштаб D(jw), что явно отмечается на графике.
r=roots(p); scale=max([1;abs(r(:))]);
w=[0 logspace(-4,log10(100*scale),900)];
d=polyval(p,1i*w)/max(1,abs(p(end)));
z=d./(1+abs(d));
end
