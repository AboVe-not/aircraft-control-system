function R = hurwitz_test(p)
% Критерий Гурвица. Положительное масштабирование не меняет знаков миноров.
p=p(:).';
R=struct('stable',false,'margin',-1,'minors',[],'status','вырождение');
if numel(p)<2 || any(~isfinite(p)) || abs(p(1))<1e-14, return; end
p=p/p(1); n=numel(p)-1;
scale=max(1,abs(p(end))^(1/n));
a=p./scale.^(0:n);
H=zeros(n);
for i=1:n
    for j=1:n
        index=2*j-i;
        if index>=0 && index<=n, H(i,j)=a(index+1); end
    end
end
H=H./max(max(abs(H),[],2),1e-100);
d=zeros(1,n);
for j=1:n, d(j)=det(H(1:j,1:j)); end
R.minors=d;
% Положительность коэффициентов необходима и помогает распознать границу.
R.margin=min([d a/max(1,max(abs(a)))]);
R.stable=R.margin>1e-12;
if R.stable, R.status='устойчива';
elseif R.margin < -1e-12, R.status='неустойчива';
else, R.status='граница / численно близко к границе'; end
end
