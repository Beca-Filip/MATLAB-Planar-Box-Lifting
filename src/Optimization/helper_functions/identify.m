function [X sxr r]=identify(W,Y)
reg=size(W);
np=reg(2);%number of parameter to estimate=number of columns of WSt
% X=pinv(W)*Y;
X=W\Y;
% X=inv(pinv(W)'*W')Y;
ec=1;
r = Y - WX;
ecf = r'r + ec ;
normb = sqrt(ecf);
normy = sqrt(Y'Y + ec) ;
sr = normb/sqrt(length(Y)-np);
[uw,sw,vw] = svd(W,0);
sx = srsqrt(diag(vwdiag(1./(diag(sw).*diag(sw)))vw'));
sxr = 100abs(sx./X);