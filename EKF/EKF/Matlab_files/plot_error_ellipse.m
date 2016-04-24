% the code is referred from researchnet.iiit.ac.in

function plot_error_ellipse(P_plus,x_k)

xy_cov=P_plus(1:2,1:2);
[b,d]=eig(xy_cov);
dir=[cos(0:0.1:2*pi);sin(0:0.1:2*pi)];
el=b*sqrtm(d)*dir;
x=[x_k(1,1);x_k(2,1)];
el=[el el(:,1)]+repmat(x,1,size(el,2)+1);
plot(el(1,:),el(2,:),'color','r');

