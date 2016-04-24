function [x_hat_plus,P_plus,res,S] = EKF_update_dist_bear(x_hat_min, P_min, z_g1,z_g2,LM)

sigma_theta = 0.01;
sigma_d = 0.01;
N = size(LM,1);
H = zeros(2*N,3);
res = zeros(2*N,1);
R = zeros(2*N,2*N);

for i=1:N
        z_hat = sqrt((LM(i,1)-x_hat_min(1,1))^2+(LM(i,2)-x_hat_min(2,1))^2);
        res(i,1) = z_g1(i,1) - z_hat;
        H(i,1) = (-LM(i,1)+x_hat_min(1,1))/z_hat;
        H(i,2) = (-LM(i,2)+x_hat_min(2,1))/z_hat;
        R(i,i) = sigma_d^2;
        z_hat1 = atan((x_hat_min(2,1)-LM(i,2))/(x_hat_min(1,1)-LM(i,1)))-x_hat_min(3,1);
        den = (LM(i,1)-x_hat_min(1,1))^2+(LM(i,2)-x_hat_min(2,1))^2;
        res(i+N,1) = z_g2(i,1) - z_hat1;
        H(i+N,1) = -(x_hat_min(2,1)-LM(i,2))/den;
        H(i+N,2) = (x_hat_min(1,1)-LM(i,1))/den;
        H(i+N,3) = -1;
        R(i+N,i+N) = sigma_theta^2;
end

S = H*P_min*H'+R;
K  = P_min*H'*inv(S);
x_hat_plus = x_hat_min+K*res;
P_plus = P_min - P_min*H'*inv(S)*H*P_min;