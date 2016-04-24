function [x_hat_plus,P_plus,res,S] = EKF_update_bear(x_hat_min, P_min, z_g,LM)

% Initializing all values
sigma_theta = 0.01;
N = size(LM,1);
H = zeros(N,3);
res = zeros(N,1);
% R = sigma_d^2;

for i=1:N
    z_hat = (atan((x_hat_min(2,1)-LM(i,2))/(x_hat_min(1,1)-LM(i,1))))-x_hat_min(3,1);
    den = (LM(i,1)-x_hat_min(1,1))^2+(LM(i,2)-x_hat_min(2,1))^2;
    res(i,1) = z_g(i,1) - z_hat;
    H(i,1) = -(x_hat_min(2,1)-LM(i,2))/den;
    H(i,2) = (x_hat_min(1,1)-LM(i,1))/den;
    H(i,3) = -1;
    R(i,i) = sigma_theta^2;
%     S = H*P_min*H'+R;
%     K  = P_min*H'*inv(S);
%     x_hat_plus = x_hat_min+K*res;
% %     P_plus = P_min - P_min*H'*inv(S)*H*P_min;
%      P_plus = (eye(3)-K*H)*P_min*(eye(3)-K*H)'+K*R*K';

end

S = H*P_min*H'+R;
K  = P_min*H'*inv(S);
x_hat_plus = x_hat_min+K*res;
P_plus = (eye(3)-K*H)*P_min*(eye(3)-K*H)'+K*R*K';
% P_plus = P_min - P_min*H'*inv(S)*H*P_min;

