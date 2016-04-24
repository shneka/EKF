function[x_hat_min,P_min] =EKF_propagate(p_plus,sigma_g,x_k,v_m,omega_m,dt)
%Propogate step in extended Kalman Filter 
P_k = eye(3);
P_k(1,3) = -v_m*dt*sin(x_k(3,1));
P_k(2,3) = v_m*dt*cos(x_k(3,1));

G_k = zeros(3,2);
G_k(1,1) = dt*cos(x_k(3,1));
G_k(2,1) = dt*sin(x_k(3,1));
G_k(3,2) = dt;

P_min = (P_k*p_plus*P_k')+(G_k*sigma_g*G_k');
x_hat_min(1,1) = x_k(1,1)+(v_m*dt*cos(x_k(3,1)));
x_hat_min(2,1) = x_k(2,1)+(v_m*dt*sin(x_k(3,1)));
x_hat_min(3,1) = x_k(3,1)+(omega_m*dt);
end