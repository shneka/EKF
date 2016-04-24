% when three landmarks are given
clear all;
close all;
clc;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Configuration settings

% change input to run different function
% 1 - Distance only measurement
% 2 - Bearing only measurement
% 3 - For distance and bearing measurement 

input = 3;

N = 150; % number of timesteps
dt = 1; % sampling time

% Trajectory
v_true = 0.2 + .01*randn(1,N-1); % true velocity. Changing this profile allows changing the trajectory
w_true = .01*ones(1,N-1); % true rotational velocity

% Updates
Msmt_avail = true; % in this Pb., there are no measurements

% Initial conditions
x_true_1 = [0 0 0]'; % initial starting pose (x y phi)
x_hat_1  = [0 0 0]'; % initial estimate
P_1 = zeros(3); % initial covariance

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Real World Simulation

% To generate random points in a circle
x1 = 0;
y1 = 0;
rc = 20;
N_L = 3;

LM = zeros(N_L,2);
 for i=1:N_L
    a=2*pi*rand;
    r=sqrt(rand);
    LM(i,1)=(rc*r)*cos(a)+x1;
    LM(i,2)=(rc*r)*sin(a)+y1;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Bookkeeping

% pre-allocate space for certain values we want to keep track of
x_hat_min = zeros(3,N); % state estimate after Propagation
x_hat_plus = zeros(3,N); % state estimate after update
P_min = zeros(3,3,N); % covariance after Propagation
P_plus = zeros(3,3,N); % covariance after update
    
if(input ==1 || input == 2)
    res = zeros(N_L,N); % measurement residual
    S = zeros(N_L,N_L,N); % residual covariance
else
    res = zeros(2*N_L,N); % measurement residual
    S = zeros(2*N_L,2*N_L,N); % residual covariance
end
% initialize those with the right values where appropriate
x_hat_plus(:,1) = x_hat_1;
P_plus(:,:,1) = P_1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EKF

% notice that we let the filter start with a propagation step. 
x_true = zeros(3,N);
for i = 2:N
    
    % TO calculate the true value
    x_true(1,i) = x_true(1,i-1)+v_true(1,i-1)*dt*cos(x_true(3,i-1));
    x_true(2,i) = x_true(2,i-1)+v_true(1,i-1)*dt*sin(x_true(3,i-1));
    x_true(3,i) = x_true(3,i-1)+w_true(1,i-1)*dt;
    [v_m, w_m,distance,bearing] = rws(v_true(1,i-1),w_true(1,i-1),LM,x_true(1,i),x_true(2,i),x_true(3,i));
   
    % Propagation
    [x_hat_min(:,i), P_min(:,:,i)] = EKF_propagate(P_plus(:,:,i-1), v_m^2*diag([0.01^2 0.04^2]),x_hat_plus(:,i-1), v_m, w_m, dt);
   
   
   % Update 
    if(input == 1)  
        [x_hat_plus(:,i),P_plus(:,:,i),res(:,i),S(:,:,i)] = EKF_update_dist(x_hat_min(:,i),P_min(:,:,i),distance(:,1),LM);
    elseif(input == 2)
        [x_hat_plus(:,i),P_plus(:,:,i),res(:,i),S(:,:,i)] = EKF_update_bear(x_hat_min(:,i),P_min(:,:,i),bearing(:,1),LM);
    else
        [x_hat_plus(:,i),P_plus(:,:,i),res(:,i),S(:,:,i)] = EKF_update_dist_bear(x_hat_min(:,i),P_min(:,:,i),distance(:,1),bearing(:,1),LM);
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualization

% labels
state = {'x (m)','y (m)','\phi (rad)'};
stateerr = {'x-x_{hat} (m)','y-y_{hat} (m)','\phi-\phi_{hat} (rad)'};


% generate time 
t = 0:dt:(N-1)*dt;
S_v = []; for i = 1:N, S_v = [S_v S(1,1,i)]; end;


% True state vs. posterior est
figure('Name','2D Trajectory'); hold on
plot(x_true(1,:), x_true(2,:), 'b'); 
plot(x_hat_plus(1,:), x_hat_plus(2,:), 'r');
xlabel('x (m)')
ylabel('y (m)')
legend('True State','Estimate')

figure('Name','Trajectory'); hold on

for i = 1:3
    subplot(3,1,i); plot(t, x_true(i,:), 'b'); hold on;
    subplot(3,1,i); plot(t, x_hat_plus(i,:), 'r'); hold on;
    ylabel(state{i})
end
xlabel('t (s)')
subplot(3,1,1); legend('True State','Posterior Estimate')





% Error
for i = 1:3
    figure('Name','Pose Error'); hold on
    plot(t, x_true(i,:)-x_hat_plus(i,:)); hold on;  
    plot(t, 3*sqrt(squeeze(P_plus(i,i,:))),'r'); hold on;
    plot(t,-3*sqrt(squeeze(P_plus(i,i,:))),'r'); hold on;
    
    xlabel('time (s)')
    ylabel(stateerr{i})
    legend('State Error','3\sigma - bound')
end


figure('Name','Error_Plot')
for j=1:5:N
    %plot_error_ellipse(P_plus(:,:,j),x_hat_plus(:,j));
    plot_error_ellipse(P_plus(:,:,j),x_hat_plus(:,j));
    hold on;
end
 
figure('Name','Residual'); hold on
plot(t, res)
plot(t, 3*sqrt(S_v),'r')
plot(t,-3*sqrt(S_v),'r')
xlabel('time (s)');
ylabel('z - z_{hat} (m)')
legend('GPS Residual','3\sigma - bound')
% Residual (only for future reference)
% figure('Name','Residual'); hold on
% nres = length(res);
% for i = 1:nres
%     subplot(nres,1,i); plot(t, res(i,:)); hold on;
%     subplot(nres,1,i); plot(t, 3*sqrt(squeeze(S(i,i,:))),'r'); hold on;
%     subplot(nres,1,i); plot(t,-3*sqrt(squeeze(S(i,i,:))),'r'); hold on;
%     ylabel(['z_' intstr(i) ' - z_{' int2str(i) '_{hat}} (m)'])
% end
% xlabel('time (s)');
% legend('Measurement Residual','3\sigma - bound')

