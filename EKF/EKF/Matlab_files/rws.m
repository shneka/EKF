% Real world simulation given linear and rotational velocity 
function[v_m, z_g, distance,bearing] = rws(v_true,g_true,LM,p_x_R,p_y_R,phi_R)

% To get the contaminated value of linear and angular velocity
std_v = 0.01 *v_true;
std_g = 0.04*g_true;

v_m = v_true + std_v*randn(1,1);
z_g = g_true + std_g*randn(1,1);
    

% standard deviation of bearing and actual distance
std_d = 0.01;
std_theta = 0.01;

% Calculating the actual distance and bearing value
N = size(LM,1);

dis = zeros(N,1);
b_t = zeros(N,1);
distance = zeros(N,1);
bearing = zeros(N,1);
for i=1:N
    dis(i,1) = sqrt((LM(i,1)-p_x_R)^2+(LM(i,2)-p_y_R)^2);
    b_t(i,1) = atan((LM(i,2)-p_y_R)/(LM(i,1)-p_x_R));
    distance(i,1) = dis(i,1)+std_d*randn(1,1);
    bearing(i,1) = b_t(i,1)-phi_R+std_theta*randn(1,1);
end


