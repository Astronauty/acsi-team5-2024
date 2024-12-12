%% LQR for Trajectory Tracking of Two-Wheeled Robot

% Parameters
r = 0.021; % Radius of the wheels (m)
m = 1.9; % Mass (kg)
D = 0.19; % Wheelbase (m)
J = 0.1; % Moment of inertia in yaw (kg-ms^2)
F = 8; % Friction force (N)


% Define the state-space matrices from Equation (29)
% A_ = [0, 0, 0, 0; 
%     0, 0, 0, 0; 
%     0, 0, (-F/J), 0;
%     0, 0, 0, (-F/J)]; 
% 
% B = [1/m, 1/m, 0, 0; 
%     -D/(2*J), D/(2*J), 0, 0; 
%     -r/J, 0, 1/J, 0;
%     0, -r/J, 0, 1/J];
% 
% C = [0, 0, 1, 0;
%      0, 0, 0, 1];

steps = linspace(0,3*pi,300);
%1m circle traveling

syms circle_x(t) circle_y(t) infty_x(t) infty_y(t)

circle_x(t) = sin(t);
circle_y(t) = cos(t); 
d_circle_x(t) = diff(circle_x);
d_circle_y(t) = diff(circle_y);
dd_circle_x(t) = diff(d_circle_x);
dd_circle_y(t) = diff(d_circle_y);

infty_x(t) = sin(t)*cos(t);
infty_y(t) = sin(t);
d_infty_x(t) = diff(infty_x);
d_infty_y(t) = diff(infty_y);
dd_infty_x(t) = diff(d_infty_x);
dd_infty_y(t) = diff(d_infty_y);

x_cir = circle_x(steps);
d_x_cir = d_circle_x(steps);
dd_x_cir = dd_circle_x(steps);
y_cir = circle_y(steps);
d_y_cir = d_circle_y(steps);
dd_y_cir = dd_circle_y(steps);


w_ref = (d_x_cir.*dd_y_cir - d_y_cir.*dd_x_cir)./(d_x_cir.^2 + d_y_cir.^2);
v_ref = sqrt(d_x_cir.^2 + d_y_cir.^2);
theta_ref = atan(d_y_cir./d_x_cir); 

% A = [0, w_ref(1), 0;
%     -w_ref(1), 0, v_ref(1);
%     0, 0, 0];

w_ref_num = double(subs(w_ref, t, steps)); % Convert symbolic to numeric
v_ref_num = double(subs(v_ref, t, steps)); % Convert symbolic to numeric

% Numeric A matrix using w_ref and v_ref
A = [0, w_ref_num, 0;
        -w_ref_num, 0, v_ref_num;
         0, 0, 0];

B = [1, 0;
     0, 0;
     0, 1];


% Cost matrices for LQR
Q = diag([1, 1, 1]); % State cost matrix
Q(3,3) = 10;
R_control = 0.0001*diag([1, 1]); % Control cost matrix

% Compute LQR gain for the prescribed velocity (system dynamics discounted)
% K = lqr(A, B, Q, R_control);
K = lqr(A, B, Q, R_control);

% Inputs
x_ref = [x_ref; y_ref; theta_ref]; % Reference state (position and orientation)
x_curr = [x_curr; y_curr; theta_curr]; % Current state

% Calculate errors (Equation 27)
error_global = x_ref - x_curr;

% Transform errors to robot frame (Equation 28)
rotation_matrix = [cos(x_curr(3)), sin(x_curr(3)), 0;
                   -sin(x_curr(3)), cos(x_curr(3)), 0;
                   0, 0, 1];
error_robot = rotation_matrix * error_global;

% Compute control inputs (Equation 30)
u = -K * error_robot;

% Extract velocities
v = u(1);     % Linear velocity (m/s)
w = u(2);     % Angular velocity (rad/s)

% Compute wheel tangential velocities (Equations 22 and 23)
v_r = v + (L/2) * w; % Right wheel
v_l = v - (L/2) * w; % Left wheel

% Display results
fprintf('Linear Velocity (v): %.2f m/s\n', v);
fprintf('Angular Velocity (w): %.2f rad/s\n', w);
fprintf('Right Wheel Tangential Velocity (v_r): %.2f m/s\n', v_r);
fprintf('Left Wheel Tangential Velocity (v_l): %.2f m/s\n', v_l);