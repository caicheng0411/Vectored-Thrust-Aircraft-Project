clear all, close all, clc

% Define the system parameters 

m = 4        % System mass unit in kg 
J = 0.0475   % System intertia unit in Kg m^2 
r = 0.25     % Radius unit in m
g = 9.8      % Gravitational constant unit in m/s^2
c = 0.05     % Ratational damping Ns/m 

% Linearized state - space A, B,A C and D
A = [0, 0, 0, 1, 0, 0; 
    0, 0, 0, 0, 1, 0; 
    0, 0, 0, 0, 0, 1; 
    0, 0, -g, -c/m, 0, 0;
    0, 0, 0, 0, -c/m, 0; 
    0, 0, 0, 0, 0, 0];
B = [0, 0;
    0, 0;
    0, 0;
    1/m, 0;
    0, 1/m;
    r/J, 0];
C = [1, 0, 0, 0, 0, 0; 0, 1, 0, 0, 0, 0]
D = [0, 0; 
    0, 0]
sys = ss(A, B, C, D)

sys_tf = tf(sys)

%% ZOH
% Find the eigenvalues and eigenvectors
[V, Di] = eig(A);

% Extract the diagonal matrix D (eigenvalues) and the matrix V (eigenvectors)
H = Di;
T = V;

% Check if the transformation is valid
if norm(A*T - T*H, 'fro') < 1e-10
    disp('Transformation successful.');
    
    % Display the transformation matrix T and the diagonal matrix H
    disp('Transformation matrix T:');
    disp(T);
    
    disp('Diagonal matrix H:');
    disp(H);
else
    disp('Error in transformation.');
end

% Matrix B_hat
B_hat = [6.66667, 0;
    20.00156, 0;
    0, 20; 
    6.66667, 0;
    0, 20.00156;
    -6.66667, 0]

% Matrix C_hat
C_hat = [1, -0.99992, 0, 1, 0, -1;0, 0, 1, 0, -0.99992, 0]

new_sys = ss(H, B_hat, C_hat, D)
sys_tf_new = tf(new_sys)

dt = 0.1;
Gd = c2d(sys, dt, 'zoh')

%% State Feedback Control 
desired_eigenvalues = [-0.5 + 0.5j; -0.5 - 0.5j; -1; -3 + 5j; -3 - 5j; -5];
% desired_eigenvalues = [-5.3853; -2.0563 + 2.3125j; -2.0563 - 2.3125j; -1.0002; 0.3751+0.3307; 0.3751-0.3307];
K = place(A, B, desired_eigenvalues)


% Define the closed-loop system with state feedback
syscl = ss(A - B * K, B, C, D);

syscl_tf = tf(syscl);

% Step Response 
figure;
step(syscl)
h = findobj(gcf, 'Type', 'line');
set(h, 'LineWidth', 2); 
grid on;

% Impulse Response 
figure;
impulse(syscl)
h = findobj(gcf, 'Type', 'line');
set(h, 'LineWidth', 2); 
grid on;

% Sinusoidal Response with Different Frequency 
% Specify the time vector for the Sinusoidal response
t = linspace(0,20,201);

% All amplitude of Sinusoid signal equals to 10
% Simulate the system response to the sinusoidal input with Frequency = 1
v1 = 10*sin(2*pi * t);

[X_0, T_0] = lsim(syscl(1,1),v1, t);
[X_1, T_1] = lsim(syscl(1,2),v1, t);
[X_2, T_2] = lsim(syscl(2,1),v1, t);
[X_3, T_3] = lsim(syscl(2,2),v1, t);


% Plot the sinusoidal response for each output
figure;

subplot(2, 2, 1);
plot(T_0, X_0, 'LineWidth', 2);
title('System Response of 10Sin(2tpi)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(2, 2, 2);
plot(T_1, X_1, 'LineWidth', 2);
title('System Response of 10Sin(2tpi)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;


subplot(2, 2, 3);
plot(T_2, X_2, 'LineWidth', 2);
title('System Response of 10Sin(2tpi)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(2, 2, 4);
plot(T_3, X_3, 'LineWidth', 2);
title('System Response of 10Sin(2tpi)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

v2 = 10*sin(20*pi*t);

[Y_0, T_00] = lsim(syscl(1,1),v2, t);
[Y_1, T_11] = lsim(syscl(1,2),v2, t);
[Y_2, T_22] = lsim(syscl(2,1),v2, t);
[Y_3, T_33] = lsim(syscl(2,2),v2, t);

% Plot the sinusoidal response for each output
figure;

subplot(2, 2, 1);
plot(T_00, Y_0, 'LineWidth', 2);
title('System Response of 10Sin(20tpi)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(2, 2, 2);
plot(T_11, Y_1, 'LineWidth', 2);
title('System Response of 10Sin(20tpi)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;


subplot(2, 2, 3);
plot(T_22, Y_2, 'LineWidth', 2);
title('System Response of 10Sin(20tpi)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(2, 2, 4);
plot(T_33, Y_3, 'LineWidth', 2);
title('System Response of 10Sin(20tpi)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

%% Observable Feedback Control
O = obsv(A, C)
rank (O)

% Desired poles for the observer
pole_des = [-5 + 5j, -5 - 5j, -10 + 10j, -10 - 10j, -3, -9];
L = place(A', C', pole_des)'

est_poles = eig(A - L*C)

% Simulation 
% Define augmented system to run the simulation
Aaug = [A, zeros(6,6); L*C, A-L*C];
Baug = [B;B];
Caug = [C, zeros(2,6)];
Daug = 0;

% Agumented Matrix Under Observer State Feedback Control
sys_obs = ss(Aaug,Baug,Caug,Daug);

% Find the Transfer function of each output
sys_tff = tf(sys_obs);

% From sys_tff we can know that there only two transfer function (1,1)
% and(2,2) position, therefore, we only plot these two 
sys_tf_x = sys_tff(1,1)
sys_tf_y = sys_tff(2,2)

% Step Response 
figure;
subplot(2,1,1)
step(sys_tf_x)
title('Step Response for Output in X Direction')
grid on;
subplot(2,1,2)
step(sys_tf_y)
title('Step Response for Output in Y Direction')
h = findobj(gcf, 'Type', 'line');
set(h, 'LineWidth', 2); 
grid on;

% Impulse Response
figure;
subplot(2,1,1)
impulse(sys_tf_x)
title('Impulse Response for Output in X Direction')
grid on;
subplot(2,1,2)
impulse(sys_tf_y)
title('Impulse Response for Output in Y Direction')
h = findobj(gcf, 'Type', 'line');
set(h, 'LineWidth', 2); 
grid on;

% Define time vector
T = linspace(0,20,201);

% Sinusoid Input Signal of Different Frequency

% All amplitude of Sinusoid signal equals to 10
% Simulate the system response to the sinusoidal input with Frequency = pi
u1 = 10*sin(2 * pi * T);

[X1, T1] = lsim(sys_tf_x,u1, T);
[Y1, T1] = lsim(sys_tf_y,u1, T);


% Simulate the system response to the sinusoidal input with Frequency = 5pi
u2 = 10*sin(5 * pi * T);

[X2, T2] = lsim(sys_tf_x,u2, T);
[Y2, T2] = lsim(sys_tf_y,u2, T);

% Simulate the system response to the sinusoidal input with Frequency = 9pi
u3 = 10*sin(9 * pi * T);
[X3, T3] = lsim(sys_tf_x,u3, T);
[Y3, T3] = lsim(sys_tf_y,u3, T);


% Plot the system response
figure;
subplot(3,2,1)
plot(T1, X1, 'LineWidth', 2);
title('System Response in X to 10Sin(2piT)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(3,2,2)
plot(T1, Y1, 'LineWidth', 2);
title('System Response in Y to 10Sin(2piT)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(3,2,3)
plot(T2, X2, 'LineWidth', 2);
title('System Response in X to 10Sin(5piT)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(3,2,4)
plot(T2, Y2, 'LineWidth', 2);
title('System Response in Y to 10Sin(5piT)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;


subplot(3,2,5)
plot(T3, X3, 'LineWidth', 2);
title('System Response in X to 10Sin(9piT)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(3,2,6)
plot(T3, Y3, 'LineWidth', 2);
title('System Response in Y to 10Sin(9piT)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;


%% LQR
v = [1,1,1,1,1,1];
Q = diag(v);
u = [1,1];
R = diag(u);
[K, S, E] = lqr(A, B, Q, R);
closed_loop_system = ss(A - B*K, B, C, D);

% Plot the pole-zero map
figure;
pzmap(closed_loop_system);
title('Pole-Zero Map');
h = findobj(gcf, 'Type', 'line');
set(h, 'LineWidth', 2); 
grid on;

% Step response 
figure;
step(closed_loop_system)
title('Step Response with LQR');
h = findobj(gcf, 'Type', 'line');
set(h, 'LineWidth', 2); 
grid on;

% Impulse response 
figure;
impulse(closed_loop_system)
title('Impulse Response with LQR');
h = findobj(gcf, 'Type', 'line');
set(h, 'LineWidth', 2); 
grid on;

% Sinusoidal Response 
a1 = sin(2*pi*T);

[W1, P_t1] = lsim(closed_loop_system(1,1), a1, T);

[W2, P_t2] = lsim(closed_loop_system(1,2), a1, T);

[W3, P_t3] = lsim(closed_loop_system(2,1), a1, T);

[W4, P_t4] = lsim(closed_loop_system(2,2), a1, T);

% Plot the system response
figure;
subplot(4,1,1)
plot(P_t1, W1, 'LineWidth', 2);
title('System Response of sin(2pit)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(4,1,2)
plot(P_t2, W2, 'LineWidth', 2);
title('System Response of sin(2pit)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(4,1,3)
plot(P_t3, W3, 'LineWidth', 2);
title('System Response of sin(2pit)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(4,1,4)
plot(P_t4,W4, 'LineWidth', 2);
title('System Response of sin(2pit)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;
