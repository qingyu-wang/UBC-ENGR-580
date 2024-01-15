%% Setting
clc;

if not(isfolder("image"))
    mkdir("image")
    fprintf("Create Folder 'image'\n")
end

%% Parameters
clear;clc;

g               =  9.81;
v               =  350.3552;
r               =  0.1275;
l               =  5.31;
m               =  320;
P               =  20000;
alpha           =  0.2618;
theta           = -0.0688;
vartheta        =  0.1930;

Z_delta_y       = -2018.0457;
Z_beta          =  13273.1810;

J_x             =  2.601;
J_y             =  751.8960;

M_x_omega_x     = -4.8379;
M_x_omega_y     = -1.7167;
M_x_beta        = -22.8888;
M_x_delta_x     = -2.5490;
M_x_delta_y     = -2.0289;

M_y_omega_x     = -15.0379;
M_y_omega_y     = -150.3792;
M_y_beta        = -2255.6880;
M_y_beta_dot    =  0;
M_y_delta_y     = -676.7064;

b11     = - M_x_omega_x/J_x;
b12     = - M_x_omega_y/J_x;
b14     = - M_x_beta/J_x;
b17     = - M_x_delta_y/J_x;
b18     = - M_x_delta_x/J_x;

b21     = - M_y_omega_x/J_y;
b22     = - M_y_omega_y/J_y;
b24     = - M_y_beta/J_y;
b24_dot = - M_y_beta_dot/J_y;
b27     = - M_y_delta_y/J_y;

b32     = - cosd(theta)/cosd(vartheta);
b34     = (P-Z_beta)/(m*v);
b35     = -(g/v)*cosd(vartheta);
b37     = -Z_delta_y/(m*v);

a33     = -(g/v)*sind(theta);
b52     = -tand(vartheta);

% State Space Function
A = [
    -b11, -b12, -b14, 0;
    -b21-b24_dot*alpha, -b22+b24_dot*b32-b24_dot*b52*alpha, -b24+b24_dot*b34-b24_dot*a33, b24_dot*b35;
    alpha, -b32+b52*alpha, -b34-a33, -b35;
    1, b52, 0, 0
];
B = [
    -b18, -b17;
    0, -b27+b24_dot*b37;
    0, -b37;
    0, 0
];
C = [
    1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1
];
D = [
    0 0;
    0 0;
    0 0;
    0 0
];
fprintf("A:\n")
disp(A)
fprintf("B:\n")
disp(B)
fprintf("C:\n")
disp(C)
fprintf("D:\n")
disp(D)

%% Handout 2 Question 3.2
clc;

% Eigenvalue
eig_values = eig(A);
fprintf("Eigenvalues of A\n")
disp(eig_values)

% Jordan Form
[jor_P, jor_J] = jordan(A);
fprintf("Jordan Form\n")
disp(jor_J)

%% Handout 2 Question 3.5
clc;

% Open Loop System
sys = ss(A, B, C, D);

poles = pole(sys);
fprintf("Poles for the Open Loop System:\n")
disp(poles)

%% Handout 2 Question 3.7 (SIMULINK)
clc;

% Homogeneous Response in Open Loop Linear Model
x_0 = [0.1; 0.1; 0.05; 0.05];
delta_x_0 = 0;
delta_y_0 = 0;
out = sim("design_project_openloop_linear.slx", 200);
X = squeeze(out.X);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Open Loop Linear Model\nHomogeneous State Response"));
xlabel("Time");
ylabel("Response");
ylim([-0.25, 0.15])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_2_question_3.7_homo_state_resp_openloop_linear.png"));

% Homogeneous Response in Open Loop Nonlinear Model
x_0 = [0.1; 0.1; 0.05; 0.05];
delta_x_0 = 0;
delta_y_0 = 0;
out = sim("design_project_openloop_nonlinear.slx", 200);
X = squeeze(out.X);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Open Loop Nonlinear Model\nHomogeneous State Response"));
xlabel("Time");
ylabel("Response");
ylim([-0.25, 0.15])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_2_question_3.7_homo_state_resp_openloop_nonlinear.png"));

%% Handout 2 Question 3.8 (SIMULINK)
clc;

% Step Response (delta_x) in Open Loop Linear Model
x_0 = [0.1; 0.1; 0.05; 0.05];
delta_x_0 = 0.01;
delta_y_0 = 0;
out = sim("design_project_openloop_linear.slx", 100);
X = squeeze(out.X);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Open Loop Linear Model\nStep Input (delta\\_x)\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-1, 0.5])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_2_question_3.8_step_input_u1_state_resp_openloop_linear.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Open Loop Model\nStep Input (delta\\_x)\nInput Response"));
xlabel("Time");
ylabel("Input");
ylim([-0.005, 0.015])
legend(["delta\_x","delta\_y"])
saveas(f, sprintf("design_project/handout_2_question_3.8_step_input_u1_input_resp_openloop.png"));

% Step Response (delta_y) in Open Loop Linear Model
x_0 = [0.1; 0.1; 0.05; 0.05];
delta_x_0 = 0;
delta_y_0 = 0.01;
out = sim("design_project_openloop_linear.slx", 100);
X = squeeze(out.X);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Open Loop Linear Model\nStep Input (delta\\_y)\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-0.5, 1.5])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_2_question_3.8_step_input_u2_state_resp_openloop_linear.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Open Loop Model\nStep Input (delta\\_y)\nInput Response"));
xlabel("Time");
ylabel("Input");
ylim([-0.005, 0.015])
legend(["delta\_x","delta\_y"])
saveas(f, sprintf("design_project/handout_2_question_3.8_step_input_u2_input_resp_openloop.png"));

% Step Response (delta_x) in Open Loop Linear Model
x_0 = [0.1; 0.1; 0.05; 0.05];
delta_x_0 = 0.01;
delta_y_0 = 0;
out = sim("design_project_openloop_nonlinear.slx", 100);
X = squeeze(out.X);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Open Loop Nonlinear Model\nStep Input (delta\\_x)\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-1, 0.5])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_2_question_3.8_step_input_u1_state_resp_openloop_nonlinear.png"));

% Step Response (delta_y) in Open Loop Linear Model
x_0 = [0.1; 0.1; 0.05; 0.05];
delta_x_0 = 0;
delta_y_0 = 0.01;
out = sim("design_project_openloop_nonlinear.slx", 100);
X = squeeze(out.X);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Open Loop Nonlinear Model\nStep Input (delta\\_y)\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-0.5, 1.5])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_2_question_3.8_step_input_u2_state_resp_openloop_nonlinear.png"));

%% Handout 3 Question 2.1
clc;

CTRB = rank(ctrb(A,B)) == 4;
fprintf("Controllability: %s\n", string(CTRB))

%% Handout 3 Question 2.3
clc;

B_u1_fail = B(:,2);
B_u2_fail = B(:,1);
CTRB_u1_fail = rank(ctrb(A,B_u1_fail)) == 4;
CTRB_u2_fail = rank(ctrb(A,B_u2_fail)) == 4;
fprintf("Controllability if u_1 (delta_x) fails: %s\n", string(CTRB_u1_fail))
fprintf("Controllability if u_2 (delta_x) fails: %s\n", string(CTRB_u2_fail))

fprintf("\n")

% stabilizability
STBL = [];
eigenvalues = eig(A);
j = 1;
for i = 1:4
    if eigenvalues(i)>0
        STBL(j) = rank([A-eigenvalues(i)*eye(4) B]) == 4;
        j = j + 1;
    end
end
fprintf("Stabilizability: %s\n", string(all(STBL)))

%% Handout 3 Question 2.5 - Linear - 1. Lyapunov Test
clc;

% Book Lecture 12.4
assert(all(eig(-1.5*eye(4)-A)<0))
Q = B*B';
P = lyap((-1.5*eye(4)-A),Q);
K = 0.5*B'*P;

% Closed Loop System
A_cl = A-B*K;
sys_cl = ss(A_cl, B, C, D);
poles = pole(sys_cl);
fprintf("Poles for the Closed Loop System:\n")
disp(poles)

eigenvalues = eig(A_cl);
fprintf("Eigenvalue of A:\n")
disp(eigenvalues)

% State Response in Closed Loop Linear Model
x_0 = [1; 1; 1; 1];
delta_x_0 = 0;
delta_y_0 = 0;
noise = 0;
out = sim("design_project_closedloop_linear.slx", 20);
X = squeeze(out.X);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nLyapunov Test\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-3, 2])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_3_question_2.5_method_1_lyap_state_resp_closedloop_linear.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nLyapunov Test\nInput Response"));
xlabel("Time");
ylabel("Input");
ylim([-3, 2])
legend(["delta\_x","delta\_y"])
saveas(f, sprintf("design_project/handout_3_question_2.5_method_1_lyap_input_resp_closedloop_linear.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(real(poles), imag(poles), "o");
title(sprintf("Closed Loop Linear Model\nLyapunov Test\nPoles"));
xlim([-5, 1]);
line([0,0], ylim, 'LineStyle', '--', 'Color', [0.5 0.5 0.5]);
line(xlim, [0,0], 'LineStyle', '--', 'Color', [0.5 0.5 0.5]); 
xlabel("Real Part");
ylabel("Imaginary Part");
saveas(f, sprintf("design_project/handout_3_question_2.5_method_1_lyap_pole_closedloop_linear.png"));

%% Handout 3 Question 2.5 - Linear - 2. Eigenvalue Assignment
clc;

poles = [-1, -2, -3, -4];
K = place(A, B, poles);

% Closed Loop System
A_cl = A-B*K;
sys_cl = ss(A_cl, B, C, D);
poles = pole(sys_cl);
fprintf("Poles for the Closed Loop System:\n")
disp(poles)

eigenvalues = eig(A_cl);
fprintf("Eigenvalue of A:\n")
disp(eigenvalues)

% State Response in Closed Loop Linear Model
x_0 = [1; 1; 1; 1];
delta_x_0 = 0;
delta_y_0 = 0;
noise = 0;
out = sim("design_project_closedloop_linear.slx", 20);
X = squeeze(out.X);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nEigenvalue Assignment\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-4, 4])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_3_question_2.5_method_2_eigassign_state_resp_closedloop_linear.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nEigenvalue Assignment\nInput Response"));
xlabel("Time");
ylabel("Input");
ylim([-15, 15])
legend(["delta\_x","delta\_y"])
saveas(f, sprintf("design_project/handout_3_question_2.5_method_2_eigassign_input_resp_closedloop_linear.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(real(poles), imag(poles), "o");
title(sprintf("Closed Loop Linear Model\nEigenvalue Assignment\nPoles"));
xlim([-5, 1]);
line([0,0], ylim, 'LineStyle', '--', 'Color', [0.5 0.5 0.5]);
line(xlim, [0,0], 'LineStyle', '--', 'Color', [0.5 0.5 0.5]); 
xlabel("Real Part");
ylabel("Imaginary Part");
saveas(f, sprintf("design_project/handout_3_question_2.5_method_2_eigassign_pole_closedloop_linear.png"));

%% Handout 3 Question 2.5 - Linear - 3. LQR Control
clc;

% LQR Control
Q = [
    1 0  0  0; % Penalize x1 error
    0 1  0  0; % Penalize X2 error
    0 0 20  0; % Penalize x3 error
    0 0  0 20; % Penalize x3 error
];
R = [
    0.1 0.0;   % Penalize u1 effort
    0.0 0.1;   % Penalize u2 effort
];
K = lqr(A, B, Q, R);

% Closed Loop System
A_cl = A - B*K;
sys_cl = ss(A_cl, B, C, D);
poles = pole(sys_cl);
fprintf("Poles for the Closed Loop System:\n")
disp(poles)

eigenvalues = eig(A_cl);
fprintf("Eigenvalue of A:\n")
disp(eigenvalues)

% State Response in Closed Loop Linear Model
x_0 = [1; 1; 1; 1];
delta_x_0 = 0;
delta_y_0 = 0;
noise = 0;
out = sim("design_project_closedloop_linear.slx", 20);
X = squeeze(out.X);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nLQR Control\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-5, 5])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_3_question_2.5_method_3_lqr_state_resp_closedloop_linear.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nLQR Control\nInput Response"));
xlabel("Time");
ylabel("Input");
ylim([-10, 25])
legend(["delta\_x","delta\_y"])
saveas(f, sprintf("design_project/handout_3_question_2.5_method_3_lqr_input_resp_closedloop_linear.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(real(poles), imag(poles), "o");
title(sprintf("Closed Loop Linear Model\nLQR Control\nPoles"));
xlim([-5, 1]);
line([0,0], ylim, 'LineStyle', '--', 'Color', [0.5 0.5 0.5]);
line(xlim, [0,0], 'LineStyle', '--', 'Color', [0.5 0.5 0.5]); 
xlabel("Real Part");
ylabel("Imaginary Part");
saveas(f, sprintf("design_project/handout_3_question_2.5_method_3_lqr_pole_closedloop_linear.png"));

%% Handout 3 Question 2.6 - Linear - 3. LQR Control
clc;

% LQR Control
Q = [
    1 0  0  0; % Penalize x1 error
    0 1  0  0; % Penalize X2 error
    0 0 20  0; % Penalize x3 error
    0 0  0 20; % Penalize x3 error
];
R = [
    0.1 0.0;   % Penalize u1 effort
    0.0 0.1;   % Penalize u2 effort
];
K = lqr(A, B, Q, R);

% State Response in Closed Loop Linear Model
x_0 = [1; 1; 1; 1];
delta_x_0 = 0;
delta_y_0 = 0;
noise = 0;
out = sim("design_project_closedloop_linear.slx", 20);
X = squeeze(out.X);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nLQR Control\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-3, 2])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_3_question_2.6_method_3_lqr_state_resp_closedloop_linear.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nLQR Control\nInput Response"));
xlabel("Time");
ylabel("Input");
ylim([-10, 25])
legend(["delta\_x","delta\_y"])
saveas(f, sprintf("design_project/handout_3_question_2.6_method_3_lqr_input_resp_closedloop_linear.png"));

%% Handout 3 Question 2.6 - Linear - 3. LQR Control x5
clc;

% LQR Control
Q = [
    1 0  0  0; % Penalize x1 error
    0 1  0  0; % Penalize X2 error
    0 0 20  0; % Penalize x3 error
    0 0  0 20; % Penalize x3 error
];
R = [
    0.1 0.0;   % Penalize u1 effort
    0.0 0.1;   % Penalize u2 effort
];
K = lqr(A, B, Q, R);

% State Response in Closed Loop Linear Model
x_0 = [1; 1; 1; 1]*5;
delta_x_0 = 0;
delta_y_0 = 0;
noise = 0;
out = sim("design_project_closedloop_linear.slx", 20);
X = squeeze(out.X);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nLQR Control (5x Initial State)\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-15, 10])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_3_question_2.6_method_3_lqr_state_resp_closedloop_linear_x5.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nLQR Control (5x Initial State)\nInput Response"));
xlabel("Time");
ylabel("Input");
ylim([-50, 125])
legend(["delta\_x","delta\_y"])
saveas(f, sprintf("design_project/handout_3_question_2.6_method_3_lqr_input_resp_closedloop_linear_x5.png"));

%% Handout 3 Question 2.6 - Nonlinear - 3. LQR Control
clc;

% LQR Control
Q = [
    1 0  0  0; % Penalize x1 error
    0 1  0  0; % Penalize X2 error
    0 0 20  0; % Penalize x3 error
    0 0  0 20; % Penalize x3 error
];
R = [
    0.1 0.0;   % Penalize u1 effort
    0.0 0.1;   % Penalize u2 effort
];
K = lqr(A, B, Q, R);

% State Response in Closed Loop Linear Model
x_0 = [1; 1; 1; 1];
delta_x_0 = 0;
delta_y_0 = 0;
noise = 0;
out = sim("design_project_closedloop_nonlinear.slx", 20);
X = squeeze(out.X);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Closed Loop Nonlinear Model\nLQR Control\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-3, 2])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_3_question_2.6_method_3_lqr_state_resp_closedloop_nonlinear.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Nonlinear Model\nLQR Control\nInput Response"));
xlabel("Time");
ylabel("Input");
ylim([-10, 25])
legend(["delta\_x","delta\_y"])
saveas(f, sprintf("design_project/handout_3_question_2.6_method_3_lqr_input_resp_closedloop_nonlinear.png"));

%% Handout 3 Question 2.6 - Nonlinear - 3. LQR Control x5
clc;

% LQR Control
Q = [
    1 0  0  0; % Penalize x1 error
    0 1  0  0; % Penalize X2 error
    0 0 20  0; % Penalize x3 error
    0 0  0 20; % Penalize x3 error
];
R = [
    0.1 0.0;   % Penalize u1 effort
    0.0 0.1;   % Penalize u2 effort
];
K = lqr(A, B, Q, R);

% State Response in Closed Loop Linear Model
x_0 = [1; 1; 1; 1]*5;
delta_x_0 = 0;
delta_y_0 = 0;
noise = 0;
out = sim("design_project_closedloop_nonlinear.slx", 20);
X = squeeze(out.X);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Closed Loop Nonlinear Model\nLQR Control (5x Initial State)\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-15, 10])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_3_question_2.6_method_3_lqr_state_resp_closedloop_nonlinear_x5.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Nonlinear Model\nLQR Control (5x Initial State)\nInput Response"));
xlabel("Time");
ylabel("Input");
ylim([-50, 125])
legend(["delta\_x","delta\_y"])
saveas(f, sprintf("design_project/handout_3_question_2.6_method_3_lqr_input_resp_closedloop_nonlinear_x5.png"));

%% Handout 3 Question 2.6 - Linear - 1. Lyapunov Test - Noise
clc;

% Book Lecture 12.4
assert(all(eig(-1.5*eye(4)-A)<0))
Q = B*B';
P = lyap((-1.5*eye(4)-A),Q);
K = 0.5*B'*P;

for noise = [0.0000, 0.0001, 0.0010, 0.0100]
    % State Response in Closed Loop Linear Model
    x_0 = [1; 1; 1; 1];
    delta_x_0 = 0;
    delta_y_0 = 0;
    noise = noise;
    out = sim("design_project_closedloop_linear.slx", 20);
    X = squeeze(out.X);
    U = squeeze(out.U);
    T = squeeze(out.tout);
    
    f = figure;
    f.Position = [0, 0, 600, 400];
    f.Visible = "off";
    plot(T', X', 'LineWidth', 1.5);
    title(sprintf("Closed Loop Linear Model\nLQR Control (noise=%.4f)\nState Response", noise));
    xlabel("Time");
    ylabel("Response");
    ylim([-5, 5])
    legend(["omega\_x","omega\_y","beta","gamma"])
    saveas(f, sprintf("design_project/handout_3_question_2.7_method_1_lyap_closedloop_linear_state_resp_noise_%.4f.png", noise));
    
    f = figure;
    f.Position = [0, 0, 600, 400];
    f.Visible = "off";
    plot(T', U', 'LineWidth', 1.5);
    title(sprintf("Closed Loop Linear Model\nLQR Control (noise=%.4f)\nInput Response", noise));
    xlabel("Time");
    ylabel("Input");
    ylim([-30, 30])
    legend(["delta\_x","delta\_y"])
    saveas(f, sprintf("design_project/handout_3_question_2.7_method_1_lyap_closedloop_linear_input_resp_noise_%.4f.png", noise));
end

%% Handout 3 Question 2.6 - Linear - 2. Eigenvalue Assignment - Noise
clc;

poles = [-1, -2, -3, -4];
K = place(A, B, poles);

for noise = [0.0000, 0.0001, 0.0010, 0.0100]
    % State Response in Closed Loop Linear Model
    x_0 = [1; 1; 1; 1];
    delta_x_0 = 0;
    delta_y_0 = 0;
    noise = noise;
    out = sim("design_project_closedloop_linear.slx", 20);
    X = squeeze(out.X);
    U = squeeze(out.U);
    T = squeeze(out.tout);
    
    f = figure;
    f.Position = [0, 0, 600, 400];
    f.Visible = "off";
    plot(T', X', 'LineWidth', 1.5);
    title(sprintf("Closed Loop Linear Model\nLQR Control (noise=%.4f)\nState Response", noise));
    xlabel("Time");
    ylabel("Response");
    ylim([-5, 5])
    legend(["omega\_x","omega\_y","beta","gamma"])
    saveas(f, sprintf("design_project/handout_3_question_2.7_method_2_eigassign_closedloop_linear_state_resp_noise_%.4f.png", noise));
    
    f = figure;
    f.Position = [0, 0, 600, 400];
    f.Visible = "off";
    plot(T', U', 'LineWidth', 1.5);
    title(sprintf("Closed Loop Linear Model\nLQR Control (noise=%.4f)\nInput Response", noise));
    xlabel("Time");
    ylabel("Input");
    ylim([-30, 30])
    legend(["delta\_x","delta\_y"])
    saveas(f, sprintf("design_project/handout_3_question_2.7_method_2_eigassign_closedloop_linear_input_resp_noise_%.4f.png", noise));
end

%% Handout 3 Question 2.6 - Linear - 3. LQR Control - Noise
clc;

% LQR Control
Q = [
    1 0  0  0; % Penalize x1 error
    0 1  0  0; % Penalize X2 error
    0 0 20  0; % Penalize x3 error
    0 0  0 20; % Penalize x3 error
];
R = [
    0.1 0.0;   % Penalize u1 effort
    0.0 0.1;   % Penalize u2 effort
];
K = lqr(A, B, Q, R);

for noise = [0.0000, 0.0001, 0.0010, 0.0100]
    % State Response in Closed Loop Linear Model
    x_0 = [1; 1; 1; 1];
    delta_x_0 = 0;
    delta_y_0 = 0;
    noise = noise;
    out = sim("design_project_closedloop_linear.slx", 20);
    X = squeeze(out.X);
    U = squeeze(out.U);
    T = squeeze(out.tout);
    
    f = figure;
    f.Position = [0, 0, 600, 400];
    f.Visible = "off";
    plot(T', X', 'LineWidth', 1.5);
    title(sprintf("Closed Loop Linear Model\nLQR Control (noise=%.4f)\nState Response", noise));
    xlabel("Time");
    ylabel("Response");
    ylim([-5, 5])
    legend(["omega\_x","omega\_y","beta","gamma"])
    saveas(f, sprintf("design_project/handout_3_question_2.7_method_3_lqr_closedloop_linear_state_resp_noise_%.4f.png", noise));
    
    f = figure;
    f.Position = [0, 0, 600, 400];
    f.Visible = "off";
    plot(T', U', 'LineWidth', 1.5);
    title(sprintf("Closed Loop Linear Model\nLQR Control (noise=%.4f)\nInput Response", noise));
    xlabel("Time");
    ylabel("Input");
    ylim([-30, 30])
    legend(["delta\_x","delta\_y"])
    saveas(f, sprintf("design_project/handout_3_question_2.7_method_3_lqr_closedloop_linear_input_resp_noise_%.4f.png", noise));
end

%% Handout 3 Question 2.7 - Nonlinear - 3. LQR Control - Noise
clc;

% LQR Control
Q = [
    1 0  0  0; % Penalize x1 error
    0 1  0  0; % Penalize X2 error
    0 0 20  0; % Penalize x3 error
    0 0  0 20; % Penalize x3 error
];
R = [
    0.1 0.0;   % Penalize u1 effort
    0.0 0.1;   % Penalize u2 effort
];
K = lqr(A, B, Q, R);

for noise = [0.0000, 0.0001, 0.0010, 0.0100]
    % State Response in Closed Loop Linear Model
    x_0 = [1; 1; 1; 1];
    delta_x_0 = 0;
    delta_y_0 = 0;
    noise = noise;
    out = sim("design_project_closedloop_nonlinear.slx", 20);
    X = squeeze(out.X);
    U = squeeze(out.U);
    T = squeeze(out.tout);
    
    f = figure;
    f.Position = [0, 0, 600, 400];
    f.Visible = "off";
    plot(T', X', 'LineWidth', 1.5);
    title(sprintf("Closed Loop Nonlinear Model\nLQR Control (noise=%.4f)\nState Response)", noise));
    xlabel("Time");
    ylabel("Response");
    ylim([-4, 4])
    legend(["omega\_x","omega\_y","beta","gamma"])
    saveas(f, sprintf("design_project/handout_3_question_2.7_method_3_lqr_closedloop_nonlinear_state_resp_noise_%.4f.png", noise));
    
    f = figure;
    f.Position = [0, 0, 600, 400];
    f.Visible = "off";
    plot(T', U', 'LineWidth', 1.5);
    title(sprintf("Closed Loop Nonlinear Model\nLQR Control (noise=%.4f)\nInput Response", noise));
    xlabel("Time");
    ylabel("Input");
    ylim([-30, 30])
    legend(["delta\_x","delta\_y"])
    saveas(f, sprintf("design_project/handout_3_question_2.7_method_3_lqr_closedloop_nonlinear_input_resp_noise_%.4f.png", noise));
end

%% Handout 4 Question 2.1
clc;

OBSV = rank(obsv(A,C)) == 4;
fprintf("Observability: %s\n", string(OBSV))

%% Handout 4 Question 2.3
clc;

% one fails
C_y1_fail = C([2,3,4],:);
C_y2_fail = C([1,3,4],:);
C_y3_fail = C([1,2,4],:);
C_y4_fail = C([1,2,3],:);
OBSV_y1_fail = rank(obsv(A,C_y1_fail)) == 4;
OBSV_y2_fail = rank(obsv(A,C_y2_fail)) == 4;
OBSV_y3_fail = rank(obsv(A,C_y3_fail)) == 4;
OBSV_y4_fail = rank(obsv(A,C_y4_fail)) == 4;
fprintf("Observability if y_1 (omega_x) fails: %s\n", string(OBSV_y1_fail))
fprintf("Observability if y_2 (omega_y) fails: %s\n", string(OBSV_y2_fail))
fprintf("Observability if y_3 (beta)    fails: %s\n", string(OBSV_y3_fail))
fprintf("Observability if y_4 (gamma)   fails: %s\n", string(OBSV_y4_fail))

fprintf("\n")

% one works
C_y1_work = C(1,:);
C_y2_work = C(2,:);
C_y3_work = C(3,:);
C_y4_work = C(4,:);
OBSV_y1_work = rank(obsv(A,C_y1_work)) == 4;
OBSV_y2_work = rank(obsv(A,C_y2_work)) == 4;
OBSV_y3_work = rank(obsv(A,C_y3_work)) == 4;
OBSV_y4_work = rank(obsv(A,C_y4_work)) == 4;
fprintf("Observability if y_1 (omega_x) works: %s\n", string(OBSV_y1_work))
fprintf("Observability if y_2 (omega_y) works: %s\n", string(OBSV_y2_work))
fprintf("Observability if y_3 (beta)    works: %s\n", string(OBSV_y3_work))
fprintf("Observability if y_4 (gamma)   works: %s\n", string(OBSV_y4_work))

fprintf("\n")

% detectability
DTCT = [];
eigenvalues = eig(A);
j = 1;
for i = 1:4
    if eigenvalues(i)>0
        DTCT(j) = rank([A-eigenvalues(i)*eye(4); C]) == 4;
        j = j + 1;
    end
end
fprintf("Detectability: %s\n", string(all(DTCT)))

%% Handout 4 Question 2.4 - Open Loop Linear - Observer - LQR Control
clc;

% LQR Control
Q = [
    1 0  0  0; % Penalize x1 error
    0 1  0  0; % Penalize X2 error
    0 0 20  0; % Penalize x3 error
    0 0  0 20; % Penalize x3 error
];
R = [
    0.1 0.0;   % Penalize u1 effort
    0.0 0.1;   % Penalize u2 effort
];
K = lqr(A, B, Q, R);
L = K';

% State Response in Closed Loop Linear Model
x_0 = [1; 1; 1; 1];
delta_x_0 = 0;
delta_y_0 = 0;
noise = 0;
out = sim("design_project_openloop_obsv_linear.slx", 20);
X = squeeze(out.X);
X_obs = squeeze(out.X_obs);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Open Loop Linear Model\nLQR\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-4, 4])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.4_openloop_linear_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X_obs', 'LineWidth', 1.5);
title(sprintf("Open Loop Linear Model\nLQR\nObserver State Response"));
xlabel("Time");
ylabel("Response");
ylim([-4, 4])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.4_openloop_linear_obsv_state_resp.png"));

%% Handout 4 Question 2.4 - Open Loop Nonlinear - Observer - LQR Control
clc;

% LQR Control
Q = [
    1 0  0  0; % Penalize x1 error
    0 1  0  0; % Penalize X2 error
    0 0 20  0; % Penalize x3 error
    0 0  0 20; % Penalize x3 error
];
R = [
    0.1 0.0;   % Penalize u1 effort
    0.0 0.1;   % Penalize u2 effort
];
K = lqr(A, B, Q, R)*0.01;
L = K';

% State Response in Closed Loop Linear Model
x_0 = [1; 1; 1; 1];
delta_x_0 = 0;
delta_y_0 = 0;
noise = 0;
out = sim("design_project_openloop_obsv_nonlinear.slx", 20);
X = squeeze(out.X);
X_obs = squeeze(out.X_obs);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Open Loop Nonlinear Model\nLQR\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-4, 4])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.4_openloop_nonlinear_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X_obs', 'LineWidth', 1.5);
title(sprintf("Open Loop Nonlinear Model\nLQR\nObserver State Response"));
xlabel("Time");
ylabel("Response");
ylim([-4, 4])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.4_openloop_nonlinear_obsv_state_resp.png"));

%% Handout 4 Question 2.5 - Closed Loop Linear - Observer - LQR Control
clc;

% LQR Control
Q = [
    1 0  0  0; % Penalize x1 error
    0 1  0  0; % Penalize X2 error
    0 0 20  0; % Penalize x3 error
    0 0  0 20; % Penalize x3 error
];
R = [
    0.1 0.0;   % Penalize u1 effort
    0.0 0.1;   % Penalize u2 effort
];
% L = acker(A', C', [-1;-1;-1;-1])';
K = lqr(A, B, Q, R);
L = K';

% State Response in Closed Loop Linear Model
x_0 = [1; 1; 1; 1];
delta_x_0 = 0;
delta_y_0 = 0;
noise = 0;
out = sim("design_project_closedloop_obsv_linear.slx", 20);
X = squeeze(out.X);
X_obs = squeeze(out.X_obs);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-3, 3])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.5_closedloop_linear_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X_obs', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nObserver State Response"));
xlabel("Time");
ylabel("Response");
ylim([-3, 3])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.5_closedloop_linear_obsv_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nInput Response"));
xlabel("Time");
ylabel("Input");
ylim([-10, 25])
legend(["delta\_x","delta\_y"])
saveas(f, sprintf("design_project/handout_4_question_2.5_closedloop_linear_input_resp.png"));

%% Handout 4 Question 2.5 - Closed Loop Nonlinear - Observer - LQR Control
clc;

% LQR Control
Q = [
    1 0  0  0; % Penalize x1 error
    0 1  0  0; % Penalize X2 error
    0 0 20  0; % Penalize x3 error
    0 0  0 20; % Penalize x3 error
];
R = [
    0.1 0.0;   % Penalize u1 effort
    0.0 0.1;   % Penalize u2 effort
];
K = lqr(A, B, Q, R)*0.01;
L = K';

% State Response in Closed Loop Linear Model
x_0 = [1; 1; 1; 1];
delta_x_0 = 0;
delta_y_0 = 0;
noise = 0;
out = sim("design_project_closedloop_obsv_nonlinear.slx", 20);
X = squeeze(out.X);
X_obs = squeeze(out.X_obs);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Closed Loop Nonlinear Model\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-3, 3])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.5_closedloop_nonlinear_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X_obs', 'LineWidth', 1.5);
title(sprintf("Closed Loop Nonlinear Model\nObserver State Response"));
xlabel("Time");
ylabel("Response");
ylim([-3, 3])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.5_closedloop_nonlinear_obsv_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Nonlinear Model\nInput Response"));
xlabel("Time");
ylabel("Input");
ylim([-0.5, 0.5])
legend(["delta\_x","delta\_y"])
saveas(f, sprintf("design_project/handout_4_question_2.5_closedloop_nonlinear_input_resp.png"));

%% Handout 4 Question 2.6 - Closed Loop Linear - Observer - Reference Tracking
clc;

% LQR Control
Q = [
    1 0  0  0; % Penalize x1 error
    0 1  0  0; % Penalize X2 error
    0 0 20  0; % Penalize x3 error
    0 0  0 20; % Penalize x3 error
];
R = [
    0.1 0.0;   % Penalize u1 effort
    0.0 0.1;   % Penalize u2 effort
];
% L = acker(A', C', [-1;-1;-1;-1])';
K = lqr(A, B, Q, R);
L = K';

% State Response in Closed Loop Linear Model
x_0 = [1; 1; 1; 1];
r = [2; 1];
delta_x_0 = 0;
delta_y_0 = 0;
noise = 0;
K_i = -10;
out = sim("design_project_closedloop_obsv_track_linear.slx", 100);
X = squeeze(out.X);
X_obs = squeeze(out.X_obs);
U = squeeze(out.U);
Y = squeeze(out.Y);
R = squeeze(out.R);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nReference Tracking\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-10, 80])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.6_closedloop_linear_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X_obs', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nReference Tracking\nObserver State Response"));
xlabel("Time");
ylabel("Response");
ylim([-10, 80])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.6_closedloop_linear_obsv_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nReference Tracking\nInput Response"));
xlabel("Time");
ylabel("Input");
ylim([-20, 30])
legend(["delta\_x","delta\_y"])
saveas(f, sprintf("design_project/handout_4_question_2.6_closedloop_linear_input_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', Y', 'LineWidth', 1.5);
hold on
plot(T', R', 'LineWidth', 1.5, 'LineStyle', '--');
hold off
title(sprintf("Closed Loop Linear Model\nReference Tracking\nOutput Response"));
xlabel("Time");
ylabel("Input");
ylim([-4, 4])
legend(["omega\_x", "beta", "ref\_omega\_x", "ref\_beta"])
saveas(f, sprintf("design_project/handout_4_question_2.6_closedloop_linear_output_resp.png"));

%% Handout 4 Question 2.6 - Closed Loop Nonlinear - Observer - Reference Tracking
clc;

% LQR Control
Q = [
    1 0  0  0; % Penalize x1 error
    0 1  0  0; % Penalize X2 error
    0 0 20  0; % Penalize x3 error
    0 0  0 20; % Penalize x3 error
];
R = [
    0.1 0.0;   % Penalize u1 effort
    0.0 0.1;   % Penalize u2 effort
];
% L = acker(A', C', [-1;-1;-1;-1])';
K = lqr(A, B, Q, R)*0.01;
L = K';

% State Response in Closed Loop Linear Model
x_0 = [1; 1; 1; 1];
r = [2; 1];
delta_x_0 = 0;
delta_y_0 = 0;
noise = 0;
K_i = -0.1;
out = sim("design_project_closedloop_obsv_track_nonlinear.slx", 100);
X = squeeze(out.X);
X_obs = squeeze(out.X_obs);
U = squeeze(out.U);
Y = squeeze(out.Y);
R = squeeze(out.R);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Closed Loop Nonlinear Model\nReference Tracking\nState Response"));
xlabel("Time");
ylabel("Response");
ylim([-10, 50])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.6_closedloop_nonlinear_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X_obs', 'LineWidth', 1.5);
title(sprintf("Closed Loop Nonlinear Model\nReference Tracking\nObserver State Response"));
xlabel("Time");
ylabel("Response");
ylim([-10, 50])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.6_closedloop_nonlinear_obsv_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Nonlinear Model\nReference Tracking\nInput Response"));
xlabel("Time");
ylabel("Input");
ylim([-10, 1])
legend(["delta\_x","delta\_y"])
saveas(f, sprintf("design_project/handout_4_question_2.6_closedloop_nonlinear_input_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', Y', 'LineWidth', 1.5);
hold on
plot(T', R', 'LineWidth', 1.5, 'LineStyle', '--');
hold off
title(sprintf("Closed Loop Nonlinear Model\nReference Tracking\nOutput Response"));
xlabel("Time");
ylabel("Input");
ylim([-4, 4])
legend(["omega\_x", "beta", "ref\_omega\_x", "ref\_beta"])
saveas(f, sprintf("design_project/handout_4_question_2.6_closedloop_nonlinear_output_resp.png"));

%% Handout 4 Question 2.7 - Closed Loop Linear - Observer - Reference Tracking - Noise
clc;

% LQR Control
Q = [
    1 0  0  0; % Penalize x1 error
    0 1  0  0; % Penalize X2 error
    0 0 20  0; % Penalize x3 error
    0 0  0 20; % Penalize x3 error
];
R = [
    0.1 0.0;   % Penalize u1 effort
    0.0 0.1;   % Penalize u2 effort
];
% L = acker(A', C', [-1;-1;-1;-1])';
K = lqr(A, B, Q, R);
L = K';

% State Response in Closed Loop Linear Model
x_0 = [1; 1; 1; 1];
r = [2; 1];
delta_x_0 = 0;
delta_y_0 = 0;
noise = 0.0001;
K_i = 0;
out = sim("design_project_closedloop_obsv_track_linear.slx", 20);
X = squeeze(out.X);
X_obs = squeeze(out.X_obs);
U = squeeze(out.U);
Y = squeeze(out.Y);
R = squeeze(out.R);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nReference Tracking (Noise=%.4f)\nState Response", 0.0001));
xlabel("Time");
ylabel("Response");
ylim([-10, 80])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.7_closedloop_linear_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X_obs', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nReference Tracking (Noise=%.4f)\nObserver State Response", 0.0001));
xlabel("Time");
ylabel("Response");
ylim([-10, 80])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.7_closedloop_linear_obsv_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nReference Tracking (Noise=%.4f)\nInput Response", 0.0001));
xlabel("Time");
ylabel("Input");
ylim([-20, 30])
legend(["delta\_x","delta\_y"])
saveas(f, sprintf("design_project/handout_4_question_2.7_closedloop_linear_input_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', Y', 'LineWidth', 1.5);
hold on
plot(T', R', 'LineWidth', 1.5, 'LineStyle', '--');
hold off
title(sprintf("Closed Loop Linear Model\nReference Tracking (Noise=%.4f)\nOutput Response", 0.0001));
xlabel("Time");
ylabel("Input");
ylim([-4, 4])
legend(["omega\_x", "beta", "ref\_omega\_x", "ref\_beta"])
saveas(f, sprintf("design_project/handout_4_question_2.7_closedloop_linear_output_resp.png"));

%% Handout 4 Question 2.6 - Closed Loop Nonlinear - Observer - Reference Tracking
clc;

% LQR Control
Q = [
    1 0  0  0; % Penalize x1 error
    0 1  0  0; % Penalize X2 error
    0 0 20  0; % Penalize x3 error
    0 0  0 20; % Penalize x3 error
];
R = [
    0.1 0.0;   % Penalize u1 effort
    0.0 0.1;   % Penalize u2 effort
];
% L = acker(A', C', [-1;-1;-1;-1])';
K = lqr(A, B, Q, R)*0.01;
L = K';

% State Response in Closed Loop Linear Model
x_0 = [1; 1; 1; 1];
r = [2; 1];
delta_x_0 = 0;
delta_y_0 = 0;
noise = 0.0001;
K_i = -0.1;
out = sim("design_project_closedloop_obsv_track_nonlinear.slx", 100);
X = squeeze(out.X);
X_obs = squeeze(out.X_obs);
U = squeeze(out.U);
Y = squeeze(out.Y);
R = squeeze(out.R);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Closed Loop Nonlinear Model\nReference Tracking (Noise=%.4f)\nState Response", 0.0001));
xlabel("Time");
ylabel("Response");
ylim([-10, 50])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.7_closedloop_nonlinear_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X_obs', 'LineWidth', 1.5);
title(sprintf("Closed Loop Nonlinear Model\nReference Tracking (Noise=%.4f)\nObserver State Response", 0.0001));
xlabel("Time");
ylabel("Response");
ylim([-10, 50])
legend(["omega\_x","omega\_y","beta","gamma"])
saveas(f, sprintf("design_project/handout_4_question_2.7_closedloop_nonlinear_obsv_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Nonlinear Model\nReference Tracking (Noise=%.4f)\nInput Response", 0.0001));
xlabel("Time");
ylabel("Input");
ylim([-10, 1])
legend(["delta\_x","delta\_y"])
saveas(f, sprintf("design_project/handout_4_question_2.7_closedloop_nonlinear_input_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', Y', 'LineWidth', 1.5);
hold on
plot(T', R', 'LineWidth', 1.5, 'LineStyle', '--');
hold off
title(sprintf("Closed Loop Nonlinear Model\nReference Tracking (Noise=%.4f)\nOutput Response", 0.0001));
xlabel("Time");
ylabel("Input");
ylim([-4, 4])
legend(["omega\_x", "beta", "ref\_omega\_x", "ref\_beta"])
saveas(f, sprintf("design_project/handout_4_question_2.7_closedloop_nonlinear_output_resp.png"));
