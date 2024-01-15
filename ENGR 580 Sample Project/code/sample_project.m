%% Setting
clc;

if not(isfolder("image"))
    mkdir("image")
    fprintf("Create Folder 'image'\n")
end

%% Parameters
clear;clc;

% with symbol
syms q_r q_u q_t;
A_syms = [
    -q_r-q_u,    0,        0,    0;
         q_u, -q_t,        0,    0;
    0.25*q_r,    0, -q_r-q_u,    0;
           0,    0,      q_u, -q_t
];
B = [
    1, 0;
    0, 0;
    0, 1;
    0, 0
];
C_syms = [
    0.2*q_r, 0, 0.15*q_r, 0
];
D = [
    0, 0;
];

A = double(subs(A_syms, {q_u, q_r, q_t}, {0.00627778, 0.00313889, 0.00418519}));
C = double(subs(C_syms, {q_u, q_r, q_t}, {0.00627778, 0.00313889, 0.00418519}));

%% Handout 2 Question 2.1
clc;

fprintf("A:\n");
disp(A_syms);
fprintf("B:\n");
disp(B);
fprintf("C:\n");
disp(C_syms);
fprintf("D:\n");
disp(D);

fprintf("A:\n");
disp(A);
fprintf("B:\n");
disp(B);
fprintf("C:\n");
disp(C);
fprintf("D:\n");
disp(D);

%% Handout 2 Question 2.3
clc;

eigenvalues_syms = eig(A_syms);
fprintf("Eigenvalues of A:\n");
disp(eigenvalues_syms);

% with value
eigenvalues = eig(A);
fprintf("Eigenvalues of A:\n");
disp(eigenvalues);

%% Handout 2 Question 2.4
clc;

[jorT_syms, jorJ_syms] = jordan(A_syms);
fprintf("A:\n");
disp(A_syms);
fprintf("Jordan Normal Form of A:\n");
disp(jorJ_syms);
fprintf("Transfer Matrix for Jordan Normal Form of A:\n");
disp(collect(jorT_syms));
fprintf("Convert back to A:\n");
disp(collect(jorT_syms*jorJ_syms/jorT_syms))

fprintf("\n");

% with value
[jorT, jorJ] = jordan(A);
fprintf("A:\n");
disp(A);
fprintf("Jordan Normal Form of A:\n");
disp(jorJ);
fprintf("Transfer Matrix for Jordan Normal Form of A:\n");
disp(jorT);
fprintf("Convert back to A:\n");
disp(jorT*jorJ/jorT);

%% Handout 2 Question 2.5
clc;

Q = eye(size(A));
P = lyap(A, Q);
is_positive_definite = all(eig(P) > 0);
fprintf("P Eigenvalues:\n");
disp(eig(P));
fprintf("P is positive definite: %s\n", string(is_positive_definite));

%% Handout 2 Question 2.6
clc;

sys_ol = ss(A, B, C, D);
poles_ol = pole(sys_ol);

fprintf("Poles of open loop system:\n");
disp(poles_ol);

%% Handout 2 Question 2.8
clc;

% Sprinkler 1
u_0 = [30000; 0];
x_0 = [30000; 45000; 30000; 45000];
A_sim = A;
B_sim = B;
C_sim = C;
D_sim = D;
out = sim("sample_project_openloop_linear.slx", 2400);
X = squeeze(out.X);
Y = squeeze(out.Y);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Open Loop Linear Model\nSprinkler 1\nState Response\n(volume of water in the soil/crops)"));
xlabel("Time (hour)");
ylabel("Response");
% ylim([-5, 5])
legend(["V_s,1","V_c,1","V_s,2","V_c,2"])
saveas(f, sprintf("sample_project/handout_2_question_2.8_openloop_linear_sp1_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', Y', 'LineWidth', 1.5);
title(sprintf("Open Loop Linear Model\nSprinkler 1\nOutput Response\n(volume of water collected by a nearby stream)"));
xlabel("Time (hour)");
ylabel("Response");
% ylim([-5, 5])
legend("y")
saveas(f, sprintf("sample_project/handout_2_question_2.8_openloop_linear_sp1_output_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Open Loop Linear Model\nSprinkler 1\nInput Response\n(inflow of water from the sprinkler)"));
xlabel("Time (hour)");
ylabel("Response");
% ylim([-5, 5])
legend(["u1", "u2"])
saveas(f, sprintf("sample_project/handout_2_question_2.8_openloop_linear_sp1_input_resp.png"));

% Sprinkler 2
u_0 = [0; 30000];
x_0 = [30000; 45000; 30000; 45000];
A_sim = A;
B_sim = B;
C_sim = C;
D_sim = D;
out = sim("sample_project_openloop_linear.slx", 2400);
X = squeeze(out.X);
Y = squeeze(out.Y);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Open Loop Linear Model\nSprinkler 2\nState Response\n(volume of water in the soil/crops)"));
xlabel("Time (hour)");
ylabel("Response");
% ylim([-5, 5])
legend(["V_s,1","V_c,1","V_s,2","V_c,2"])
saveas(f, sprintf("sample_project/handout_2_question_2.8_openloop_linear_sp2_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', Y', 'LineWidth', 1.5);
title(sprintf("Open Loop Linear Model\nSprinkler 2\nOutput Response\n(volume of water collected by a nearby stream)"));
xlabel("Time (hour)");
ylabel("Response");
% ylim([-5, 5])
legend("y")
saveas(f, sprintf("sample_project/handout_2_question_2.8_openloop_linear_sp2_output_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Open Loop Linear Model\nSprinkler 2\nInput Response\n(inflow of water from the sprinkler)"));
xlabel("Time (hour)");
ylabel("Response");
% ylim([-5, 5])
legend(["u1", "u2"])
saveas(f, sprintf("sample_project/handout_2_question_2.8_openloop_linear_sp2_input_resp.png"));

%% Handout 2 Question 2.9
clc;

% with symbol
syms q_r_hot q_u_hot q_t_hot;

% hot weather increate flow rate
q_r_hot = q_r * 1.5;
q_u_hot = q_u * 1.5;
q_t_hot = q_t * 1.5;

A_hot_syms = [
    -q_r_hot-q_u_hot,        0,                0,        0;
             q_u_hot, -q_t_hot,                0,        0;
        0.25*q_r_hot,        0, -q_r_hot-q_u_hot,        0;
                   0,        0,          q_u_hot, -q_t_hot
];
C_hot_syms = [
    0.2*q_r_hot, 0, 0.15*q_r_hot, 0
];

A_hot = double(subs(A_hot_syms, {q_u, q_r, q_t}, {0.00627778, 0.00313889, 0.00418519}));
C_hot = double(subs(C_hot_syms, {q_u, q_r, q_t}, {0.00627778, 0.00313889, 0.00418519}));


% Initial 1
u_0 = [30000; 30000];
x_0 = [0; 0; 0; 0];
A_sim = A_hot;
B_sim = B;
C_sim = C_hot;
D_sim = D;
out = sim("sample_project_openloop_linear.slx", 2400);
X = squeeze(out.X);
Y = squeeze(out.Y);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
title(sprintf("Open Loop Linear Model\nDry condition & Hot Weather\nState Response\n(volume of water in the soil/crops)"));
xlabel("Time (hour)");
ylabel("Response");
% ylim([-5, 5])
legend(["V_s,1","V_c,1","V_s,2","V_c,2"])
saveas(f, sprintf("sample_project/handout_2_question_2.9_openloop_linear_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', Y', 'LineWidth', 1.5);
title(sprintf("Open Loop Linear Model\nDry condition & Hot Weather\nOutput Response\n(volume of water collected by a nearby stream)"));
xlabel("Time (hour)");
ylabel("Response");
% ylim([-5, 5])
legend("y")
saveas(f, sprintf("sample_project/handout_2_question_2.9_openloop_linear_output_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Open Loop Linear Model\nDry condition & Hot Weather\nInput Response\n(inflow of water from the sprinkler)"));
xlabel("Time (hour)");
ylabel("Response");
% ylim([-5, 5])
legend(["u1", "u2"])
saveas(f, sprintf("sample_project/handout_2_question_2.9_openloop_linear_input_resp.png"));

%% Handout 3 Question 1.1
clc;

CTRB = rank(ctrb(A,B)) == 4;
fprintf("Controllability: %s\n", string(CTRB))

%% Handout 3 Question 1.2
clc;

u_0    = [0; 0];
x_stbl = [60000; 90000; 60000; 90000];

A_sim = A;
B_sim = B;
C_sim = C;
D_sim = D;

x_minus = [500 1000 2000 5000];
poles_sets = [
    -0.12+0.0001i, -0.12+0.0002i, -0.12-0.0001i, -0.12-0.0002i;
    -0.15+0.0001i, -0.15+0.0002i, -0.15-0.0001i, -0.15-0.0002i;
    -0.16+0.0001i, -0.16+0.0002i, -0.16-0.0001i, -0.16-0.0002i;
    -0.18+0.0001i, -0.18+0.0002i, -0.18-0.0001i, -0.18-0.0002i;
];
for i = 1:4
    x_0 = x_stbl - x_minus(i);
    % Eigenvalue Assignment
    poles = poles_sets(i,:);
    K = place(A, B, poles);

    out = sim("sample_project_closedloop_linear.slx", 48);
    X = squeeze(out.X);
    Y = squeeze(out.Y);
    U = squeeze(out.U);
    T = squeeze(out.tout);
    
    f = figure;
    f.Position = [0, 0, 600, 400];
    f.Visible = "off";
    plot(T', X', 'LineWidth', 1.5);
    yline(60000)
    yline(90000)
    title(sprintf("Closed Loop Linear Model\nLoss %sL Water\nState Response\n(volume of water in the soil/crops)", string(x_minus(i))));
    xlabel("Time (hour)");
    ylabel("Response");
    ylim([50000, 110000])
    legend(["V_s,1","V_c,1","V_s,2","V_c,2"])
    saveas(f, sprintf("sample_project/handout_3_question_1.2_closedloop_linear_init%s_state_resp.png", string(i)));

    f = figure;
    f.Position = [0, 0, 600, 400];
    f.Visible = "off";
    plot(T', U', 'LineWidth', 1.5);
    yline(0)
    title(sprintf("Closed Loop Linear Model\nLoss %sL Water\nInput Response\n(inflow of water from the sprinkler)", string(x_minus(i))));
    xlabel("Time (hour)");
    ylabel("Response");
    ylim([-10000, 30000])
    legend(["u1", "u2"])
    saveas(f, sprintf("sample_project/handout_3_question_1.2_closedloop_linear_init%s_input_resp.png", string(i)));
end

%% Handout 3 Question 1.3
clc;

u_0    = [0; 0];
x_stbl = [60000; 90000; 60000; 90000];
x_0    = x_stbl - 5000;

A_sim = A;
B_sim = B;
C_sim = C;
D_sim = D;

% LQR Control
Q = [
    1 0 0 0; % Penalize x1 error
    0 10 0 0; % Penalize X2 error
    0 0 1 0; % Penalize x3 error
    0 0 0 10; % Penalize x3 error
];
R = [
    10 0;   % Penalize u1 effort
    0 10;   % Penalize u2 effort
];
K = lqr(A, B, Q, R);

out = sim("sample_project_closedloop_linear.slx", 480);
X = squeeze(out.X);
Y = squeeze(out.Y);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
yline(60000)
yline(90000)
title(sprintf("Closed Loop Linear Model\nLoss %sL Water\nState Response\n(volume of water in the soil/crops)", "5000"));
xlabel("Time (hour)");
ylabel("Response");
ylim([50000, 110000])
legend(["V_s,1","V_c,1","V_s,2","V_c,2"])
saveas(f, sprintf("sample_project/handout_3_question_1.3_closedloop_linear_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
yline(0)
title(sprintf("Closed Loop Linear Model\nLoss %sL Water\nInput Response\n(inflow of water from the sprinkler)", "5000"));
xlabel("Time (hour)");
ylabel("Response");
ylim([-10000, 30000])
legend(["u1", "u2"])
saveas(f, sprintf("sample_project/handout_3_question_1.3_closedloop_linear_input_resp.png"));

%% Handout 3 Question 1.4
clc;

B_u2_fail = B(:,1);

%% Handout 3 Question 1.5
clc;

CTRB_u2_fail = rank(ctrb(A,B_u2_fail)) == 4;
fprintf("Controllability if u2 fails: %s\n", string(CTRB_u2_fail))

[A_bar,B_bar,C_bar,T] = ctrbf(A,B_u2_fail,C);
fprintf("A_bar:\n")
disp(A_bar)
fprintf("B_bar:\n")
disp(B_bar)
fprintf("C_bar:\n")
disp(C_bar)

STBL = [];
eigenvalues = eig(A);
j = 1;
for i = 1:4
    if eigenvalues(i)>0
        STBL(j) = rank([A-eigenvalues(i)*eye(4) B_u2_fail]) == 4;
        j = j + 1;
    end
end
fprintf("Stabilizability if u2 fails: %s\n", string(all(STBL)))

%% Handout 3 Question 1.6
clc;

u_0     = [0; 0];
x_stbl  = [60000; 90000; 60000; 90000];
x_0     = x_stbl - 500;

% u2 work
A_sim = A;
B_sim = B;
C_sim = C;
D_sim = D;

poles = [-0.12+0.0001i, -0.12+0.0002i, -0.12-0.0001i, -0.12-0.0002i];
K = place(A, B, poles);

out = sim("sample_project_closedloop_linear.slx", 48);
X = squeeze(out.X);
Y = squeeze(out.Y);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
yline(60000)
yline(90000)
title(sprintf("Closed Loop Linear Model\nLoss %sL Water & u2 Fail\nState Response\n(volume of water in the soil/crops)", "500"));
xlabel("Time (hour)");
ylabel("Response");
ylim([30000, 100000])
legend(["V_s,1","V_c,1","V_s,2","V_c,2"])
saveas(f, sprintf("sample_project/handout_3_question_1.6_closedloop_linear_u2_work_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', Y', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nLoss %sL Water & u2 Fail\nOutput Response\n(volume of water in the soil/crops)", "500"));
xlabel("Time (hour)");
ylabel("Response");
% ylim([30000, 100000])
legend("y")
saveas(f, sprintf("sample_project/handout_3_question_1.6_closedloop_linear_u2_work_output_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nLoss %sL Water & u2 Fail\nInput Response\n(inflow of water from the sprinkler)", "500"));
xlabel("Time (hour)");
ylabel("Response");
ylim([0, 14000])
legend(["u1", "u2"])
saveas(f, sprintf("sample_project/handout_3_question_1.6_closedloop_linear_u2_work_input_resp.png"));

% u2 fail
B_u2_fail = B;
B_u2_fail(:,2) = 0;

A_sim = A;
B_sim = B_u2_fail;
C_sim = C;
D_sim = D;

poles = [-0.12+0.0001i, -0.12+0.0002i, -0.12-0.0001i, -0.12-0.0002i];
K = place(A, B, poles);

out = sim("sample_project_closedloop_linear.slx", 48);
X = squeeze(out.X);
Y = squeeze(out.Y);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
yline(60000)
yline(90000)
title(sprintf("Closed Loop Linear Model\nLoss %sL Water & u2 Fail\nState Response\n(volume of water in the soil/crops)", "500"));
xlabel("Time (hour)");
ylabel("Response");
ylim([30000, 100000])
legend(["V_s,1","V_c,1","V_s,2","V_c,2"])
saveas(f, sprintf("sample_project/handout_3_question_1.6_closedloop_linear_u2_fail_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', Y', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nLoss %sL Water & u2 Fail\nOutput Response\n(volume of water in the soil/crops)", "500"));
xlabel("Time (hour)");
ylabel("Response");
% ylim([30000, 100000])
legend("y")
saveas(f, sprintf("sample_project/handout_3_question_1.6_closedloop_linear_u2_fail_output_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nLoss %sL Water & u2 Fail\nInput Response\n(inflow of water from the sprinkler)", "500"));
xlabel("Time (hour)");
ylabel("Response");
ylim([0, 14000])
legend(["u1", "u2"])
saveas(f, sprintf("sample_project/handout_3_question_1.6_closedloop_linear_u2_fail_input_resp.png"));

%% Handout 3 Question 1.6 - New Design
clc;

% u2 fail update
B_u2_fail = B;
B_u2_fail(:,2) = 0;

A_sim = A;
B_sim = B_u2_fail;
C_sim = C;
D_sim = D;

poles = [-0.12+0.0001i, -0.12+0.0002i, -0.12-0.0001i, -0.12-0.0002i];
K = place(A, B, poles);

out = sim("sample_project_closedloop_linear_unctrb.slx", 2400);
X = squeeze(out.X);
Y = squeeze(out.Y);
U = squeeze(out.U);
T = squeeze(out.tout);

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', X', 'LineWidth', 1.5);
yline(60000)
yline(90000)
title(sprintf("Closed Loop Linear Model\nLoss %sL Water & u2 Fail\nState Response\n(volume of water in the soil/crops)", "500"));
xlabel("Time (hour)");
ylabel("Response");
ylim([0, 180000])
legend(["V_s,1","V_c,1","V_s,2","V_c,2"])
saveas(f, sprintf("sample_project/handout_3_question_1.6_closedloop_linear_u2_fail_edit_state_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', Y', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nLoss %sL Water & u2 Fail\nOutput Response\n(volume of water in the soil/crops)", "500"));
xlabel("Time (hour)");
ylabel("Response");
% ylim([30000, 100000])
legend("y")
saveas(f, sprintf("sample_project/handout_3_question_1.6_closedloop_linear_u2_fail_edit_output_resp.png"));

f = figure;
f.Position = [0, 0, 600, 400];
f.Visible = "off";
plot(T', U', 'LineWidth', 1.5);
title(sprintf("Closed Loop Linear Model\nLoss %sL Water & u2 Fail\nInput Response\n(inflow of water from the sprinkler)", "500"));
xlabel("Time (hour)");
ylabel("Response");
ylim([0, 3000])
legend(["u1", "u2"])
saveas(f, sprintf("sample_project/handout_3_question_1.6_closedloop_linear_u2_fail_edit_input_resp.png"));

%% Handout 4 Question 1.1
clc;

OBSV = rank(obsv(A,C)) == 4;
fprintf("Observability: %s\n", string(OBSV))

%% Handout 4 Question 1.2
clc;

num_state = size(A, 1);
min_num_sensor = num_state - rank(obsv(A,C));
fprintf("Minimal Number of Sensors: %s\n", string(min_num_sensor))

%% Handout 4 Question 1.3
clc;

[~, U] = minreal(ss(A,B,C,D));

A_kd = U*A*U';
B_kd = U*B;
C_kd = C*U';
fprintf("Kalman Decomposition A:\n")
disp(A_kd)
fprintf("Kalman Decomposition B:\n")
disp(B_kd)
fprintf("Kalman Decomposition C:\n")
disp(C_kd)
