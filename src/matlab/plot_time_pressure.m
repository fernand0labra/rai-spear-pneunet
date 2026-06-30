close all
clear all
clc

%% PRESSURE to SOFTDRONE
dt = 0.01;                      % Sampling interval (s)
fs = 1/dt;                      % Sampling frequency (Hz)
n = 800;                        % Number of samples per ramp (for total ~40 s)
pause_time = 2;                 % Pause duration (s)
pause_len = round(fs * pause_time);

p_max = 1.3;                    % Maximum pressure
y = linspace(0, p_max/2, n);    % Pressure ramp

%% Initialize arrays
pd1_all = [];
pd2_all = [];

%% ---- 1st Phase ---- (p1 down, p2 up)
for i = 1:n
    pd_1 = p_max/2 - y(i);
    pd_2 = p_max/2 + y(i);
    pd1_all(end+1) = pd_1;
    pd2_all(end+1) = pd_2;
end
pd1_all = [pd1_all, repmat(pd1_all(end), 1, pause_len)];
pd2_all = [pd2_all, repmat(pd2_all(end), 1, pause_len)];

%% ---- 2nd Phase ---- (p1 up, p2 down)
for i = 1:n
    pd_1 = y(i);
    pd_2 = p_max - y(i);
    pd1_all(end+1) = pd_1;
    pd2_all(end+1) = pd_2;
end
pd1_all = [pd1_all, repmat(pd1_all(end), 1, pause_len)];
pd2_all = [pd2_all, repmat(pd2_all(end), 1, pause_len)];

%% ---- 3rd Phase ---- (p1 up, p2 down reversed)
for i = 1:n
    pd_1 = p_max/2 + y(i);
    pd_2 = p_max/2 - y(i);
    pd1_all(end+1) = pd_1;
    pd2_all(end+1) = pd_2;
end
pd1_all = [pd1_all, repmat(pd1_all(end), 1, pause_len)];
pd2_all = [pd2_all, repmat(pd2_all(end), 1, pause_len)];

%% ---- 4th Phase ---- (p1 high to low, p2 low to high)
for i = 1:n
    pd_1 = p_max - y(i);
    pd_2 = y(i);
    pd1_all(end+1) = pd_1;
    pd2_all(end+1) = pd_2;
end
pd1_all = [pd1_all, repmat(pd1_all(end), 1, pause_len)];
pd2_all = [pd2_all, repmat(pd2_all(end), 1, pause_len)];

%% ---- Time vector ----
t = (0:length(pd1_all)-1) / fs;

%% ---- Plot results ----
figure;
plot(t, pd1_all, 'b', 'LineWidth', 1.5); hold on;
plot(t, pd2_all, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Pressure (KPa)');
legend('pd_1', 'pd_2', 'Location', 'Best');
grid on;

%% ---- Mark phase boundaries ----
phase1_end = n + pause_len;
phase2_end = phase1_end + n + pause_len;
phase3_end = phase2_end + n + pause_len;
phase4_end = phase3_end + n + pause_len;

%% ---- Verify total time ----
total_time = length(pd1_all) / fs;
fprintf('Total duration: %.2f seconds\n', total_time);
