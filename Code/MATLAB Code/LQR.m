%%=========================================================%%
%                   LQR Controller
%%=========================================================%%
%   Masters Thesis, Continuum Actuator, Mechatronics, KTH
%   Written by Seshagopalan T M and Ruihao Zhu
%   8th October 2020
%%=========================================================%%

Continuum_Model_Est;                                                      % Get the system model

%set(0,'DefaultFigureVisible','off');
set(0,'DefaultFigureVisible','on');

sys1 = ss(mp.A, mp.B, mp.C, mp.D);                                          % Create a copy of the state space system model

index = 1;                                                                  % Counter index

for i =0.001:0.001:5                                                        % For loop to iterate various optimization weights
    
Q = eye(size(sys1.A, 2)).*i;                                                % Q matrix to set optimization weight
R = 0.01;                                                                   % R matrix, need to test different values
K = lqr(sys1.A, sys1.B, Q, R);                                              % K matrix which satisfies the Ricati equation
sys2 = ss(sys1.A-sys1.B*K, sys1.B, sys1.C, sys1.D);                         % Closed loop system 'sys2' with no scaling

lsim_LQR =  lsim(sys2, [Reference_A, Reference_B, Reference_C], Time/1.5);    % Apply the reference with half time on the system

y_A = lsim_LQR(:,1);                                                        % Achieved position of motor A
y_B = lsim_LQR(:,2);                                                        % Achieved position of motor B
y_C = lsim_LQR(:,3);                                                        % Achieved position of motor C

RMSE_A = sqrt(mean((y_A - Reference_A).^2));                                % RMS error of achieved position with respect to reference
RMSE_B = sqrt(mean((y_B - Reference_B).^2));                                % RMS error of achieved position with respect to reference
RMSE_C = sqrt(mean((y_C - Reference_C).^2));                                % RMS error of achieved position with respect to reference
RMSE_mean = (RMSE_A+ RMSE_B+ RMSE_C)/3;                                     % Average of RMS errors of the 3 motors

Effort_A = sum(abs(gradient(gradient(y_A - Reference_A))));                 % Calculate the control effort, defined as integral acceleration
Effort_B = sum(abs(gradient(gradient(y_B - Reference_B))));                 % Calculate the control effort, defined as integral acceleration
Effort_C = sum(abs(gradient(gradient(y_C - Reference_C))));                 % Calculate the control effort, defined as integral acceleration
Effort_mean = (Effort_A+ Effort_B+ Effort_C)/3;                             % Average of the control effor of the 3 motors

Parameters(index,1) = RMSE_mean;                                            % Add RMS mean to a matrix
Parameters(index,2) = Effort_mean;                                          % Add control effot mean to a matrix
Parameters_Minima(index) = sqrt((RMSE_mean^2)+(Effort_mean^2));             % Find the distance to the origin, the lowest distance is the first local minima

index = index + 1;                                                          % Increment index
end

figure;                                                                     % Declare a new figure
plot(Parameters(:,1), Parameters(:,2))                                      % Plot the RMS and Control Effort mean

Minima = find(Parameters_Minima == min(Parameters_Minima));                 % Find the minimal value for the Q parameter

Q = eye(size(sys1.A, 2)).*(Minima*0.001);                                   % Re-establish Q matrix with this minimal value
K = lqr(sys1.A, sys1.B, Q, R);                                              % Re-calculate K matrix
sys2 = ss(sys1.A-sys1.B*K, sys1.B, sys1.C, sys1.D);                         % Re-write the system with the minimal Q parameter

figure;                                                                     % Declare a new figure
lsim(sys2, [Reference_A, Reference_B, Reference_C], Time/1);                % Plot the system with respect to reference with half the time

Kdc = dcgain(sys2);                                                         % Calculate the DC gain of the closed loop system
Kr = inv(Kdc);                                                              % Find the necessary gain compensation matrix to get a closed loop gain of 1
sys3 = sys2;                                                                % Create a new system which will contain the gain compensation
sys3.B = sys2.B*Kr;                                                         % Provide the system with the gain compensation

figure;                                                                     % Declare a new figure
lsim(sys3, [Reference_A, Reference_B, Reference_C], Time/1);                % Plot the system with respect to reference with half the time

figure;                                                                     % Declare a new figure
step(sys2, sys3)                                                            % Compare the step responses
