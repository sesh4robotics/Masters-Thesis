%%=========================================================%%
%           Model Estimation Based on Data
%%=========================================================%%
%   Masters Thesis, Continuum Actuator, Mechatronics, KTH
%   Written by Seshagopalan T M and Ruihao Zhu
%   8th October 2020
%%=========================================================%%

set(0,'DefaultFigureVisible','on');

clear;
close all;

load Continuum_Actuator_Model.mat;

Motor_Position_A = Motor_Position_A - 56.5;
Motor_Position_B = Motor_Position_B - 56.5;
Motor_Position_C = Motor_Position_C - 56.5;

Reference_A = Reference_A - 56.5;
Reference_B = Reference_B - 56.5;
Reference_C = Reference_C - 56.5;

Time = Time - Time(1);

continuumactuator = iddata([Motor_Position_A, Motor_Position_B, ...
    Motor_Position_C],[Reference_A, Reference_B, Reference_C],0.05);
continuumactuator.InputName  = {'AngleRefA'; 'AngleRefB'; 'AngleRefC'};
continuumactuator.OutputName = {'PositionOutA'; 'PositionOutB'; ...
    'PositionOutC'};
mp = ssest(continuumactuator(500:2000), 3, 'form', 'canonical', ...
    'DisturbanceModel', 'none');

figure;
step(continuumactuator)

figure;
step(mp)

figure;
compare(continuumactuator, mp)

figure;
lsim(mp, [Reference_A, Reference_B, Reference_C], Time)