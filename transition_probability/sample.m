clear; clc;

initi  % start NATS
addpath('/home/pzhao28/Documents/Research/Software/Muti_Aircraft_Avoidance/transition_probability');
% set current state
h=[100,-50];  % initial relative height: ft
h_dot=[10 0];  % initial relative rate of climb: ft/s
tau=[10 30];  % initial time to collision: s

Scurrent=[h; h_dot; tau];

% initial speed: knots
v=[300 300 300];  % maginitude: knots
angel=[90/180 315/180 180/180];  % course angel: rad
V=[v; angel];

% action
a=5;

% collision point
P(1,:)=[-112, 35];
P(2,:)=[-111.9722409397640, 35];

futureS=f_collision(Scurrent,V,a,P,2);