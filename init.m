%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                CentraleSupelec - Elective course - 2020                %
%                     "Interactive Robotic Systems"                      %
%                                                                        %
%                                                                        %
%                            Simulink model                              %
%                              TD 1 and 2                                %
%                              Students :                                %
%                     João Victor EVANGELISTA MATOSO                     %
%                   Vítor ALBUQUERQUE MARANHÃO RIBEIRO                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1: Initialize simulation by running init.m
% 2: Run simulink simulation

clear all; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Parameters initialization  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Parameters for the display of plots:
displayQ5 = false;
displayQ7 = false;
displayQ9 = false;
displayQ10 = false;
displayQ11 = false;

% Robot parameters
% Geometric parameters from table 1 (TD1)
d3 = 0.7; %[m]
r1 = 0.5; %[m]
r4 = 0.2; %[m]
rE = 0.1; %[m]

alphas = [0 ; pi/2; 0  ; pi/2; -pi/2; pi/2;  0];
ds =     [0 ;   0 ; d3 ;  0  ;    0 ;   0 ;  0];
rs =     [r1;   0 ; 0  ; r4  ;    0 ;   0 ; rE];
Offset = [0; 0; pi/2; 0; 0; 0];

[N_joints,~] = size(alphas);
N_joints = N_joints - 1;     % Discounting the end-effect parameters

% Geometric parameters from table 1 (TD2)
G1 = [  0  ;  0  ;-0.25]; % Coordinates of G1 given in frame R1 [m]
G2 = [0.35 ;  0  ;   0 ]; % Coordinates of G2 given in frame R2 [m]
G3 = [  0  ; -0.1;   0 ]; % Coordinates of G3 given in frame R3 [m]
G4 = [  0  ;  0  ;   0 ]; % Coordinates of G4 given in frame R4 [m]
G5 = [  0  ;  0  ;   0 ]; % Coordinates of G5 given in frame R5 [m]
G6 = [  0  ;  0  ;   0 ]; % Coordinates of G6 given in frame R6 [m]
Gs = [G1,G2,G3,G4,G5,G6];

m1 = 15.0; % Mass of the body 1 [kg]
m2 = 10.0; % Mass of the body 2 [kg]
m3 =  1.0; % Mass of the body 3 [kg]
m4 =  7.0; % Mass of the body 4 [kg]
m5 =  1.0; % Mass of the body 5 [kg]
m6 =  0.5; % Mass of the body 6 [kg]
m = [m1;m2;m3;m4;m5;m6];

I1 = [0.80 ,   0  , 0.05;
        0  , 0.80 ,  0  ;  % Inertial tensor of the body 1 
      0.05 ,   0  , 0.10]; % in the R1 frame[kg*m^2]

I2 = [0.10 ,   0  , 0.10;
        0  , 1.50 ,  0  ;  % Inertial tensor of the body 2 [kg*m^2]
      0.10 ,   0  , 1.50]; % in the R2 frame[kg*m^2]
  
I3 = [0.05 ,   0  ,  0 ;
        0  , 0.01 ,  0 ;   % Inertial tensor of the body 3 [kg*m^2]
        0  ,   0  ,0.05];  % in the R3 frame[kg*m^2]
  
I4 = [0.50 ,   0  ,  0  ;
        0  , 0.50 ,  0  ;  % Inertial tensor of the body 4 [kg*m^2]
        0  ,   0  , 0.05]; % in the R4 frame[kg*m^2]
  
I5 = [0.01 ,   0  ,  0  ;
        0  , 0.01 ,  0  ;  % Inertial tensor of the body 5 [kg*m^2]
        0  ,   0  , 0.01]; % in the R5 frame[kg*m^2]
  
I6 = [0.01 ,   0  ,  0  ;
        0  , 0.01 ,  0  ;  % Inertial tensor of the body 6 [kg*m^2]
        0  ,   0  , 0.01]; % in the R6 frame[kg*m^2]

I = cat(3, I1,I2,I3,I4,I5,I6);

Jm = 10e-6.*[1;1;1;1;1;1]; % Moment of inertia of the actuator rotor [kg*m^2]

red = [100;100;100;70;70;70]; % Reduction ratio

Fv = [10;10;10;10;10;10]; % Joint viscous frictions [N*m*(rad^-1)*s]

tmax = [5;5;5;5;5;5]; % Maximal motor torques [N*m]

Gravity = [0;0;-9.81]; %Acceleration of gravity in the reference frame [m*s^-2]

% Trajectory
q_dot_i = [0;0;0;0;0;0];    % [rad/s] Initial velocity
q_dot_f = [0;0;0;0;0;0];    % [rad/s] Final velocity

qdi = [-1;0;-1;-1;-1;-1];   % [rad] Initial position
qdf = [0;1;0;0;0;0];        % [rad] Final position

% Exercise 19
Ts = 1e-4;      % [s] Sample time
tf = 0.5;       % [s] Minimum time

q_dot_max = 15/8.*(qdf - qdi)./tf;
q_dot_dot_max = (10/sqrt(3)) .* (qdf-qdi)./(tf^2);

% Exercise 20
Kp = [5000;8000;5500;800;5500;2500];
Kd = [1000;1000;500;100;100;500];

%dhparams = [0    , 0  ,    q1     , r1;
%            pi/2 , 0  ,    q2     , 0 ;
%            0    , d3 , q3 + pi/2 , 0 ;
%            pi/2 , 0  ,    q4     , r4;
%            -pi/2, 0  ,    q5     , 0 ;
%            pi/2 , 0  ,    q6     , 0 ;
%            0    , 0  ,    0      , rE];