%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                CentraleSupelec - Elective course - 2020                %
%                     "Interactive Robotic Systems"                      %
%                                                                        %
%                              TD 1 and 2                                %
%                              Students :                                %
%                     João Victor EVANGELISTA MATOSO                     %
%                   Vítor ALBUQUERQUE MARANHÃO RIBEIRO                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; close all; clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Parameters initialization  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Parameters for the display of plots:

displayQ5 = true;
displayQ7 = true;
displayQ9 = true; % setting this to false slows greatly the runtime
animation_rate = 47; % the step of frames to skip during the animation of Q9
%animation_rate = 127; % set it equal to multiples of 2, 5 or 127 to have the
%final figure of the animation on the final time
displayQ10 = true;
displayQ11 = true;

% Geometric parameters from table 1 (TD1)
d3 = 0.7; %[m]
r1 = 0.5; %[m]
r4 = 0.2; %[m]
rE = 0.1; %[m]

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

g = [0;0;-9.81]; %Acceleration of gravity in the reference frame [m*s^-2]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Direct geometric model     %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Geometric parameters of the robot frp,

%dhparams = [0    , 0  ,    q1     , r1;
%            pi/2 , 0  ,    q2     , 0 ;
%            0    , d3 , q3 + pi/2 , 0 ;
%            pi/2 , 0  ,    q4     , r4;
%            -pi/2, 0  ,    q5     , 0 ;
%            pi/2 , 0  ,    q6     , 0 ;
%            0    , 0  ,    0      , rE];

alphas = [0 ; pi/2; 0  ; pi/2; -pi/2; pi/2;  0];
ds =     [0 ;   0 ; d3 ;  0  ;    0 ;   0 ;  0];
rs =     [r1;   0 ; 0  ; r4  ;    0 ;   0 ; rE];
[N_joints,~] = size(alphas);
N_joints = N_joints -1; % Discounting the end-effect parameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Direct kinematic model     %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Question 4

% For the configuration i

% Vector of thetas
q_i = do_offset_theta([-pi/2; 0; -pi/2; -pi/2; -pi/2; -pi/2]);

% Computation of the DGM. X = [px, py, pz, alpha, betta, gamma]'
[X_i,R0E_i] = ComputeDGM(alphas, ds, [q_i;0] , rs); %[q_i;0] for accounting the E reference
p_i = X_i(1:3,1)
euler_angles_i = X_i(4:6,1);

[ang_rot_qi, vec_rot_qi] = inv_Rot(R0E_i)

% Configuration f (same process as before)
q_f = do_offset_theta([0; pi/4; 0; pi/2; pi/2; 0]);
[X_f,R0E_f] = ComputeDGM(alphas, ds, [q_f;0], rs);
p_f = X_f(1:3,1)
euler_angles_f = X_f(4:6,1);

[ang_rot_qf, vec_rot_qf] = inv_Rot(R0E_f)

% Question 5

if displayQ5
    PlotFrame(alphas,ds,q_i,rs);
    PlotFrame(alphas,ds,q_f,rs);
end

% Question 6
q_dot = [0.5; 1.0; -0.5; 0.5; 1.0; -0.5];

J_0_i = ComputeJac(alphas, ds, [q_i;0], rs);
J_0_f = ComputeJac(alphas, ds, [q_f;0], rs);

twist_qi = J_0_i * q_dot
twist_qf = J_0_f * q_dot

% Question 7

if displayQ7
    ellips = true;
    PlotFrame(alphas,ds,q_i,rs,ellips);
    PlotFrame(alphas,ds,q_f,rs,ellips);
end

% 1) What is the preferred direction to transmit velocity in the task space?
%  - The first principal axis (i.e: U(:,1))
% 2) What are the corresponding velocity manipulabilities?
%  - The first eigen value of S (i.e: S(1,1))

% Configuration qi
J_0_v_i = J_0_i(1:3,:);
[U_i,S_i,~] = svd(J_0_v_i);
preffered_direction_qi = U_i(:,1)
i = 1;

% Performing the productory over the diagonal of S to compute the volume of
% the ellipsoid
vol_i = 1;
while (i<size(S_i,1)) && (S_i(i,i)>0)
    vol_i = vol_i * S_i(i,i);
    i = i + 1;
end
velocity_manipulability_qi = vol_i % the volume

% Configuration qf --> same reasoning
J_0_v_f = J_0_f(1:3,:);
[U_f,S_f,~] = svd(J_0_v_f);
preffered_direction_qf = U_f(:,1)
vel_manip_f = S_f(1,1);
vol_f = 1;
while (i<size(S_i,1)) && (S_i(i,i)>0)
    vol_f = vol_f * S_fi(i,i);
    i = i + 1;
end
velocity_manipulability_qf = vol_f


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Inverse geometric model    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Question 8
% Question 8.1
Xd_i = [-0.1; -0.7; 0.3]; % Xi desired
q0_i = do_offset_theta([-1.57; 0; -1.47; -1.47; -1.47; -1.47]);
kmax = 100;
error = 1e-3; %[mm]

%met1 = 'inverse';
%met2 = 'gradient';

tic;
q_i = ComputeIGM(Xd_i, q0_i, kmax, error, alphas, ds, rs);
t_calc_i_met1 = toc;
[Xc_i,~] = ComputeDGM(alphas, ds, [q_i;0], rs);
% Calculation error of the IGM on case i with method 1 (Inverse)
calc_error_i_met1 = norm(Xd_i - Xc_i(1:3));

tic;
q_i = ComputeIGM(Xd_i, q0_i, kmax, error, alphas, ds, rs,'gradient');
t_calc_i_met2 = toc;
[Xc_i,~] = ComputeDGM(alphas, ds, [q_i;0], rs);
% Calculation error of the IGM on case i with method 2 (Gradient)
calc_error_i_met2 = norm(Xd_i - Xc_i(1:3));

% Question 8.2
Xd_f = [0.64; -0.10; 1.14]; % Xf desired
q0_f = do_offset_theta([0; 0.8; 0; 1; 2; 0]);

tic;
q_f = ComputeIGM(Xd_f, q0_f, kmax, error, alphas, ds, rs, 'inverse');
t_calc_f_met1 = toc;
[Xc_f,~] = ComputeDGM(alphas, ds, [q_f;0], rs); % Xf calulated
% Calculation error of the IGM on case f with method 1 (Inverse)
calc_error_f_met1 = norm(Xd_f - Xc_f(1:3));

tic;
q_f = ComputeIGM(Xd_f, q0_f, kmax, error, alphas, ds, rs,'gradient');
t_calc_f_met2 = toc;
[Xc_f,~] = ComputeDGM(alphas, ds, [q_f;0], rs);
% Calculation error of the IGM on case f with method 2 (Gradient)
calc_error_f_met2 = norm(Xd_f - Xc_f(1:3));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Inverse kinematic model    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Question 9
V = 1;         % Norm of the velocity [m/s]
Ts = 1e-3;     % Sampling time [s]
Xd_i = [-0.1; -0.7; 0.3]; % Desired initial point on the task space [m]
Xd_f = [0.64; -0.10; 1.14]; % Desired final point on the task space [m]
q_i = do_offset_theta([-pi/2; 0; -pi/2; -pi/2; -pi/2; -pi/2]);

% Computing the IKM
qd = ComputeIKM(Xd_i, Xd_f, V, Ts, q_i,alphas,ds,rs, kmax, error);

if displayQ9
   PlotFrameQ9(qd,Xd_i, Xd_f,Ts,alphas,ds,rs,animation_rate);
end

% Undoing the offset of theta done for the calculations
for i = 1:size(qd,2)
    qd(:,i) = undo_offset_theta(qd(:,i));
end

% Question 10
q_min = [-pi; -pi/2 ; -pi ;  -pi ; -pi/2 ;  -pi];
q_max = [ 0 ;  pi/2 ;  0  ; pi/2 ;  pi/2 ; pi/2];

if displayQ10
    PlotQs(qd,Ts,q_min,q_max);
end

% Question 11
alpha = 1;
q_min_offset = do_offset_theta(q_min);
q_max_offset = do_offset_theta(q_max);
q_limits = ComputeIKMlimits(Xd_i, Xd_f, V, Ts,q_i,alpha,q_min_offset,q_max_offset,...
                             alphas,ds,rs, kmax, error);
                         
for i = 1:size(qd,2)
q_limits(:,i) = undo_offset_theta(q_limits(:,i));
end

if displayQ11
    PlotQs(q_limits,Ts,q_min,q_max);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Dynamic model              %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exercise 12
% Example of JvG6 and JwG6 for q = q_test
q_test = do_offset_theta([pi/4; -pi/8; -pi/3 ; 0; -pi/2; -pi/7]);
[Jv0G6_test,Jw0G6_test] = ComputeJacGi(alphas, ds, q_test, rs, Gs, 6)

% Exercise 13
A_q_i = ComputeMatInert(q_i,Gs,m,I,red,Jm,alphas,ds,rs);
A_q_f = ComputeMatInert(q_f,Gs,m,I,red,Jm,alphas,ds,rs);
A_q_test = ComputeMatInert(q_test,Gs,m,I,red,Jm,alphas,ds,rs)

% Exercise 14 and 16
% For revolute joints, mi_1 and mi_2 for the lower and the upper
% bounds of the inertia matrix are constant
% [Slide 365] these scalar quantities can be defined as the minimal and maximal
% eigenvalues (Lamb_min, Lamb_max) of A(q), for all q in [q_min,q_max]

% Performing a sweep on the allowed values of q between q_max and q_min

n_interval = 4; % Size of the mesh of the sweep

% all_qs = q_min:n_interval:q_max
all_qs = zeros(N_joints,n_interval);
for j =1:N_joints
    all_qs(j,:) = linspace(q_min_offset(j),q_max_offset(j),n_interval);
end

% Getting all the combination of indexes N_interval 6 to 6, with repetition
all_index_combinations = permn(1:n_interval,6);

% Variables to store the maximum and minimum values found during the loop
eig_max = 0;
eig_min = inf;
g_max = 0;

for i = 1:size(all_index_combinations,1) % For each combination
    indexes = all_index_combinations(i,:);
    q_sweep = zeros(N_joints,1);
    for j = 1:N_joints % for each joint
        q_sweep(j) = all_qs(j,indexes(j));
    end

    %Computing the functions we want to analyze
    A_sweep = ComputeMatInert(q_sweep,Gs,m,I,red,Jm,alphas,ds,rs);
    eig_A = eig(A_sweep); % calculating the eigen values of A

    G_sweep = ComputeGravTorque(q_sweep,alphas, ds, rs, Gs, m,g);
    G_norm1 = norm(G_sweep,1);

    % updating eig_max,eig_min and g_max
    if max(eig_A) > eig_max
        eig_max = max(eig_A);
    end
    if min(eig_A) < eig_min
        eig_min = min(eig_A);
    end
    if G_norm1 > g_max
        g_max = G_norm1;
    end
end

mi_2 = eig_max
mi_1 = eig_min
g_b = g_max

% Exercise 15

G_q_i = ComputeGravTorque(q_i,alphas, ds, rs, Gs, m,g);
G_q_f = ComputeGravTorque(q_f,alphas, ds, rs, Gs, m,g);


% Exercise 17
% Example of the joint friction for q_dot = [0.5; 1.0; -0.5; 0.5; 1.0; -0.5];
fric = ComputeFrictionTorque(q_dot,Fv);

% Simulink

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Trajectory generation in the joint space     %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exercise 18
q_dot_i = [0;0;0;0;0;0];    % [rad/s] Initial velocity
q_dot_f = [0;0;0;0;0;0];    % [rad/s] Final velocity

qdi = [-1;0;-1;-1;-1;-1];   % [rad] Initial position
qdf = [0;1;0;0;0;0];        % [rad] Final position

D = qdf - qdi;
ka = tmax.*red/mi_2

tf = sqrt(10*abs(D)./(sqrt(3)*ka));
tf_min = max(tf)

% Exercise 19
% Simulink

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Joint control law          %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulink


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   TD Functions               %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function g = TransformMatElem(a,d,t,r)
% Exercise 3.a (taken from slide 96)
% Computes the homogeneous transform matrix g between two successive frames
% Inputs: (alpha [rad] , d [m] , theta [rad] , r [m]) on MDH convention

% Expression: g = R_x(alpha)*Trans(x,di)*Rz(theta)*Trans(z,r)

% g = Rot_x(a)*Trans([d;0;0])*Rot_z(t)*Trans([0;0;r]);

% Alternative way of computation (faster computation)
g = [cos(t)       , -sin(t)      , 0      , d        ;
     cos(a)*sin(t), cos(a)*cos(t), -sin(a), -r*sin(a);
     sin(a)*sin(t), sin(a)*cos(t), cos(a) , r*cos(a) ;
     0            , 0            , 0      , 1        ];
end


function [X, R0N] = ComputeDGM(alphas, ds, thetas, rs)
% Exercise 3.b (taken from slide 97)
% Computes the Direct geometric model g between two successive frames
% Inputs: (alphas [rad] , ds [m] , thetas [rad] , rs [m]) on MDH convention
%          each with dimension N x 1
% Output: X(q) = [px(q),py(q),pz(q),alpha(q),betta(q),gamma(q)]
%                                  (with the Euler angles convention)

[N_frames,~] = size(alphas);

% Computation of g0n recursively
g = 1;
for i = 1:N_frames
    g = g * TransformMatElem(alphas(i), ds(i), thetas(i), rs(i));
end

R0N = g(1:3,1:3);
euler_angs = get_euler_angles(R0N);
p0N = g(1:3,4);
X = [p0N; euler_angs];
end


function Jac = ComputeJac(a,d,t,r)
% Exercise 6 (taken from slide 107)
% Computes the Jacobian matrix of a robot tree with only revolute joints
% Inputs: (alphas [rad] , ds [m] , thetas [rad] , rs [m]) on MDH convention
%          with dimension N_joints + 1 x 1 (due to the end-effector)
% Output: J_0(q) (in R6xN)

N_joints = size(a,1) -1;
J = zeros(6,N_joints);
Zi = [0;0;1]; % The unit vector of the i-th joint given in frame Ri
g = 1;

[X0E,~] = ComputeDGM(a,d,t,r); % counting the end-effect
p0E = X0E(1:3);

for i = 1:N_joints
    g = g * TransformMatElem(a(i),d(i),t(i),r(i));
    R0i = g(1:3,1:3);
    p0i = g(1:3,4);
    
    pin = p0E - p0i;
    
    J(:,i) = [cross(R0i*Zi, pin);
                R0i*Zi]; % Since they are all revolute joints
end

Jac = J;
end


function q = ComputeIGM(Xd, q0, kmax, error, alphas, ds, rs, method)
% Question 8
% Computes de Inverse Geometric Model of the robot tree by an iterative
% Calculates the coordinates of q for a given destination position Xd
% algotithm
% Input: (Xd --> desired task position on the zero reference frame
%         q0 --> initial conditions for the degrees of freedom of the joints
%         kmax (in N*) --> maximum number of iterations
%         error [m] (in R>0) --> tolerated Cartesian error
%        (alphas[rad],ds[m],rs[m]) --> robot tree parameters on MDH convention
% Optional: method = 'inverse' (Default) or method = 'gradient'
if nargin<8
  method = 'inverse';
end

k = 0;
qk = q0;
alp = 1.2;

[Xk,~] = ComputeDGM(alphas, ds, [q0;0], rs);

while (k < kmax) && (norm(Xd - Xk(1:3)) > error)
    k = k + 1;
    
    Jac_k = ComputeJac(alphas, ds, [qk;0], rs);
    
    % Newton-Raphson method (for M = N)
    if strcmp(method,'inverse')
        qk = qk + pinv(Jac_k(1:3,:))*(Xd - Xk(1:3));
    elseif strcmp(method,'gradient')
        % Gradient-based method
        qk = qk + alp*Jac_k(1:3,:)'*(Xd - Xk(1:3));
    end
    [Xk,~] = ComputeDGM(alphas, ds, [qk;0], rs);
end

q = qk;

end


function q_d = ComputeIKM(Xd_i, Xd_f, V, Ts, q_i,alphas,ds,rs, kmax, error)
% Question 9 - Computation of the Inverse Kinematic Model
% Provides the series of setpoint values qdk corresponding
% to the Xdk to the joint drivers
% using the inverse differential kinematic model.
% Input: (Xdi --> initial task position on the zero reference frame
%         Xdf --> final task position on the zero reference frame
%         V [m/s] (in R>0) --> tolerated Cartesian error
%         Ts [s] (in R>0) --> sampling time 
%         qi --> the robot's initial configuration
%        (alphas[rad],ds[m],rs[m]) --> robot tree parameters on MDH convention
%         kmax (in N*) --> maximum number of iterations
%         error [m] (in R>0) --> tolerated Cartesian error

% Calculating the X trajectory
dist = norm(Xd_f - Xd_i); % distance between the points
Tf = dist/V;
t = 0:Ts:Tf;
N_t = length(t); %number of samples during the time
p = linspace(0,1,N_t)';
Xd = ((1-p)*Xd_i' + p*(Xd_f'))'; %3d linear parametrization for a line

q = zeros(size(q_i,1),size(Xd,2)); %q is in R^(6xN_t)

% Initializing
q(:,1) = q_i;

for k=2:size(Xd,2)
    q(:,k) = ComputeIGM(Xd(:,k), q(:,k-1), kmax, error, alphas, ds, rs);
end

q_d = q;
end


function q_lim = ComputeIKMlimits(Xd_i, Xd_f, V, Ts,q_i,alpha, q_min, q_max,alphas,ds,rs, kmax, error)
% Question 11 - Computation of the Inverse Kinematic Model with a Secondary
% task (slide 138)
% Provides the series of setpoint values qdk corresponding
% to the Xdk to the joint drivers
% using the inverse differential kinematic model and considering the
% joints' limits as a secondary task on the task space
% Input: (Xdi --> initial task position on the zero reference frame
%         Xdf --> final task position on the zero reference frame
%         V [m/s] (in R>0) --> tolerated Cartesian error
%         Ts [s] (in R>0) --> sampling time 
%         qi --> the robot's initial configuration
%         alfa (in R>0) --> tradeoff between primary and secondary tasks
%         q_min, q_max --> the minimum and maximum geometric values for each joint
%        (alphas[rad],ds[m],rs[m]) --> robot tree parameters on MDH convention
%         kmax (in N*) --> maximum number of iterations
%         error [m] (in R>0) --> tolerated Cartesian error

% Calculating the Xd desired trajectory
dist = norm(Xd_f - Xd_i); % distance between the points
Tf = dist/V;
t = 0:Ts:Tf;
N_t = length(t);
p = linspace(0,1,N_t)';
Xd = ((1-p)*Xd_i' + p*(Xd_f'))'; %3d linear parametrization for a line

% Getting the Xd_dot
dir = (Xd_f - Xd_i)/dist; % unitary direction vector
Xd_dot = V*dir(:).*ones(size(Xd));

% Getting the desired joint coordinates for Xd
qd = ComputeIKM(Xd_i, Xd_f, V, Ts, q_i,alphas,ds,rs, kmax, error);

q_dot = zeros(size(q_i,1),size(Xd_dot,2));

N = size(qd,1);
W = eye(N); %weighting matrix
q_limits_square = (q_max - q_min).^2;
q_bar = (q_max(:) + q_min(:))./2;

for k = 1:size(q_dot,2)
    qd_k = qd(:,k);
    J = ComputeJac(alphas, ds, [qd_k;0], rs); %J(qd_k)
    J_v = J(1:3,:);
    
    Jinv_W = inv(W)*(J_v')*inv(J_v*inv(W)*(J_v'));
    %Jinv_W = pinv(J_v); %alternative calculation if W = eye(N)
    
    q_dot0 = zeros(N,1);
    for j = 1:N
        % The gradient of H_lim(q)
        q_dot0(j) = -alpha*2*(qd_k(j) - q_bar(j))/q_limits_square(j);
    end
    
    homog_sol = (eye(N) - Jinv_W*J_v)*q_dot0;
    
    q_dot(:,k) = Jinv_W*Xd_dot(:,k) + homog_sol;
end

% Integrating q_dot into q
q = zeros(size(q_dot));
for i = 1:size(q_dot,1)
    q(i,:) = cumtrapz(t,q_dot(i,:));
end

q_lim = q(:,:) + q_i; % Adding the initial value
end


function [JvGi,JwGi] = ComputeJacGi(alphas, ds, thetas, rs, Gs, n)
% Exercise 12
% Slide 168
% Returns the Jacobian matrix with respect center of mass of the body Ci and no longer the
% velocity of the origin of the robot's terminal frame
% Inputs: (alphas [rad] , ds [m] , rs [m]) on MDH convention
%               each with dimension (N+1) x 1 with the End-effector's
%               parameters
%          thetas [rad] (in R nx1) --> joint-coordinates of the robot
%               (without the end-effector's parameter included)
%          Gs (in R 3xN) --> position of the barycenter of the bodies in
%               relation to their respective reference frames
%               i.e: OiGi in ref. frame Ri
%          i (in N*) between 1 and N --> index of the body to compute the
%               JacGi

J0E = ComputeJac(alphas,ds,[thetas;0],rs);
% p0E = Vector O0Oe in the 0 reference frame
[p0E,~] = ComputeDGM(alphas, ds, [thetas;0], rs); 

JGi = zeros(size(J0E));

% Getting the DGM until body i (g0i)
g = 1;
for i=1:n
    g = g * TransformMatElem(alphas(i),ds(i),thetas(i),rs(i));
end

O0Oi_0 = g(1:3,4); % Vector O0Oi in the 0 reference frame

pOiGi_0 = g*[Gs(:,n);0]; 
pOiGi_0 = pOiGi_0(1:3); % Vector OiGi in the 0 reference frame

Oe_Gi = O0Oi_0 + pOiGi_0 - p0E(1:3); % Vector OeGi in 0 reference frame
Oe_Gi_matrix = vec_to_cross_prod(Oe_Gi);

% Varignon formula : V(Gi) = V(OE) + w x OiGi (in matrix form) 
JGi_aux = [ eye(3)  , -Oe_Gi_matrix;
            zeros(3),      eye(3)  ]*J0E;

JGi(:,1:n) = JGi_aux(:,1:n);

JvGi = JGi(1:3,:);
JwGi = JGi(4:6,:);
end


function A_q = ComputeMatInert(q,Gs,m,I,red,Jm,alphas,ds,rs)
% Exercise 13
% Computes the Inertia matrix of the manipulator A(q) in Rnxn
% Inputs:  q [rad] (in R nx1) --> joint-coordinates of the robot
%               (without the end-effector's parameter included)
%          Gs [m] (in R 3xN) --> position of the barycenter of the bodies
%               in relation to their respective reference frames
%          m [kg] (in RNx1) --> mass of the bodies
%          I [kg*m^2] (R3x3xN) --> table of the matrixes of the intertia 
%               tensors of each body referenced to the point Oi 
%               on the reference frame Ri
%          red (in RNx1) --> Reduction ratio of each actuator rotor
%          Jm (in RNx1) --> Moment of inertia of each actuator rotor
%          (alphas [rad] , ds [m] , rs [m]) on MDH convention
%               each with dimension (N+1) x 1 with the End-effector's
%               parameters

N = size(q,1);
A = zeros(N);
g = 1;

for i = 1:N
    %g0i
    g = g * TransformMatElem(alphas(i), ds(i), q(i), rs(i));
    
    R0Oi = g(1:3,1:3);
    
    % Inertia tensor of body i in relation to the point Oi on reference frame i
    I_iOi = I(:,:,i);
    
    % Passing the inertia tensor to the point Gi
    I_iGi = ComputeHuygens(I_iOi, m(i), [0;0;0], Gs(:,i));
    
    % Passing the inertia tensor in relation to the 0 reference frame
    I_0Gi = R0Oi*I_iGi*R0Oi';
    
    [JvGi,JwGi] = ComputeJacGi(alphas, ds, q, rs, Gs, i);
    
    A = A + m(i).*(JvGi'*JvGi) + (JwGi'*I_0Gi*JwGi);
end

A_q = A + diag(Jm)*diag(red.^2); % adding the inertia of the actuators
end


function G_q = ComputeGravTorque(q,alphas, ds, rs, Gs, m,grav)
% Exercise 15 (expression on TD2)
% Returns the vector of joint torques due
% to the gravity
% Inputs:  q [rad] (in R nx1) --> joint-coordinates of the robot
%               (without the end-effector's parameter included)
%          (alphas [rad] , ds [m] , rs [m]) on MDH convention
%               each with dimension (N+1) x 1 with the End-effector's
%               parameters
%          Gs [m] (in R 3xN) --> position of the barycenter of the bodies
%               in relation to their respective reference frames
%          m [kg] (in RNx1) --> mass of the bodies
%          grav (in R3x1) --> the acceleration of the gravity in the 0
%               reference frame
% Output:  G(q) in R^Nx1.

G = 0;
N = length(q);

for i = 1:N
    [JvGi,~] = ComputeJacGi(alphas, ds, q, rs, Gs, i);
    G = G - m(i).*(JvGi'*grav);
end

G_q = G;
end


function fric = ComputeFrictionTorque(q_dot,Fv)
% Exercise 17
% return the vector of joint torques produced by joint friction
% Friction model used:
% tau_f_i(q_dot_i) = diag(q_dot_i)*Fv_i for i = 1:6
% Inputs:  q_dot [rad*s^(-1)] (in R^(Nx1)) --> speed of the joint coordinates
%               of the robot (without the end-effector's parameter included)
%          Fv (in R^(Nx1)) --> Joint viscous frictions [N*m*(rad^-1)*s]

fric = diag(q_dot)*Fv;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Plotting functions        %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function PlotFrame(alphas,ds,q,rs,ellips)
% Plot of the robot's configuration
% Input:(alphas,ds,rs) --> on the MDH convention and include the end-effector's
% parameters
%        q --> joint coordinates (do not include end-effector)
% Optional: ellips = true For showing the ellipsoid of velocities
%                         transmition between joint and task spaces

if nargin<5
  ellips = false;
end

% Plots:
figure;
view(3);
hold on;
axis equal;

% Question 5
% Computation of the DGM iterativelly to draw the intermediate joint points
N_joints = size(alphas,1)-1;

% Getting the intermediate joint points from the robot tree
g = 1;
ps = zeros(3,N_joints+1); % the position of each intermediate joint in the 0 reference frame

% plot3(...,'HandleVisibility','off') is for not be shown in the legend
for i = 1:(N_joints+1)
    if i == N_joints+1 %from g0N to g0E, theta = 0
        g = g * TransformMatElem(alphas(i),ds(i),0,rs(i));
    else
        g = g * TransformMatElem(alphas(i),ds(i),q(i),rs(i));
    end
    
    ps(:,i) = g(1:3,4);
    if i == 1 % Draw the line from o_0 to o_1
        plot3([0 ps(1,i)],[0 ps(2,i)],[0 ps(3,i)],...
            '--r','LineWidth',1,'HandleVisibility','off');
    else % Draw the line from o_i-1 to o_i
        plot3([ps(1,i-1) ps(1,i)],[ps(2,i-1) ps(2,i)],[ps(3,i-1) ps(3,i)],...
            '--r','LineWidth',1,'HandleVisibility','off');
    end
end

p0E = ps(:,N_joints+1);
R0E = g(1:3,1:3);
[rotation_ang, n] = inv_Rot(R0E);

% Draw the line from o_N to o_E (p0E is already calculated)
%plot3([ps(1,end) p0E(1)],[ps(2,end) p0E(2)],[ps(3,end) p0E(3)],...
%    '--r','LineWidth',1, 'HandleVisibility','off');


% Plotting the 0 reference frame on the image of Question 5 and 7
x0 = [0.5,0,0];
y0 = [0,0.5,0];
z0 = [0,0,0.5];
o0 = [0,0,0];

plot3([o0(1) x0(1)],[o0(2) x0(2)],[o0(3) x0(3)],'LineWidth',3);
plot3([o0(1) y0(1)],[o0(2) y0(2)],[o0(3) y0(3)],'LineWidth',3);
plot3([o0(1) z0(1)],[o0(2) z0(2)],[o0(3) z0(3)],'LineWidth',3);
    
% Draw the Rotation vector between R0 and RE on Question 5
% Draw 3D vector: quiver3(x,y,z,nx,ny,nz);
if ~ellips
    quiver3(0,0,0,n(1),n(2),n(3));
end

% Plotting the E reference frame

% Slide 77: Rij(1:3,1) designates the unit vector x_j of frame Rj expressed in frame Ri.
% i.e: x_j = Rij(1:3,1) = [x_j(x_i) ; x_j(y_i) ; x_j(z_i)]
% x0E = R(1:3,1), y0E = R(1:3,2), zoE = R(1:3,3)

if ~ellips % Plot only on Question 5
    quiver3(p0E(1),p0E(2),p0E(3),R0E(1,1),R0E(2,1),R0E(3,1)); %x0E vector
    quiver3(p0E(1),p0E(2),p0E(3),R0E(1,2),R0E(2,2),R0E(3,2)); %y0E vector
    quiver3(p0E(1),p0E(2),p0E(3),R0E(1,3),R0E(2,3),R0E(3,3)); %z0E vector
    plot3(p0E(1),p0E(2),p0E(3),'o');
end

% Question 7 - Plotting the ellipsoid of velocities
% Plots if the argument "elispoid" is true
%[x,y,z] = ellipsoid(xc,yc,zc,xr,yr,zr,n) generates the ellipsoid mesh
% with center (xc,yc,zc) and semi-axis lengths (xr,yr,zr)
%rotate(h,direction,alpha,center) rotates the graphics object h by alpha degrees.
if ellips
    J_0 = ComputeJac(alphas, ds, [q;0] , rs);
    J_0_v = J_0(1:3,:);
    [U,S,~] = svd(J_0_v);
    [x_e, y_e, z_e] = ellipsoid(p0E(1),p0E(2),p0E(3),S(1,1),S(2,2),S(3,3),30);
    s = surf(x_e,y_e,z_e);
    [theta, w] = inv_Rot(U);
    rotate(s, w, theta*180/pi,[p0E(1),p0E(2),p0E(3)]);
    quiver3(p0E(1),p0E(2),p0E(3),U(1,1),U(2,1),U(3,1),'LineWidth',2); %1st ellipsoid principal vector
    quiver3(p0E(1),p0E(2),p0E(3),U(1,2),U(2,2),U(3,2),'LineWidth',2); %2nd ellipsoid principal vector
    quiver3(p0E(1),p0E(2),p0E(3),U(1,3),U(2,3),U(3,3),'LineWidth',2); %3rd ellipsoid principal vector
end

% Plot configurations
grid on;
xlabel('X_0');
ylabel('Y_0');
zlabel('Z_0');

% Title
plot_title = ['Reference frame of the End-effector for q = '];
% Getting the values of q to string for diplpaying in the title
for i = 1:(size(q,1))
    if i ~= size(q,1)
        if i == 3
            q(i) = q(i) - pi/2;
        end
        plot_title = [plot_title, num2str(q(i)), ', '];
    else
        plot_title = [plot_title, num2str(q(end)),' .'];
    end
end
title(plot_title);

% Legend

if ellips
    set(gcf, 'Position',  [100, 100, 850, 400]);
    legend({'x_0','y_0','z_0',...
            'Ellipsoid of velocities transmition', 'x^1_{ellipsoid}',...
            'x^2_{ellipsoid}','x^3_{ellipsoid}'},...
           'Location','southeastoutside');
else
    rot = ['Rot Axis of ', num2str(round(rotation_ang*180/pi)),'º'];
    set(gcf, 'Position',  [100, 100, 750, 400]);
    legend({'x_0','y_0','z_0',rot,'x_E','y_E','z_E','o_E'},'Location','southeastoutside');
end
end


function PlotFrameQ9(qs,Xd_i, Xd_f,Ts,alphas,ds,rs,animation_rate)
% Plot of the robot's configuration
figure();
N_t = size(qs,2);
X0Es = zeros(3,N_t);

% Performing the DGM for Calculating the position of every Xd0E from the qd
for j = 1:N_t
    [p0Ej,~] = ComputeDGM(alphas, ds, [qs(:,j);0], rs);
    X0Es(:,j) = p0Ej(1:3,1);
end

% Animating the plot
for k = 1:animation_rate:N_t %Skips "animation_rate" frames
    % Setting the plot
    clf; % Clear figure
    view(3);
    %view([-121 20]); %other view of the motion
    axis equal;
    grid on;
    xlabel('X_0');
    ylabel('Y_0');
    zlabel('Z_0');
    hold on;
    
    % Plotting the 0 Reference frame
    x0 = [0.3,0,0];
    y0 = [0,0.3,0];
    z0 = [0,0,0.3];
    o0 = [0,0,0];

    plot3(0,0,0,'ob','HandleVisibility','off');
    plot3([o0(1) x0(1)],[o0(2) x0(2)],[o0(3) x0(3)],'LineWidth',3);
    plot3([o0(1) y0(1)],[o0(2) y0(2)],[o0(3) y0(3)],'LineWidth',3);
    plot3([o0(1) z0(1)],[o0(2) z0(2)],[o0(3) z0(3)],'LineWidth',3);

    % Plotting the initial and final position
    plot3(Xd_i(1),Xd_i(2),Xd_i(3),'ob');
    plot3(Xd_f(1),Xd_f(2),Xd_f(3),'xr');

    % Plotting the robot tree position
    % Computation of the DGM iterativelly to draw the intermediate joint points
    N_joints = size(alphas,1)-1;

    % Getting the intermediate joint points from the robot tree
    q_k = qs(:,k);
    g = 1;
    ps = zeros(3,N_joints+1); % the position of each intermediate joint in the 0 reference frame
    
    % plot3(...,'HandleVisibility','off') is for not be shown in the legend
    for i = 1:(N_joints+1)
        if i == N_joints+1 %from g0N to g0E, theta = 0
            g = g * TransformMatElem(alphas(i),ds(i),0,rs(i));
        else
            g = g * TransformMatElem(alphas(i),ds(i),q_k(i),rs(i));
        end

        ps(:,i) = g(1:3,4);
        if i == 1 % Draw the line from o_0 to o_1
            plot3([0 ps(1,i)],[0 ps(2,i)],[0 ps(3,i)],...
                '--r','LineWidth',1,'HandleVisibility','off');
        else % Draw the line from o_i-1 to o_i
            plot3([ps(1,i-1) ps(1,i)],[ps(2,i-1) ps(2,i)],[ps(3,i-1) ps(3,i)],...
                '--r','LineWidth',1,'HandleVisibility','off');
        end
    end

    p0E_k = ps(:,N_joints+1);
    R0E_k = g(1:3,1:3);
    
    % Plotting the E reference frame
    plot3(p0E_k(1),p0E_k(2),p0E_k(3),'or'); %End-effector's position at time k
    plot3(X0Es(1,1:k),X0Es(2,1:k),X0Es(3,1:k),'-b'); %End-effector's trajectory
    quiver3(p0E_k(1),p0E_k(2),p0E_k(3),R0E_k(1,1),R0E_k(2,1),R0E_k(3,1)); %x0E vector
    quiver3(p0E_k(1),p0E_k(2),p0E_k(3),R0E_k(1,2),R0E_k(2,2),R0E_k(3,2)); %y0E vector
    quiver3(p0E_k(1),p0E_k(2),p0E_k(3),R0E_k(1,3),R0E_k(2,3),R0E_k(3,3)); %z0E vector
    
    % Title
    plot_title = ['Reference frame of the End-effector at t = ', num2str(k*Ts),'s.'];
    title(plot_title);

    % Legend
    legend({'x_0','y_0','z_0','X_i','X_f','o_E','Path','x_E','y_E','z_E'},'Location','southeastoutside');
    drawnow;
end

end


function PlotQs(qd,Ts,q_min, q_max)
% Plot of the robot's configuration
% Input:(alphas,ds,rs) --> on the MDH convention and include the end-effector's
% parameters
figure;
N = size(qd,2);
const = ones(1,N);
t = 0:Ts:(N-1)*Ts;

% subplot(column,row,fig)
for joint_index = 1:size(qd,1)
    subplot(3,2,joint_index);
    hold on;
    plot(t(1:N),q_max(joint_index).*const,'--r','LineWidth',1);
    plot(t(1:N),qd(joint_index,:),'b','LineWidth',1.5);
    plot(t(1:N),q_min(joint_index).*const,'--m','LineWidth',1);
    q_name = ['q',num2str(joint_index)];
    title(['Time evolution of ',q_name]);
    xlabel('t(s)');
    legend({[q_name,'_{max}'],q_name,[q_name,'_{min}']},'Location','best');
    grid on;
    hold off;
end
set(gcf, 'Position',  [150, 150, 800, 500]);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Auxiliary functions        %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function theta = do_offset_theta(q)
theta = q;
theta(3) = q(3) + pi/2;
end


function theta = undo_offset_theta(q)
theta = q;
theta(3) = q(3) - pi/2;
end


function g = affine_to_homogeneous(R,p)
% Slide 77
% Computes the homogeneus transformation matrix
% Given a rotation matrix R and a translation vector p
g = [    R    , p;
    zeros(1,3), 1];
end


function I_G = ComputeHuygens(I_O, m, O, G)
% Huygens Formula - Slide 166
% Transports the Inertia tensor from one point to the Barycenter of the
% body

O_G = G - O;

I_G = I_O - m*[O_G(2)*O_G(2)+O_G(3)*O_G(3) -O_G(1)*O_G(2) -O_G(1)*O_G(3);
                -O_G(1)*O_G(2) O_G(1)*O_G(1)+O_G(3)*O_G(3) -O_G(2)*O_G(3);
                -O_G(1)*O_G(3) -O_G(2)*O_G(3) O_G(1)*O_G(1)+O_G(2)*O_G(2)];
end


function W = vec_to_cross_prod(w)
% Slide 60
% Computes the the skew-symmetric matrix associated 
% to the cross product of vectors w and u as follows w x u = Wu

W = [  0   , -w(3) ,  w(2);
      w(3) ,   0   , -w(1);
     -w(2) ,  w(1) ,   0  ];
end


function [theta, w] = inv_Rot(R)
% Slide 61 - Inverse relationship of Rodrigues formula
% Identifcation of the axis of rotation w and theta (0<theta<pi)
% This function does not work if R is symmetric
% Input: Rotation matrix R (in SO(3))

theta = atan2(0.5*sqrt((R(3,2)-R(2,3))^2 + (R(1,3)-R(3,1))^2 + (R(2,1) - R(1,2))^2),...
              0.5*(R(1,1) + R(2,2) + R(3,3) - 1));

tol = 1e-3;
if (0 < sin(theta)) && (sin(theta)< tol)
    w = [0;0;0];
    theta = 0;
else
    w = [(R(3,2)-R(2,3));...
        (R(1,3)-R(3,1)) ;...
        (R(2,1)-R(1,2))]./(2*sin(theta));
end
end


function euler_angles = get_euler_angles(R)
% Slide 66 - Inverse relationship of euler angles
% Computes the Euler angles (alpha, betta, gamma) of a rotation matrix
% Input: Rotation matrix R (in SO(3))
g = atan2(R(3,1),R(3,2));
b = atan2(R(3,1)*sin(g) + R(3,2)*cos(g), R(3,3));
a = atan2(R(2,1)*cos(g) + R(2,2)*sin(g), R(1,1)*cos(g) - R(1,2)*sin(g));

euler_angles = [a;b;g];
end


function g = Trans(p)
% Slide 77
% Computation of g_ij the homogeneous transformation matrix
% of a translation from frame i to frame j
% Input: p_ij = [a;b;c] (in R3x1) is the translation vector along the axis xi,yi,zi

I = eye(3);
g = affine_to_homogeneous(I,p);
% g = [1,0,0,a;
%      0,1,0,b;
%      0,0,1,c;
%      0,0,0,1];
end


function Rx_homog = Rot_x(t)
% Slide 59
% Calculates the rotation matrix  of theta around the axis x
% Inputs: (theta [rad])

Rx = [1, 0     , 0      ;
      0, cos(t), -sin(t);
      0, sin(t), cos(t)];
  
Rx_homog = affine_to_homogeneous(Rx, [0;0;0]);

end


function Ry_homog = Rot_y(t)
% Slide 59
% Calculates the rotation matrix  of theta around the axis y
% Inputs: t (theta [rad])

Ry = [cos(t),   0   , sin(t) ;
        0   ,   1   ,  0     ;
     -sin(t),   0   , cos(t)];

Ry_homog = affine_to_homogeneous(Ry, [0;0;0]);
end


function Rz_homog = Rot_z(t)
% Slide 59
% Calculates the rotation matrix  of theta around the axis z
% Inputs: (theta [rad])

Rz = [cos(t),-sin(t), 0 ;
      sin(t),cos(t),  0 ;
        0   ,   0  ,  1];

Rz_homog = affine_to_homogeneous(Rz, [0;0;0]);
end


function R_homog = Rot(w,t)
% Slide 60 - Rodrigues formula
% Calculation of the rotation matrix, given an axis of rotation w and theta
% Input: (w (in R3), theta (in R))
w = w./norm(w);
wx = w(1);wy = w(2); wz = w(3);

R_primary = [(wx^2)*(1 - cos(t)) + cos(t), wx*wy*(1 - cos(t)) - wz*sin(t),wx*wz*(1 - cos(t)) + wy*sin(t);
                            0            , (wy^2)*(1 - cos(t)) + cos(t)  ,wy*wz*(1 - cos(t)) - wx*sin(t);
                            0            ,              0                , (wz^2)*(1 - cos(t)) + cos(t)];
 
 % get the symmetrical rotation matrix
 R = (R_primary+R_primary') - eye(size(R_primary,1)).*diag(R_primary);
 R_homog = affine_to_homogeneous(R, [0;0;0]);
end


function R_euler_homog = Rot_euler(euler_angles)
% Slide 66 - Rotation matrix of euler angles
% Computes the Euler rotation matrix R (in SO(3) with the Euler angles
% Input: [alpha, betta, gamma]'[rad] (in R3x1) in Euler convenction
a = euler_angles(1);
b = euler_angles(2);
g = euler_angles(3);

R_euler = [cos(a)*cos(g) - sin(a)*cos(b)*sin(g), -cos(a)*sin(g) - sin(a)*cos(b)*cos(g),  sin(a)*sin(b);
           sin(a)*cos(g) + cos(a)*cos(b)*sin(g), -sin(a)*sin(g) + cos(a)*cos(b)*cos(g), -cos(a)*sin(b);
                      sin(b)*sin(g)            ,              sin(b)*cos(g)           ,       cos(b) ]; 
R_euler_homog = affine_to_homogeneous(R_euler, [0;0;0]);
end
