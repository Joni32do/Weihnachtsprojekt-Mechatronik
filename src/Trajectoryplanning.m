%% Modellierung und Simulation in der Mechatronik WS 2018 / 2019
% Homework trajectory planing
% Institut fuer Technische und Numerische Mechanik, Uni Stuttgart
% Profs Eberhard / Fehr / Hanss
% WS 2018 / 2019
% Jun. Prof. J. Fehr    Dipl.-Ing. C. Kleinbach
% last mod. 2015-12-16 Joerg Fehr
% 
function Trajectoryplanning
close all
%% First example for trajectory planning
% Aim is a smooth transition of one link from 
% starting position q0=10 [rad], v0=0 [rad/s], to
%    final position q0=-20 [rad], v0=0 [rad/s],
t0=0;    % initial time [s]
q0=10;   % initial joint position in [rad]
v0=0;    % initial velocity in [rad/s]
tf=1;    % final time [s]
qf=-20;  % final joint position in [rad]
vf=0 ;   % final velocity in [rad/s]

a=cubic_func(t0,q0,v0,tf,qf,vf);
%% Question 1: Write a Matlab function for quintic splines
% 1.a) Write a function |quintic_func(t0,q0,v0,a0,tf,qf,vf,af)| which
%      returns the polynominal coefficient of the quintic spline
%      which fullfils the boundary conditions q0,v0,a0 at t0 and qf,vf,af at tf.
% 
% 1.b) Plot the joint angle, joint velocity and joint acceleration for the
% initial and final values given below.
% 
% 1.c) Compare the joint angle, joint velocity and joint acceleration calculated with the quintic spline 
%      with the cubic spline.
%      

t0=0;    % initial time
q0=10;   % initial joint position in rad
v0=0;    % initial velocity in [rad/s]
a0=0;    % initial acceleration in [rad/s^2]
tf=1;    % final time
qf=-20;  % final joint position in [rad]
vf=0;    % final velocity in [rad/s]
af=0;    % final acceleration in [rad/s^2]
% cubic_func(t0,q0,v0,tf,qf,vf)
% quintic_func(t0,q0,v0,a0,tf,qf,vf,af)
% quintic_func(.....)

%%
%% Question 2: Write a Function which provides a Trajectory based on "Linear Segments with Parabolic Blends LSPB"
% 
% A LSPB consists of three parts: 
%    * a cubic spline up to the blend time, to accelerate the joint with a constant acceleration up to a
%    certain velocity,
%    * a linear function where the joint moves with a constant velocity (v_desired),
%    * a cubic spline, to decelerate the joint with a constant acceleration to the final state (position/velocity).
%  
% <<LSPB.PNG>>
% 
% 2.a) Write a LSPB function which calculates a LSPB spline 
%      which fullfils the boundary conditions q0,v0 at t0 and qf,vf which
%      moves with a constant velocity v_c after the blend time.
% 
% 2.b) Calculate the blend time for acceleration and deceleration based on the given parameter and the
% constant acceleration.
% 
% 2.c) Make a qualitative sketch of the joint velocity and joint acceleration.
%      Compare your qualitative sketch with the plot produced with Matlab.
%      
% 2.d) How can you improve LSPB splines?
% 
% 2.e) Make a proper documentation of your code if you want a review of your code  hand in your code via ILIAS to get a feedback
%
% * t0:   'start time'
% * q0:   'postion at start time'
% * tf:   'final time'
% * qf:   'postion at final time'
% * V:    'constant velocity after blend time
t0=0;     % initial time
q0=10;    % initial joint position in [rad]
tf=1;     % final time
qf=-20;   % final joint position in [rad]
V=-34;    % constant velocity [rad/s]
% [q,dq,ddq,tb,a1,a2]=LSPB_func(...)


%% Question 3: Make a literature/internet survey regarding B-Splines
% 
% 3.a) What are the advantages of B-Splines
% 
% 3.b) Implement, respectively download a toolbox to use B-Splines
%      for defining a smooth trajectory
%


end

%% Example Cubic Polynomial Trajectories 
function [a] = cubic_func(t0,q0,v0,tf,qf,vf)
% Syntax
% ======
%   a=cubic_func(t0,q0,v0,tf,qf,vf)
% Description
% ===========
%   Computes the polynomial coefficients of a cubic polynominal
%   and plots the polynominal, its derivate and second derivative
%
% Input Arguments
% ===============
%
% -t0:   'start time'
% -q0:   'postion at start time'
% -v0:   'velocity at start time'
% -tf:   'final time'
% -qf:   'postion at final time'
% -vf:   'velocity at final time'
%
% Output Arguments
% ================
%   -a: coefficients of polynominal
%--------------------------------------------------------------------------
% Vorlesung Modellierung und Simulation in der Mechatronik
% Copyright ITM University of Stuttgart, <a href="matlab:
%  web('http://www.itm.uni-stuttgart.de')">www.itm.uni-stuttgart.de</a>
%--------------------------------------------------------------------------

% Coefficient matrix for cubic trajectory and its derivative
% at initial and final joint values.
A = [1,  t0,  t0^2, t0^3;   ...
    0,  1,   2*t0, 3*t0^2; ...
    1,  tf,  tf^2, tf^3;   ...
    0,  1,   2*tf, 3*tf^2];
% Vector of initial and final joint positions and velocities
b = [q0; v0; qf; vf];
% Compute coefficients of trajectory polynomial using
% notion of a = inv(A)*b, for large scale systems the calculation of the inverse
% is not recommended, instead always use Gaussian Elimination backslash operator 
% of Matlab
a = A\b;
% Evaluate cubic polynomial 500 times
t = t0:(tf-t0)/500:tf;   % define the 
q     = a(1) + a(2)*t + a(3)*t.^2 + a(4)*t.^3;
qdot  = a(2) + 2*a(3)*t + 3*a(4)*t.^2;
qddot = 2*a(3) + 6*a(4)*t;

% Plot the trajectories
figure(1);

subplot(1,3,1);
plot(t,q,'r-.','LineWidth',2);

leg=get(legend,'String');
leg{end+1}='q x^3';
legend(leg);


xlabel('time [s]'); ylabel('joint angle [rad]');
% title('Trajectory using Cubic Polynomial');
hold on;

subplot(1,3,2);
plot(t,qdot,'r-.','LineWidth',2);


leg=get(legend,'String');
leg{end+1}='dq/dt x^3';
legend(leg);
xlabel('time [s]'); ylabel('veclocity [rad/s]');
% title('Trajectory using Cubic Polynomial');
hold on;

subplot(1,3,3);
plot(t,qddot,'r-.','LineWidth',2);

leg=get(legend,'String');
leg{end+1}='d^2q/dt^2 x^3';
legend(leg);
xlabel('time [s]'); ylabel('acceleration [rad/s^2]');
% title('Trajectory using Cubic Polynomial');
hold on;
end
