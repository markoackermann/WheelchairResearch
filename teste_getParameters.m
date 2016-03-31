clear all
close all

H  = 1.8;       % [m]
Px = -0.056;    % Actual x position of the axle rear in the Jaguaribe wheelchair
Py = -0.249;    % Actual y position of the axle rear in the Jaguaribe wheelchair

% If you desire to enter with the mass value, just uncomment here. In
% contrast, the mass according to BMI (Font: Brasil-2003) will be used.
%
% M  = 80;   
% [q, J] = parameters(H, Px, Py, M);

[q, J] = parameters(H, Px, Py);

initial_angle = atan2(q(2,1),q(1,1));