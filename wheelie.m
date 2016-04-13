% Inverted pendulum problem
clear; close all

% Parameters
H  = 1.75;       % [m] stature
% Px = -0.056;    % [m] Actual x position of the rear axle  with respect to the "hip" in the Jaguaribe wheelchair (x positive forwards)
Px = 0;
Py = -0.249;    % [m] Actual y position of the rear axle with respect to the "hip" in the Jaguaribe wheelchair (y positive upwars)

% If you desire to enter with the mass value, just uncomment here. In
% contrast, the mass according to upper limit for normal BMI according to WHO (25) will be used.
Mu  = 75; %[kg] user's mass 
[qcm, Jc] = parameters(H, Px, Py, Mu);
% [qcm, Jc, Mu] = parameters(H, Px, Py)
% Jc - [kg.m^2] momento de inérica da cadeira sem rodas + usuário em relação ao seu próprio CG
% qcm - [m] vetor 2x1 com a posição do CG do sistema em relação ao eixo da roda traseira

initial_angle = atan2(qcm(1,1),qcm(2,1));
L = sqrt(qcm(2,1)^2 + qcm(1,1)^2);

% Initial and final conditions
phi0 = atan2(qcm(1,1),qcm(2,1)); % initial angle for wheelchair front wheel on the ground
phif = 0; % final angle


%% Specify number of nodes
nn = 100; % number of nodes

w = sys_wheelchair;

%% Parameters
g = w.g;
Mcad_tot = w.Mc; % [kg] massa total com as rodas
Mr = 2*w.Mr; % [kg] masssa das duas rodas
Mcad = Mcad_tot - Mr; % [kg] massa da cadeira sem as rodas
Mc = Mu + Mcad; % [kg] massa total do sistema
Jr = 2*w.Jr; % [kg.m^2] momento de inércia das duas rodas
R = w.R; % [m] raio da roda traseira
Frol = w.Frol; % [N]


tau_lim = (Mc + Jr/(R^2))*R*L*sin(phi0)*g/(L*cos(phi0)+R)

pars.g = g;
pars.Mr = Mr;
pars.Mc = Mc;
pars.Jr = Jr;
pars.R = R;
pars.H = H;
pars.L = L;
pars.phi0 = phi0;
pars.Px = Px;
pars.Py = Py;
pars.tau_lim = tau_lim;
pars.Jc = Jc;


%% Define the independent variable, and phase:
% It is common for the independent variable to be named "t",
% but any legal tomSym name is possible.
% The length of the time-interval is variable, so another tomSym
% symbol is created for that.
% Note: for free final time declare tf here as well (as in five-link MK2 Robot example)
% Phase: single phase
toms t tf    
phase = tomPhase('phase', t, 0, tf, nn);


setPhase(phase);
tomStates theta thetad phi phid
tomControls tau

%% Define a list of states
% After a phase has been defined, states can be created
% Note that the states can be given meaningful names, even though
% they are simply named x1...x3 in this particular problem.
%
% States
% Add description

% Lower and upper bounds on states
cbox = {0.01 <= tf <= 5
    -abs(phi0) <= icollocate(phi) <= abs(phi0)
    -10*pi <= icollocate(phid) <= 10*pi};
%     -0.25 <= icollocate(x) <= 0.25
%     -20 <= icollocate(xdd) <= 20};

% Load previous solution
load('test20.mat');


%% Initial Guess for states
setPhase(phase)
x0 = {tf == 1
    icollocate({
    theta == 0*t/tf;
    thetad == 0*t/tf;
    phi == phi0 + (phif - phi0)*t/tf;
    phid == 0*t/tf;
    })};

% x0 = {tf == opt.tf
%     icollocate({
%     theta == interp1p(opt.ti, opt.theta,t);
%     thetad == interp1p(opt.ti, opt.thetad,t);
%     x == interp1p(opt.ti, opt.x,t);
%     xd == interp1p(opt.ti, opt.xd,t);
%     })};

% Initial guess for controls
setPhase(phase)
x0 = {x0
    collocate(tau == 0)};

% x0 = {x0
%     collocate(xdd == interp1p(opt.tc, opt.xdd,t))};


%% Contraints
% Lower and upper bounds on controls
setPhase(phase)
cbox = {cbox
      -150 <= collocate(tau) <= 150}; 

% % Inequality constraints
% setPhase(phase)
% cineq = {0 <= icollocate(alpha1 - beta1) <= pi};   % elbow angle
     
% Equations of motion
setPhase(phase)
M = [Jr + (Mc+Mr)*R^2, Mc*R*L*cos(phi); Mc*R*L*cos(phi), Jc+Mc*L^2];
k = [-Mc*R*L*(phid^2)*sin(phi); -Mc*g*L*sin(phi)];
ceq = collocate({thetad == dot(theta); phid == dot(phi); M*dot([thetad; phid]) == -k + [tau; -tau]});


% Boundary constraints on initial and final states
setPhase(phase)  
cbnd = {
        initial({theta == 0})
        initial({thetad == 0})
        initial({phi == phi0})
        initial({phid == 0})
%         final({theta == 0})
        final({thetad == 0})
        final({phi == phif})
        final({phid == 0})};



% Objective function
objective = (integrate(phase, tau^2));
% objective = (integrate(phase, tau^2) + 1000*integrate(phase, theta^2));
% objective = ((integrate(phase1, tau_s1^2) + integrate(phase1, tau_e1^2) + integrate(phase2, tau_s2^2) + integrate(phase2, tau_e2^2))/(tf2*speed));


%% Build the .m files and general TOMLAB problem
%%%%%%%%%%%
% Computing equivalent Prob structure
% display('computing equivalent Prob struct ...')
% Prob = sym2prob('con',objective,{cbox, ceq, cbnd},x0,options);
% 
% Prob.SOL.optPar(30) = 200000; % maximal sum of minor iterations (max(10000,20*m))
% Prob.SOL.optPar(36) = 5000; % maximal number of minor iterations in the solution of the QP problem (500)
% 
% % Prob.FUNCS.f = 'f_B3';
% % Prob.FUNCS.g = 'g_B3';
% % Prob.FUNCS.H = 'H_B3';
% Prob.FUNCS.c = 'c_B3';
% Prob.FUNCS.dc = 'dc_B3';
% % Prob.FUNCS.d2c = 'd2c_B3';
% Prob.FUNCS.d2c = [];
% 
% % % Impose null upper and lower bounds for F2
% % Prob.x_L(324:403) = 0;
% % Prob.x_U(324:403) = 0;
% 
% % Solve the problem using any TOMLAB solver
% display('solver called ...')
% result = tomRun('snopt', Prob, 1);
% 
% % Extract solution
% % Obs.: state vectors include initial state but not final state
% % Obs.: control vectors include only nodal values of controls
% solution = getSolution(result);
%%%%%%%%

%%%%%%%%
% Directly
options = struct;
options.name = 'wheelie';
% options.scale = 'auto';
% options.Prob.SOL.optPar(30) = 200000; % maximal sum of minor iterations (max(10000,20*m))
% options.Prob.SOL.optPar(36) = 5000; % maximal number of minor iterations in the solution of the QP problem (500)
options.PriLevOpt = 1;
constr = {cbox, cbnd, ceq};
solution = ezsolve(objective, constr, x0, options);


opt.tf = subs(tf,solution);
opt.ti  = subs(icollocate(phase,t),solution);
opt.tc  = subs(collocate(phase,t),solution);
opt.phi = subs(icollocate(phase,phi),solution);
opt.phid = subs(icollocate(phase,phid),solution);
opt.theta = subs(icollocate(phase,theta),solution);
opt.thetad = subs(icollocate(phase,thetad),solution);
opt.tau = subs(collocate(phase,tau),solution);   
opt.objective = subs(objective,solution);
opt.nn = nn;


%% Plot kinematics
figure
hold on
plot(opt.ti,opt.phi*180/pi)
hold off
title('phi')

figure
hold on
plot(opt.ti,opt.phid*180/pi)
hold off
title('phid')

figure
hold on
plot(opt.ti,opt.theta*180/pi)
hold off
title('theta')

figure
hold on
plot(opt.ti,opt.thetad*180/pi)
hold off
title('thetad')


%% Plot joint moments
figure
hold on
plot(opt.tc,opt.tau)
hold off
title('tau')



% %% Stick figure
% % Shoulder position
% rsx = 0*opt.t;
% rsy = 0*opt.t;
% 
% % Elbow position
% rex = B*sin(opt.beta);
% rey = -B*cos(opt.beta);
% 
% % Hand position
% rhx = B*sin(opt.beta) + A*sin(opt.alpha);
% rhy = -B*cos(opt.beta) - A*cos(opt.alpha);
% 
% % Rim Position
% rrx = h + R1*cos(pi - phi - opt.x/R2);
% rry = -v + R1*sin(pi - phi - opt.x/R2);
% 
% % Interpolate results for snapshots
% n_shots = 20;
% time = opt.t1(1):(opt.t2(end)-opt.t1(1))/n_shots:opt.t2(end);
% rsx_int = interp1(opt.t, rsx, time);
% rsy_int = interp1(opt.t, rsy, time);
% rex_int = interp1(opt.t, rex, time);
% rey_int = interp1(opt.t, rey, time);
% rhx_int = interp1(opt.t, rhx, time);
% rhy_int = interp1(opt.t, rhy, time);
% rrx_int = interp1(opt.t, rrx, time);
% rry_int = interp1(opt.t, rry, time);
% 
% % Stick-figures
% figure
% hold on
% for i = 1:length(time)
%     plot([rsx_int(i) rex_int(i)],[rsy_int(i) rey_int(i)],'r'); % arm
%     plot([rex_int(i) rhx_int(i)],[rey_int(i) rhy_int(i)],'k'); % forearm
%     plot(rrx_int(i),rry_int(i),'og'); % rim position
% end
% % plot([-1 1],[0 0]); % wheelrim
% axis equal
% hold off
% 
% % Save solution (file name result_ang??_bf??_ini?.mat)
% filename = 'wheelie_Px2.mat';
% % filename = 'wheelchair_ni20_Froll15_mC025_nn40_v05_scaled.mat';
% save(filename,'solution','opt','pars');