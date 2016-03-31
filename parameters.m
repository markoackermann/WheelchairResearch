function [ q, Jr ] = parameters(varargin)
    %  [q, Jr] : Function returns the vector formed by the rear axle and
    %  center of mass. In addition, it returns the moment of inertial.
    %  
    %  H:  Height of the user
    %  Px: Longitutinal distance between pelvis and rear axle
    %  Py: Horizontal distance between pelvis and rear axle
    %  M:  Mass of the user
    %
    %  q: [qx qy]: Vector of the center of mass to center of wheel
    %  Jpg: Moment of Inertial in Gp coordinates

    % Inputs of the function
    H  = varargin{1}; 
    Px = varargin{2};
    Py = varargin{3};
    switch nargin
        case 3
            M  = 25*H^2;     % Using BMI
        case 4
            M  = varargin{4};
        otherwise
            error('Unexpected inputs')
    end
    
    % Angles of the joints
    syms a1 a2 a3 a4 a5;
    
     % Body Segment lenghts ------------------------------------------------
    l(1) = (2/3)*0.152*H;               % Foot
    l(2) = 0.039*H ;                    % 
    l(3) = 0.285*H - (l(2));            % Leg - Medial malleollus - Femoral condyles 
    l(4) = 0.530*H - (l(2)+l(3));       % Thigh
    l(5) = 0.818*H - (l(2)+l(3)+l(4));  % HAT

    % Mass of each segment ------------------------------------------------
    m(1) = 0.0145*M;  % Mass of foot
    m(2) = 0;         % 
    m(3) = 0.0465*M;  % Mass of leg
    m(4) = 0.1000*M;  % Mass of thigh
    m(5) = 0.6780*M;  % Mass of HAT - Head, arms and trunk

    % Center of mass of each segment --------------------------------------
    cm(1) = sqrt((0.0351*H/1.8)^2+(0.0768*H/1.8)^2); % Center of Mass - foot
    cm(2) = 0;             %
    cm(3) = 0.433*l(3);    % Center of Mass - leg
    cm(4) = 0.433*l(4);    % Center of Mass - thigh
    cm(5) = 0.626*l(5);    % Center of Mass - HAT
    
    % Radius of Gyration/Segment Length -----------------------------------
    rog(1) = 0.475;   % Foot
    rog(2) = 0;    
    rog(3) = 0.302;   % Leg
    rog(4) = 0.323;   % Thigh
    rog(5) = 0.496;   % HAT

    % Direct kinematics of ------------------------------------------------
    b1 = a1;
    b2 = b1 + a2;
    b3 = b2 + a3;
    b4 = b3 + a4;
    b5 = b4 + a5;

    x1 = l(1)*cos(b1);
    y1 = l(1)*sin(b1);
    x2 = l(1)*cos(b1) + l(2)*cos(b2);
    y2 = l(1)*sin(b1) + l(2)*sin(b2);
    x3 = l(1)*cos(b1) + l(2)*cos(b2) + l(3)*cos(b3);
    y3 = l(1)*sin(b1) + l(2)*sin(b2) + l(3)*sin(b3);
    x4 = l(1)*cos(b1) + l(2)*cos(b2) + l(3)*cos(b3) + l(4)*cos(b4);
    y4 = l(1)*sin(b1) + l(2)*sin(b2) + l(3)*sin(b3) + l(4)*sin(b4);
    x5 = l(1)*cos(b1) + l(2)*cos(b2) + l(3)*cos(b3) + l(4)*cos(b4) + l(5)*cos(b5);
    y5 = l(1)*sin(b1) + l(2)*sin(b2) + l(3)*sin(b3) + l(4)*sin(b4) + l(5)*sin(b5);

    % Stantard position of user -------------------------------------------
    a1 = pi;
    a2 = -pi/2;
    a3 = 0;
    a4 = pi/2;
    a5 = -pi/2;

    % Coordinates of joints -----------------------------------------------
    X(1) = 0;
    Y(1) = 0;
    X(2) = eval(x1);
    Y(2) = eval(y1);
    X(3) = eval(x2);
    Y(3) = eval(y2);
    X(4) = eval(x3);
    Y(4) = eval(y3);
    X(5) = eval(x4);
    Y(5) = eval(y4);
    X(6) = eval(x5);
    Y(6) = eval(y5);

    % Coordinates of center of mass for each segment ----------------------
    cx1 = 0.5*l(1)*cos(b1);
    cy1 = 0.5*l(1)*sin(b1);
    cx2 = l(1)*cos(b1) + 0.5*l(2)*cos(b2);
    cy2 = l(1)*sin(b1) + 0.5*l(2)*sin(b2);
    cx3 = l(1)*cos(b1) + l(2)*cos(b2) + (l(3)-cm(3))*cos(b3);
    cy3 = l(1)*sin(b1) + l(2)*sin(b2) + (l(3)-cm(3))*sin(b3);
    cx4 = l(1)*cos(b1) + l(2)*cos(b2) + l(3)*cos(b3) + (l(4)-cm(4))*cos(b4);
    cy4 = l(1)*sin(b1) + l(2)*sin(b2) + l(3)*sin(b3) + (l(4)-cm(4))*sin(b4);
    cx5 = l(1)*cos(b1) + l(2)*cos(b2) + l(3)*cos(b3) + l(4)*cos(b4) + cm(5)*cos(b5);
    cy5 = l(1)*sin(b1) + l(2)*sin(b2) + l(3)*sin(b3) + l(4)*sin(b4) + cm(5)*sin(b5);

    Cx(1) = eval(cx1);
    Cy(1) = eval(cy1);
    Cx(2) = eval(cx2);
    Cy(2) = eval(cy2);
    Cx(3) = eval(cx3);
    Cy(3) = eval(cy3);
    Cx(4) = eval(cx4);
    Cy(4) = eval(cy4);
    Cx(5) = eval(cx5);
    Cy(5) = eval(cy5);

    % Fix the (0,0) in the pelvis
    X  = X + (l(1) + l(4)); 
    Y  = Y - (l(2) + l(3));
    Cx = Cx + (l(1) + l(4));
    Cy = Cy - (l(2) + l(3));
    
    % Center of Mass of the Pacient ---------------------------------------
    CMpx = 0; CMpy = 0; Mt = 0;
    for i=1:5
        if i<5                          % foot, leg and thigh (2X)
            CMpx = CMpx + 2*Cx(i)*m(i);
            CMpy = CMpy + 2*Cy(i)*m(i);
            Mt = Mt + 2*m(i);
        else                            % HAT (1K)
            CMpx = CMpx + Cx(i)*m(i);
            CMpy = CMpy + Cy(i)*m(i);
            Mt = Mt + m(i);
        end
    end
    CMpx = CMpx/Mt;
    CMpy = CMpy/Mt;
      
    % Moment of Inertia of Pacient in CMp ---------------------------------
    Jog = zeros(5,1); Jo = zeros(5,1);
    for i=1:5
        Jo(i) = m(i) * (l(i) * rog(i))^2;   % Moment of Inertia for each joint
        if i<5                              % foot, leg and thigh (2X)
            Jog(i)= 2*(Jo(i) + m(i) * ((Cx(i)-CMpx)^2 + (Cy(i)-CMpy)^2));% (Cx, Cy) : Center of Mass of each Joint
                                                                         % (CMpx, CMpy) : Center of Mass of User                                                                   
        else                                % HAT (1X)
            Jog(i)= Jo(i) + m(i) * ((Cx(i)-CMpx)^2 + (Cy(i)-CMpy)^2);   % (Cx, Cy) : Center of Mass of each Joint
                                                                        % (CMpx, CMpy) : Center of Mass of User                                                                   
        end
    end
    Jpg = sum(Jog); % Sum of the Moment of Inertia 
    
    w = sys_wheelchair;
        
    % Moment of Inertia of Wheelchair in CMc ------------------------------
    Jcg  = w.Jc;    % [kg*m^2]    Obtained experimentally
    CMcx = 0.193;   % [m]   Obtained experimentally
    CMcy = -0.173;  % [m]   Obtained experimentally
    
    % Resultant center of mass --------------------------------------------
    CMr = zeros(2,1);
    CMr(1,1) = (CMpx*Mt + CMcx*w.Mc) / (Mt + w.Mc);
    CMr(2,1) = (CMpy*Mt + CMcy*w.Mc) / (Mt + w.Mc);
    
    % Vector q ------------------------------------------------------------
    q = zeros(2,1);
    q(1,1) = CMr(1,1) - Px;
    q(2,1) = CMr(2,1) - Py;
    
    % Resultant moment of inertia in CG -----------------------------------
    Jr = Jpg + Mt * ((CMpx - CMr(1,1))^2 + (CMpy - CMr(2,1))^2) + ...
        Jcg + w.Mc *((CMcx - CMr(1,1))^2 + (CMcy - CMr(2,1))^2);
    
    % Plots ---------------------------------------------------------------
    figure
    hold on
    plot(X,Y,'b -o');
    plot(CMr(1,1), CMr(2,1), 'r x');
    plot(Cx, Cy, 'k d');
    plot(CMpx, CMpy, 'm x');
    plot(CMcx, CMcy, 'm x');  
    plot(Px, Py, 'k x');
    circle(Px,Py,0.305, 'k');
    
    title('Center of Mass of the Human Body');  
    axis([-1 1 -1 1.5]);
    axis equal;
    xlabel('Meters [m]');
    ylabel('Meters [m]');
    grid
    
end


