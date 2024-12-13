%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();
disp('iTj_0')
disp(iTj_0);
jointType = [0 0 0 0 0 1 0]; % specify two possible link type: Rotational, Prismatic.
q0 = [0,0,0,0,0,0,0]'; 

%% Define the tool frame rigidly attached to the end-effector
% Tool frame definition
eeta_te = [0, 0, pi/10];
eO_t = [0.2; 0; 0];

eRt = YPRToRot(eeta_te(1), eeta_te(2), eeta_te(3));

eTt = eye(4);
eTt(1:3, 1:3) = eRt;
eTt(1:3, 4) = eO_t;

disp(eTt);

%% Initialize Geometric Model (GM) and Kinematic Model (KM)

% Initialize geometric model with q0
gm = geometricModel(iTj_0,jointType,eTt);

% Initialize the kinematic model given the goemetric model
km = kinematicModel(gm);

%% Compute trasformation of the tool w.r.t. the base frame
% Update direct geoemtry given q=q0

bTt = gm.getToolTransformWrtBase();

disp("eTt");
disp(eTt);
disp('bTt q = q0');
disp(bTt);

%% Define the goal frame and initialize cartesian control
% Goal definition 
bO_g = [0.15; -0.85; 0.3];
beta_gb = [0, pi/6, 0];

bRg = YPRToRot(beta_gb(1), beta_gb(2), beta_gb(3));
bTg = eye(4);
bTg(1:3, 1:3) = bRg;
bTg(1:3, 4) = bO_g;
disp('bTg')
disp(bTg)

% control proportional gain 
k_a = 0.8;
k_l = 0.8;

% Cartesian control initialization
cc = cartesianControl(gm,k_a,k_l);

% initial configuration 
q = [pi/2, -pi/4, 0, -pi/4, 0, 0.15, pi/4]';

%% Initialize control loop 

% Simulation variables
samples = 100;
t_start = 0.0;
t_end = 10.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

% preallocation variables
bTi = zeros(4, 4, gm.jointNumber);
bri = zeros(3, gm.jointNumber+1);

% joints upper and lower bounds 
qmin = -3.14 * ones(7,1);
qmin(6) = 0;
qmax = +3.14 * ones(7,1);
qmax(6) = 1;

% list for data plot
x_dot_hist = [];
t_hist = [];

% Show simulation ? %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
show_simulation = false;
tool = true;

% init plot
figure
grid on 
hold on
title('MOTION OF THE MANIPULATOR')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
az = 48;
el = 25;
view(az,el)
cindex = 1;
csize = length(t);
cmap = colormap(parula(csize));
color = cmap(mod(cindex,csize)+1,:);
plot3(bTg(1,4),bTg(2,4),bTg(3,4),'ro')

%%%%%%% Kinematic Simulation %%%%%%%
for i = t
    % Update geometric and kinematic model and use the cartesian control ... to do
    gm.updateDirectGeometry(q);
    km.updateJacobian();


    %% INVERSE KINEMATIC
    % Compute desired joint velocities ... to do
    x_dot = cc.getCartesianReference(bTg);

    q_dot = pinv(km.J) * x_dot;

    % simulating the robot - implement KinematicSimulation
    q = KinematicSimulation(q, q_dot, dt, qmin, qmax);
    
    %% Plot Script (do not change)
    % computing the actual velocity and saving the unitary direction for plot
    % do NOT change
    x_dot_actual = km.J*q_dot;

    x_dot_hist = [x_dot_hist; (x_dot_actual/norm(x_dot_actual))'];
    t_hist = [t_hist; i];

    %% ... Plot the motion of the robot 
    if (rem(i,0.1) == 0) % only every 0.1 sec
        
        for j=1:gm.jointNumber
            bTi(:,:,j) = gm.getTransformWrtBase(j); 
        end
    
        bri(:,1) = [0; 0; 0];
        % Plot joints
        for j = 1:gm.jointNumber
            bri(:,j+1) = bTi(1:3,4,j);              
        end
        bTt = gm.getToolTransformWrtBase();
        bri(:,gm.jointNumber+2) = bTt(1:3,4); 

        % Plot links
        for j = 1:gm.jointNumber+1
            plot3(bri(1,j), bri(2,j), bri(3,j),'bo')           
        end
        plot3(bri(1,gm.jointNumber+2),bri(2,gm.jointNumber+2),bri(3,gm.jointNumber+2),'go') 
    
        color = cmap(mod(cindex,csize)+1,:);
        cindex = cindex + 1;
    
        line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5, 'Color', color)

    end

    if show_simulation == true
        drawnow
    end    
    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01)
        disp('Reached Requested Pose')
        break
    end

end

%% Plot the final configuration of the robot and the direction of the end-effector velocities
figure
grid on 
hold on
title('FINAL CONFIGURATION')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
az = 48;
el = 25;
view(az,el)
cindex = 1;
for j = 1:gm.jointNumber+2
    plot3(bri(1,j), bri(2,j), bri(3,j),'bo')
    
end

color = cmap(mod(cindex,csize)+1,:);
cindex = cindex + 1;

line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5, 'Color', color)

figure
hold on;
title('DIRECTION OF THE END-EFFECTOR VELOCITIES')
plot(t_hist, x_dot_hist)
legend('omega x', 'omega y', 'omega z', 'xdot', 'ydot', 'zdot')