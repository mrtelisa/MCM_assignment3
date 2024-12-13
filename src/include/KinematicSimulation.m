%% Kinematic Simulation function
%
% Inputs
% - q current robot configuration
% - q_dot joints velocity
% - ts sample time
% - q_min lower joints bound
% - q_max upper joints bound
%
% Outputs
% - q new joint configuration

function [q] = KinematicSimulation(q, q_dot, ts, qmin, qmax)
    q = q + q_dot * ts; 
    
    q = max(q, qmin);
    q = min(q, qmax);
end