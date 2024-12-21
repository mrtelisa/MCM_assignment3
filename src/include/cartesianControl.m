%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end
        function [x_dot]=getCartesianReference(self,bTg)
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control

            lambda = eye(6);

            lambda(1:3, 1:3) = self.k_a * eye(3); 
            lambda(4:6, 4:6) = self.k_l * eye(3);
            
            bTt = self.gm.getToolTransformWrtBase();
            bRt = bTt(1:3, 1:3);
            tRb = transpose(bRt);

            tRg = tRb * bTg(1:3, 1:3);
            

            %TODO: check if we need to use ypr or angle axis
            be = zeros(6, 1);
            [h, theta] = RotToAngleAxis(tRg);

            trho = h * theta;

            be(1:3) = trho * bRt;
            be(4:6) = bTg(1:3, 4) - bTt (1:3, 4);

            x_dot = lambda * be;
        end
    end
end

