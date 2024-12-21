%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end
        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix
                
            S_te = eye(6);
            bTe = self.gm.getTransformWrtBase(self.gm.jointNumber);
            bRe = bTe(1:3, 1:3);
            er_te = self.gm.eTt(1:3, 4);
            br_te = bRe * er_te;
            S_te (4:6, 1:3) = transpose(skew(br_te));

            self.J = zeros(6, self.gm.jointNumber);
            iTe = eye(4);
            ik_i = [0; 0; 1];
            for i = self.gm.jointNumber:-1:1
                ir_ei = iTe(1:3, 4);
                bTi = self.gm.getTransformWrtBase(i);
                bRi = bTi(1:3, 1:3);
                bk_i = bRi * ik_i;
                if self.gm.jointType(i) == 0
                    self.J(:, i) = [bk_i; bRi * cross(ik_i, ir_ei)];
                else
                    self.J(:, i) = [zeros(3, 1); bk_i];
                end
                iTe = self.gm.iTj(:,:,i) * iTe;
            end

            self.J = S_te * self.J;

        end
    end
end
