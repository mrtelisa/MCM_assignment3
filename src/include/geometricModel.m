%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    % eTt (OPTIONAL) add a tool to the model rigid attached to the
    % end-effector
    properties
        iTj_0
        jointType
        jointNumber
        iTj
        q
        eTt
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType,eTt)
            if nargin > 1
                 if ~exist('eTt','var')
                     % third parameter does not exist, so default it to something
                      eTt = eye(4);
                 end
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
                self.eTt = eTt;
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end
        function updateDirectGeometry(self, q)
            %%% GetDirectGeometryFunction
            % This method update the matrices iTj.
            % Inputs:
            % q : joints current position ;

            % The function updates:
            % - iTj: vector of matrices containing the transformation matrices from link i to link j for the input q.
            % The size of iTj is equal to (4,4,numberOfLinks)
            if length(q) ~= self.jointNumber
                error("Invalid number of joint variables, expected: %d, given: %d", self.jointNumber, length(q));
            end
            self.q = q;

            for i = 1:length(q)
                Tz = eye(4,4);
                if self.jointType(i) == 0
                    s = sin(q(i));
                    c = cos(q(i));
                    Tz(1:2, 1:2) = [c -s; s c];
                else
                    Tz(3, 4) = q(i);
                end
                self.iTj(:,:,i) =  self.iTj_0(:,:,i) * Tz;
            end
        end

        function [bTk] = getTransformWrtBase(self,k)
            %% GetTransformatioWrtBase function
            % Inputs :
            % k: the idx for which computing the transformation matrix
            % outputs
            % bTk : transformation matrix from the manipulator base to the k-th joint in
            % the configuration identified by iTj.
            if (k < 1)
                error("invalid frame number");
            end

            bTk = eye(4,4);
            for i = 1:k
                bTk = bTk * self.iTj(:,:,i);
            end
        end

        function [bTt] = getToolTransformWrtBase(self)
            %% getToolTransformWrtBase function
            % Inputs :
            % None 
            % bTt : transformation matrix from the manipulator base to the
            % tool
            bTt = self.getTransformWrtBase(self.jointNumber) * self.eTt;
        end
    end
end


