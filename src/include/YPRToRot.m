
function [R] = YPRToRot(psi, theta, phi)

% Rotation matrix around z-axis (yaw)     
Rz = [cos(psi), -sin(psi), 0;
      sin(psi),  cos(psi), 0;
      0,          0,      1];

% Rotation matrix around y-axis (pitch)
Ry = [cos(theta),  0, sin(theta);
      0,          1,         0;
      -sin(theta), 0, cos(theta)];

% Rotation matrix around x-axis (roll)
Rx = [1,        0,         0;
      0, cos(phi), -sin(phi);
      0, sin(phi),  cos(phi)];

% Combined rotation matrix
R = Rz * Ry * Rx;
end
