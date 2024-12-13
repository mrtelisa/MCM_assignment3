
function [h, theta] = RotToAngleAxis(R)
    % Check matrix R to see if its size is 3x3
    tolerance = 10e-10;

    if (norm(inv(R) - transpose(R)) > tolerance || abs(det(R) - 1)  > tolerance) 
        error("The matrix is not a rotation matrix");
    end

    theta = acos((trace(R) - 1) / 2);
    if theta < tolerance
        h = [1, 0, 0];
    elseif abs(theta - pi) < tolerance
        h1 = sqrt((R(1,1) + 1)/2);
        h2 = my_sign(h1) * my_sign(R(1,2)) * sqrt((R(2,2) + 1)/2);
        h3 = my_sign(h1) * my_sign(R(1,3)) * sqrt((R(3,3) + 1)/2);
        
        h = [h1, h2, h3];
    else
       rrt = (R - transpose(R)) /2;
       a = vex(rrt);
       h = a / sin(theta);
    end
end

 
function a = vex(S_a)
% input: skew matrix S_a (3x3)
% output: the original a vector (3x1)
    a = [S_a(3, 2), S_a(1, 3), S_a(2, 1)];
end

% the sign function in matlab returns 0 in case of 0, this is not what we
% want for the computation of the h vector, this function solves this
% problem
function s = my_sign(x)
    if x == 0
        s = 1;
    else
        s = sign(x);
    end
end