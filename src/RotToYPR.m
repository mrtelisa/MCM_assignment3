function [psi,theta,phi] = RotToYPR(R)
    tolerance = 10e-10;
    
    if (norm(inv(R) - transpose(R)) > tolerance || abs(det(R) - 1)  > tolerance) 
        error("The matrix is not a rotation matrix");
    end
    
    theta = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));
    if cos(theta) ~= 0
        psi = atan2(R(2, 1), R(1, 1));
        phi = atan2(R(3, 2), R(3, 3));
    else
        error("This configuration could be a singularity");
    end
end


