function R = AngleAxisToRot(h,theta)
    skew_h = skew(h);
    
    I = eye(3);
    R = I + sin(theta) * skew_h + (1-cos(theta)) * (skew_h ^ 2);

end