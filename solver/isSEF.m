function b = isSEF(pose)

global phi_1;
global phi_2;

angle_diff = wrapTo2Pi(pose.phi - pose.theta);

if angle_diff >= phi_1 && angle_diff <= phi_2
    b = true;
else
    b = false;
end

end
