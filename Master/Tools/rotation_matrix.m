% Calculate rotation matrix
function R = rotation_matrix(psi)
    R = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
end