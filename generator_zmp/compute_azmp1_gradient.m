%Compute the gradient matrix of zmp position
function [A B]=compute_azmp1_gradient(Azmp1_coeff_gradient,Bzmp1_coeff_gradient,dddt)
% dt=compute_coeff_dt1_matrix(discretization,frequency,cutting);

A=dddt*Azmp1_coeff_gradient;
B=dddt*Bzmp1_coeff_gradient;
end