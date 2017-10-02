%Compute the gradient matrix of zmp position
function [A B]=compute_szmp1_gradient(Azmp1_coeff_gradient,Bzmp1_coeff_gradient,ddt)
% dt=compute_coeff_dt1_matrix(discretization,frequency,cutting);

A=ddt*Azmp1_coeff_gradient;
B=ddt*Bzmp1_coeff_gradient;
end