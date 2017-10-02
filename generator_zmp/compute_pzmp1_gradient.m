%Compute the gradient matrix of zmp position
function [A B]=compute_pzmp1_gradient(Azmp1_coeff_gradient,Bzmp1_coeff_gradient,dt)
% dt=compute_coeff_dt1_matrix(discretization,frequency,cutting);

A=dt*Azmp1_coeff_gradient;
B=dt*Bzmp1_coeff_gradient;
end