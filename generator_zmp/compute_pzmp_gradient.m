%Compute the gradient matrix of zmp position
function [A B]=compute_pzmp_gradient(Azmp_gradient,Bzmp_gradient,dt)
% [Azmp_gradient Bzmp_gradient]=compute_coeff_zmp_gradient(tpassage,psa_zmpinit,psa_zmpfin);
% dt=compute_coeff_dt_matrix(discretization,frequency);
%dtg=dt*g;
%B=[dtg(:,1:3) dtg(:,size(dtg,2)-2:size(dtg,2))];
%A=dtg(:,4:size(dtg,2)-3);

A=dt*Azmp_gradient;
B=dt*Bzmp_gradient;
end