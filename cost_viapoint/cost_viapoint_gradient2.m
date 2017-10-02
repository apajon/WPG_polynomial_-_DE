function [A B] = cost_viapoint_gradient2(Afcom2,Bfcom2,Atorque2ssp,Btorque2ssp,lambda,mu)

% [Afcom2 Bfcom2] = fcom2_gradient(Afcom,Bfcom);
% [Atorque2ssp Btorque2ssp] = torque2_ankle_SSP_gradient2(Apzmp,Bpzmp,Afcom,Bfcom,pankinit,pankfin,discretization,mg,ha);

A=lambda*[Afcom2 zeros(size(Afcom2,1),size(Atorque2ssp,2)-size(Afcom2,2));zeros(size(Atorque2ssp,1)-size(Afcom2,1),size(Afcom2,2)) zeros(size(Atorque2ssp,1)-size(Afcom2,1),size(Atorque2ssp,2)-size(Afcom2,2))]+mu*Atorque2ssp;
B=lambda*[Bfcom2 zeros(1,size(Btorque2ssp,2)-size(Bfcom2,2))] +mu*Btorque2ssp;
end
