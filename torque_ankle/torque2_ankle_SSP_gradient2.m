function [A B] = torque2_ankle_SSP_gradient2(Apzmp,Bpzmp,Afcom,Bfcom,pankinit,pankfin,discretization,mg,ha)

[Atorquessp Btorquessp] = torque_ankle_SSP_gradient2(Apzmp,Bpzmp,Afcom,Bfcom,pankinit,pankfin,discretization,mg,ha);

A=transpose(Atorquessp)*Atorquessp;
B=transpose(Btorquessp)*Atorquessp;

end