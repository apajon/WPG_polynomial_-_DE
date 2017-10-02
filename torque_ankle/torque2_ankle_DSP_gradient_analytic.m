function [A B] = torque2_ankle_DSP_gradient_analytic(Apzmp1,Bpzmp1,Afcom,Bfcom,Apankle,Bpankle,mg,ha,Afb)

[Atorquedsp Btorquedsp] = torque_ankle_DSP_gradient_analytic(Apzmp1,Bpzmp1,Afcom,Bfcom,Apankle,Bpankle,mg,ha,Afb);

A=transpose(Atorquedsp)*Atorquedsp;
B=transpose(Btorquedsp)*Atorquedsp;

end