function [A B] = torque2_ankle_DSP_gradient_22(Apzmp2,Bpzmp2,Afcom,Bfcom,Apankle,Bpankle,mg,ha,Afb)

[Atorquedsp Btorquedsp] = torque_ankle_DSP_gradient_22(Apzmp2,Bpzmp2,Afcom,Bfcom,Apankle,Bpankle,mg,ha,Afb);

A=transpose(Atorquedsp)*Atorquedsp;
B=transpose(Btorquedsp)*Atorquedsp;

end