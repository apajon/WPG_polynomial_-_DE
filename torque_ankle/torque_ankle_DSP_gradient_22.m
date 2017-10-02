%A penser à inverser les y et x
%le couple en x est fonction des y
%le couple en y est fonction des x
%attention les pieds étant parallèles au debut, penser à inverser les
function [A B] = torque_ankle_DSP_gradient_22(Apzmp2,Bpzmp2,Afcom,Bfcom,Apankle,Bpankle,mg,ha,Afb)

IAfb=eye(size(Afb,1))-Afb;

A1=ha*IAfb*([Afcom zeros(size(Afcom,1),size(Apzmp2,2)-size(Afcom,2))]);
A2=+IAfb*mg*Apzmp2;
A3=[zeros(size(Afcom)) -IAfb*mg*Apankle zeros(size(Afcom,1),size(Apzmp2,2)-size(Afcom,2)-size(Apankle,2))];
A=A1+A2+A3;

B=ha*IAfb*(Bfcom)+IAfb*mg*Bpzmp2-IAfb*mg*Bpankle;

end