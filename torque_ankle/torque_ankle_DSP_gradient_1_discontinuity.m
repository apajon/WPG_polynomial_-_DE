%A penser à inverser les y et x
%le couple en x est fonction des y
%le couple en y est fonction des x
%attention les pieds étant parallèles au debut, penser à inverser les
function [A B] = torque_ankle_DSP_gradient_1_discontinuity(Apzmp1,Bpzmp1,Afcom,Bfcom,Apankle,Bpankle,mg,ha,Afb)

A1=ha*Afb*([-Afcom zeros(size(Apankle)) zeros(size(Apzmp1))]);
A2=+Afb*mg*[zeros(size(Afcom)) zeros(size(Apankle)) Apzmp1];
A3=[zeros(size(Afcom)) -Afb*mg*Apankle zeros(size(Apzmp1))];
A=A1+A2+A3;
B=ha*Afb*(-Bfcom)+Afb*mg*Bpzmp1-Afb*mg*Bpankle;

A=A(any(any(Apankle,2)+any(Bpankle,2),2),:);
B=B(any(any(Apankle,2)+any(Bpankle,2),2),:);

end