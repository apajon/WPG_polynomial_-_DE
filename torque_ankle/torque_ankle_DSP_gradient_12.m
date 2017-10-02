%A penser à inverser les y et x
%le couple en x est fonction des y
%le couple en y est fonction des x
%attention les pieds étant parallèles au debut, penser à inverser les
function [A B] = torque_ankle_DSP_gradient_12(Apzmp1,Bpzmp1,Afcom,Bfcom,Apankle,Bpankle,mg,ha,Afb)

A1=ha*Afb*([Afcom zeros(size(Afcom,1),size(Apzmp1,2)-size(Afcom,2))]);
A2=+Afb*mg*Apzmp1;
A3=[zeros(size(Afcom)) -Afb*mg*Apankle zeros(size(Afcom,1),size(Apzmp1,2)-size(Afcom,2)-size(Apankle,2))];
A=A1+A2+A3;
B=ha*Afb*(Bfcom)+Afb*mg*Bpzmp1-Afb*mg*Bpankle;

A=A(any(any(Apankle,2)+any(Bpankle,2),2),:);
B=B(any(any(Apankle,2)+any(Bpankle,2),2),:);

end