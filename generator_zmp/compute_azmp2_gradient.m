%Compute the gradient matrix of zmp2 acceleration
function [A B]=compute_azmp2_gradient(Apzmp,Bpzmp,Apzmp1,Bpzmp1,Aszmp,Bszmp,Aszmp1,Bszmp1,Aazmp,Bazmp,Aazmp1,Bazmp1,Afb,dAfb,ddAfb)
Afb_=(eye(length(Afb))-Afb);
dAfb_=-dAfb;
ddAfb_=-ddAfb;


A1=-(Afb_\(Aazmp1-[Aazmp zeros(size(Aazmp,1),size(Aazmp1,2)-size(Aazmp,2))]));
A2=+2*dAfb_*((Afb_)\((Afb_)\(Aszmp1-[Aszmp zeros(size(Aszmp,1),size(Aszmp1,2)-size(Aszmp,2))])));
A3=+ddAfb_*((Afb_)\((Afb_)\(Apzmp1-[Apzmp zeros(size(Apzmp,1),size(Apzmp1,2)-size(Apzmp,2))])))-2*dAfb_*dAfb_*((Afb_)\((Afb_)\((Afb_)\(Apzmp1-[Apzmp zeros(size(Apzmp,1),size(Apzmp1,2)-size(Apzmp,2))]))));
% A1=0;
% A2=0;
% A3=0;

B1=-(Afb_\(Bazmp1-Bazmp));
B2=+2*dAfb_*((Afb_)\((Afb_)\(Bszmp1-Bszmp)));
B3=+ddAfb_*((Afb_)\((Afb_)\(Bpzmp1-Bpzmp)))-2*dAfb_*dAfb_*((Afb_)\((Afb_)\((Afb_)\(Bpzmp1-Bpzmp))));
% B1=0;
% B2=0;
% B3=0;

A=A1+A2+A3+Aazmp1;
B=B1+B2+B3+Bazmp1;

end
