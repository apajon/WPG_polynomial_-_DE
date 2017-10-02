%Compute the gradient matrix of zmp position
function [A B]=compute_pzmp2_gradient(Apzmp,Bpzmp,Apzmp1,Bpzmp1,Afb,mg)
xf2=(eye(length(Afb))-Afb)*mg;
% ixf2=inv(xf2);

A=-mg*(xf2\(Apzmp1-[Apzmp zeros(size(Apzmp,1),size(Apzmp1,2)-size(Apzmp,2))]))+Apzmp1;
B=-mg*(xf2\(Bpzmp1-Bpzmp))+Bpzmp1;
end
% Compute the gradient matrix of zmp position
% function [A B]=compute_pzmp2_gradient(Apzmp,Bpzmp,Apzmp1,Bpzmp1,Afb,mg)
% xf2=1-Afb;
% 
% AA=(Apzmp1-[Apzmp zeros(size(Apzmp,1),size(Apzmp1,2)-size(Apzmp,2))]);
% BB=(Bpzmp1-Bpzmp);
% 
% for i=1:size(AA,1)
%    AA(i,:)=AA(i,:)/xf2(i);
%    BB(i,:)=BB(i,:)/xf2(i);
% end
% 
% 
% A=-mg*AA+Apzmp1;
% B=-mg*BB+Bpzmp1;
% end