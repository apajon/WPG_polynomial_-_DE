function [A B]=pankle_constraint_max_x(pankinit,pankfin,xankmax,nbparamank)

B1=[-pankinit;zeros(nbparamank-1,1);pankfin];
B2=xankmax*ones(nbparamank+1,1);
B=B2-B1;

A=zeros(nbparamank+1,nbparamank);

for i=1:nbparamank
    A(i,i)=1;
    A(i+1,i)=-1;
end
%A=-A;

end