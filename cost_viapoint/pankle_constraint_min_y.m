function [A B]=pankle_constraint_min_y(pankinit,pankfin,yankmin,nbparamank)

B1=[pankinit;zeros(nbparamank-1,1);(-1)^(nbparamank+1)*pankfin];
B2=yankmin*ones(nbparamank+1,1);
B=B1-B2;

A=zeros(nbparamank+1,nbparamank);

for i=1:nbparamank
    A(i,i)=(-1)^i;
    A(i+1,i)=(-1)^i;
end
A=-A;

end