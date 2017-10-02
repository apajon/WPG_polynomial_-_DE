function [A B] = acc2_zmp1_gradient(Aazmp1,Bazmp1)

A=transpose(Aazmp1)*Aazmp1;
B=transpose(Bazmp1)*Aazmp1;

end