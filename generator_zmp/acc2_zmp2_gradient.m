function [A B] = acc2_zmp2_gradient(Aazmp2,Bazmp2)

A=transpose(Aazmp2)*Aazmp2;
B=transpose(Bazmp2)*Aazmp2;

end