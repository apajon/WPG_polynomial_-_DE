function [A B]=com_morisawa_VW_gradient(A_zmp_gradient,B_zmp_gradient,A_gradient,tpassage,w)

[A_global B_global]=com_morisawa_VW_gradient_global(A_gradient,tpassage,w);

A=[zeros(size(A_global,1),size(A_zmp_gradient,2)-4) B_global]+A_global*A_zmp_gradient;
B=A_global*B_zmp_gradient;
end