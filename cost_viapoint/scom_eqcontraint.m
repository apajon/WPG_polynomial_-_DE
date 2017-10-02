function [A B] = scom_eqcontraint(A_zmp_gradient,B_zmp_gradient,AVW,BVW,A_gradient,tpassage,nbphases,w)

dti=0;
dtf=tpassage(nbphases+1)-tpassage(nbphases);
mscwt=[w*sinh(w*dti) w*cosh(w*dti) zeros(1,(nbphases-1)*2);
    zeros(1,(nbphases-1)*2) w*sinh(w*dtf) w*cosh(w*dtf)];
mdt=[0 1 2*dti 3*dti^2 4*dti^3 5*dti^4 zeros(1,(nbphases-1)*6);
    zeros(1,(nbphases-1)*6) 0 1 2*dtf 3*dtf^2 4*dtf^3 5*dtf^4];

A=mscwt*AVW+mdt*A_gradient*A_zmp_gradient-[zeros(2,size(A_zmp_gradient,2)-2) [1 0;0 1]];
B=mscwt*BVW+mdt*A_gradient*B_zmp_gradient;
end