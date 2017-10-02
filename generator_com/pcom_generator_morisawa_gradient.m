%%%generate the COM trajectory gradient%%%
%Use Morisawa algorithm with 5th zmp polynomial coeff known
function [A B] = pcom_generator_morisawa_gradient(A_zmp_gradient,B_zmp_gradient,AVW,BVW,A_gradient,mdt,mscwt) %rajouter le calcul de scom

% [AVW BVW]=com_morisawa_VW_gradient(tpassage,psa_zmpinit,psa_zmpfin,pcominit,pcomfin,w);
% [mscwt] = compute_coeff_swt_cwt_matrix(discretization,w,frequency);
% [mdt] = compute_coeff_dt_matrix(discretization,frequency);
% A_gradient=com_morisawa_A_gradient(length(tpassage)-1,w);
% [A_zmp_gradient B_zmp_gradient]=compute_coeff_zmp_gradient(tpassage,psa_zmpinit,psa_zmpfin);

A=mscwt*AVW+mdt*A_gradient*A_zmp_gradient;
B=mscwt*BVW+mdt*A_gradient*B_zmp_gradient;
%com=mscwt*(BVW+AVW*psa_abcd)+mdt*A_gradient*(A_zmp_gradient*psa_abcd +B_zmp_gradient);

end