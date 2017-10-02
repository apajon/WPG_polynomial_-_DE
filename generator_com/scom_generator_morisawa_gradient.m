%%%generate the COM speed gradient%%%
%Use Morisawa algorithm with 5th zmp polynomial coeff known
function [A B] = scom_generator_morisawa_gradient(A_zmp_gradient,B_zmp_gradient,AVW,BVW,A_gradient,discretization,w,frequency,mddt) %rajouter le calcul de scom

% [AVW BVW]=com_morisawa_VW_gradient(tpassage,psa_zmpinit,psa_zmpfin,pcominit,pcomfin,w);
[mdsdcwt] = compute_coeff_dswt_dcwt_matrix(discretization,w,frequency);
% [mddt] = compute_coeff_ddt_matrix(discretization,frequency);
% A_gradient=com_morisawa_A_gradient(length(tpassage)-1,w);
% [A_zmp_gradient B_zmp_gradient]=compute_coeff_zmp_gradient(tpassage,psa_zmpinit,psa_zmpfin);

A=mdsdcwt*AVW+mddt*A_gradient*A_zmp_gradient;
B=mdsdcwt*BVW+mddt*A_gradient*B_zmp_gradient;
%com=mscwt*(BVW+AVW*psa_abcd)+mdt*A_gradient*(A_zmp_gradient*psa_abcd +B_zmp_gradient);

end