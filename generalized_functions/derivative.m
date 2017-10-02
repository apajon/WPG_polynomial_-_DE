function []=derivative(walking_param)

% load('trajectories.mat')

ddt=compute_coeff_ddt_matrix(walking_param.discretization,walking_param.frequency);

[xAzmp_gradient xBzmp_gradient]=compute_coeff_zmp_gradient(walking_param.tpassage,walking_param.xpsa_zmpinit,walking_param.xpsa_zmpfin,walking_param.nbphases,walking_param.nbparamABCD-4);
[yAzmp_gradient yBzmp_gradient]=compute_coeff_zmp_gradient(walking_param.tpassage,walking_param.ypsa_zmpinit,walking_param.ypsa_zmpfin,walking_param.nbphases,walking_param.nbparamABCD-4);

A_gradient=com_morisawa_A_gradient(length(walking_param.tpassage)-1,walking_param.w);

[xAVW xBVW]=com_morisawa_VW_gradient(xAzmp_gradient,xBzmp_gradient,A_gradient,walking_param.tpassage,walking_param.w);
[yAVW yBVW]=com_morisawa_VW_gradient(yAzmp_gradient,yBzmp_gradient,A_gradient,walking_param.tpassage,walking_param.w);

%% %%%speed of com computation
[xAscom xBscom]=scom_generator_morisawa_gradient(xAzmp_gradient,xBzmp_gradient,xAVW,xBVW,A_gradient,walking_param.discretization,walking_param.w,walking_param.frequency,ddt);
walking_param.xscom=xAscom*walking_param.psa_abcd(1:length(walking_param.psa_abcd)/2)+xBscom;
[yAscom yBscom]=scom_generator_morisawa_gradient(yAzmp_gradient,yBzmp_gradient,yAVW,yBVW,A_gradient,walking_param.discretization,walking_param.w,walking_param.frequency,ddt);
walking_param.yscom=yAscom*walking_param.psa_abcd(length(walking_param.psa_abcd)/2+1:length(walking_param.psa_abcd))+yBscom;

%% %%%acceleration of com computation
walking_param.xacom=(walking_param.xpcom-walking_param.xpzmp)*walking_param.g/walking_param.z;
walking_param.yacom=(walking_param.ypcom-walking_param.ypzmp)*walking_param.g/walking_param.z;

%% %%%writing ZMP and COM trajectories in 'zmp&com.txt' file%%%%
walking_param.xsankle_l=([0;(walking_param.xpankle_l(2:end)-walking_param.xpankle_l(1:end-1))*walking_param.frequency;]);
walking_param.ysankle_l=([0;(walking_param.ypankle_l(2:end)-walking_param.ypankle_l(1:end-1))*walking_param.frequency;]);
walking_param.zsankle_l=([0;(walking_param.zpankle_l(2:end)-walking_param.zpankle_l(1:end-1))*walking_param.frequency;]);
walking_param.xsankle_r=([0;(walking_param.xpankle_r(2:end)-walking_param.xpankle_r(1:end-1))*walking_param.frequency;]);
walking_param.ysankle_r=([0;(walking_param.ypankle_r(2:end)-walking_param.ypankle_r(1:end-1))*walking_param.frequency;]);
walking_param.zsankle_r=([0;(walking_param.zpankle_r(2:end)-walking_param.zpankle_r(1:end-1))*walking_param.frequency;]);

walking_param.xaankle_l=([0;(walking_param.xpankle_l(3:end)+walking_param.xpankle_l(1:end-2)-2*walking_param.xpankle_l(2:end-1))*walking_param.frequency;0]);
walking_param.yaankle_l=([0;(walking_param.ypankle_l(3:end)+walking_param.ypankle_l(1:end-2)-2*walking_param.ypankle_l(2:end-1))*walking_param.frequency;0]);
walking_param.zaankle_l=([0;(walking_param.zpankle_l(3:end)+walking_param.zpankle_l(1:end-2)-2*walking_param.zpankle_l(2:end-1))*walking_param.frequency;0]);
walking_param.xaankle_r=([0;(walking_param.xpankle_r(3:end)+walking_param.xpankle_r(1:end-2)-2*walking_param.xpankle_r(2:end-1))*walking_param.frequency;0]);
walking_param.yaankle_r=([0;(walking_param.ypankle_r(3:end)+walking_param.ypankle_r(1:end-2)-2*walking_param.ypankle_r(2:end-1))*walking_param.frequency;0]);
walking_param.zaankle_r=([0;(walking_param.zpankle_r(3:end)+walking_param.zpankle_r(1:end-2)-2*walking_param.zpankle_r(2:end-1))*walking_param.frequency;0]);

end