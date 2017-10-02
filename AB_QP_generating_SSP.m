clc

% %for windows
% addpath .\cost_viapoint
% addpath .\generator_zmp
% addpath .\generator_com
% addpath .\f_com
% addpath .\torque_ankle
% addpath .\divers

%for linux
% addpath ./cost_viapoint
% addpath ./generator_zmp
% addpath ./generator_com
% addpath ./f_com
% addpath ./torque_ankle
% addpath ./divers

tic
%% %compute the matrix of time discretization
dt=compute_coeff_dt_matrix(discretization,frequency,nbphases,nbpointdiscret);
%% %matrix of sinh(wt) and cosh(wt)
scwt = compute_coeff_swt_cwt_matrix(discretization,w,frequency,nbphases);

%% %coeff gradient computation of zmp_poly
[xAzmp_gradient xBzmp_gradient]=compute_coeff_zmp_gradient(tpassage,xpsa_zmpinit,xpsa_zmpfin,nbphases,nbparamABCD);
[yAzmp_gradient yBzmp_gradient]=compute_coeff_zmp_gradient(tpassage,ypsa_zmpinit,ypsa_zmpfin,nbphases,nbparamABCD);
%% %A coeff of COM poly
A_gradient=com_morisawa_A_gradient(length(tpassage)-1,w);
%% %VW gradient
[xAVW xBVW]=com_morisawa_VW_gradient(xAzmp_gradient,xBzmp_gradient,A_gradient,tpassage,w);
[yAVW yBVW]=com_morisawa_VW_gradient(yAzmp_gradient,yBzmp_gradient,A_gradient,tpassage,w);
%% %position of ZMP and COM in x-axis
[xApzmp xBpzmp] = compute_pzmp_gradient(xAzmp_gradient,xBzmp_gradient,dt);
[xApcom xBpcom] = pcom_generator_morisawa_gradient(xAzmp_gradient,xBzmp_gradient,xAVW,xBVW,A_gradient,dt,scwt);
%% %position of ZMP and COM in y-axis
[yApzmp yBpzmp] = compute_pzmp_gradient(yAzmp_gradient,yBzmp_gradient,dt);
[yApcom yBpcom] = pcom_generator_morisawa_gradient(yAzmp_gradient,yBzmp_gradient,yAVW,yBVW,A_gradient,dt,scwt);

%% %force on com in x and y axis
[xAfcom xBfcom] = fcom_gradient(xApzmp,xBpzmp,xApcom,xBpcom,mg,h);
[yAfcom yBfcom] = fcom_gradient(yApzmp,yBpzmp,yApcom,yBpcom,mg,h);

%% %square sum of force on COM
[xAfcom2 xBfcom2] = fcom2_gradient(xAfcom,xBfcom);
[yAfcom2 yBfcom2] = fcom2_gradient(yAfcom,yBfcom);

%% %square sum of torques in akle in SSP
[xAtorque2ssp xBtorque2ssp] = torque2_ankle_SSP_gradient2(xApzmp,xBpzmp,xAfcom,xBfcom,pankinit2(1),pankfin1(1),discretization,mg,ha+(e-e_));
[yAtorque2ssp yBtorque2ssp] = torque2_ankle_SSP_gradient2(yApzmp,yBpzmp,yAfcom,yBfcom,pankinit2(2),pankfin1(2),discretization,mg,ha+(e-e_));
[xAtorquessp xBtorquessp] = torque_ankle_SSP_gradient2(xApzmp,xBpzmp,xAfcom,xBfcom,pankinit2(1),pankfin1(1),discretization,mg,ha+(e-e_));
[yAtorquessp yBtorquessp] = torque_ankle_SSP_gradient2(yApzmp,yBpzmp,yAfcom,yBfcom,pankinit2(2),pankfin1(2),discretization,mg,ha+(e-e_));
%% %constraint ineq in SSP
[xApankle xBpankle]=torque_ankle_positions_SSP2(pankinit2(1),pankfin1(1),discretization);
[yApankle yBpankle]=torque_ankle_positions_SSP2(pankinit2(2),pankfin1(2),discretization);
[Apzmpconstraintssp Bpzmpconstraintssp]=zmp_constraint_stability_SSP_xy(xApzmp,xBpzmp,yApzmp,yBpzmp,xApankle,xBpankle,yApankle,yBpankle,discretization,backtoankle,fronttoankle,exttoankle,inttoankle,sole_margin,psi,type_phase,nbparamBb,firstSS);
%% %Constraint eq for com speed initial and final
%Warning : A*P+B=0 => put -B in constraint in optimized function
[xAscomeq xBscomeq] = scom_eqcontraint(xAzmp_gradient,xBzmp_gradient,xAVW,xBVW,A_gradient,tpassage,nbphases,w);
[yAscomeq yBscomeq] = scom_eqcontraint(yAzmp_gradient,yBzmp_gradient,yAVW,yBVW,A_gradient,tpassage,nbphases,w);
%% %%%concatenation%%%
Ascomeq=[xAscomeq zeros(size(yAscomeq)) zeros(size(xAscomeq,1),size(nbparamank,2)) zeros(size(yAscomeq,1),size(nbparamank,2));
    zeros(size(xAscomeq)) zeros(size(xAscomeq,1),size(nbparamank,2)) yAscomeq zeros(size(yAscomeq,1),size(nbparamank,2))];
Bscomeq=[xBscomeq;
    yBscomeq];
%%%%%%%

'time to generate qp SSP'
toc

%% %%%%%%%
open('AC_QP_generating_DSP_08_icra2015_azmp.m')