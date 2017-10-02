function [xAzmp_gradient xBzmp_gradient yAzmp_gradient yBzmp_gradient xApzmp xBpzmp yApzmp yBpzmp xApcom xBpcom yApcom yBpcom xAfcom xBfcom yAfcom yBfcom xAfcom2 xBfcom2 yAfcom2 yBfcom2 xAtorque2ssp xBtorque2ssp yAtorque2ssp yBtorque2ssp Apzmpconstraintssp Bpzmpconstraintssp xAscomeq yAscomeq Bscomeq]=qp_generating_SSP(walking_param)
tic
%% %compute the matrix of time discretization
dt=compute_coeff_dt_matrix(walking_param.discretization,walking_param.frequency,walking_param.nbphases,walking_param.nbpointdiscret);
%% %matrix of sinh(wt) and cosh(wt)
scwt = compute_coeff_swt_cwt_matrix(walking_param.discretization,walking_param.w,walking_param.frequency,walking_param.nbphases);

%% %coeff gradient computation of zmp_poly
[xAzmp_gradient xBzmp_gradient]=compute_coeff_zmp_gradient(walking_param.tpassage,walking_param.xpsa_zmpinit,walking_param.xpsa_zmpfin,walking_param.nbphases,walking_param.nbparamABCD);
[yAzmp_gradient yBzmp_gradient]=compute_coeff_zmp_gradient(walking_param.tpassage,walking_param.ypsa_zmpinit,walking_param.ypsa_zmpfin,walking_param.nbphases,walking_param.nbparamABCD);
%% %A coeff of COM poly
A_gradient=com_morisawa_A_gradient(length(walking_param.tpassage)-1,walking_param.w);
%% %VW gradient
[xAVW xBVW]=com_morisawa_VW_gradient(xAzmp_gradient,xBzmp_gradient,A_gradient,walking_param.tpassage,walking_param.w);
[yAVW yBVW]=com_morisawa_VW_gradient(yAzmp_gradient,yBzmp_gradient,A_gradient,walking_param.tpassage,walking_param.w);
%% %position of ZMP and COM in x-axis
[xApzmp xBpzmp] = compute_pzmp_gradient(xAzmp_gradient,xBzmp_gradient,dt);
[xApcom xBpcom] = pcom_generator_morisawa_gradient(xAzmp_gradient,xBzmp_gradient,xAVW,xBVW,A_gradient,dt,scwt);
%% %position of ZMP and COM in y-axis
[yApzmp yBpzmp] = compute_pzmp_gradient(yAzmp_gradient,yBzmp_gradient,dt);
[yApcom yBpcom] = pcom_generator_morisawa_gradient(yAzmp_gradient,yBzmp_gradient,yAVW,yBVW,A_gradient,dt,scwt);

%% %force on com in x and y axis
[xAfcom xBfcom] = fcom_gradient(xApzmp,xBpzmp,xApcom,xBpcom,walking_param.mg,walking_param.z);
[yAfcom yBfcom] = fcom_gradient(yApzmp,yBpzmp,yApcom,yBpcom,walking_param.mg,walking_param.z);

%% %square sum of force on COM
[xAfcom2 xBfcom2] = fcom2_gradient(xAfcom,xBfcom);
[yAfcom2 yBfcom2] = fcom2_gradient(yAfcom,yBfcom);

%% %square sum of torques in akle in SSP
[xAtorque2ssp xBtorque2ssp] = torque2_ankle_SSP_gradient2(xApzmp,xBpzmp,xAfcom,xBfcom,walking_param.pankinit_firstSS(1),walking_param.pankfin_lastSS(1),walking_param.discretization,walking_param.mg,walking_param.ha);
[yAtorque2ssp yBtorque2ssp] = torque2_ankle_SSP_gradient2(yApzmp,yBpzmp,yAfcom,yBfcom,walking_param.pankinit_firstSS(2),walking_param.pankfin_lastSS(2),walking_param.discretization,walking_param.mg,walking_param.ha);

%% %constraint ineq in SSP
[xApankle xBpankle]=torque_ankle_positions_SSP2(walking_param.pankinit_firstSS(1),walking_param.pankfin_lastSS(1),walking_param.discretization);
[yApankle yBpankle]=torque_ankle_positions_SSP2(walking_param.pankinit_firstSS(2),walking_param.pankfin_lastSS(2),walking_param.discretization);
[Apzmpconstraintssp Bpzmpconstraintssp]=zmp_constraint_stability_SSP_xy(xApzmp,xBpzmp,yApzmp,yBpzmp,xApankle,xBpankle,yApankle,yBpankle,walking_param.discretization,walking_param.backtoankle,walking_param.fronttoankle,walking_param.exttoankle,walking_param.inttoankle,walking_param.sole_margin,walking_param.psi,walking_param.type_phase,walking_param.nbparamBb,walking_param.firstSS);

%% %Constraint eq for com speed initial and final
%Warning : A*P+B=0 => put -B in constraint in optimized function
[xAscomeq xBscomeq] = scom_eqcontraint(xAzmp_gradient,xBzmp_gradient,xAVW,xBVW,A_gradient,walking_param.tpassage,walking_param.nbphases,walking_param.w);
[yAscomeq yBscomeq] = scom_eqcontraint(yAzmp_gradient,yBzmp_gradient,yAVW,yBVW,A_gradient,walking_param.tpassage,walking_param.nbphases,walking_param.w);
%% %%%concatenation%%%
% Ascomeq=[xAscomeq zeros(size(yAscomeq)) zeros(size(xAscomeq,1),size(walking_param.nbparamank,2)) zeros(size(yAscomeq,1),size(walking_param.nbparamank,2));
%     zeros(size(xAscomeq)) zeros(size(xAscomeq,1),size(walking_param.nbparamank,2)) yAscomeq zeros(size(yAscomeq,1),size(walking_param.nbparamank,2))];
Bscomeq=[xBscomeq;
    yBscomeq];
%%%%%%%

'time to generate qp SSP'
toc
end