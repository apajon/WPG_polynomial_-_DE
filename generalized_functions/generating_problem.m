function [H f A b Aeq beq]=generating_problem(walking_param)
    [xAzmp_gradient xBzmp_gradient yAzmp_gradient yBzmp_gradient xApzmp xBpzmp yApzmp yBpzmp xApcom xBpcom yApcom yBpcom xAfcom xBfcom yAfcom yBfcom xAfcom2 xBfcom2 yAfcom2 yBfcom2 xAtorque2ssp xBtorque2ssp yAtorque2ssp yBtorque2ssp Apzmpconstraintssp Bpzmpconstraintssp xAscomeq yAscomeq Bscomeq]=qp_generating_SSP(walking_param);
    
    %% %%%%%%%
    zmp_type=8;
    %0 : solution ICRA2015 + min acc zmp1
    %1 : zmp1 analytical
    %2 : zmp1 with force repartition which allow discontinuity
    %3 : zmp1 with force repartition which allow discontinuity and constraint zmp1&2 speed >0
    %4 : solution ICRA2015 and constraint zmp1&2 speed >0
    %5 : solution ICRA2015 + min acc zmp1 + min acc zmp2 + zmp1&2 speed >0
    %6 : solution ICRA2015 + szmp1&2 near szmp
    %7 : solution ICRA2015 + pzmp1&2 near pzmp
    %8 : solution ICRA2015 + min acc zmp1 + min acc zmp2

    %%%%%%%

    %%%next file to compute next part of the code
    switch(walking_param.zmp_type)
    %     case 0 
    %         open('AC_QP_generating_DSP_00_icra2015.m');
    %     case 1 
    %         open('AC_QP_generating_DSP_02_analitical.m')
    %     case 2
    %         open('AC_QP_generating_DSP_02_discontinuity.m')
    %     case 3
    %         open('AC_QP_generating_03_discontinuity_szmppositiv.m')
    %     case 4
    %         open('AC_QP_generating_DSP_04_icra2015_szmppositiv.m')
    %     case 5 
    %         open('AC_QP_generating_DSP_05_icra2015_azmp_szmppositiv.m')
    %     case 6
    %         open('AC_QP_generating_DSP_06_icra2015_szmp.m');
    %     case 7 
    %         open('AC_QP_generating_DSP_07_icra2015_pzmp.m')
        case 8 
            [xApzmp1 xBpzmp1 yApzmp1 yBpzmp1 xApzmp2 xBpzmp2 yApzmp2 yBpzmp2 xAtorque2_1 xBtorque2_1 yAtorque2_1 yBtorque2_1 xAtorque2_2 xBtorque2_2 yAtorque2_2 yBtorque2_2 xAa2zmp1 xBa2zmp1 yAa2zmp1 yBa2zmp1 xAa2zmp2 xBa2zmp2 yAa2zmp2 yBa2zmp2 A b AscomeqDSP]=qp_generating_DSP_08_icra2015_azmp(walking_param,xAzmp_gradient,xBzmp_gradient,yAzmp_gradient,yBzmp_gradient,xApzmp,xBpzmp,yApzmp,yBpzmp,xAfcom,xBfcom,yAfcom,yBfcom,Apzmpconstraintssp,Bpzmpconstraintssp,xAscomeq,yAscomeq);
    end
    
    save('generalized_functions/trajectories.mat','xApzmp','xBpzmp','yApzmp','yBpzmp','xApcom','xBpcom','yApcom','yBpcom','xApzmp1','xBpzmp1','yApzmp1','yBpzmp1','xApzmp2','xBpzmp2','yApzmp2','yBpzmp2','xAfcom','xBfcom','yAfcom','yBfcom')

    [H f Aeq beq]=qp_generating_problem_matrix(walking_param,xAfcom2,xBfcom2,yAfcom2,yBfcom2,xAtorque2ssp,xBtorque2ssp,yAtorque2ssp,yBtorque2ssp,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,xAa2zmp1,xBa2zmp1,yAa2zmp1,yBa2zmp1,xAa2zmp2,xBa2zmp2,yAa2zmp2,yBa2zmp2,AscomeqDSP,Bscomeq);
    
%      C=walking_param.lambda*(xBfcom'*xBfcom+yBfcom'*yBfcom)+(1-walking_param.lambda)*(xBtorquessp'*xBtorquessp+yBtorquessp'*yBtorquessp)+walking_param.epsilon*(xBazmp1'*xBazmp1+yBazmp1'*yBazmp1+xBazmp2'*xBazmp2+yBazmp2'*yBazmp2)+0.5*(1-walking_param.lambda)*(xBtorquedsp1'*xBtorquedsp1+yBtorquedsp1'*yBtorquedsp1+xBtorquedsp2'*xBtorquedsp2+yBtorquedsp2'*yBtorquedsp2);

    
end