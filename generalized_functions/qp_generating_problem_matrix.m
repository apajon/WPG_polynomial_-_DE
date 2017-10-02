function [H f Aeq beq]=qp_generating_problem_matrix(walking_param,xAfcom2,xBfcom2,yAfcom2,yBfcom2,xAtorque2ssp,xBtorque2ssp,yAtorque2ssp,yBtorque2ssp,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,xAa2zmp1,xBa2zmp1,yAa2zmp1,yBa2zmp1,xAa2zmp2,xBa2zmp2,yAa2zmp2,yBa2zmp2,AscomeqDSP,Bscomeq)

% %%%%weight of optimization criterion%%%
% lambda=0.6; %weight of fcom
% mu=1-lambda; %weight of ankle torques
% epsilon=1; %weight of ZMP1&2 acceleration 250

% %%%%%%%
% optim_type=3;
% %1 : optim cost with only torque in ankle during DSP 
% %2 : optim cost with only acceleration of ZMP1
% %3 : optim cost with both torque in ankle during DSP and acceleration of ZMP1-2 
% %4 : optim with zmp1 analytical and cost with only torque in ankle during DSP 
% %%%%%%%


%%%position, speed and acceleration gradient of viapoints%%%
[xAviapoint xBviapoint] = cost_viapoint_gradient2(xAfcom2,xBfcom2,xAtorque2ssp,xBtorque2ssp,walking_param.lambda,walking_param.mu);
[yAviapoint yBviapoint] = cost_viapoint_gradient2(yAfcom2,yBfcom2,yAtorque2ssp,yBtorque2ssp,walking_param.lambda,walking_param.mu);


switch(walking_param.optim_type)
   case 1
        %%%%cost taking into account torque in DSP%%%
        [xAviapointDSP xBviapointDSP] = cost_viapoint_gradient_withDSP(xAviapoint,xBviapoint,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,0.5*(walking_param.tds/(walking_param.tds+walking_param.tss))*walking_param.mu);
        [yAviapointDSP yBviapointDSP] = cost_viapoint_gradient_withDSP(yAviapoint,yBviapoint,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,0.5*(walking_param.tds/(walking_param.tds+walking_param.tss))*walking_param.mu);

    case 2
        %%%%cost taking into account azmp1 only%%%
        [xAviapointDSP xBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(xAviapoint,xBviapoint,zeros(size(xAtorque2_1)),zeros(size(xBtorque2_1)),zeros(size(xAtorque2_2)),zeros(size(xBtorque2_2)),xAa2zmp1+xAa2zmp2,xBa2zmp1+xBa2zmp2,epsilon,0.5*(walking_param.tds/(walking_param.tds+walking_param.tss))*walking_param.mu);
        [yAviapointDSP yBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(yAviapoint,yBviapoint,zeros(size(yAtorque2_1)),zeros(size(yBtorque2_1)),zeros(size(yAtorque2_2)),zeros(size(yBtorque2_2)),yAa2zmp1+yAa2zmp2,yBa2zmp1+yBa2zmp2,epsilon,0.5*(walking_param.tds/(walking_param.tds+walking_param.tss))*walking_param.mu);

    case 3
        %%%%cost taking into account torque in DSP and azmp1%%%
%         [xAviapointDSP xBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(xAviapoint,xBviapoint,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,xAa2zmp1,xBa2zmp1,epsilon,0.5*mu);
%         [yAviapointDSP yBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(yAviapoint,yBviapoint,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,yAa2zmp1,yBa2zmp1,epsilon,0.5*mu);
%         [xAviapointDSP xBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(xAviapoint,xBviapoint,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,xAa2zmp2,xBa2zmp2,epsilon,0.5*mu);
%         [yAviapointDSP yBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(yAviapoint,yBviapoint,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,yAa2zmp2,yBa2zmp2,epsilon,0.5*mu);
        [xAviapointDSP xBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(xAviapoint,xBviapoint,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,xAa2zmp1+xAa2zmp2,xBa2zmp1+xBa2zmp2,walking_param.epsilon,0.5*walking_param.mu);
        [yAviapointDSP yBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(yAviapoint,yBviapoint,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,yAa2zmp1+yAa2zmp2,yBa2zmp1+yBa2zmp2,walking_param.epsilon,0.5*walking_param.mu);
%         [xAviapointDSP xBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(xAviapoint,xBviapoint,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,xAs2zmp1zmp+xAs2zmp2zmp,xBs2zmp1zmp+xBs2zmp2zmp,epsilon,0.5*mu);
%         [yAviapointDSP yBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(yAviapoint,yBviapoint,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,yAs2zmp1zmp+yAs2zmp2zmp,yBs2zmp1zmp+yBs2zmp2zmp,epsilon,0.5*mu);
%         [xAviapointDSP xBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(xAviapoint,xBviapoint,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,xAs2zmp1zmp+xAs2zmp,xBs2zmp1zmp+xBs2zmp,epsilon,0.5*mu);
%         [yAviapointDSP yBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(yAviapoint,yBviapoint,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,yAs2zmp1zmp+yAs2zmp,yBs2zmp1zmp+yBs2zmp,epsilon,0.5*mu);
%         [xAviapointDSP xBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(xAviapoint,xBviapoint,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,xAs2zmp1zmp+xAs2zmp2zmp+xAs2zmp,xBs2zmp1zmp+xBs2zmp2zmp+xBs2zmp,epsilon,0.5*mu);
%         [yAviapointDSP yBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(yAviapoint,yBviapoint,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,yAs2zmp1zmp+yAs2zmp2zmp+yAs2zmp,yBs2zmp1zmp+yBs2zmp2zmp+yBs2zmp,epsilon,0.5*mu);

        
    case 4
        %%%%cost taking into account torque in DSP%%%
        [xAviapointDSP xBviapointDSP] = cost_viapoint_gradient_withDSP(xAviapoint,xBviapoint,xAtorque2_1_anal,xBtorque2_1_anal,xAtorque2_2_anal,xBtorque2_2_anal,0.5*walking_param.mu);
        [yAviapointDSP yBviapointDSP] = cost_viapoint_gradient_withDSP(yAviapoint,yBviapoint,yAtorque2_1_anal,yBtorque2_1_anal,yAtorque2_2_anal,yBtorque2_2_anal,0.5*walking_param.mu);


end

%%%viapointcost with DSP%%%
H=[xAviapointDSP zeros(size(yAviapointDSP));zeros(size(xAviapointDSP)) yAviapointDSP];
f=[xBviapointDSP yBviapointDSP]';
      
switch(walking_param.optim_type)
    case {1,2,3}
        AscomeqDSP_path=AscomeqDSP;
%         AscomeqDSP_path=[];
        Bscomeq_path=Bscomeq;
%         Bscomeq_path=[];
    case 4
        AscomeqDSP_path=AscomeqDSP_anal;
        Bscomeq_path=Bscomeq;        
end
%%%%%%%%%%

%%%%Contrainte sur des ankle positions%%%
switch(walking_param.optim_type)
    case {1,2,3}
        [Aeq beq]=pankle_fixed_path(AscomeqDSP_path,Bscomeq_path,walking_param.nbparamABCD,walking_param.nbparamank,walking_param.nbparamBb,walking_param.step_number_pankle_fixed);
    case 4
        [Aeq beq]=pankle_fixed_path(AscomeqDSP_path,Bscomeq_path,walking_param.nbparamtotal-walking_param.nbparamBb,walking_param.nbparamABCD,walking_param.nbparamank,0,walking_param.step_number_pankle_fixed);
end
%%%%%%%%%%

%% %%%contrainte position et vitesse initial et final du COM%%%
xpA=[zeros(4,walking_param.nbparamABCD) [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1] zeros(4,walking_param.nbparamank) zeros(4,walking_param.nbparamBb) zeros(4,walking_param.nbparamABCD+4+walking_param.nbparamank+walking_param.nbparamBb)];
Aeq=[Aeq;xpA];
beq=[beq;-walking_param.xpcominit;-walking_param.xpcomfin;-walking_param.xscominit;-walking_param.xscomfin];
ypA=[zeros(4,walking_param.nbparamABCD+4+walking_param.nbparamank+walking_param.nbparamBb) zeros(4,walking_param.nbparamABCD) [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1] zeros(4,walking_param.nbparamank) zeros(4,walking_param.nbparamBb)];
Aeq=[Aeq;ypA];
beq=[beq;-walking_param.ypcominit;-walking_param.ypcomfin;-walking_param.yscominit;-walking_param.yscomfin];
%%%%%%%%%%

end