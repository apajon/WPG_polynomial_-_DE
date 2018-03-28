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

%trjx=[]; trjy=[]; xcom=[]; ycom=[]; xvcom=[]; yvcom=[];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%weight of optimization criterion%%%
lambda=0.6; %weight of fcom
mu=1-lambda; %weight of ankle torques
epsilon=100; %weight of ZMP1&2 acceleration 250

%%%%%%%
optim_type=3;
%1 : optim cost with only torque in ankle during DSP 
%2 : optim cost with only acceleration of ZMP1
%3 : optim cost with both torque in ankle during DSP and acceleration of ZMP1-2 
%4 : optim with zmp1 analytical and cost with only torque in ankle during DSP 
%%%%%%%


%% %%%position, speed and acceleration gradient of viapoints%%%
[xAviapoint xBviapoint] = cost_viapoint_gradient2(xAfcom2,xBfcom2,xAtorque2ssp,xBtorque2ssp,lambda,mu);
[yAviapoint yBviapoint] = cost_viapoint_gradient2(yAfcom2,yBfcom2,yAtorque2ssp,yBtorque2ssp,lambda,mu);


switch(optim_type)
   case 1
        %%%%cost taking into account torque in DSP%%%
        [xAviapointDSP xBviapointDSP] = cost_viapoint_gradient_withDSP(xAviapoint,xBviapoint,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,0.5*(tds/(tds+tss))*mu);
        [yAviapointDSP yBviapointDSP] = cost_viapoint_gradient_withDSP(yAviapoint,yBviapoint,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,0.5*(tds/(tds+tss))*mu);

    case 2
        %%%%cost taking into account azmp1 only%%%
        [xAviapointDSP xBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(xAviapoint,xBviapoint,zeros(size(xAtorque2_1)),zeros(size(xBtorque2_1)),zeros(size(xAtorque2_2)),zeros(size(xBtorque2_2)),xAa2zmp1+xAa2zmp2,xBa2zmp1+xBa2zmp2,epsilon,0.5*(tds/(tds+tss))*mu);
        [yAviapointDSP yBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(yAviapoint,yBviapoint,zeros(size(yAtorque2_1)),zeros(size(yBtorque2_1)),zeros(size(yAtorque2_2)),zeros(size(yBtorque2_2)),yAa2zmp1+yAa2zmp2,yBa2zmp1+yBa2zmp2,epsilon,0.5*(tds/(tds+tss))*mu);

    case 3
        %%%%cost taking into account torque in DSP and azmp1%%%
%         [xAviapointDSP xBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(xAviapoint,xBviapoint,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,xAa2zmp1,xBa2zmp1,epsilon,0.5*mu);
%         [yAviapointDSP yBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(yAviapoint,yBviapoint,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,yAa2zmp1,yBa2zmp1,epsilon,0.5*mu);
%         [xAviapointDSP xBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(xAviapoint,xBviapoint,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,xAa2zmp2,xBa2zmp2,epsilon,0.5*mu);
%         [yAviapointDSP yBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(yAviapoint,yBviapoint,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,yAa2zmp2,yBa2zmp2,epsilon,0.5*mu);
        [xAviapointDSP xBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(xAviapoint,xBviapoint,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,xAa2zmp1+xAa2zmp2,xBa2zmp1+xBa2zmp2,epsilon,0.5*mu);
        [yAviapointDSP yBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(yAviapoint,yBviapoint,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,yAa2zmp1+yAa2zmp2,yBa2zmp1+yBa2zmp2,epsilon,0.5*mu);
%         [xAviapointDSP xBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(xAviapoint,xBviapoint,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,xAs2zmp1zmp+xAs2zmp2zmp,xBs2zmp1zmp+xBs2zmp2zmp,epsilon,0.5*mu);
%         [yAviapointDSP yBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(yAviapoint,yBviapoint,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,yAs2zmp1zmp+yAs2zmp2zmp,yBs2zmp1zmp+yBs2zmp2zmp,epsilon,0.5*mu);
%         [xAviapointDSP xBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(xAviapoint,xBviapoint,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,xAs2zmp1zmp+xAs2zmp,xBs2zmp1zmp+xBs2zmp,epsilon,0.5*mu);
%         [yAviapointDSP yBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(yAviapoint,yBviapoint,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,yAs2zmp1zmp+yAs2zmp,yBs2zmp1zmp+yBs2zmp,epsilon,0.5*mu);
%         [xAviapointDSP xBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(xAviapoint,xBviapoint,xAtorque2_1,xBtorque2_1,xAtorque2_2,xBtorque2_2,xAs2zmp1zmp+xAs2zmp2zmp+xAs2zmp,xBs2zmp1zmp+xBs2zmp2zmp+xBs2zmp,epsilon,0.5*mu);
%         [yAviapointDSP yBviapointDSP]=cost_viapoint_gradient_withDSP_azmp1(yAviapoint,yBviapoint,yAtorque2_1,yBtorque2_1,yAtorque2_2,yBtorque2_2,yAs2zmp1zmp+yAs2zmp2zmp+yAs2zmp,yBs2zmp1zmp+yBs2zmp2zmp+yBs2zmp,epsilon,0.5*mu);

        
    case 4
        %%%%cost taking into account torque in DSP%%%
        [xAviapointDSP xBviapointDSP] = cost_viapoint_gradient_withDSP(xAviapoint,xBviapoint,xAtorque2_1_anal,xBtorque2_1_anal,xAtorque2_2_anal,xBtorque2_2_anal,0.5*mu);
        [yAviapointDSP yBviapointDSP] = cost_viapoint_gradient_withDSP(yAviapoint,yBviapoint,yAtorque2_1_anal,yBtorque2_1_anal,yAtorque2_2_anal,yBtorque2_2_anal,0.5*mu);


end

%% %%%viapointcost with DSP%%%
AviapointDSP=[xAviapointDSP zeros(size(yAviapointDSP));zeros(size(xAviapointDSP)) yAviapointDSP];
BviapointDSP=[xBviapointDSP yBviapointDSP]';
      
switch(optim_type)
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
%%
% step_number_pankle_fixed=[];
% % step_number_pankle_fixed=[...0.0885   -0.0664;
% %     0.1855    0.0816;
% %     0.2906   -0.0664;
% %     0.3982    0.0816;
% %     0.5066   -0.0664;
% %     0.6150    0.0816;
% %     0.7228   -0.0664;
% %     0.8285    0.0816;
% %     ...0.9277   -0.0664
% %     ];
% % step_number_pankle_fixed=[[2:nbparamank-1]' step_number_pankle_fixed];
%% %%%Contrainte sur des ankle positions%%%
switch(optim_type)
    case {1,2,3}
        [AscomeqDSP_path Bscomeq_path]=pankle_fixed_path(AscomeqDSP_path,Bscomeq_path,nbparamABCD,nbparamank,nbparamBb,step_number_pankle_fixed);
    case 4
        [AscomeqDSP_path Bscomeq_path]=pankle_fixed_path(AscomeqDSP_path,Bscomeq_path,nbparamtotal-nbparamBb,nbparamABCD,nbparamank,0,step_number_pankle_fixed);
end
%%%%%%%%%%

%% %%%contrainte position et vitesse initial et final du COM%%%
xpA=[zeros(4,nbparamABCD) [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1] zeros(4,nbparamank) zeros(4,nbparamBb) zeros(4,nbparamABCD+4+nbparamank+nbparamBb)];
AscomeqDSP_path=[AscomeqDSP_path;xpA];
Bscomeq_path=[Bscomeq_path;-xpcominit;-xpcomfin;-xscominit;-xscomfin];
ypA=[zeros(4,nbparamABCD+4+nbparamank+nbparamBb) zeros(4,nbparamABCD) [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1] zeros(4,nbparamank) zeros(4,nbparamBb)];
AscomeqDSP_path=[AscomeqDSP_path;ypA];
Bscomeq_path=[Bscomeq_path;-ypcominit;-ypcomfin;-yscominit;-yscomfin];

%% %%%optimization with torque in DSP%%%
tic
opt = optimset('Algorithm', 'interior-point-convex','Display','iter');
switch(optim_type)
    case {1,2,3}
        psa_abcdDSP=quadprog(sparse(AviapointDSP),sparse(BviapointDSP),sparse(AconstraintDSP),sparse(BconstraintDSP),sparse(AscomeqDSP_path),sparse(-Bscomeq_path),[],[],[],opt);
    case 4
        psa_abcdDSP=quadprog(AviapointDSP,BviapointDSP,AconstraintDSP_anal,BconstraintDSP_anal,AscomeqDSP_path,-Bscomeq_path,[],[],[],opt);
end
'optimization time'
toc

open('AE_drawing.m');