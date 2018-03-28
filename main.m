clear all
clear walking_param
clc

addpath ./cost_viapoint
addpath ./generator_zmp
addpath ./generator_com
addpath ./f_com
addpath ./torque_ankle
addpath ./divers
addpath ./Simulation_3D
addpath ./Simulation_3D/input
addpath ./Simulation_3D/results
addpath ./Simulation_3D/FEM
addpath ./generalized_functions

%% create walking parameters
robot=2;
type_traj=1;
firstSS=0;
frequency=200;
lambda=0.6;
epsilon=15;
e=0.021;
walking_param=new_walking_param(robot,type_traj,firstSS,frequency,lambda,epsilon,e);
%% %fix ankle positions
% load('step_number_pankle_fixed.mat');
% walking_param.choose_fixed_ankle_position(step_number_pankle_fixed)
%% generate Qp problem
[H f A b Aeq beq]=generating_problem(walking_param);

%% %optimization quadratic
opt = optimset('Algorithm', 'interior-point-convex','Display','iter');%,'MaxIter',1000,'TolFun',1e-6);
switch(walking_param.optim_type)
    case {1,2,3}
        [walking_param.psa_abcdDSP toto]=quadprog(sparse(H),sparse(f),sparse(A),sparse(b),sparse(Aeq),sparse(-beq),[],[],[],opt);
%     case 4
%         psa_abcdDSP=quadprog(AviapointDSP,BviapointDSP,AconstraintDSP_anal,BconstraintDSP_anal,AscomeqDSP_path,-Bscomeq_path,[],[],[],opt);
end
%% %optimmization non-linear
% %% generate Qp problem
% func= @(pbc_) compute_cost(pbc_,H,f);
% % save('generalized_functions/H','H');
% % hessianfunc(0,0)
% % opt = optimset('Display','iter','Algorithm','Interior-Point','MaxFunEvals',3000000000000000000000000000000000,'GradObj','on','Hessian','user-supplied','HessFcn',@hessianfunc);%,'TolX',1e-8,'TolFun',1e-8,'TolCon',1e-8);
% % opt = optimset('Display','iter','Algorithm','sqp','MaxFunEvals',3000000000000000000000000000000000);%,'GradObj','on','Hessian','user-supplied','HessFcn',@hessianfunc);%,'TolX',1e-8,'TolFun',1e-8,'TolCon',1e-8);
% opt = optimset('Display','iter','Algorithm','sqp','MaxFunEvals',3000000000000000000000000000000000,'GradObj','on');
% 
% 
% pbc0= walking_param.psa_abcdDSP;
% pbc0(11)=0;
% pbc0(12)=0;
% % pbc0(2:3:walking_param.nbparamABCD)=0;
% % pbc0(3:3:walking_param.nbparamABCD)=0;
% % pbc0(walking_param.nbparamtotal+4+2:3:walking_param.nbparamtotal+4+walking_param.nbparamABCD)=0;
% % pbc0(walking_param.nbparamtotal+4+3:3:walking_param.nbparamtotal+4+walking_param.nbparamABCD)=0;
% pbc=fmincon(func,pbc0,A,b,Aeq,-beq,[],[],[],opt);
% %%
% pbc(11)=0;
% pbc(12)=0;
% walking_param.psa_abcdDSP=pbc;
%% %draw trajectories
%save ZMP, COM, ZMP1, ZMP2, foot step position in walking_param
drawing(walking_param,1);
%% %extract fixed ankle position from a first optimization%
% step_number_pankle_fixed=[1:walking_param.nbparamank walking_param.pstep(3:end-2,:)];
% save('step_number_pankle_fixed.mat','step_number_pankle_fixed');
%% %foot rolling generating
flexible_sole(walking_param,1:5,120001,0.3,'generalized_functions/result_FEM_06_01_1step_080000.mat');

%% %generate support foot ankle positions
ankleDSP(walking_param,'generalized_functions/result_FEM_06_01_1step_080000.mat','generalized_functions/result_ankleDSP_06_01_1step_080000.mat');

%% %generate in air foot ankle positions
drawing3D(walking_param,'generalized_functions/result_ankleDSP_06_01_1step_080000.mat');

%% %compute scom, acom, sankle,aankle
derivative(walking_param);

%% write results to export to control robot
writing_in_txt(walking_param,'pstep_06_10_1step.txt','zmp_com_06_10_1step.txt')