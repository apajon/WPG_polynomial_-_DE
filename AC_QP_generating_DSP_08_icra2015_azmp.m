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
        %%%gestion du DSP%%%
        %%%gestion de zmp1%%%
        %where to cut the beginning and ending of zmp1 in %
        cutting=1/2;
        %compute the matrix of time discretization for zmp1
        dt1=compute_coeff_dt1_matrix(discretization,frequency,cutting);
        dt1=dt1(any(dt_type_phase==0,2),:);
        %coeff gradient computation of zmp1_poly
        [xAzmp1_coeff_gradient xBzmp1_coeff_gradient]=compute_coeff_zmp1_gradient2(tpassage,xpsa_zmp1init,xpsa_zmp1fin,cutting);
        [yAzmp1_coeff_gradient yBzmp1_coeff_gradient]=compute_coeff_zmp1_gradient2(tpassage,ypsa_zmp1init,ypsa_zmp1fin,cutting);
        %position of ZMP1
        [xApzmp1 xBpzmp1]=compute_pzmp1_gradient(xAzmp1_coeff_gradient,xBzmp1_coeff_gradient,dt1);
        [yApzmp1 yBpzmp1]=compute_pzmp1_gradient(yAzmp1_coeff_gradient,yBzmp1_coeff_gradient,dt1);
        %matrix of force repartition in DSP
        [Ac]=compute_repartition_matrix(tpassage,cutting);
        [Afb] = force1_repartition(Ac,dt1);
        %% %ankle position uses for torques by zmp1
        [xApankle1 xBpankle1]=torque_ankle_positions_DSP_12(pankinit1(1),pankinit2(1),pankfin1(1),discretization);
        [yApankle1 yBpankle1]=torque_ankle_positions_DSP_12(pankinit1(2),pankinit2(2),pankfin1(2),discretization);
%         yBpankle1(1:discretization(1)+1)=-yBpankle1(1:discretization(1)+1);%les pieds sont parallèles au debut, La création des pankle commence par le pied gauche or zmp1 est sous le pieds droit
        xApankle1=xApankle1(any(dt_type_phase==0,2),:);
        xBpankle1=xBpankle1(any(dt_type_phase==0,2),:);
        yApankle1=yApankle1(any(dt_type_phase==0,2),:);
        yBpankle1=yBpankle1(any(dt_type_phase==0,2),:);
        %% %torques by zmp1
        [xAtorque2_1 xBtorque2_1] = torque2_ankle_DSP_gradient_12(xApzmp1,xBpzmp1,xAfcom(any(dt_type_phase==0,2),:),xBfcom(any(dt_type_phase==0,2),:),xApankle1,xBpankle1,mg,ha+(e-e_),Afb);
        [yAtorque2_1 yBtorque2_1] = torque2_ankle_DSP_gradient_12(yApzmp1,yBpzmp1,yAfcom(any(dt_type_phase==0,2),:),yBfcom(any(dt_type_phase==0,2),:),yApankle1,yBpankle1,mg,ha+(e-e_),Afb);
        [xAtorque_1 xBtorque_1] = torque_ankle_DSP_gradient_12(xApzmp1,xBpzmp1,xAfcom(any(dt_type_phase==0,2),:),xBfcom(any(dt_type_phase==0,2),:),xApankle1,xBpankle1,mg,ha+(e-e_),Afb);
        [yAtorque_1 yBtorque_1] = torque_ankle_DSP_gradient_12(yApzmp1,yBpzmp1,yAfcom(any(dt_type_phase==0,2),:),yBfcom(any(dt_type_phase==0,2),:),yApankle1,yBpankle1,mg,ha+(e-e_),Afb);
        %% %%%gestion de zmp2%%%
        %% %position of ZMP2
        [xApzmp2 xBpzmp2]=compute_pzmp2_gradient(xApzmp(any(dt_type_phase==0,2),:),xBpzmp(any(dt_type_phase==0,2),:),xApzmp1,xBpzmp1,Afb,mg);
        [yApzmp2 yBpzmp2]=compute_pzmp2_gradient(yApzmp(any(dt_type_phase==0,2),:),yBpzmp(any(dt_type_phase==0,2),:),yApzmp1,yBpzmp1,Afb,mg);
        %% %ankle position uses for torques by zmp2
        [xApankle2 xBpankle2]=torque_ankle_positions_DSP_22(pankinit2(1),pankfin1(1),pankfin2(1),discretization);
        [yApankle2 yBpankle2]=torque_ankle_positions_DSP_22(pankinit2(2),pankfin1(2),pankfin2(2),discretization);
%         yBpankle2(end-discretization(end):end)=-yBpankle2(end-discretization(end):end);
%         yBpankle2(end-discretization(end)+1:end)=yBpankle2(end-discretization(end)+1:end)-(-1)^(nbstep)*0.095*2;
        xApankle2=xApankle2(any(dt_type_phase==0,2),:);
        xBpankle2=xBpankle2(any(dt_type_phase==0,2),:);
        yApankle2=yApankle2(any(dt_type_phase==0,2),:);
        yBpankle2=yBpankle2(any(dt_type_phase==0,2),:);
        %% %torques by zmp2
        [xAtorque2_2 xBtorque2_2] = torque2_ankle_DSP_gradient_22(xApzmp2,xBpzmp2,xAfcom(any(dt_type_phase==0,2),:),xBfcom(any(dt_type_phase==0,2),:),xApankle2,xBpankle2,mg,ha+(e-e_),Afb);
        [yAtorque2_2 yBtorque2_2] = torque2_ankle_DSP_gradient_22(yApzmp2,yBpzmp2,yAfcom(any(dt_type_phase==0,2),:),yBfcom(any(dt_type_phase==0,2),:),yApankle2,yBpankle2,mg,ha+(e-e_),Afb);
        [xAtorque_2 xBtorque_2] = torque_ankle_DSP_gradient_22(xApzmp2,xBpzmp2,xAfcom(any(dt_type_phase==0,2),:),xBfcom(any(dt_type_phase==0,2),:),xApankle2,xBpankle2,mg,ha+(e-e_),Afb);
        [yAtorque_2 yBtorque_2] = torque_ankle_DSP_gradient_22(yApzmp2,yBpzmp2,yAfcom(any(dt_type_phase==0,2),:),yBfcom(any(dt_type_phase==0,2),:),yApankle2,yBpankle2,mg,ha+(e-e_),Afb);
        %% %%%cost function acceleration zmp1 & 2%%%
        dddt1=compute_coeff_dddt1_matrix(discretization,frequency,cutting);
        dddt1=dddt1(any(dt_type_phase==0,2),:);
        %% %acceleration of zmp1
        [xAazmp1 xBazmp1]=compute_azmp1_gradient(xAzmp1_coeff_gradient,xBzmp1_coeff_gradient,dddt1);
        [yAazmp1 yBazmp1]=compute_azmp1_gradient(yAzmp1_coeff_gradient,yBzmp1_coeff_gradient,dddt1);
        %% %acceleration2 of zmp1
        [xAa2zmp1 xBa2zmp1]=acc2_zmp1_gradient(xAazmp1,xBazmp1);
        [yAa2zmp1 yBa2zmp1]=acc2_zmp1_gradient(yAazmp1,yBazmp1);
       
        %% %speed of zmp1
        ddt1=compute_coeff_ddt1_matrix(discretization,frequency,cutting);
        ddt1=ddt1(any(dt_type_phase==0,2),:);
        [xAszmp1 xBszmp1]=compute_szmp1_gradient(xAzmp1_coeff_gradient,xBzmp1_coeff_gradient,ddt1);
        [yAszmp1 yBszmp1]=compute_szmp1_gradient(yAzmp1_coeff_gradient,yBzmp1_coeff_gradient,ddt1);
        %% %speed of zmp
        ddt=compute_coeff_ddt_matrix(discretization,frequency);
        [xAszmp xBszmp] = compute_pzmp_gradient(xAzmp_gradient,xBzmp_gradient,ddt);
	    [yAszmp yBszmp] = compute_pzmp_gradient(yAzmp_gradient,yBzmp_gradient,ddt);
        %% %acceleration of zmp
        dddt=compute_coeff_dddt_matrix(discretization,frequency);
        [xAazmp xBazmp] = compute_pzmp_gradient(xAzmp_gradient,xBzmp_gradient,dddt);
	    [yAazmp yBazmp] = compute_pzmp_gradient(yAzmp_gradient,yBzmp_gradient,dddt);
        %% %acceleration of zmp2
        [dAfb] = force1_repartition(Ac,ddt1);
        [ddAfb] = force1_repartition(Ac,dddt1);
        [xAazmp2 xBazmp2]=compute_azmp2_gradient(xApzmp(any(dt_type_phase==0,2),:),xBpzmp(any(dt_type_phase==0,2),:),xApzmp1,xBpzmp1,xAszmp(any(dt_type_phase==0,2),:),xBszmp(any(dt_type_phase==0,2),:),xAszmp1,xBszmp1,xAazmp(any(dt_type_phase==0,2),:),xBazmp(any(dt_type_phase==0,2),:),xAazmp1,xBazmp1,Afb,dAfb,ddAfb);
        [yAazmp2 yBazmp2]=compute_azmp2_gradient(yApzmp(any(dt_type_phase==0,2),:),yBpzmp(any(dt_type_phase==0,2),:),yApzmp1,yBpzmp1,yAszmp(any(dt_type_phase==0,2),:),yBszmp(any(dt_type_phase==0,2),:),yAszmp1,yBszmp1,yAazmp(any(dt_type_phase==0,2),:),yBazmp(any(dt_type_phase==0,2),:),yAazmp1,yBazmp1,Afb,dAfb,ddAfb);
        %acceleration2 of zmp1
        [xAa2zmp2 xBa2zmp2]=acc2_zmp2_gradient(xAazmp2,xBazmp2);
        [yAa2zmp2 yBa2zmp2]=acc2_zmp2_gradient(yAazmp2,yBazmp2);
        
        %% %COM velocity
        [xAscom xBscom]=scom_generator_morisawa_gradient(xAzmp_gradient,xBzmp_gradient,xAVW,xBVW,A_gradient,discretization,w,frequency,ddt);
        [yAscom yBscom]=scom_generator_morisawa_gradient(yAzmp_gradient,yBzmp_gradient,yAVW,yBVW,A_gradient,discretization,w,frequency,ddt);
        %%
        [xAs2com xBs2com]=acc2_zmp2_gradient(xAscom,xBscom);
        [yAs2com yBs2com]=acc2_zmp2_gradient(yAscom,yBscom);
        
        %% %%%constraint stability zmp1&2
%         [Aconstraint1 Bconstraint1]=zmp_constraint_stability_DSP_xy_1(xApzmp1,xBpzmp1,yApzmp1,yBpzmp1,pankinit1,pankinit2,pankfin1,discretization,backtoankle,fronttoankle,exttoankle,inttoankle,sole_margin,psi,type_phase,nbparamBb);
%         [Aconstraint2 Bconstraint2]=zmp_constraint_stability_DSP_xy_2(xApzmp2,xBpzmp2,yApzmp2,yBpzmp2,pankinit2,pankfin1,pankfin2,discretization,backtoankle,fronttoankle,exttoankle,inttoankle,sole_margin,psi,type_phase,nbparamBb);    
        [Aconstraint1 Bconstraint1]=zmp_constraint_stability_DSP_xy_1(xApzmp1,xBpzmp1,yApzmp1,yBpzmp1,xApankle1,xBpankle1,yApankle1,yBpankle1,discretization,backtoankle,fronttoankle,exttoankle,inttoankle,sole_margin,psi,type_phase,nbparamABCD,nbparamBb,firstSS);
        [Aconstraint2 Bconstraint2]=zmp_constraint_stability_DSP_xy_2(xApzmp2,xBpzmp2,yApzmp2,yBpzmp2,xApankle2,xBpankle2,yApankle2,yBpankle2,discretization,backtoankle,fronttoankle,exttoankle,inttoankle,sole_margin,psi,type_phase,nbparamABCD,nbparamBb,firstSS);
        %% %%%contrainte de non chevauchement des pieds%%%
        [Apankconst Bpankconst]=zmp_constraint_ankle_pos(pankinit2,pankfin1,backtoankle,fronttoankle,exttoankle,inttoankle,xankmax,xankmin,yankmax,yankmin,psi,nbparamABCD,nbparamank,nbparamBb);
        
        AconstraintDSP=[Apzmpconstraintssp;    Aconstraint1;   Aconstraint2;   Apankconst];
        BconstraintDSP=[Bpzmpconstraintssp;    Bconstraint1;   Bconstraint2;   Bpankconst];

        
        %adaptation de Aeq avec DSP
        AscomeqDSP=[xAscomeq                 zeros(size(xAscomeq,1),nbparamank) zeros(size(xAscomeq,1),nbparamBb) zeros(size(yAscomeq)) zeros(size(xAscomeq,1),nbparamank) zeros(size(xAscomeq,1),nbparamBb);
                    zeros(size(xAscomeq))    zeros(size(yAscomeq,1),nbparamank) zeros(size(yAscomeq,1),nbparamBb) yAscomeq              zeros(size(yAscomeq,1),nbparamank) zeros(size(yAscomeq,1),nbparamBb)];

'time to generate qp DSP'
toc

open('AD_QP_optimization.m');