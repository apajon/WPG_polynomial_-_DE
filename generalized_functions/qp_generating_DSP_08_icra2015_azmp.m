function [xApzmp1 xBpzmp1 yApzmp1 yBpzmp1 xApzmp2 xBpzmp2 yApzmp2 yBpzmp2 xAtorque2_1 xBtorque2_1 yAtorque2_1 yBtorque2_1 xAtorque2_2 xBtorque2_2 yAtorque2_2 yBtorque2_2 xAa2zmp1 xBa2zmp1 yAa2zmp1 yBa2zmp1 xAa2zmp2 xBa2zmp2 yAa2zmp2 yBa2zmp2 AconstraintDSP BconstraintDSP AscomeqDSP]=qp_generating_DSP_08_icra2015_azmp(walking_param,xAzmp_gradient,xBzmp_gradient,yAzmp_gradient,yBzmp_gradient,xApzmp,xBpzmp,yApzmp,yBpzmp,xAfcom,xBfcom,yAfcom,yBfcom,Apzmpconstraintssp,Bpzmpconstraintssp,xAscomeq,yAscomeq)
tic
        %%%gestion du DSP%%%
        %%%gestion de zmp1%%%
        %where to cut the beginning and ending of zmp1 in %
        cutting=1/2;
        %compute the matrix of time discretization for zmp1
        dt1=compute_coeff_dt1_matrix(walking_param.discretization,walking_param.frequency,cutting);
        dt1=dt1(any(walking_param.dt_type_phase==0,2),:);
        %coeff gradient computation of zmp1_poly
        [xAzmp1_coeff_gradient xBzmp1_coeff_gradient]=compute_coeff_zmp1_gradient2(walking_param.tpassage,walking_param.xpsa_zmp1init,walking_param.xpsa_zmp1fin,cutting);
        [yAzmp1_coeff_gradient yBzmp1_coeff_gradient]=compute_coeff_zmp1_gradient2(walking_param.tpassage,walking_param.ypsa_zmp1init,walking_param.ypsa_zmp1fin,cutting);
        %position of ZMP1
        [xApzmp1 xBpzmp1]=compute_pzmp1_gradient(xAzmp1_coeff_gradient,xBzmp1_coeff_gradient,dt1);
        [yApzmp1 yBpzmp1]=compute_pzmp1_gradient(yAzmp1_coeff_gradient,yBzmp1_coeff_gradient,dt1);
        %matrix of force repartition in DSP
        [Ac]=compute_repartition_matrix(walking_param.tpassage,cutting);
        [Afb] = force1_repartition(Ac,dt1);
        %% %ankle position uses for torques by zmp1
        [xApankle1 xBpankle1]=torque_ankle_positions_DSP_12(walking_param.pankinit_firstinair(1),walking_param.pankinit_firstSS(1),walking_param.pankfin_lastSS(1),walking_param.discretization);
        [yApankle1 yBpankle1]=torque_ankle_positions_DSP_12(walking_param.pankinit_firstinair(2),walking_param.pankinit_firstSS(2),walking_param.pankfin_lastSS(2),walking_param.discretization);
%         yBpankle1(1:walking_param.discretization(1)+1)=-yBpankle1(1:walking_param.discretization(1)+1);%les pieds sont parallèles au debut, La création des pankle commence par le pied gauche or zmp1 est sous le pieds droit
        xApankle1=xApankle1(any(walking_param.dt_type_phase==0,2),:);
        xBpankle1=xBpankle1(any(walking_param.dt_type_phase==0,2),:);
        yApankle1=yApankle1(any(walking_param.dt_type_phase==0,2),:);
        yBpankle1=yBpankle1(any(walking_param.dt_type_phase==0,2),:);
        %% %torques by zmp1
        [xAtorque2_1 xBtorque2_1] = torque2_ankle_DSP_gradient_12(xApzmp1,xBpzmp1,xAfcom(any(walking_param.dt_type_phase==0,2),:),xBfcom(any(walking_param.dt_type_phase==0,2),:),xApankle1,xBpankle1,walking_param.mg,walking_param.ha,Afb);
        [yAtorque2_1 yBtorque2_1] = torque2_ankle_DSP_gradient_12(yApzmp1,yBpzmp1,yAfcom(any(walking_param.dt_type_phase==0,2),:),yBfcom(any(walking_param.dt_type_phase==0,2),:),yApankle1,yBpankle1,walking_param.mg,walking_param.ha,Afb);

        %% %%%gestion de zmp2%%%
        %% %position of ZMP2
        [xApzmp2 xBpzmp2]=compute_pzmp2_gradient(xApzmp(any(walking_param.dt_type_phase==0,2),:),xBpzmp(any(walking_param.dt_type_phase==0,2),:),xApzmp1,xBpzmp1,Afb,walking_param.mg);
        [yApzmp2 yBpzmp2]=compute_pzmp2_gradient(yApzmp(any(walking_param.dt_type_phase==0,2),:),yBpzmp(any(walking_param.dt_type_phase==0,2),:),yApzmp1,yBpzmp1,Afb,walking_param.mg);
        %% %ankle position uses for torques by zmp2
        [xApankle2 xBpankle2]=torque_ankle_positions_DSP_22(walking_param.pankinit_firstSS(1),walking_param.pankfin_lastSS(1),walking_param.pankfin_lastinair(1),walking_param.discretization);
        [yApankle2 yBpankle2]=torque_ankle_positions_DSP_22(walking_param.pankinit_firstSS(2),walking_param.pankfin_lastSS(2),walking_param.pankfin_lastinair(2),walking_param.discretization);
%         yBpankle2(end-discretization(end):end)=-yBpankle2(end-discretization(end):end);
%         yBpankle2(end-walking_param.discretization(end)+1:end)=yBpankle2(end-walking_param.discretization(end)+1:end)-(-1)^(walking_param.nbstep)*0.095*2;
        xApankle2=xApankle2(any(walking_param.dt_type_phase==0,2),:);
        xBpankle2=xBpankle2(any(walking_param.dt_type_phase==0,2),:);
        yApankle2=yApankle2(any(walking_param.dt_type_phase==0,2),:);
        yBpankle2=yBpankle2(any(walking_param.dt_type_phase==0,2),:);
        %% %torques by zmp2
        [xAtorque2_2 xBtorque2_2] = torque2_ankle_DSP_gradient_22(xApzmp2,xBpzmp2,xAfcom(any(walking_param.dt_type_phase==0,2),:),xBfcom(any(walking_param.dt_type_phase==0,2),:),xApankle2,xBpankle2,walking_param.mg,walking_param.ha,Afb);
        [yAtorque2_2 yBtorque2_2] = torque2_ankle_DSP_gradient_22(yApzmp2,yBpzmp2,yAfcom(any(walking_param.dt_type_phase==0,2),:),yBfcom(any(walking_param.dt_type_phase==0,2),:),yApankle2,yBpankle2,walking_param.mg,walking_param.ha,Afb);

        %% %%%cost function acceleration zmp1 & 2%%%
        dddt1=compute_coeff_dddt1_matrix(walking_param.discretization,walking_param.frequency,cutting);
        dddt1=dddt1(any(walking_param.dt_type_phase==0,2),:);
        %% %acceleration of zmp1
        [xAazmp1 xBazmp1]=compute_azmp1_gradient(xAzmp1_coeff_gradient,xBzmp1_coeff_gradient,dddt1);
        [yAazmp1 yBazmp1]=compute_azmp1_gradient(yAzmp1_coeff_gradient,yBzmp1_coeff_gradient,dddt1);
        %% %acceleration2 of zmp1
        [xAa2zmp1 xBa2zmp1]=acc2_zmp1_gradient(xAazmp1,xBazmp1);
        [yAa2zmp1 yBa2zmp1]=acc2_zmp1_gradient(yAazmp1,yBazmp1);
       
        %% %speed of zmp1
        ddt1=compute_coeff_ddt1_matrix(walking_param.discretization,walking_param.frequency,cutting);
        ddt1=ddt1(any(walking_param.dt_type_phase==0,2),:);
        [xAszmp1 xBszmp1]=compute_szmp1_gradient(xAzmp1_coeff_gradient,xBzmp1_coeff_gradient,ddt1);
        [yAszmp1 yBszmp1]=compute_szmp1_gradient(yAzmp1_coeff_gradient,yBzmp1_coeff_gradient,ddt1);
        %% %speed of zmp
        ddt=compute_coeff_ddt_matrix(walking_param.discretization,walking_param.frequency);
        [xAszmp xBszmp] = compute_pzmp_gradient(xAzmp_gradient,xBzmp_gradient,ddt);
	    [yAszmp yBszmp] = compute_pzmp_gradient(yAzmp_gradient,yBzmp_gradient,ddt);
        %% %acceleration of zmp
        dddt=compute_coeff_dddt_matrix(walking_param.discretization,walking_param.frequency);
        [xAazmp xBazmp] = compute_pzmp_gradient(xAzmp_gradient,xBzmp_gradient,dddt);
	    [yAazmp yBazmp] = compute_pzmp_gradient(yAzmp_gradient,yBzmp_gradient,dddt);
        %% %acceleration of zmp2
        [dAfb] = force1_repartition(Ac,ddt1);
        [ddAfb] = force1_repartition(Ac,dddt1);
        [xAazmp2 xBazmp2]=compute_azmp2_gradient(xApzmp(any(walking_param.dt_type_phase==0,2),:),xBpzmp(any(walking_param.dt_type_phase==0,2),:),xApzmp1,xBpzmp1,xAszmp(any(walking_param.dt_type_phase==0,2),:),xBszmp(any(walking_param.dt_type_phase==0,2),:),xAszmp1,xBszmp1,xAazmp(any(walking_param.dt_type_phase==0,2),:),xBazmp(any(walking_param.dt_type_phase==0,2),:),xAazmp1,xBazmp1,Afb,dAfb,ddAfb);
        [yAazmp2 yBazmp2]=compute_azmp2_gradient(yApzmp(any(walking_param.dt_type_phase==0,2),:),yBpzmp(any(walking_param.dt_type_phase==0,2),:),yApzmp1,yBpzmp1,yAszmp(any(walking_param.dt_type_phase==0,2),:),yBszmp(any(walking_param.dt_type_phase==0,2),:),yAszmp1,yBszmp1,yAazmp(any(walking_param.dt_type_phase==0,2),:),yBazmp(any(walking_param.dt_type_phase==0,2),:),yAazmp1,yBazmp1,Afb,dAfb,ddAfb);
        %acceleration2 of zmp1
        [xAa2zmp2 xBa2zmp2]=acc2_zmp2_gradient(xAazmp2,xBazmp2);
        [yAa2zmp2 yBa2zmp2]=acc2_zmp2_gradient(yAazmp2,yBazmp2);
        
        %% %%%constraint stability zmp1&2
        [Aconstraint1 Bconstraint1]=zmp_constraint_stability_DSP_xy_1(xApzmp1,xBpzmp1,yApzmp1,yBpzmp1,xApankle1,xBpankle1,yApankle1,yBpankle1,walking_param.discretization,walking_param.backtoankle,walking_param.fronttoankle,walking_param.exttoankle,walking_param.inttoankle,walking_param.sole_margin,walking_param.psi,walking_param.type_phase,walking_param.nbparamABCD,walking_param.nbparamBb,walking_param.firstSS);
        [Aconstraint2 Bconstraint2]=zmp_constraint_stability_DSP_xy_2(xApzmp2,xBpzmp2,yApzmp2,yBpzmp2,xApankle2,xBpankle2,yApankle2,yBpankle2,walking_param.discretization,walking_param.backtoankle,walking_param.fronttoankle,walking_param.exttoankle,walking_param.inttoankle,walking_param.sole_margin,walking_param.psi,walking_param.type_phase,walking_param.nbparamABCD,walking_param.nbparamBb,walking_param.firstSS);
        %% %%%contrainte de non chevauchement des pieds%%%
        [Apankconst Bpankconst]=zmp_constraint_ankle_pos(walking_param.pankinit_firstSS,walking_param.pankfin_lastSS,walking_param.backtoankle,walking_param.fronttoankle,walking_param.exttoankle,walking_param.inttoankle,walking_param.xankmax,walking_param.xankmin,walking_param.yankmax,walking_param.yankmin,walking_param.psi,walking_param.nbparamABCD,walking_param.nbparamank,walking_param.nbparamBb);
        
        AconstraintDSP=[Apzmpconstraintssp;    Aconstraint1;   Aconstraint2;   Apankconst];
        BconstraintDSP=[Bpzmpconstraintssp;    Bconstraint1;   Bconstraint2;   Bpankconst];

        
        %adaptation de Aeq avec DSP
        AscomeqDSP=[xAscomeq                 zeros(size(xAscomeq,1),walking_param.nbparamank) zeros(size(xAscomeq,1),walking_param.nbparamBb) zeros(size(yAscomeq)) zeros(size(xAscomeq,1),walking_param.nbparamank) zeros(size(xAscomeq,1),walking_param.nbparamBb);
                    zeros(size(xAscomeq))    zeros(size(yAscomeq,1),walking_param.nbparamank) zeros(size(yAscomeq,1),walking_param.nbparamBb) yAscomeq              zeros(size(yAscomeq,1),walking_param.nbparamank) zeros(size(yAscomeq,1),walking_param.nbparamBb)];

'time to generate qp DSP'
toc
end