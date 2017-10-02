classdef new_walking_param<handle
    
    properties
        robot%preset robot chosen
        type_traj%preset trajectory chosen
        firstSS%preset firstSS chosen
        backtoankle %distance from back to ankle of foot
        fronttoankle %distance from  front to ankle of foot
        exttoankle %distance from exterior to ankle of foot
        inttoankle %distance from interior to ankle of foot
        sole_margin %sole margin to keep stability
        xankmax%stepping forward max
        xankmin%stepping forward min (if negative, it means stepping backward max)
        yankmin%width min between ankles
        yankmax%width max between ankles
        nbstep %number of foot step
        pankinit_firstSS %initial position of the first SS foot
        pankinit_firstinair %initial position of non first SS foot
        pankfin_lastSS %position of the last SS foot
        pankfin_lastinair %position of the non last SS foot
        xpsa_zmpinit %initial position, speed and acceleration of ZMP on x axis
        xpsa_zmpfin %final position, speed and acceleration of ZMP on x axis
        ypsa_zmpinit %initial position, speed and acceleration of ZMP on y axis
        ypsa_zmpfin %final position, speed and acceleration of ZMP on y axis
        xpcominit %initial position of COM on x axis
        xpcomfin %final position of COM on x axis
        ypcominit %initial position of COM on y axis
        ypcomfin %final position of COM on y axis
        xscominit %initial speed of COM on x axis
        xscomfin %final speed of COM on x axis
        yscominit %initial speed of COM on y axis
        yscomfin %final speed of COM on y axis
        xpsa_zmp1init %initial position, speed and acceleration of ZMP1 on x axis
        xpsa_zmp1fin %final position, speed and acceleration of ZMP1 on x axis
        ypsa_zmp1init %initial position, speed and acceleration of ZMP1 on y axis
        ypsa_zmp1fin %final position, speed and acceleration of ZMP1 on y axis
        z%COM altitude in m
        g%gravity constant en m/s²
        Tc%time constant
        w%inverse time constant
        m%robot mass
        mg%robot weight
        ha%ankle height in m
        he%max ankle height in air in m
        frequency%discretization frequency
        tss%time duration of SSP in ms
        tss_half%half duration of SSP in ms
        tds%time duration of DSP in ms
        tpi%time duration of starting phase in ms
        tpf%time duration of stopping phase in ms
        tpassage%time border of phases
        nbphases%number of phases
        discretization%number of discrete point of each phases. !Warning!, during phase one, time=0 is not counted
        type_phase %type of each phase, 0 means DSP, 1 means SSP after landing, 2 means SSP before take off
        nbpointdiscret %number of discrete points
        dt_type_phase%discretization of type_phase
        nbparamABCD%number of optimization parameter linked to points ABCD
        nbparamBb%number of optimization parameters linked to point B' of ZMP1 aka number of double support phases +2 (because ZMP1 is cut during starting and stopping phases), +6 because initial and final DSP are cut in two
        nbparamank%number of ankle position
        nbparamtotal %number of optimization parameters in one direction
        nbpankle %number of foot positions on the floor
        step_number_pankle_fixed%fixed ankle positions
        psi%fixed ankle angles
        zmp_type%choose the
        lambda %weight of fcom
        mu %weight of ankle torques
        epsilon %weight of ZMP1&2 acceleration
        optim_type
        psa_abcdDSP%all optimized parameters
        psa_abcd%ABCD boundary conditions optimized parameters
        pstep%step positions
        pabcd%via-point positions
        xpzmp%zmp positions on x axis
        ypzmp%zmp positions on y axis
        xpzmp1%zmp1 positions on x axis
        ypzmp1%zmp1 positions on y axis
        xpzmp2%zmp2 positions on x axis
        ypzmp2%zmp2 positions on y axis
        xpcom%com position on x axis
        ypcom%com position on y axis
        xscom%com speed on x axis
        yscom%com speed on y axis
        xacom%com acceleration on x axis
        yacom%com acceleration on y axis
        e%real sole width without constraint
        e_%sole width needed to have ankle at the same height
        xpankle_l%left ankle positions on x axis
        ypankle_l%left ankle positions on y axis
        zpankle_l%left ankle positions on z axis
        xpankle_r%right ankle positions on x axis
        ypankle_r%right ankle positions on y axis
        zpankle_r%right ankle positions on z axis
        xsankle_l%left ankle speed on x axis
        ysankle_l%left ankle speed on y axis
        zsankle_l%left ankle speed on z axis
        xsankle_r%right ankle speed on x axis
        ysankle_r%right ankle speed on y axis
        zsankle_r%right ankle speed on z axis
        xaankle_l%left ankle acceleration on x axis
        yaankle_l%left ankle acceleration on y axis
        zaankle_l%left ankle acceleration on z axis
        xaankle_r%right ankle acceleration on x axis
        yaankle_r%right ankle acceleration on y axis
        zaankle_r%right ankle acceleration on z axis
        rpsi_l%left ankle orientation around z axis
        rphi_l%left ankle orientation around y axis
        rtheta_l%left ankle orientation around x axis
        rpsi_r%right ankle orientation around z axis
        rphi_r%right ankle orientation around y axis
        rtheta_r%right ankle orientation around x axis
    end
    methods
        function obj=new_walking_param(robot,type_traj,firstSS,frequency,lambda,epsilon,e)
            obj.g=9.81;%gravity constant en m/s²
            %% %%%discretization frequency%%%
            obj.frequency=frequency;
            %% %choose robot
            obj.choose_robot(robot,type_traj,firstSS);
            
            %% %zmp type wanted
            %only 8 is currently enable
            obj.zmp_type=8;
            %0 : solution ICRA2015 + min acc zmp1
            %1 : zmp1 analytical
            %2 : zmp1 with force repartition which allow discontinuity
            %3 : zmp1 with force repartition which allow discontinuity and constraint zmp1&2 speed >0
            %4 : solution ICRA2015 and constraint zmp1&2 speed >0
            %5 : solution ICRA2015 + min acc zmp1 + min acc zmp2 + zmp1&2 speed >0
            %6 : solution ICRA2015 + szmp1&2 near szmp
            %7 : solution ICRA2015 + pzmp1&2 near pzmp
            %8 : solution ICRA2015 + min acc zmp1 + min acc zmp2
            
            %% %%%%weight of optimization criterion%%%
            obj.lambda=lambda; %weight of fcom
            obj.mu=1-obj.lambda; %weight of ankle torques
            obj.epsilon=epsilon; %weight of ZMP1&2 acceleration

            %% %choose optimization type
            obj.optim_type=3;
            %1 : optim cost with only torque in ankle during DSP 
            %2 : optim cost with only acceleration of ZMP1
            %3 : optim cost with both torque in ankle during DSP and acceleration of ZMP1-2 
            %4 : optim with zmp1 analytical and cost with only torque in ankle during DSP 
            %%%%%%%

            %% %choose sole width
            
            switch robot
                case 0
                    'use obj.choose_sole_width(e,e_) to define the robot sole widths'
                case 1
                    'sole width of HRP-2 not set'
                    'use obj.choose_sole_width(e,e_) to define HRP-2 sole widths'
                case 2
                    e_=0.11;
                    obj.choose_sole_width(e,e_);
            end
        end
        function obj=choose_robot(obj,robot,type_traj,firstSS)
            %% %%%choose the robot%%%
            if isnan(robot) || isempty(robot) || ~(any(robot==[0 1 2]))
                msg='choose a robot: \n';
                msg0='0: personalised robot\n';
                msg1='1: hrp2 \n';
                msg2='2 : hrp4';
                errormsg=[msg msg0 msg1 msg2];
                error(errormsg,[])
            end
            
            obj.robot=robot;
            
            obj.choose_trajectory(type_traj,firstSS);
        end
        function obj=choose_trajectory(obj,type_traj,firstSS)
            %% %%%choose the trajectory%%%
            if isnan(type_traj) || isempty(type_traj) || ~(any(type_traj==[0 1 2 3 4 5 6]))
                msg='choose a type_traj :\n';
                msg0='0: personalised trajectory\n';
                msg1='1 : forward walking 1m, 10step\n';
                msg2='2 : forward+diagonal+stretch, 30step\n';
                msg3='3 : forward walking 1.5m, 9step\n';
                msg4='4 : forward walking 3m, 10 steps, human like\n';
                msg5='5 : arc circle walking\n';
                msg6='6 : 1 step walking';
                errormsg=[msg msg0 msg1 msg2 msg3 msg4 msg5 msg6];
                error(errormsg,[])
            end
            obj.type_traj=type_traj;
            
            obj.choose_firtSS(firstSS);
            
        end
        function obj=choose_firtSS(obj,firstSS)
                        %% %first SS foot%%%
            if isnan(firstSS) || isempty(firstSS) || ~(any(firstSS==[0 1]))
                msg0='choose a firstSS foot:\n';
                msg1='0 : left\n';
                msg2='1 : right';
                errormsg=[msg0 msg1 msg2];
                error(errormsg,[])
            end
            
             obj.firstSS=firstSS;
            %% %%%foot dimension%%%
            switch(obj.robot)
                case 0
                    'use obj.choose_foot_dim(backtoankle,fronttoankle,exttoankle,inttoankle) to define foot dimension'
                case 1
                    backtoankle=0.1; %from back to ankle of foot
                    fronttoankle=0.13; %from  front to ankle of foot
                    exttoankle=0.075; %from exterior to ankle of foot
                    inttoankle=0.055; %from interior to ankle of foot
                case 2
                    backtoankle=0.098; %from back to ankle of foot
                    fronttoankle=0.128; %from  front to ankle of foot
                    exttoankle=0.076; %from exterior to ankle of foot
                    inttoankle=0.054; %from interior to ankle of foot      
            end
            
            if obj.robot~=0
                obj.choose_foot_dim(backtoankle,fronttoankle,exttoankle,inttoankle);
            end
            %%%%%%%
        end
        function obj=choose_foot_dim(obj,backtoankle,fronttoankle,exttoankle,inttoankle)
            obj.backtoankle=backtoankle; %from back to ankle of foot
            obj.fronttoankle=fronttoankle; %from  front to ankle of foot
            obj.exttoankle=exttoankle; %from exterior to ankle of foot
            obj.inttoankle=inttoankle; %from interior to ankle of foot
            
%             b=obj.backtoankle+obj.fronttoankle; %length of foot
%             a=obj.exttoankle+obj.inttoankle; %width of foot
%             obj.sole_margin=1/20*(5*a+5*b-sqrt(5)*sqrt(5*a^2+8*a*b+5*b^2))+0.01;%marge a enlever pour avoir marge de stabilité à 10% +0.01m pour améliorer la stabilité
            obj.sole_margin=0.02;
            %% %%%foot step positions constraint%%%
            switch(obj.robot)
                case 0
                    'use choose_foot_pos_constraint(obj,xankmax,xankmin,yankmin,yankmax) to define constraint on foot positions'
                case 1
                    xankmax=0.4;%stepping forward max
                    xankmin=-0.4;%stepping forward min (if negative, it means stepping backward max)
                    yankmin=obj.inttoankle+0.03;%width min between ankles
                    yankmax=obj.inttoankle+0.4;%width max between ankles
                case 2
                    xankmax=0.4;%stepping forward max
                    xankmin=-0.4;%stepping forward min (if negative, it means stepping backward max)
                    yankmin=obj.inttoankle+0.04;%width min between ankles
                    yankmax=obj.inttoankle+0.4;%width max between ankles
            end
            
            if obj.robot~=0
                obj.choose_foot_pos_constraint(xankmax,xankmin,yankmin,yankmax);
            end
        end
        function obj=choose_foot_pos_constraint(obj,xankmax,xankmin,yankmin,yankmax)
             %% %%%foot step positions constraint%%%
            obj.xankmax=xankmax;%stepping forward max
            obj.xankmin=xankmin;%stepping forward min (if negative, it means stepping backward max)
            obj.yankmin=yankmin;%width min between ankles
            obj.yankmax=yankmax;%width max between ankles
            %% %%%define the number of foot step%%%
            switch(obj.type_traj)
                case 0
                    'use choose_number_step(obj,nbstep) to define the number of foot step'
                case 1
                    nbstep=10;
                case 2
                    nbstep=30; 
                case 3
                    nbstep=9;
                case 4
                    nbstep=10;
                    yankmin=0.00;
                case 5
                    nbstep=10;
                    obj.xankmax=1;%stepping forward max
                    obj.xankmin=-1;%stepping forward min (if negative, it means stepping backward max)
                    obj.yankmin=obj.inttoankle+0.03;%width min between ankles
                    obj.yankmax=obj.inttoankle+1;%width max between ankles
                case 6
                    nbstep=2;
            end
            if obj.type_traj~=0
                obj.choose_number_step(nbstep);
            end
        end
        function obj=choose_number_step(obj,nbstep)
            %% %%%define the number of foot step%%%
            obj.nbstep=nbstep;
            %% %%%define initial - final positions of right and left foot%%%
            switch(obj.robot)
                case 0
                    'use choose_init_fin_foot_step_position(pankinit_firstSS,pankinit_firstinair,pankfin_lastSS,pankfin_lastinair) to define initial and final foot positions'
                case 1
                    pankinit_firstSS=[0.0095 (-1)^(obj.firstSS)*0.095];%initial position of left foot. 
                    pankinit_firstinair=[0.0095 (-1)^(obj.firstSS+1)*-0.095];%right foot is supposed to be symetrical by x axis to the left foot with initial pose
                    switch(obj.type_traj)%final position of left foot. right foot is supposed to be symetrical by x axis to the left foot.
                        case 0
                            'use choose_init_fin_foot_step_position(pankinit_firstSS,pankinit_firstinair,pankfin_lastSS,pankfin_lastinair) to define initial and final foot positions'
                        case 1
                            pankfin_lastSS=[1.0095 (-1)^(obj.firstSS)*0.095];
                            pankfin_lastinair=[1.0095 (-1)^(obj.firstSS+1)*0.095];
                        case 2
                            pankfin_lastSS=[1 (-1)^(obj.firstSS+1)*0.1];
                            pankfin_lastinair=[1 (-1)^(obj.firstSS+1)*(0.1+0.095*2)];
                        case 3
                            pankfin_lastSS=[1.5 (-1)^(obj.firstSS+1)*0.5+(0.095*2+0.01)*(obj.firstSS==1)];
                            pankfin_lastinair=[1.53 (-1)^(obj.firstSS+1)*0.5+(0.095*2+0.01)*(obj.firstSS==0)];
                        case 4
                            pankfin_lastSS=[3 0.095-0.095*2*(obj.firstSS==1)];
                            pankfin_lastinair=[3 0.095-0.095*2*(obj.firstSS==0)];
                        case 5
                            pankfin_lastSS=[1+0.095+0.0095 (-1)^(obj.firstSS+1)*1];
                            pankfin_lastinair=[1-0.095+0.0095 (-1)^(obj.firstSS+1)*1];
                        case 6
                            pankfin_lastSS=[(-1)^(obj.firstSS)*0.3 (-1)^(obj.firstSS)*0.095];
                            pankfin_lastinair=[(-1)^(obj.firstSS)*0.3 (-1)^(obj.firstSS+1)*0.095];
                    end
                case 2
                pankinit_firstSS=[0.0095 (-1)^(obj.firstSS)*0.0815817];%initial position of left foot. 
                pankinit_firstinair=[0.0095 (-1)^(obj.firstSS+1)*0.0815817];%right foot is supposed to be symetrical by x axis to the left foot with initial pose
                    switch(obj.type_traj)%final position of left foot. right foot is supposed to be symetrical by x axis to the left foot.
                        case 0
                            'use choose_init_fin_foot_step_position(pankinit_firstSS,pankinit_firstinair,pankfin_lastSS,pankfin_lastinair) to define initial and final foot positions'
                        case 1
                            pankfin_lastSS=[1.0095 (-1)^(obj.firstSS)*0.0815817];
                            pankfin_lastinair=[1.0095 (-1)^(obj.firstSS+1)*0.0815817];;
                        case 2
                            pankfin_lastSS=[1 (-1)^(obj.firstSS+1)*0.1];
                            pankfin_lastinair=[1 (-1)^(obj.firstSS+1)*(0.1+0.0815817*2)];
                        case 3
                            pankfin_lastSS=[1.5 (-1)^(obj.firstSS+1)*0.5+(0.0815817*2+0.01)*(obj.firstSS==1)];
                            pankfin_lastinair=[1.53 (-1)^(obj.firstSS+1)*0.5+(0.0815817*2+0.01)*(obj.firstSS==0)];
                        case 4
                            pankfin_lastSS=[3 0.0815817-0.0815817*2*(obj.firstSS==1)];
                            pankfin_lastinair=[3 0.0815817-0.0815817*2*(obj.firstSS==0)];
                        case 5
                            pankfin_lastSS=[1+0.0815817+0.0095 (-1)^(obj.firstSS+1)*1];
                            pankfin_lastinair=[1-0.0815817+0.0095 (-1)^(obj.firstSS+1)*1];
                        case 6
                            pankfin_lastSS=[(-1)^(obj.firstSS)*0.3 (-1)^(obj.firstSS)*0.0815817];
                            pankfin_lastinair=[(-1)^(obj.firstSS)*0.3 (-1)^(obj.firstSS+1)*0.0815817];
                    end
            end
            
            if obj.robot~=0 && obj.type_traj~=0
                obj.choose_init_step_fin_foot_position(pankinit_firstSS,pankinit_firstinair,pankfin_lastSS,pankfin_lastinair);
            end
            %%%%%%%
        end
        function obj=choose_init_step_fin_foot_position(obj,pankinit_firstSS,pankinit_firstinair,pankfin_lastSS,pankfin_lastinair)
            obj.pankinit_firstSS=pankinit_firstSS;
            obj.pankinit_firstinair=pankinit_firstinair;
            obj.pankfin_lastSS=pankfin_lastSS;
            obj.pankfin_lastinair=pankfin_lastinair;
            
            %% %%%initial and final zmp%%%
            switch(obj.type_traj)
                case 0
                'use choose_zmp_viapoint_init_fin(obj,xpsa_zmpinit,ypsa_zmpinit,xpsa_zmpfin,ypsa_zmpfin) to define initial and final zmp viapoint'
                case {1,2,3,4,5,6}
                    xpsa_zmpinit=[(obj.pankinit_firstSS(1)+obj.pankinit_firstinair(1))/2+0.0095;0;0];    xpsa_zmpfin=[(obj.pankfin_lastSS(1)+obj.pankfin_lastinair(1))/2+0.0095;0;0];
                    ypsa_zmpinit=[(obj.pankinit_firstSS(2)+obj.pankinit_firstinair(2))/2;0;0];           ypsa_zmpfin=[(obj.pankfin_lastSS(2)+obj.pankfin_lastinair(2))/2;0;0];
            end
            
            if obj.type_traj~=0
                obj.choose_zmp_viapoint_init_fin(xpsa_zmpinit,ypsa_zmpinit,xpsa_zmpfin,ypsa_zmpfin);
            end
        end
        function obj=choose_zmp_viapoint_init_fin(obj,xpsa_zmpinit,ypsa_zmpinit,xpsa_zmpfin,ypsa_zmpfin)
            %%%initial and final zmp via-point (position, speed,acceleration)
            obj.xpsa_zmpinit=xpsa_zmpinit;  obj.xpsa_zmpfin=xpsa_zmpfin;
            obj.ypsa_zmpinit=ypsa_zmpinit;  obj.ypsa_zmpfin=ypsa_zmpfin;
            
            %% %%%initial and final com%%%
            switch(obj.type_traj)
                case 0
                	'use choose_com_init_fin(obj,xpcominit,ypcominit,xpcomfin,ypcomfin,xscominit,yscominit,xscomfin,yscomfin) to define initial and final COM position and speed viapoint'
                case {1,2,3,4,5,6}
                    xpcominit=(obj.pankinit_firstSS(1)+obj.pankinit_firstinair(1))/2+0.0095;             xpcomfin=(obj.pankfin_lastSS(1)+obj.pankfin_lastinair(1))/2+0.0095;
                    ypcominit=(obj.pankinit_firstSS(2)+obj.pankinit_firstinair(2))/2;                    ypcomfin=(obj.pankfin_lastSS(2)+obj.pankfin_lastinair(2))/2;
            end
            
            if obj.type_traj~=0
                obj.choose_com_init_fin(xpcominit,ypcominit,xpcomfin,ypcomfin,0,0,0,0);
            end
        end
        function obj=choose_com_init_fin(obj,xpcominit,ypcominit,xpcomfin,ypcomfin,xscominit,yscominit,xscomfin,yscomfin)
            %%%initial and final com positions
            obj.xpcominit=xpcominit;	obj.xpcomfin=xpcomfin;
            obj.ypcominit=ypcominit;    obj.ypcomfin=ypcomfin;
            obj.xscominit=xscominit;	obj.xscomfin=xscomfin;
            obj.yscominit=yscominit;    obj.yscomfin=yscomfin;

            %% %initial and final position of zmp1
            switch(obj.type_traj)
                case 0
                    'use choose_init_fin_choose_zmp1_viapoint_init_fin(obj,xpsa_zmp1init,ypsa_zmp1init,xpsa_zmp1fin,ypsa_zmp1fin) to define initial and final zmp1 viapoint'
                case{1,2,3,4,6}
                    xpsa_zmp1init=obj.xpsa_zmpinit;
                    ypsa_zmp1init=[obj.pankinit_firstinair(2);0;0];
                    xpsa_zmp1fin=obj.xpsa_zmpfin;
                    ypsa_zmp1fin=[obj.pankfin_lastSS(2);0;0];
                case 5
                    xpsa_zmp1init=obj.xpsa_zmpinit;
                    ypsa_zmp1init=[obj.pankinit_firstinair(2);0;0];
                    xpsa_zmp1fin=[obj.pankfin_lastSS(1);0;0];
                    ypsa_zmp1fin=obj.ypsa_zmpfin;
            end
            if obj.type_traj~=0
                obj.choose_zmp1_viapoint_init_fin(xpsa_zmp1init,ypsa_zmp1init,xpsa_zmp1fin,ypsa_zmp1fin);
            end
        end
        function obj=choose_zmp1_viapoint_init_fin(obj,xpsa_zmp1init,ypsa_zmp1init,xpsa_zmp1fin,ypsa_zmp1fin)
            obj.xpsa_zmp1init=xpsa_zmp1init;
            obj.ypsa_zmp1init=ypsa_zmp1init;
            obj.xpsa_zmp1fin=xpsa_zmp1fin;
            obj.ypsa_zmp1fin=ypsa_zmp1fin;
            
            %% %%%constant time%%%
            switch(obj.robot)
                case 0
                    'use choose_robot_com_height(obj,z) to define the height of robot com'
                case 1
                    z=0.808511;%COM altitude in m
                case 2
                    z=0.781739;%COM altitude in m
            end
            
            if obj.robot~=0
                obj.choose_robot_com_height(z);
            end
        end
        function obj=choose_robot_com_height(obj,z)
            %% %robot com height
            obj.z=z;
            %% %time constant
            obj.Tc=sqrt(obj.z/obj.g);%time constant
            obj.w=1/obj.Tc;
            %% %%%robot mass%%%
            switch(obj.robot)
                case 0
                    'use choose_robot_mass(obj,m) to define the robot mass'
                case 1
                    m=80;%robot mass in kg
                case 2
                    m=43;%robot mass in kg
            end
            
            if obj.robot~=0
                obj.choose_robot_mass(m);
            end
            %%%%%%%
        end
        function obj=choose_robot_mass(obj,m)
            obj.m=m;
            obj.mg=obj.m*obj.g;%robot weight in N
            
            %% %%%robot height%%%
            switch(obj.robot)
                case 0
                    'use choose_ankle_height(obj,ha,he) to define the ankle height and max ankle height in air'
                case 1
                    ha=0.12;%ankle height in m
                    he=0.2;%max ankle height in air in m
                case 2
                    ha=0.093;%ankle height in m
                    he=ha+0.08;%max ankle height in air in m
            end
            
            if obj.robot~=0
                obj.choose_ankle_height(ha,he);
            end
            %%%%%%%
        end
        function obj=choose_ankle_height(obj,ha,he)
            obj.ha=ha;
            obj.he=he;
        obj.choose_frequency(obj.frequency);
        end
        function obj=choose_frequency(obj,frequency)
            %% %discretization frequency
            obj.frequency=frequency;
                        %% %%%time on passage points%%%
            switch(obj.robot)
                case 0
                    'use choose_phase_duration(obj,tss,tds,tpi,tpf) to define phase duration or choose_time_viapoint(obj,tpassage) to define time of viapoints'
                case 1
                    tss=.800; %time duration of single support phases in ms
                    tss_half=.400;
                    tds=.200; %time duration of double support phases in ms
                    tpi=.805; %time duration of starting phase in ms
                    tpf=.805; %time duration of stopping phase in ms
                case 2
                    tss=.700; %time duration of single support phases in ms
                    tss_half=.350;
                    tds=.200; %time duration of double support phases in ms
                    tpi=.600; %time duration of starting phase in ms
                    tpf=.600; %time duration of stopping phase in ms
            end
            
            if obj.robot~=0
                obj.choose_phase_duration(tss,tds,tpi,tpf);
            end
            %%%%%%%
        end
        function obj=choose_phase_duration(obj,tss,tds,tpi,tpf)
            obj.tss=tss; %time duration of single support phases in ms
            obj.tss_half=tss/2;
            obj.tds=tds; %time duration of double support phases in ms
            obj.tpi=tpi; %time duration of starting phase in ms
            obj.tpf=tpf; %time duration of stopping phase in ms
            
            obj.compute_time_viapoint();
        end
        function obj=compute_time_viapoint(obj)
            %% %%%computation of phase time duration without starting and stopping%%%
            tpassage=[0];
            for i=0:obj.nbstep-1
                if(i~=0)
                    tpassage((i)*3+2)= tpassage((i)*3+2-1)+obj.tss_half;

                else
                    tpassage((i)*3+2)= tpassage((i)*3+2-1)+obj.tss;
                end

                tpassage((i)*3+3)= tpassage((i)*3+3-1)+obj.tds;

                if(i~=obj.nbstep-1) %29
                    tpassage((i)*3+4)= tpassage((i)*3+4-1)+obj.tss_half;
                else
                    tpassage((i)*3+4)= tpassage((i)*3+4-1)+obj.tss;
                end
            end
            %add starting and stopping duration%%%
            tpassage=[0 tpassage+obj.tpi tpassage(length(tpassage))+obj.tpi+obj.tpf];
            %%%%%%%
            
            obj.choose_time_viapoint(tpassage);
        end
        function obj=choose_time_viapoint(obj,tpassage)
            obj.tpassage=tpassage;
            %% %%%number of phases in the whole walk%%%
            obj.nbphases=length(tpassage)-1;
            %%%%%%%
            obj.compute_dicretization_type_phase();
        end
        function obj=compute_dicretization_type_phase(obj)
            %% %%%computation of the number of points during each phase and the type of phase :%%%
            %0 means DSP
            %1 means SSP after landing
            %2 means SSP before take off
            obj.discretization=zeros(1,obj.nbphases);
            obj.type_phase=zeros(1,obj.nbphases);
            for i=1:obj.nbphases
                obj.discretization(i)=round(obj.frequency*(obj.tpassage(i+1)-obj.tpassage(i)));
                obj.type_phase(i)=mod(i,3);
            end
            obj.type_phase(1)=0;
            obj.type_phase(end)=0;
            %%%%%%%
            
            %% %%%number of discretization points%%%
            obj.nbpointdiscret=sum(obj.discretization)+1;
            %%%%%%%

            %% %%%discretize the type of phase%%%
            obj.dt_type_phase=compute_dt_type_phase(obj.type_phase,obj.discretization,obj.nbphases,obj.nbpointdiscret);
            %%%%%%%

            %% %%%constantes%%%
            %number of optimization parameter linked to points ABCD
            obj.nbparamABCD=(obj.nbstep*3+2+1)*3-6;
            %number of optimization parameters linked to point B' of ZMP1 aka number of double support phases +2 (because ZMP1 is cut during starting and stopping phases)
            obj.nbparamBb=(obj.nbstep+2+1)*3;
%             obj.nbparamBb=sum(any(obj.type_phase==0,1))*6+6; %+6 because initial and final DSP are cut in two
            %number of ankle position we are looking at
            obj.nbparamank=obj.nbstep-1;
            %number of foot positions on the floor
            obj.nbpankle=obj.nbstep+3;
            %number of optimization parameters in one direction
            obj.nbparamtotal=obj.nbparamABCD+obj.nbparamBb+obj.nbparamank;
            %%%%%%%
            %% %%%fixed ankle position%%%
            switch(obj.type_traj)
                case 0
                    'use choose_fixed_ankle_position(obj,step_number_pankle_fixed) to define fixed ankle positions'
                case 1
                    step_number_pankle_fixed=[];
                case 2
                    step_number_pankle_fixed=[6 0.5 (-1)^(obj.firstSS)*0.095;17 2 (-1)^(obj.firstSS)*0.75;23 2 (-1)^(obj.firstSS)*0.2]; 
                case 3
                    step_number_pankle_fixed=[];
                case 4
                    step_number_pankle_fixed=[];
                case 5
                    step_number_pankle_fixed=[];
                case 6
                    step_number_pankle_fixed=[];
            end

            if obj.type_traj~=0
                obj.choose_fixed_ankle_position(step_number_pankle_fixed);
            end
            %%%%%%%
        end
        function obj=choose_fixed_ankle_position(obj,step_number_pankle_fixed)
            %% %fixed ankle position (out of initial and final positions)
            %choose [number_of_footstep xposition yposition]
%             if(isempty(step_number_pankle_fixed))
%                 obj.step_number_pankle_fixed=[1 obj.pankinit_firstinair;2 obj.pankinit_firstSS;obj.nbparamank-1 obj.pankfin_lastSS;obj.nbparamank obj.pankfin_lastinair];
%             else
%                 obj.step_number_pankle_fixed=[1 obj.pankinit_firstinair;2 obj.pankinit_firstSS;step_number_pankle_fixed(:,1)+2 step_number_pankle_fixed(:,2:3);obj.nbparamank-1 obj.pankfin_lastSS;obj.nbparamank obj.pankfin_lastinair];
%             end
            obj.step_number_pankle_fixed=step_number_pankle_fixed;
            %% %%%fixed ankle angular position%%%
            psi=zeros(1,length(obj.discretization)+1);

            switch(obj.type_traj)
                case 0
                    'use cchoose_fixed_ankle_orientation(obj,psi) to define fixed ankle orientation'
                case 2
                    for i=6:17
                        psi((i-1)*3+4)=(-1)^(obj.firstSS)*(pi/4-abs((i-6-6)*(pi/4/(17-6)*2)));
                        psi((i-1)*3+5)=(-1)^(obj.firstSS)*(pi/4-abs((i-6-6)*(pi/4/(17-6)*2)));
                        psi((i-1)*3+6)=(-1)^(obj.firstSS)*(pi/4-abs((i-6-6)*(pi/4/(17-6)*2)));
                    end
                    for i=23:obj.nbstep-1
                        psi((i-1)*3+4)=(-1)^(obj.firstSS)*(pi/4-abs((i-23-3)*(pi/4/(obj.nbstep-1-23)*2)));
                        psi((i-1)*3+5)=(-1)^(obj.firstSS)*(pi/4-abs((i-23-3)*(pi/4/(obj.nbstep-1-23)*2)));
                        psi((i-1)*3+6)=(-1)^(obj.firstSS)*(pi/4-abs((i-23-3)*(pi/4/(obj.nbstep-1-23)*2)));
                    end
                case 3
                    for i=4:length(psi)-3
                        if (mod(i,6)==4||mod(i,6)==5||mod(i,6)==0)
                            psi(i)=(-1)^(obj.firstSS+1)*pi/4;
                        else
                            psi(i)=(-1)^(obj.firstSS)*pi/4;
                        end
                    end
                case 4
                    for i=4:length(psi)-3
                        if (mod(i,6)==4||mod(i,6)==5||mod(i,6)==0)
                            psi(i)=(-1)^(obj.firstSS+1)*pi/12;
                        else
                            psi(i)=(-1)^(obj.firstSS)*pi/12;
                        end
                    end
                case 5
                    for i=1:obj.nbstep
                        psi((i-1)*3+4)=(-1)^(obj.firstSS+1)*i*(pi/2/obj.nbstep);
                        psi((i-1)*3+5)=(-1)^(obj.firstSS+1)*i*(pi/2/obj.nbstep);
                        psi((i-1)*3+6)=(-1)^(obj.firstSS+1)*i*(pi/2/obj.nbstep);
                    end
                    psi(end-2:end)=(-1)^(obj.firstSS+1)*pi/2;
            end
            if obj.type_traj~=0
                obj.choose_fixed_ankle_orientation(psi);
            end
        end
        function obj=choose_fixed_ankle_orientation(obj,psi)
            obj.psi=psi;
        end
        function obj=choose_sole_width(obj,e,e_)
            obj.e=e;%real sole width
            obj.e_=e_;%max sole width
        end
    end
end