clear all
clc
%%
%for windows
addpath .\cost_viapoint
addpath .\generator_zmp
addpath .\generator_com
addpath .\f_com
addpath .\torque_ankle
addpath .\divers

%for linux
% addpath ./cost_viapoint
% addpath ./generator_zmp
% addpath ./generator_com
% addpath ./f_com
% addpath ./torque_ankle
% addpath ./divers

%% %%%chosse the robot%%%
robot=2;
%1 : hrp2
%2 : hrp4
%%%%%%%%%%%

%% %%%choose the trajectory%%%
type_traj=1;
%1 : forward walking 1m, 10step
%2 : forward+diagonal+stretch, 30step
%3 : forward walking 1.5m, 9step
%4 : forward walking 3m, 10 steps, human like
%5 : arc circle walking
%6 : 1 step walking
%7 : lateral walking
%8 : arc circle little
%9 : com sinusoidal movement
%%%%%%%%%%%

%% %%%first SS foot%%%
firstSS=0;
%0 : left
%1 : right

%% %%%foot characteristics%%%
switch(robot)
    case 1
        backtoankle=0.1; %from back to ankle of foot
        fronttoankle=0.13; %from  front to ankle of foot
        exttoankle=0.075; %from exterior to ankle of foot
        inttoankle=0.055; %from interior to ankle of foot
    case 2
        toto=0.0;
        backtoankle=0.098; %from back to ankle of foot
        fronttoankle=0.128; %from  front to ankle of foot
        exttoankle=0.076; %from exterior to ankle of foot
        inttoankle=0.054-toto; %from interior to ankle of foot      
end
b=backtoankle+fronttoankle; %length of foot
a=exttoankle+inttoankle; %width of foot
% d=0.2; %width between foot edges in frontal plane
% da=(d+inttoankle)/2; %length between foot center and sagittal plane
% sole_margin=1/20*(5*a+5*b-sqrt(5)*sqrt(5*a^2+8*a*b+5*b^2))+0.01;%marge a enlever pour avoir marge de stabilité à 10% +0.01m pour améliorer la stabilité
sole_margin=0.02;
%%%%%%%%%%%

%% %%%foot step positions%%%
switch(robot)
    case 1
        xankmax=0.4;%stepping forward max
        xankmin=-0.4;%stepping forward min (if negative, it means stepping backward max)
        yankmin=inttoankle+0.03;%width min between ankles
        yankmax=inttoankle+0.4;%width max between ankles
    case 2
        xankmax=0.4;%stepping forward max
        xankmin=-0.4;%stepping forward min (if negative, it means stepping backward max)
        yankmin=inttoankle+0.1;%0.108;%width min between ankles
        yankmax=inttoankle+0.4;%width max between ankles
end
%%%%%%%%%%%

%% %%%define the number of foot step%%%
switch(type_traj)
    case 1
        nbstep=10;
%         nbstep=4;
    case 2
        nbstep=30; 
    case 3
        nbstep=9;
    case 4
        nbstep=10;
%         xankmax=0.5;
%         xankmin=fronttoankle+0.03;
        yankmin=0.00;
    case 5
        nbstep=10;
        xankmax=1;%stepping forward max
        xankmin=-1;%stepping forward min (if negative, it means stepping backward max)
        yankmin=inttoankle+0.03;%width min between ankles
        yankmax=inttoankle+1;%width max between ankles
    case 6
        nbstep=2;
    case 7
        nbstep=10;
    case 8
        nbstep=10;
%         xankmax=1;%stepping forward max
%         xankmin=-1;%stepping forward min (if negative, it means stepping backward max)
%         yankmin=inttoankle+0.03;%width min between ankles
%         yankmax=inttoankle+1;%width max between ankles
    case 9
        nbstep=4;
        yankmin=0;%inttoankle+0.1;%width min between ankles
        yankmax=inttoankle+1;%width max between ankles
%         inttoankle=0.08;
end
%%%%%%%


%% %%%define initial - final positions of right and left foot%%%
yHSFootPos=0.0815817;
% yHSFootPos=0.108;
switch (robot)
    case 1
        pankinit2=[0.0095 0.095];%initial position of left foot. 
        pankinit1=[0.0095 -0.095];%right foot is supposed to be symetrical by x axis to the left foot with initial pose
    case 2
        pankinit2=[0.0095 (-1)^(firstSS)*yHSFootPos];%initial position of left foot. 
        pankinit1=[0.0095 (-1)^(firstSS+1)*yHSFootPos];%right foot is supposed to be symetrical by x axis to the left foot with initial pose
end
switch(type_traj)%final position of left foot. right foot is supposed to be symetrical by x axis to the left foot.
    case 1
        switch (robot)
            case 1
                pankfin1=[1.0095 0.095];
                pankfin2=[1.0095 -0.095];
            case 2
%                 pankfin1=[1.0095 (-1)^(firstSS)*yHSFootPos];
%                 pankfin2=[1.0095 (-1)^(firstSS+1)*yHSFootPos];
%                 pankfin1=[0.5095 (-1)^(firstSS)*yHSFootPos];
%                 pankfin2=[0.5095 (-1)^(firstSS+1)*yHSFootPos];
                pankfin1=[1.0095 (-1)^(firstSS)*yHSFootPos];
                pankfin2=[1.0095 (-1)^(firstSS+1)*yHSFootPos];
        end
    case 2
        pankfin1=[1 (-1)^(firstSS+1)*0.1];
        pankfin2=[1 (-1)^(firstSS+1)*0.1+(-1)^(firstSS+1)*0.095*2];
    case 3
        pankfin1=[1.5 (-1)^(firstSS+1)*0.5+(0.095*2+0.01)*(firstSS==1)];
        pankfin2=[1.53 (-1)^(firstSS+1)*0.5+(0.095*2+0.01)*(firstSS==0)];
    case 4
        pankfin1=[3.0095 0.095-0.095*2*(firstSS==1)];
        pankfin2=[3.0095 0.095-0.095*2*(firstSS==0)];
    case 5
        pankfin1=[1+0.095+0.0095 (-1)^(firstSS+1)*1];
        pankfin2=[1-0.095+0.0095 (-1)^(firstSS+1)*1];
    case 6
%         pankfin1=[(-1)^(firstSS)*0.3 (-1)^(firstSS)*0.095];
%         pankfin2=[(-1)^(firstSS)*0.3 (-1)^(firstSS+1)*0.095];
        switch (robot)
            case 1
               pankfin1=[(-1)^(firstSS)*0.3 (-1)^(firstSS)*0.095];
               pankfin2=[(-1)^(firstSS)*0.3 (-1)^(firstSS+1)*0.095];
            case 2
                pankfin1=[0.3095 (-1)^(firstSS)*yHSFootPos];
                pankfin2=[0.3095 (-1)^(firstSS+1)*yHSFootPos];
        end
    case 7
        switch (robot)
            case 1
                pankfin1=[0.0095 0.095-1];
                pankfin2=[0.0095 -0.095-1];
            case 2
                pankfin1=[0.0095 (-1)^(firstSS)*yHSFootPos-0.5];
                pankfin2=[0.0095 (-1)^(firstSS+1)*yHSFootPos-0.5];
        end
    case 8
        pankfin1=[0.5+0.095+0.0095 (-1)^(firstSS+1)*0.5];
        pankfin2=[0.5-0.095+0.0095 (-1)^(firstSS+1)*0.5];
    case 9
%         pankinit2=[0.0095 (-1)^(firstSS)*0.0715817];
%         pankinit1=[0.0095 (-1)^(firstSS+1)*0.0715817];
        pankfin1=pankinit2+[0. 0];
        pankfin2=pankinit1+[0. 0];
end
%%%%%%%

%% %%%initial and final zmp and COM%%%
switch (robot)
    case 1
        xpsa_zmpinit=[(pankinit2(1)+pankinit1(1))/2+0.0095;0;0];    xpsa_zmpfin=[(pankfin1(1)+pankfin2(1))/2+0.0095;0;0];
        ypsa_zmpinit=[(pankinit2(2)+pankinit1(2))/2;0;0];           ypsa_zmpfin=[(pankfin1(2)+pankfin2(2))/2;0;0];
        xpcominit=(pankinit2(1)+pankinit1(1))/2+0.0095;             xpcomfin=(pankfin1(1)+pankfin2(1))/2+0.0095;
        ypcominit=(pankinit2(2)+pankinit1(2))/2;                    ypcomfin=(pankfin1(2)+pankfin2(2))/2;
    case 2
%         xpsa_zmpinit=[(pankinit2(1)+pankinit1(1))/2+0.0095;0;0];    xpsa_zmpfin=[(pankfin1(1)+pankfin2(1))/2+0.0095;0;0];
%         ypsa_zmpinit=[(pankinit2(2)+pankinit1(2))/2;0;0];           ypsa_zmpfin=[(pankfin1(2)+pankfin2(2))/2;0;0];
%         xpcominit=(pankinit2(1)+pankinit1(1))/2+0.0095;             xpcomfin=(pankfin1(1)+pankfin2(1))/2+0.0095;
%         ypcominit=(pankinit2(2)+pankinit1(2))/2;                    ypcomfin=(pankfin1(2)+pankfin2(2))/2;
        xpsa_zmpinit=[(pankinit2(1)+pankinit1(1))/2-0.00365306;0;0];             xpsa_zmpfin=[(pankfin1(1)+pankfin2(1))/2+-0.00365306;0;0];
        ypsa_zmpinit=[(pankinit2(2)+pankinit1(2))/2+0.00047291;0;0];             ypsa_zmpfin=[(pankfin1(2)+pankfin2(2))/2+0.00047291;0;0];
        xpcominit=(pankinit2(1)+pankinit1(1))/2-0.00365306;                      xpcomfin=(pankfin1(1)+pankfin2(1))/2-0.00365306;
        ypcominit=(pankinit2(2)+pankinit1(2))/2+0.00047291;                      ypcomfin=(pankfin1(2)+pankfin2(2))/2+0.00047291;
end
%%%%%%%

%% %%%initial and final zmp and COM%%%
xscominit=0;    xscomfin=0;
yscominit=0;    yscomfin=0;
%%%%%%%

%% %initial and final position of zmp1
switch(type_traj)
    case{1,2,3,4,6,7,9}
        xpsa_zmp1init=xpsa_zmpinit;
        ypsa_zmp1init=[pankinit1(2);0;0];
        xpsa_zmp1fin=xpsa_zmpfin;
        ypsa_zmp1fin=[pankfin1(2);0;0];
    case {5,8}
        xpsa_zmp1init=xpsa_zmpinit;
        ypsa_zmp1init=[pankinit1(2);0;0];
        xpsa_zmp1fin=[pankfin1(1);0;0];
        ypsa_zmp1fin=ypsa_zmpfin;
end
%%%%%%%

%% %%%constant time%%%
switch(robot)
    case 1
        z=0.808511;%COM altitude in m
        
        e=0.015;%real sole width
%         e=0.021;%real sole width
%         e=0.006;
        e_=0.011;%max sole width
%         z=0.781739;%COM altitude in m
%         z=0.779615+(e-e_);%COM altitude in m
    case 2
        e=0.015;%real sole width
%         e=0.021;%real sole width
%         e=0.006;
        e_=0.011;%max sole width
%         z=0.781739;%COM altitude in m
%         z=0.779615+(e-e_);%COM altitude in m
        z=0.78051+(e-e_);%COM altitude in m
        z=0.780678;

end
g=9.81;%gravity constant en m/s²
Tc=sqrt(z/g);%time constant
w=1/Tc;
%%%%%%%

%% %%%robot mass%%%
switch(robot)
    case 1
        m=80;%robot mass in kg
    case 2
        m=43;%robot mass in kg
end
mg=m*g;%robot weight in N
%%%%%%%

%% %%%robot height%%%
h=z;%COM heitght in m
switch(robot)
    case 1
        ha=0.12;%ankle height in m
        he=0.2;%max ankle height in air in m
    case 2
        ha=0.093;%ankle height in m
        he=ha+0.05;%max ankle height in air in m
%         if type_traj==9
%             he=ha;
%         end
end
%%%%%%%

%% %%%discretization frequency%%%
frequency=200; %discretization in Hz
%%%%%%%

%% %%%time on passage points%%%
switch(robot)
    case 1
        tss=.800; %time duration of single support phases in ms
        tss_half=.400;
        tds=.200; %time duration of double support phases in ms
        tpi=.805; %time duration of starting phase in ms
        tpf=.805; %time duration of stopping phase in ms
    case 2
%         tss=700; %time duration of single support phases in ms
%         tss_half=350;
%         tds=200; %time duration of double support phases in ms
%         tpi=600; %time duration of starting phase in ms
%         tpf=600; %time duration of stopping phase in ms
        tss=0.80; %time duration of single support phases in ms
        tss_half=0.40;
        tds=0.8; %time duration of double support phases in ms
        tpi=0.805; %time duration of starting phase in ms
        tpf=0.805; %time duration of stopping phase in ms
        if type_traj==9
            tss=0.020; %time duration of single support phases in ms
            tds=5.000; %time duration of double support phases in ms
            tss=1.0;
            tds=.50;
            tpi=2.5; %time duration of starting phase in ms
            tpf=2.5; %time duration of stopping phase in ms
%             tpi=0.85; %time duration of starting phase in ms
%             tpf=0.85; %time duration of stopping phase in ms

        end
end
%%%%%%%

%% %%%computation of phase time duration without starting and stopping%%%
tpassage=[0];
for i=0:nbstep-1 %29 %10
    if(i~=0)
        tpassage((i)*3+2)= tpassage((i)*3+2-1)+tss/2;
    else
        tpassage((i)*3+2)= tpassage((i)*3+2-1)+tss;
    end
    
    tpassage((i)*3+3)= tpassage((i)*3+3-1)+tds;
    
    if(i~=nbstep-1) %29
        tpassage((i)*3+4)= tpassage((i)*3+4-1)+tss/2;
    else
        tpassage((i)*3+4)= tpassage((i)*3+4-1)+tss;
    end
end

%add starting and stopping duration%%%
tpassage=[0 tpassage+tpi tpassage(length(tpassage))+tpi+tpf];

%% %%%number of phases in the whole walk%%%
nbphases=length(tpassage)-1;
%%%%%%%

%% %%%computation of the number of points during each phase and the type of
%phase :%%%
%0 means DSP
%1 means SSP after landing
%2 means SSP before take off
discretization=zeros(1,length(tpassage)-1);
type_phase=zeros(1,length(tpassage)-1);
for i=1:length(tpassage)-1
    discretization(i)=round(frequency*(tpassage(i+1)-tpassage(i)));
    type_phase(i)=mod(i,3);
end
type_phase(1)=0;
type_phase(end)=0;
%%%%%%%

%% %%%number of discretization points%%%
nbpointdiscret=sum(discretization)+1;
%%%%%%%

%% %%%discretize the type of phase%%%
dt_type_phase=compute_dt_type_phase(type_phase,discretization,nbphases,nbpointdiscret);
%%%%%%%

%% %%%constantes in each direction%%%
%number of parameter linked to points ABCD -6 because initial and final ZMP are known
nbparamABCD=(nbstep*3+2+1)*3-6;
%number parameters linked to point B' of ZMP1 aka number of double support phases
%+2 : because ZMP1 is cut during starting
%+1 : because ZMP1 is cut durings stopping and the last ZMP1 is known
nbparamBb=(nbstep+2+1)*3;
%number of ankle position we are looking at
nbparamank=nbstep-1;
%number of foot positions on the floor
nbpankle=nbstep+3;
%total number of optimisation parameters
nbparamtotal=nbparamABCD+nbparamBb+nbparamank; %number of optimization parameters in one direction
%%%%%%%

%% %%%fixed ankle position%%%
switch(type_traj)
    case 1
        step_number_pankle_fixed=[];
%         step_number_pankle_fixed=[2 0.1964    0.0816; ...
%                                    3 0.3007   -0.1264; ...
%                                    4 0.4056    0.0816; ...
%                                    5 0.5106   -0.1264; ...
%                                    6 0.6155    0.0816; ...
%                                    7 0.7204   -0.1264; ...
%                                    8 0.8246    0.0816];
%          step_number_pankle_fixed=[2 0.2927    0.0816; ...
%                                    3 0.4482   -0.1264; ...
%                                    4 0.6042    0.0816; ...
%                                    5 0.7604   -0.1264; ...
%                                    6 0.9165    0.0816; ...
%                                    7 1.0725   -0.1264; ...
%                                    8 1.2279    0.0816];
    case 2
        step_number_pankle_fixed=[6 0.5 (-1)^(firstSS)*0.095;17 2 (-1)^(firstSS)*0.75;23 2 (-1)^(firstSS)*0.2]; 
    case 3
        step_number_pankle_fixed=[];
    case 4
        step_number_pankle_fixed=[];
    case 5
        step_number_pankle_fixed=[];
    case 6
        step_number_pankle_fixed=[];
%         step_number_pankle_fixed=[1 0.125+0.1 -0.0815817;
%                                     2 0.250+0.1 0.0815817;
%                                     3 0.375+0.1 -0.0815817;];
    case 7
        step_number_pankle_fixed=[];
    case 8
        step_number_pankle_fixed=[];
    case 9
        step_number_pankle_fixed=[1 pankinit1(1)+0.    pankinit1(2); ...
                            2 pankinit1(1)+0.    -pankinit1(2); ...
                            3 pankinit1(1)+0.   pankinit1(2); ...
                           ];
%         step_number_pankle_fixed=[];
end
%%%%%%%

%% %%%fixed ankle angular position%%%
psi=zeros(1,length(discretization)+1);

switch(type_traj)
    case 2
        if firstSS==0
            for i=6:17
                psi((i-1)*3+4)=(pi/4-abs((i-6-6)*(pi/4/(17-6)*2)));
                psi((i-1)*3+5)=(pi/4-abs((i-6-6)*(pi/4/(17-6)*2)));
                psi((i-1)*3+6)=(pi/4-abs((i-6-6)*(pi/4/(17-6)*2)));
            end
            for i=23:nbstep-1
                psi((i-1)*3+4)=(pi/4-abs((i-23-3)*(pi/4/(nbstep-1-23)*2)));
                psi((i-1)*3+5)=(pi/4-abs((i-23-3)*(pi/4/(nbstep-1-23)*2)));
                psi((i-1)*3+6)=(pi/4-abs((i-23-3)*(pi/4/(nbstep-1-23)*2)));
            end
        elseif firstSS==1
            for i=6:17
                psi((i-1)*3+4)=-(pi/4-abs((i-6-6)*(pi/4/(17-6)*2)));
                psi((i-1)*3+5)=-(pi/4-abs((i-6-6)*(pi/4/(17-6)*2)));
                psi((i-1)*3+6)=-(pi/4-abs((i-6-6)*(pi/4/(17-6)*2)));
            end
            for i=23:nbstep-1
                psi((i-1)*3+4)=-(pi/4-abs((i-23-3)*(pi/4/(nbstep-1-23)*2)));
                psi((i-1)*3+5)=-(pi/4-abs((i-23-3)*(pi/4/(nbstep-1-23)*2)));
                psi((i-1)*3+6)=-(pi/4-abs((i-23-3)*(pi/4/(nbstep-1-23)*2)));
            end
        end
    case 3
        for i=4:length(psi)-3
            if (mod(i,6)==4||mod(i,6)==5||mod(i,6)==0)
                psi(i)=(-1)^(firstSS+1)*pi/4;
            else
                psi(i)=(-1)^(firstSS)*pi/4;
            end
        end
    case 4
        for i=4:length(psi)-3
            if (mod(i,6)==4||mod(i,6)==5||mod(i,6)==0)
                psi(i)=(-1)^(firstSS+1)*pi/12;
            else
                psi(i)=(-1)^(firstSS)*pi/12;
            end
        end
    case 5
        for i=1:nbstep
            psi((i-1)*3+4)=(-1)^(firstSS+1)*i*(pi/2/nbstep);
            psi((i-1)*3+5)=(-1)^(firstSS+1)*i*(pi/2/nbstep);
            psi((i-1)*3+6)=(-1)^(firstSS+1)*i*(pi/2/nbstep);
        end
        psi(end-2:end)=(-1)^(firstSS+1)*pi/2;
    case 8
        for i=1:nbstep
            psi((i-1)*3+4)=(-1)^(firstSS+1)*i*(pi/2/nbstep);
            psi((i-1)*3+5)=(-1)^(firstSS+1)*i*(pi/2/nbstep);
            psi((i-1)*3+6)=(-1)^(firstSS+1)*i*(pi/2/nbstep);
        end
        psi(end-2:end)=(-1)^(firstSS+1)*pi/2;
end
%%%%%%%

%% %%%next file to compute next part of the code
open('AB_QP_generating_SSP.m');