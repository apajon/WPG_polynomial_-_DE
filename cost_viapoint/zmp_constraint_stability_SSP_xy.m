function [Acons Bcons] = zmp_constraint_stability_SSP_xy(xApzmp,xBpzmp,yApzmp,yBpzmp,xApankle,xBpankle,yApankle,yBpankle,discretization,backtoankle,fronttoankle,exttoankle,inttoankle,sole_margin,theta,type_phase,nbparamBb,firstSS)

% [xApankle xBpankle]=torque_ankle_positions_SSP2(pankinit(1),pankfin(1),discretization);
% [yApankle yBpankle]=torque_ankle_positions_SSP2(pankinit(2),pankfin(2),discretization);

xApzmp_=[xApzmp zeros(size(xApankle)) zeros(size(xApzmp,1),nbparamBb)];
yApzmp_=[yApzmp zeros(size(yApankle)) zeros(size(yApzmp,1),nbparamBb)];
xApankle=[zeros(size(xApzmp)) xApankle zeros(size(xApzmp,1),nbparamBb) zeros(size(yApzmp_))];
yApankle=[zeros(size(xApzmp_)) zeros(size(yApzmp)) yApankle zeros(size(yApzmp,1),nbparamBb)];

xApzmp_=[xApzmp_ zeros(size(xApzmp_))];
yApzmp_=[zeros(size(yApzmp_)) yApzmp_];

Acons1=zeros(size(xApankle));
Bcons1=zeros(size(xBpankle));
Acons2=zeros(size(xApankle));
Bcons2=zeros(size(xBpankle));
Acons3=zeros(size(xApankle));
Bcons3=zeros(size(xBpankle));
Acons4=zeros(size(xApankle));
Bcons4=zeros(size(xBpankle));

j=1;
%constraint direction inverse clock-wise
t=theta(j);
dx=[cos(t) -sin(t) -cos(t) sin(t)];
dy=[sin(t) cos(t) -sin(t) -cos(t)];


xA=xApzmp_-xApankle;
yA=yApzmp_-yApankle;
xB=xBpzmp-xBpankle;
yB=yBpzmp-yBpankle;

for i=1:sum(discretization)+1
    if(i>sum(discretization(1:j))+1)
            j=j+1;
            %constraint direction inverse clock-wise
            t=theta(j);
            dx=[cos(t) -sin(t) -cos(t) sin(t)];
            dy=[sin(t) cos(t) -sin(t) -cos(t)];
    end
    
    if type_phase(j)~=0
        Acons1(i,:)=dx(1)*(xA(i,:))+dy(1)*(yA(i,:));
        Bcons1(i)=-dx(1)*(xB(i,:))-dy(1)*(yB(i,:))+fronttoankle-sole_margin;
        Acons2(i,:)=dx(2)*(xA(i,:))+dy(2)*(yA(i,:));
        Bcons2(i)=-dx(2)*(xB(i,:))-dy(2)*(yB(i,:));
        Acons3(i,:)=dx(3)*(xA(i,:))+dy(3)*(yA(i,:));
        Bcons3(i)=-dx(3)*(xB(i,:))-dy(3)*(yB(i,:))+backtoankle-sole_margin;
        Acons4(i,:)=dx(4)*(xA(i,:))+dy(4)*(yA(i,:));
        Bcons4(i)=-dx(4)*(xB(i,:))-dy(4)*(yB(i,:));
        if(firstSS==0)
            if(type_phase(j)==1)
            Bcons2(i)=Bcons2(i)+((inttoankle*(mod(j,6)==4)+exttoankle*(mod(j,6)==1)-sole_margin));
            Bcons4(i)=Bcons4(i)+((inttoankle*(mod(j,6)==1)+exttoankle*(mod(j,6)==4)-sole_margin));
            elseif(type_phase(j)==2)
            Bcons2(i)=Bcons2(i)+((inttoankle*(mod(j,6)==5)+exttoankle*(mod(j,6)==2)-sole_margin));
            Bcons4(i)=Bcons4(i)+((inttoankle*(mod(j,6)==2)+exttoankle*(mod(j,6)==5)-sole_margin));
            end
        elseif(firstSS==1)
            if(type_phase(j)==1)
            Bcons2(i)=Bcons2(i)+((inttoankle*(mod(j,6)==1)+exttoankle*(mod(j,6)==4)-sole_margin));
            Bcons4(i)=Bcons4(i)+((inttoankle*(mod(j,6)==4)+exttoankle*(mod(j,6)==1)-sole_margin));
            elseif(type_phase(j)==2)
            Bcons2(i)=Bcons2(i)+((inttoankle*(mod(j,6)==2)+exttoankle*(mod(j,6)==5)-sole_margin));
            Bcons4(i)=Bcons4(i)+((inttoankle*(mod(j,6)==5)+exttoankle*(mod(j,6)==2)-sole_margin));
            end
        else
           'Choose which foot is the first SS'
        end

    end
end

Acons=[Acons1(any(any(Acons1,2)+any(Bcons1,2),2),:);
       Acons2(any(any(Acons2,2)+any(Bcons2,2),2),:);
       Acons3(any(any(Acons3,2)+any(Bcons3,2),2),:);
       Acons4(any(any(Acons4,2)+any(Bcons4,2),2),:)];
Bcons=[Bcons1(any(any(Acons1,2)+any(Bcons1,2),2),:);
       Bcons2(any(any(Acons2,2)+any(Bcons2,2),2),:);
       Bcons3(any(any(Acons3,2)+any(Bcons3,2),2),:);
       Bcons4(any(any(Acons4,2)+any(Bcons4,2),2),:)];
end