function [Acons Bcons] = zmp_constraint_ankle_pos(pankinit,pankfin,backtoankle,fronttoankle,exttoankle,inttoankle,xankmax,xankmin,yankmax,yankmin,theta,nbparamABCD,nbparamank,nbparamBb)

nbparamtotal=nbparamABCD+4+nbparamank+nbparamBb;
xApankle=[zeros(1,nbparamtotal*2);
    zeros(nbparamank,nbparamABCD+4) eye(nbparamank) zeros(nbparamank,nbparamBb+nbparamtotal);
    zeros(1,nbparamtotal*2)];
xBpankle=[pankinit(1);zeros(nbparamank,1);pankfin(1)];
yApankle=[zeros(1,nbparamtotal*2);
    zeros(nbparamank,nbparamtotal+nbparamABCD+4) eye(nbparamank) zeros(nbparamank,nbparamBb);
    zeros(1,nbparamtotal*2)];
yBpankle=[pankinit(2);zeros(nbparamank,1);pankfin(2)];

%clock-wise turn from upper right if foot in x direction
ABCDleft=[fronttoankle -backtoankle -backtoankle fronttoankle;
           exttoankle exttoankle -inttoankle -inttoankle];

ABCDright=[fronttoankle -backtoankle -backtoankle fronttoankle;
            inttoankle inttoankle -exttoankle -exttoankle];

Acons=zeros((nbparamank+1)*16*4,nbparamtotal*2);
Bcons=zeros((nbparamank+1)*16*4,1);

for i=1:nbparamank+1
    t1=theta(i*3-1);
    t2=theta(i*3+1);
    
    rot=[cos(t2) -sin(t2);sin(t2) cos(t2)];
    dx=[cos(t1) -sin(t1) -cos(t1) sin(t1)];
    dy=[sin(t1) cos(t1) -sin(t1) -cos(t1)];
    
    if(mod(i,2))
        ABCD_=ABCDright;
    else
        ABCD_=ABCDleft;
    end
    [Acons((i-1)*16+1:(i-1)*16+4,:)       Bcons((i-1)*16+1:(i-1)*16+4)]         =constraint_builder (rot,dx,dy,ABCD_(:,1),xApankle(i,:),xBpankle(i),yApankle(i,:),yBpankle(i),xApankle(i+1,:),xBpankle(i+1),yApankle(i+1,:),yBpankle(i+1),xankmax,xankmin,yankmax,yankmin,i);
    [Acons((i-1)*16+1+4:(i-1)*16+4+4,:)   Bcons((i-1)*16+1+4:(i-1)*16+4+4)]     =constraint_builder (rot,dx,dy,ABCD_(:,2),xApankle(i,:),xBpankle(i),yApankle(i,:),yBpankle(i),xApankle(i+1,:),xBpankle(i+1),yApankle(i+1,:),yBpankle(i+1),xankmax,xankmin,yankmax,yankmin,i);
    [Acons((i-1)*16+1+8:(i-1)*16+4+8,:)   Bcons((i-1)*16+1+8:(i-1)*16+4+8)]     =constraint_builder (rot,dx,dy,ABCD_(:,3),xApankle(i,:),xBpankle(i),yApankle(i,:),yBpankle(i),xApankle(i+1,:),xBpankle(i+1),yApankle(i+1,:),yBpankle(i+1),xankmax,xankmin,yankmax,yankmin,i);
    [Acons((i-1)*16+1+12:(i-1)*16+4+12,:) Bcons((i-1)*16+1+12:(i-1)*16+4+12)]   =constraint_builder (rot,dx,dy,ABCD_(:,4),xApankle(i,:),xBpankle(i),yApankle(i,:),yBpankle(i),xApankle(i+1,:),xBpankle(i+1),yApankle(i+1,:),yBpankle(i+1),xankmax,xankmin,yankmax,yankmin,i);
end
end

function [Acons Bcons]=constraint_builder (rot,dx,dy,ABCD,xApankle1,xBpankle1,yApankle1,yBpankle1,xApankle2,xBpankle2,yApankle2,yBpankle2,xankmax,xankmin,yankmax,yankmin,j)

ABCD_=rot*ABCD;
xApankle2_=xApankle2;
xBpankle2_=xBpankle2+ABCD_(1);
yApankle2_=yApankle2;
yBpankle2_=yBpankle2+ABCD_(2);

Acons1=dx(1)*(xApankle2_-xApankle1)+dy(1)*(yApankle2_-yApankle1);
Bcons1=-dx(1)*(xBpankle2_-xBpankle1)-dy(1)*(yBpankle2_-yBpankle1)+xankmax;
Acons2=dx(2)*(xApankle2_-xApankle1)+dy(2)*(yApankle2_-yApankle1);
Bcons2=-dx(2)*(xBpankle2_-xBpankle1)-dy(2)*(yBpankle2_-yBpankle1);
Acons3=dx(3)*(xApankle2_-xApankle1)+dy(3)*(yApankle2_-yApankle1);
Bcons3=-dx(3)*(xBpankle2_-xBpankle1)-dy(3)*(yBpankle2_-yBpankle1)-xankmin;
Acons4=dx(4)*(xApankle2_-xApankle1)+dy(4)*(yApankle2_-yApankle1);
Bcons4=-dx(4)*(xBpankle2_-xBpankle1)-dy(4)*(yBpankle2_-yBpankle1);

Bcons2=Bcons2 +yankmax*mod(j-1,2)  -yankmin*mod(j,2);
Bcons4=Bcons4 +yankmax*mod(j,2)    -yankmin*mod(j-1,2);

Acons=[Acons1;Acons2;Acons3;Acons4];
Bcons=[Bcons1;Bcons2;Bcons3;Bcons4];

end