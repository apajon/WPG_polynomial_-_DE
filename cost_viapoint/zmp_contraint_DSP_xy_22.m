function [xlA xlB ylA ylB xuA xuB yuA yuB] = zmp_contraint_DSP_xy_22(pankinit,pankfin,discretization,backtoankle,fronttoankle,exttoankle,inttoankle,sole_margin)

% nbviapoints=length(tpassage);
nbphases=length(discretization);

xlA=[];
ylA=[];
xuA=[];
yuA=[];
xlB=[];
ylB=[];
xuB=[];
yuB=[];

xlB=-(backtoankle-sole_margin)+pankinit(1);
ylB=-(inttoankle-sole_margin)+pankinit(2);
% l=[-(backtoankle-sole_margin)+pstep(1,1) -(exttoankle-sole_margin)-pstep(1,2)];
xuB=(fronttoankle-sole_margin)+pankinit(1);
yuB=+(exttoankle-sole_margin)+pankinit(2);
% u=[(fronttoankle-sole_margin)+pstep(1,1) +(inttoankle-sole_margin)-pstep(1,2)];


for i=1:discretization(1)
    xlB=[xlB;-(backtoankle-sole_margin)+pankinit(1)];
    ylB=[ylB;-(inttoankle-sole_margin)+pankinit(2)];
%     l=[l;-(backtoankle-sole_margin)+pstep(1,1) -(exttoankle-sole_margin)-pstep(1,2)];
    xuB=[xuB;(fronttoankle-sole_margin)+pankinit(1)];
    yuB=[yuB;+(exttoankle-sole_margin)+pankinit(2)];
%     u=[u;(fronttoankle-sole_margin)+pstep(1,1) +(inttoankle-sole_margin)-pstep(1,2)];
end

xlA=[xlA;zeros(discretization(1)+1,1)];
ylA=[ylA;zeros(discretization(1)+1,1)];
xuA=[xuA;zeros(discretization(1)+1,1)];
yuA=[yuA;zeros(discretization(1)+1,1)];

for j=3:nbphases-3
    nb=discretization(j);
    if(mod(j,3)==0)
        for i=1:nb
            xlB=[xlB;-(backtoankle-sole_margin)];
            ylB=[ylB;-(inttoankle*(mod(j,6)==0)+exttoankle*(mod(j,6)==3)-sole_margin)];
            
            xlA(end+1,ceil(j/3))=1;
            ylA(end+1,ceil(j/3))=1;
%             l=[l;-(backtoankle-sole_margin)+pstep(ceil(j/3),1) -(inttoankle*(mod(j,6)==3)+exttoankle*(mod(j,6)==0)-sole_margin)+pstep(ceil(j/3),2)];
            xuB=[xuB;(fronttoankle-sole_margin)];
            yuB=[yuB;+(inttoankle*(mod(j,6)==3)+exttoankle*(mod(j,6)==0)-sole_margin)];
            
            xuA(end+1,ceil(j/3))=1;
            yuA(end+1,ceil(j/3))=1;
%             u=[u;(fronttoankle-sole_margin)+pstep(ceil(j/3),1) +(inttoankle*(mod(j,6)==0)+exttoankle*(mod(j,6)==3)-sole_margin)+pstep(ceil(j/3),2)];
        end
    end
end

for i=1:discretization(end-2)
    xlB=[xlB;-(backtoankle-sole_margin)+pankfin(1)];
    ylB=[ylB;-(inttoankle*(mod(nbphases-2,6)==0)+exttoankle*(mod(nbphases-2,6)==3)-sole_margin)+pankfin(2)];
%             l=[l;-(backtoankle-sole_margin)+pstep(ceil(j/3),1) -(inttoankle*(mod(j,6)==3)+exttoankle*(mod(j,6)==0)-sole_margin)+pstep(ceil(j/3),2)];
    xuB=[xuB;(fronttoankle-sole_margin)+pankfin(1)];
    yuB=[yuB;+(inttoankle*(mod(nbphases-2,6)==3)+exttoankle*(mod(nbphases-2,6)==0)-sole_margin)+pankfin(2)];
%             u=[u;(fronttoankle-sole_margin)+pstep(ceil(j/3),1) +(inttoankle*(mod(j,6)==0)+exttoankle*(mod(j,6)==3)-sole_margin)+pstep(ceil(j/3),2)];
end

for i=1:discretization(end)
    xlB=[xlB;-(backtoankle-sole_margin)+pankfin(1)];
%     ylB=[ylB;-(inttoankle*(mod(nbphases+1,6)==0)+exttoankle*(mod(nbphases+1,6)==3)-sole_margin)-pankfin(2)];
    ylB=[ylB;-(inttoankle*(mod(nbphases+1,6)==0)+exttoankle*(mod(nbphases+1,6)==3)-sole_margin)+pankfin(2)-(-1)^((nbphases-2)/3)*0.095*2];
%             l=[l;-(backtoankle-sole_margin)+pstep(ceil(j/3),1) -(inttoankle*(mod(j,6)==3)+exttoankle*(mod(j,6)==0)-sole_margin)+pstep(ceil(j/3),2)];
    xuB=[xuB;(fronttoankle-sole_margin)+pankfin(1)];
%     yuB=[yuB;+(inttoankle*(mod(nbphases+1,6)==3)+exttoankle*(mod(nbphases+1,6)==0)-sole_margin)-pankfin(2)];
    yuB=[yuB;+(inttoankle*(mod(nbphases+1,6)==3)+exttoankle*(mod(nbphases+1,6)==0)-sole_margin)+pankfin(2)-(-1)^((nbphases-2)/3)*0.095*2];
%             u=[u;(fronttoankle-sole_margin)+pstep(ceil(j/3),1) +(inttoankle*(mod(j,6)==0)+exttoankle*(mod(j,6)==3)-sole_margin)+pstep(ceil(j/3),2)];
end

xlA=[xlA;zeros(discretization(end)+discretization(end-2),size(xlA,2))];
ylA=[ylA;zeros(discretization(end)+discretization(end-2),size(ylA,2))];
xuA=[xuA;zeros(discretization(end)+discretization(end-2),size(xuA,2))];
yuA=[yuA;zeros(discretization(end)+discretization(end-2),size(yuA,2))];


end