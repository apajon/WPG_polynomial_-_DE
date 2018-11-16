function [A B]=pankle_fixed_path(AscomeqDSP,Bscomeq,nbparamABCD,nbparamank,nbparamBb,step_number_pankle_fixed)

AscomeqDSP_path=AscomeqDSP;
Bscomeq_path=Bscomeq;
if size(step_number_pankle_fixed,1)~=0
    for i=1:size(step_number_pankle_fixed)
        [AscomeqDSP_path Bscomeq_path]=pankle_fixed(AscomeqDSP_path,Bscomeq_path,nbparamABCD,nbparamank,nbparamBb,step_number_pankle_fixed(i,1),step_number_pankle_fixed(i,2:3));
    end
end

A=AscomeqDSP_path;
B=Bscomeq_path;