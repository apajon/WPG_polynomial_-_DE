function [A B]=pankle_fixed(AscomeqDSP,Bscomeq,nbparamABCD,nbparamank,nbparamBb,step_number,pankle_fixed)

nbparamtotal=nbparamABCD+nbparamank+nbparamBb;

Ascomeq_path=[AscomeqDSP;zeros(1,nbparamABCD+4+step_number-1) 1 zeros(1,nbparamtotal+4+nbparamBb+(nbparamank-step_number))];
Bscomeq_path=[Bscomeq;-pankle_fixed(1)];

Ascomeq_path=[Ascomeq_path;zeros(1,nbparamtotal+4+nbparamABCD+4+step_number-1) 1 zeros(1,nbparamBb+(nbparamank-step_number))];
Bscomeq_path=[Bscomeq_path;-pankle_fixed(2)];
 
A=Ascomeq_path;
B=Bscomeq_path;