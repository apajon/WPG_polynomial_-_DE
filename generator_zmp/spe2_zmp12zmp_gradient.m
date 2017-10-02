function [A B] = spe2_zmp12zmp_gradient(Aszmp,Bszmp,Aszmp12,Bszmp12)

Aszmp12zmp_=Aszmp12-[Aszmp zeros(max(size(Aszmp,1),size(Aszmp12,1)),abs(size(Aszmp12,2)-size(Aszmp,2)))];
Bszmp12zmp_=Bszmp12-Bszmp;

A=transpose(Aszmp12zmp_)*Aszmp12zmp_;
B=transpose(Bszmp12zmp_)*Aszmp12zmp_;

end