function [Afb] = force1_repartition(Ac,dt1)
%dt=compute_coeff_dt1_matrix(discretization,frequency,cutting);

Afb=diag(dt1*Ac);
end