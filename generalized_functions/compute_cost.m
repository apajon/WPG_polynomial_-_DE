function [cost gradient]=compute_cost(x,H,f)
cost=1/2*x'*H*x+f'*x;
% gradient=H*x;
gradient=H*x+f;
end