function [u,t]=prbs1(N,Tmin,Tmax);
% PRBS1 
% [u,t] = prbs1(N,Tmin,Tmax)
% PURPOSE:
% Make a Pseudo Random Binary Signal of lenght N samples.
% The signal is constant for a random interval of T samples.
% The random interval T is bounded by a specified band, Tmax <= T <= Tmax.
% ON INPUT:
% N     INTEGER. Number of samples in input signal u, (u_t for all t=1,...,N).
% Tmin  INTEGER. Minimal interval for which u_t is constant.
% Tmax  INTEGER. Maximal interval for which u_t is constant.
% ON OUTPUT:
% u     REAL. The PRBS input signal of lenght N samples.
% ALGORITHM:
% Simply a home made algorithm. DDiR, revised 17.2.2000
%-------------------------------------------------------------------------

is=1;
Tmin= Tmin-1;
dT  = Tmax-Tmin;

u=zeros(N,1);                 % Make space for input signal.
if is==0
 s=sign(randn);                % Sign of input (change) at time 0.
else
 s=1;
end
k=1;                          % Initialize integer counter for time to switch.
it=1;                         % Initialize integer counter for # of intervals.

while k < N+1
 T=Tmin+dT*rand; T=ceil(T);   % Compute random time horizon T in
 u(k:k+T-1)=s*ones(T,1);      % the interval [Tmin <= T <= Tmax].
 s=s*(-1);                    % Update sign variable s which is either -1 or 1.
 k=k+T;                       % Update time counter.

 t(it)=T;                     % Save intervals, not necessary.
 it=it+1;
end

u=u(1:N);                     % Last interval T may be shorter than Tmin.
%
% END FUNCTION PRBS1


