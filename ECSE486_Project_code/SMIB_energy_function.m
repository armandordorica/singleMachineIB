%Calculation of on-fault tranjectory and Lyapunov (energy) function

%All the necesasry data must be added to Workspace by running the
%initialization script prior to running this script.


f = 60;
w0 = 2*pi*f;
H = 7 ;
D = 24;


w(1) = 1;
delta(1) = delta_0;

%Simulation parameters:
dt = 0.001;
t_fin = 20 ;
i = 1;
time_elapsed = 0:dt:t_fin+dt;
Pe = 0;

%Calculation of the on-fault trajectory
for t = 0:dt:t_fin
    
    %Preliminary step
    %Angle state
    fd1 = w0*(w(i)-1);
    delta_next_step_p = delta(i)+fd1*dt;
    
    %Speed state
    fw1 = 1/2/H*(Pm - Pe);
    w_next_step_p = w(i) + fw1*dt;
    
    %Actual step
    fd2 = w0*(w_next_step_p-1);
    delta(i+1) = delta(i) + 0.5*dt*(fd1+fd2);
    
    fw2 = 1/2/H*(Pm - Pe); 
    w(i+1) = w(i)+0.5*dt*(fw1+fw2);
    i = i+1;
     
end

%Energy calculation
disp('Kinetic energy')
Wk = H*w0*(w-1).^2 %Kinetic energy
disp('Potential energy')
Wp = -(Pm*(delta-delta_0)+Ep*V_IB/Xeq*(cos(delta)-cos(delta_0))) %Potential energy

W_tot= Wp +Wk; 
plot(time_elapsed, Wk)



