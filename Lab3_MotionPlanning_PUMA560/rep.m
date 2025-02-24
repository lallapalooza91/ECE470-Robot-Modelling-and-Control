function [tau] = rep(q,myrobot,obs)
    %calculating the forward kinematics for each link of the robot
    Hs = eye(4);
    Os = zeros(3,6);
    z_vec = zeros(3,6);
    for i = 1:6 
        %calculate forward for q and extract the z vectors and the origins
        Hs = Hs*forward_one_link(q(i),myrobot, i);
        Os(:,i) = Hs(1:3,4);
        z_vec(:,i) = Hs(1:3,3);
    end
    %define Oi - b
    Oib=zeros(3,6);
    for i=1:6
        if(obs.type=='sph')
            %if link is outside the sphere calculate Oi-b
            if(norm(Os(:,i)-obs.c) > obs.R)
               Oib(:,i)= (Os(:,i)- obs.c)*(1-(obs.R/(norm(Os(:,i)-obs.c))));
            end
            %Oi - b should be zero when link inside the sphere
            if(norm(Os(:,i)-obs.c) <= obs.R)
                Oib(:,i)= zeros(3,1);
            end
        end
        if(obs.type=='cyl')
            %if link is outside the cylinder calculate Oi-b
            if(norm(Os(1:2,i)-obs.c)>obs.R)
                Oib(1:2,i)= (Os(1:2,i)-obs.c)-(((Os(1:2,i)-obs.c))/(norm((Os(1:2,i)-obs.c)))*obs.R);
            end
            %Oi - b should be zero when link inside the cylinder
            if(norm(Os(1:2,i)-obs.c)<=obs.R)
                Oib(1:2,i)=zeros(2,1);
            end
        end         
    end  
    %calculating the forces for the robot
    etha = 1;
    F = zeros(3,6);
    for i = 1:6
        %calculate the rho grad
        rhograd=Oib(:,i)/norm(Oib(:,i));
        %if we are outside the workspace distance of influence 
        %then repulsive force = 0
        if(norm(Oib(:,i)) > obs.rho0)
            F(:,i) = zeros(3,1);
        end
        %if we are in the workspace distance of influence
        %then the repulsive force > 0
        if (norm(Oib(:,i)) <= obs.rho0)
            F(:,i)=(etha)*((1/norm(Oib(:,i))-(1/obs.rho0)))*(1/((norm(Oib(:,i)))^2))*(rhograd)*10^6;
        end
       
    end
    %calculating the Jacobian and tau
    tau = zeros(6,1);
    for i  = 1:6
        J = zeros(3,6);
        %loop through each joint
        for j = 1:i 
            %for joint 1 the z0 is 001
             if j == 1
                 J(:,j) = cross([0;0;1],(Os(:,i)-[0;0;0]));
             end
             if j > 1
                J(:,j) = cross(z_vec(:,j-1),(Os(:,i) - Os(:,j-1)));
             end
        end
        %calculate using the formula from handout
        tau = tau + transpose(J)*F(:,i);
    end
    %for a non-zero torque normalize the torques calculated else output 0
    if (norm(tau)~= 0)    
        tau = tau/norm(tau);
        tau = transpose(tau);
        return
    end
    %don't normalize zero torque case
    if (norm(tau) == 0)
        tau = transpose(zeros(6,1));
    end
end