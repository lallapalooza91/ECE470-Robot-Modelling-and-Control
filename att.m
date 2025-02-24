function [tau] = att(q, q2, myrobot)
    %calculating the forward kinematics for each link of the robot
    Hs = eye(4);
    Hf = eye(4);
    Os = zeros(3,6);
    Of = zeros(3,6);
    z_vec = zeros(3,6);
    for i = 1:6 
        %calculate forward for q
        Hs = Hs*forward_one_link(q(i),myrobot, i);
        Os(:,i) = Hs(1:3,4);
        z_vec(:,i) = Hs(1:3,3);
        %calculate forward for q2
        Hf = Hf*forward_one_link(q2(i),myrobot, i);
        Of(:,i) = Hf(1:3,4);
    end
    %calculating the forces for the robot
    zeta = 1;
    F = zeros(3,6);
    for i = 1:6
        %since d is the norm of Oi(q) - Oi(qf) always case 1
        F(:,i) = (-zeta)*(Os(:,i) - Of(:,i)); %to check values if right
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
    %print force and normalize tau
    tau = tau/norm(tau);
    tau = transpose(tau);
    F;
end