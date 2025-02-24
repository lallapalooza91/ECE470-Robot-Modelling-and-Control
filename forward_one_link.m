function [H]=forward_one_link(joint,robot, j)
    %calculate the forward kinematics for a single link
    H = zeros(4,4);    
    H(1,1) = cos(joint);
    H(1,2) = -sin(joint)*cos(robot.alpha(j));
    H(1,3) = sin(joint)*sin(robot.alpha(j));
    H(1,4) = robot.a(j)*cos(joint);
    H(2,1) = sin(joint);
    H(2,2) = cos(joint)*cos(robot.alpha(j));
    H(2,3) = -cos(joint)*sin(robot.alpha(j));
    H(2,4) = robot.a(j)*sin(joint);
    H(3,1) = 0;
    H(3,2) = sin(robot.alpha(j));
    H(3,3) = cos(robot.alpha(j));
    H(3,4) = robot.d(j);
    H(4,4) = 1;
end