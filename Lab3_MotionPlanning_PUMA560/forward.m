function [I]=forward(joint,robot)
    I=eye(4);    
    for j=6:-1:1
        %extract DH parameters for robot 
        t=joint(j);
        a=robot.alpha(j);
        d=robot.d(j);
        A=robot.a(j);
        %calculate H matrix for current parameters
        H1=[cos(t),-sin(t)*cos(a),sin(t)*sin(a),A*cos(t);sin(t),cos(t)*cos(a),-cos(t)*sin(a),A*sin(t);0, sin(a), cos(a),d;0,0,0,1];
        %update the H matrix to be returned
        I=H1*I;
    end
end
    
