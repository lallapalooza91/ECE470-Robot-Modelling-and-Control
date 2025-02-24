function [vec]=inverse(H,robot)
    %extract the desired Rotation matrix and the vector between the origins
    R_d=H(1:3,1:3);
    o_d=H(1:3,4);
    %vector from Origin 1 to 3 
    oc=o_d-(R_d*[0;0;20]);
    %extracting DH parameters 
    d=robot.d;
    a=robot.a;
    %calculating parameters r,s,D from diagram for end effector
    r = sqrt(oc(1)^2 + oc(2)^2);
    s = oc(3) - robot.d(1);
    %calculating angle of joint 1 to 3
    theta1 = atan2(oc(2), oc(1)) - atan2(-d(2),real(sqrt(r^2 - d(2)^2)));
    D = (r^2 - (d(2)^2) +s^2 - a(2)^2 - d(4)^2)/(2*d(4)*a(2));
    theta3=atan2(D,real(sqrt(1-D^2)));    
    theta2 = atan2(s,real(sqrt(r^2-d(2)^2)))-atan2(-d(4)*cos(theta3),a(2)+d(4)*sin(theta3));
    T=[theta1,theta2,theta3];
    %calculating H matrix from frame 0 to 3
    H30=eye(4);
    for j=3:-1:1
        t=T(j);
        a=robot.alpha(j);
        d=robot.d(j);
        A=robot.a(j);
        H1=[cos(t),-sin(t)*cos(a),sin(t)*sin(a),A*cos(t);sin(t),cos(t)*cos(a),-cos(t)*sin(a),A*sin(t);0, sin(a), cos(a),d;0,0,0,1];
        H30=H1*H30;
    end
    %extracting R30 from H30 and calculating the R36 matrix
    R30=H30(1:3,1:3);
    R03=transpose(R30);
    R36=R03*R_d
    %calculating theta 4 to 6 using the euler angles
    theta4=-atan2(R36(2,3),R36(1,3));
    theta5=atan2(real(sqrt(1-R36(3,3)^2)),R36(3,3));
    theta6=atan2(R36(3,2),-R36(3,1));
    %return all angles
    vec=[theta1,theta2,theta3,theta4,theta5,theta6];
     
end