
function [sixlink]= mypuma560(DH)
    %define the 6 revolute joints for robot
    L(1)= Link(DH(1,:),'revolute');
    L(2)= Link(DH(2,:),'revolute');
    L(3)= Link(DH(3,:),'revolute');
    L(4)= Link(DH(4,:),'revolute');
    L(5)= Link(DH(5,:),'revolute');
    L(6)= Link(DH(6,:),'revolute');
    %connect all links to create robot
    sixlink = SerialLink(L, 'name', 'sixlink');
end
