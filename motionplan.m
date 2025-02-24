function [qref] = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    %initial sum should = q0
    qsum=q0;
    alpha = 0.01;
    i=1;
    while((norm(qsum(1:5)-q2(1:5)) > tol))
        %calculate the repulsive forces for all obstacles
        repsum=zeros(1,6);
        for k=1:length(obs)
            repsum=repsum+rep(qsum,myrobot,obs{k});
        end
        %update the sum with both attractive and repulsive
        qsum=qsum+((alpha)*(att(qsum,q2,myrobot)+ repsum));
        qk(i,:)=qsum;
        i=i+1;
    end

    %update q6
    q6=transpose(linspace(q0(6),q2(6),size(qk,1)));
    for i=1:size(qk,1)
       qk(i,6)=q6(i);
    end
    t=linspace(t1,t2,size(qk,1));
    qref=spline(t,transpose(qk));    
end