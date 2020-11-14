function [ plt ] = UpdateDataPlot( plt, uvms, t, loop )

% this function samples the variables contained in the structure uvms
% and saves them in arrays inside the struct plt
% this allows to have the time history of the data for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script

plt.t(loop) = t;

plt.toolPos(:, loop) = uvms.wTt(1:3,4);

plt.q(:, loop) = uvms.q;
plt.q_dot(:, loop) = uvms.q_dot;

plt.p(:, loop) = uvms.p;
plt.p_dot(:, loop) = uvms.p_dot;

%plt.xdot_jl(:, loop) = uvms.xdot.jl;
%plt.xdot_mu(:, loop) = uvms.xdot.mu;
plt.xdot_t(:, loop) =  blkdiag(uvms.wTv(1:3,1:3), uvms.wTv(1:3,1:3))*uvms.xdot.t;

plt.a(1:7, loop) = diag(uvms.A.jl);
plt.a(8, loop) = uvms.A.mu;
plt.a(9, loop) = uvms.A.ha(1,1);

plt.toolx(:,loop) = uvms.wTt(1,4);
plt.tooly(:,loop) = uvms.wTt(2,4);

i = [1 0 0]';
j = [0 1 0]';
k = [0 0 1]';
plt.history.x(loop) = i'*uvms.wTv(1:3,4);
plt.history.y(loop) = j'*uvms.wTv(1:3,4);
plt.history.z(loop) = k'*uvms.wTv(1:3,4);

scale = 0.1;
i_ = [scale 0 0 1]';
j_ = [0 scale 0 1]';
k_ = [0 0 scale 1]';
if t == floor(t)
    index = floor(t) + 1;
    plt.axis(index).x = uvms.wTv(1:3,1:3)*i;
    plt.axis(index).y = uvms.wTv(1:3,1:3)*j;
    plt.axis(index).z = uvms.wTv(1:3,1:3)*k; 
    
    plt.point(index).i = uvms.wTv*i_;
    plt.point(index).j = uvms.wTv*j_;
    plt.point(index).k = uvms.wTv*k_;
    
    plt.vehi.x(index) = i'*uvms.wTv(1:3,4);
    plt.vehi.y(index) = j'*uvms.wTv(1:3,4);
    plt.vehi.z(index) = k'*uvms.wTv(1:3,4);
end


plt.distance(loop) = uvms.w_a;
plt.min_offset = uvms.min_offset;
plt.range = uvms.range_offset;

end