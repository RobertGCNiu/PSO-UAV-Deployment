function [SINR,P ] = Flow_UAV(ITER,X,users_sets)

k_center = size(X,2)/3;
for i = 1: k_center
    centerpoint(i,:) = [X(i*3-2),X(i*3-1)];
    h(i) = X(i*3);
end
N_users = users_sets;
u_distance = 500;
%all_users = u_distance * rand(200,2);
%load('alluser.mat', 'all_users');
s=ITER;
rng(s);
all_users = 500*rand(users_sets,2);
%%   pathloss max choice
N_total = 20;
placement = zeros(N_total,N_users);
for u = 1:N_users
    dist_u = zeros(k_center,1);
    for N =  1:k_center
        dist_u(N) = norm(all_users(u,:) - centerpoint(N,:));
        pathloss(u,N) = pathloss3D(dist_u(N),h(N));
    end
    [N_dist, N_belong] = min(pathloss(u,:));
    label(u) =  N_belong;
    
    n=hist(label,[1:40]);
    for N_tot = 1:N_total
        if n(N_tot)>20
            pathloss(u,N_tot)=[10^30];pathloss(u+1,N_tot)=[10^30];
        end
    end
    [N_dist, N_belong] = min(pathloss(u,:));
    label(u) =  N_belong;
    placement(label(u),u) = 1;
    dist_all(u) = N_dist;
    
    power_normolization(1,u)=1;
    yita(u) = 10^-12;
end
%% pathloss
alpha = [0.0001,0.001];

for u_row = 1:N_users
    for u_col = 1:N_users
        R_deno = norm(all_users(u_row,:)-centerpoint(label(u_row),:));
        R_nume = norm(all_users(u_row,:)-centerpoint(label(u_col),:));% the distance between the u_col-th user and the u_row-th user's BS.
        g(u_row, u_col,:) = 1/pathloss3D(R_nume, h(label(u_col)));
    end
    r(u_row) = 1/pathloss3D(R_deno, h(label(u_row)));
end
diag_r = diag(r);
f = g-diag_r*eye(N_users);
for K = 1:k_center                                                      %damping factors
    for i = 1:N_users
        for j = 1:N_users
            A(i,j) = alpha((label(i)==label(j))+1);
        end
    end
    DC(:,:,K) = diag(1./r) * ( A.*f+ 10*yita'*placement(K,:) );                 %SINR model
    [a(:,:,K),b(:,:,K)] = eig(DC(:,:,K));
    eigenvalue = diag(b(:,:,K));
    lamda(K) = max(eigenvalue);
    for i = 1:length(DC(:,:,K))
        if lamda(K) == eigenvalue(i)
            break;
        end
    end
    y_lamda(:,K) = a(:,i,K);
end

[rho,index] = max(lamda);
SINR = 1/rho;
P(:) = 0.1*y_lamda(:,index)/(placement(index,:)*y_lamda(:,index)); %real power
end