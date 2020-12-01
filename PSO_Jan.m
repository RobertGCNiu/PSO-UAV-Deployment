
clc;clear all;close all;
%% 初始化种群


%f= Flow_BS(X); % 函数表达式
%figure(1);ezplot(f,[0,0.01,20]);
fym = -1;
% rec_kmeans = zeros(500,100);
% P = zeros(50,50);
%ITER=1;
for ITER = 1:1
    k_center = 5;
    N = 300;                         % 初始种群个数
    d = k_center*3;                          % 空间维数
    ger = 50;                      % 最大迭代次数
    
    limit = [0, 500;0,500;10,100;
        0, 500;0,500;10,100;
        0, 500;0,500;10,100;
        0, 500;0,500;10,100;
        0, 500;0,500;10,100;];                % 设置位置参数限制
    
    
    vlimit = [-15, 15];               % 设置速度限制
    w = 0.8;                        % 惯性权重
    c1 = 0.6;                       % 自我学习因子
    c2 = 0.9;                       % 群体学习因子
    % N_users = 50;
    % all_users = 100*rand(N_users,2);
    
    for users_sets = 80
        
        %load('alluser.mat', 'all_users');
        s=ITER;
        rng(s);
        all_users = 500*rand(users_sets,2);
        C(1,:) = zeros(1,k_center*2);
        for i = 1:N
            [A_label,B] = kmeans(all_users,k_center);
            C(i+1,:) = B(:);
            if C(i+1) == C(i)
                [A_label,B] = kmeans(all_users,k_center);
            end
            %     h = 10+90*rand(k_center,1);
            for j = 1:k_center
                h(j,1) = 50;
            end
            B = [B h]';
            X(i,:) = B(:); %初始种群的位置
        end
        C(1,:) = [];
        
        %     for i = 1:d
        %       X(:,i) = limit(i, 1) + (limit(i, 2) - limit(i, 1)) * rand(N, 1);%初始种群的位置
        %     end
        
        v = 15*rand(N, 3*k_center);                  % 初始种群的速度
        xm = X;                          % 每个个体的历史最佳位置
        ym = zeros(1, d);                % 种群的历史最佳位置
        fxm = zeros(N, 1);               % 每个个体的历史最佳适应度
        fym = -inf;                      % 种群历史最佳适应度
        hold on
        % plot(xm, f(xm), 'ro');title('初始状态图');
        % figure(2)
        %% 群体更新
        iter = 1;
        
        record = zeros(ger, 1);          % 记录器
        while iter <= ger
            %      fx = Flow_BS(X) ; % 个体当前适应度
            parfor i = 1:N
                [fx(i),p] = Flow_UAV(ITER,X(i,:),users_sets) ; % 个体当前适应度
            end
            for i = 1:N
                
                if fxm(i) < fx(i)
                    fxm(i) = fx(i);     % 更新个体历史最佳适应度
                    xm(i,:) = X(i,:);   % 更新个体历史最佳位置
                end
            end
            
            if fym < max(fxm)
                [fym, nmax] = max(fxm);   % 更新群体历史最佳适应度
                ym = xm(nmax, :);      % 更新群体历史最佳位置
            end
            w= 0.8-0.6*iter/ger;
            best(iter,:)= ym;
            v = v * w + c1 * rand * (xm - X) + c2 * rand * (repmat(ym, N, 1) - X);% 速度更新
            
            %边界速度处理
            v(v > vlimit(2)) = vlimit(2);
            v(v < vlimit(1)) = vlimit(1);
            X = X + v;% 位置更新
            % 边界位置处理
            for i = 1:N
                for j = 1:d
                    if X(i,[j]) > 500
                        X(i,[j]) = 500;
                    end
                    if X(i,[j]) < 0
                        X(i,[j]) = 0;
                    end
                end
                for j = 3:3:d
                    if X(i,[j]) > 100
                        X(i,[j]) = 100;
                    end
                    if X(i,[j]) < 30
                        X(i,[j]) = 30;
                    end
                end
                
                %         if X(i,[3]) > 100
                %            X(i,[3]) = 100;
                %         end
                %         if  X(i,[3]) < 10
                %             X(i,[3]) = 10;
                %         end
                %         if X(i,[6]) > 100
                %            X(i,[6]) = 100;
                %         end
                %         if  X(i,[6]) < 10
                %             X(i,[6]) = 10;
                %         end
                %         if X(i,[9]) > 100
                %            X(i,[9]) = 100;
                %         end
                %         if  X(i,[9]) < 10
                %             X(i,[9]) = 10;
                %         end
                
            end
            record(iter) = fym;%最大值记录 real value
            %     rec_randm(ITER,iter) = rec_kmeans(ITER,iter)+record(iter);
            %     record_random1(ITER,users_sets) = record(end);
            %     record_kmeans1(ITER,users_sets) = record(end);
            %     x0 = 0 : 0.01 : 20;
            %     plot(x0, f(x0), 'b-', x, f(x), 'ro');title('状态位置变化')
            %     pause(0.1)
            % [I,J]=find(tril(true(50),-1)) ;
            % dist_iter(iter)=sum(sqrt((X(I,1)-X(J,1)).^2+(X(I,2)-X(J,2)).^2+(X(I,3)-X(J,3)).^2))/(length(I));
            % SINR(ITER,iter) = fym;
            iter = iter+1;
            
            
            
        end
        SINR(ITER,users_sets)  = fym;
        
        % SINR(ITER,users_sets) = record(end);
        % if SINR(users_sets)<1
        %     k_center = k_center+1;
        %    users_sets=users_sets;
        % end
        [none,p]=Flow_UAV(ITER,best(end,:),users_sets);
        for i = 1:users_sets
            P(i,users_sets) = p(i);
        end
        UAV_num(ITER,users_sets) = k_center;
        % SINR_value(ITER,users_sets) = SINR(ITER,users_sets);
        % users_sets = users_sets+1;
        C=[];X = [];v=[];xm=[];B = [];%best = [];
    end
    
    error(1) = 0;
    % for i = 1:(iter-2)
    %     error(i+1) = norm(best(i+1,:)-best(i,:));
    % end
    C=[];X = [];v=[];xm=[];B = [];%best = [];
end


% rec(ITER) = db(fym)/2;
% k_center = k_center + 1;

figure(3);plot(record);title('收敛过程')
x0 = 0 : 0.01 : 20;
%figure(4);plot(x0, f(x0), 'b-', x, f(x), 'ro');title('最终状态位置')
disp(['最大值：',num2str(fym)]);
disp(['变量取值：',num2str(ym)]);
figure(1)
plot3(best(:,1),best(:,2),best(:,3),'-*',best(:,4),best(:,5),best(:,6),'-^',best(:,7),best(:,8),best(:,9),'-+');
figure
plot3(best(:,1),best(:,2),best(:,3),'*',best(:,4),best(:,5),best(:,6),'^',best(:,7),best(:,8),best(:,9),'+',best(:,10),best(:,11),best(:,12),'+',best(:,13),best(:,14),best(:,15),'+');
% on;grid on
%
% a = zeros(50,1);
%  plot3( all_users(:,1),all_users(:,2) ,a(:,1) ,'.' ),hold on
% title(' Convergence of PSO Algorithm');
%  xlabel('Horizontal Location(m)');
%  ylabel('Horizontal Location(m)');
%  zlabel('Attitude(m)');

% plot3(best(:,1),best(:,2),best(:,3),'*',best(:,4),best(:,5),best(:,6),'^',best(:,7),best(:,8),best(:,9),'+',best(:,10),best(:,11),best(:,12),'+',best(:,13),best(:,14),best(:,15),'+');
% a = zeros(50,1);
% plot3( all_users(:,1),all_users(:,2) ,a(:,1) ,'.' ),hold on
%  title(' Convergence of PSO Algorithm');
%  xlabel('Horizontal Location(m)');
%   ylabel('Horizontal Location(m)');
%  zlabel('Attitude(m)');



%  figure(2)
%  plot(dist_iter);
%  xlabel('ITERARION');
%  ylabel('Distance between Particle ');hold on;
