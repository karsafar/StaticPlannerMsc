function n_path = add_variance_con(centreline,lateral_dist,min_n)


%assign a std of 1/3m in x and y throughout
centreline_var = repmat([1/9 0;0 1/9],1,1,length(centreline));
theta_diff = diff(centreline(3,:));
fix_mat = (abs(theta_diff)>= pi);
theta_diff(fix_mat)= theta_diff(fix_mat) + 2*pi*sign(theta_diff(fix_mat))*-1;
%find where the diff is greater than 40deg
index_aug = find(theta_diff>40/180*pi) +1 ;
%place a variance of 1m -> 3m population;
%matrix_tolin = [index_aug(1)*2 - 1 ; index_aug(1)*2 - 1 ;index_aug(1)*2 ; index_aug(1)*2 ];
%linear_ind = sub2ind(size(centreline_var),repmat([1;2],length(matrix_tolin(:))/2 ,1),matrix_tolin(:) );


%% changed it for straight line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isempty(index_aug) ~= 1
    linear_ind = sub2ind(size(centreline_var),1,1,index_aug(1) );
    
    linear_ind = sub2ind(size(centreline_var),1,1,index_aug(2) -1 );
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    variance_norm = 1^2;
    centreline_var(linear_ind:end) = repmat([1 0 0 variance_norm],1,1,(length(centreline_var(:))-linear_ind +1)/4);
    %n_path = ceil(sqrt(centreline_var(1,1:2:end))*3/lateral_dist)*2 +1;
    %n_path = max(cat(2,ceil(sqrt(centreline_var(1,1,:))*3/lateral_dist)*2 +1, repmat(min_n,1,1, length(centreline_var(1,1,:)))),[],2);
end
n_path = max(cat(2,ceil(sqrt(centreline_var(2,2,:))*3/lateral_dist)*2 +1, repmat(min_n,1,1, length(centreline_var(1,1,:)))),[],2);

n_path = min(cat(2,n_path, ones(1,1,length(n_path))*21),[],2);%21
n_path = reshape(n_path,length(n_path),1);

% xhat_global = centreline;
% P = centreline_var;
% for k =1:length(P)
%
%     Rot= [cos(xhat_global(3,k) -pi/2) -sin(xhat_global(3,k) -pi/2) 0; sin(xhat_global(3,k) -pi/2) cos(xhat_global(3,k) -pi/2) 0; 0 0 1];
%
%     P_global(:,:,k)= Rot*[P(1,1,k) 0 P(1,2,k);0 0 0;P(2,1,k) 0 P(2,2,k)]*Rot';
%
%     uncertain =mvnrnd([xhat_global(1,k) xhat_global(2,k) xhat_global(3,k)],P_global(:,:,k),500);
%     figure(1)
%     h = plot(uncertain(:,1),uncertain(:,2),'+b');
%     set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
%
%     show2d([xhat_global(1,k) xhat_global(2,k)]', P_global(1:2,1:2,k), 3, 100, gca);
% end














end