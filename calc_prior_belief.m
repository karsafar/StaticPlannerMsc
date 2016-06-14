function centreline_var = calc_prior_belief(centreline)

%assign a std of 1/3m in x and y throughout
centreline_var = repmat([1/9 0;0 0.0001],1,1,length(centreline));
theta_diff = diff(centreline(3,:));
fix_mat = (abs(theta_diff)>= pi);
theta_diff(fix_mat)= theta_diff(fix_mat) + 2*pi*sign(theta_diff(fix_mat))*-1;
%find where the diff is greater than 40deg
index_aug = find(theta_diff>40/180*pi) +1 ;
%place a variance of 1m -> 3m population around the roundabout exit;


linear_ind = sub2ind(size(centreline_var),1,1,index_aug(2) -1 );
centreline_var(linear_ind:end) = repmat([1 0 0 0.2],1,1,(length(centreline_var(:))-linear_ind +1)/4);

end