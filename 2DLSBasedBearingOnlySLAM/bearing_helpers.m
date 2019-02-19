% given a list of local points, compute the correspondent list of local bearings
function B = bearings(points)
	B=zeros(1,size(points,2));
	for i=1:size(B,2)
		B(1,i)=atan2(points(2,i),points(1,i));
	endfor
endfunction;


% given a list of local bearings and global poses, as well as the associations linking each local bearing to a global pose, compute the correspondent list of global bearings
function global_B = global_bearings(local_bearings,global_poses,associations)
	global_B=zeros(1,size(local_bearings,2));
	for i=1:size(global_B,2)
		local_bearing=local_bearings(:,i);
		global_pose=global_poses(:,:,associations(1,i));
		theta=t2v(global_pose)(3);
		global_bearing=normalizeAngle(theta+local_bearing);
		global_B(1,i)=global_bearing;
	endfor
endfunction;


% given the total number of landmarks and all the pose-landmark associations, returns 1 counter per landmark storing how many bearings are associated to that landmark
function C = count_global_bearings(associations,num_landmarks)
	C=zeros(1,num_landmarks);
	for i=1:size(associations,2)
		current_landmark=associations(2,i);
		C(1,current_landmark)+=1;
	endfor
endfunction;



% given the full list of global bearings, the total number of landmarks and all the pose-landmark associations, returns the global bearings reordered per landmark (they are stored in a matrix 
% having a row per landmark and as many coloumns as the maximum number of bearings associated to a landmark; -10 represents an empty element)
function global_B_per_landmark = split_global_bearings(global_bearings,associations,num_landmarks,max_counter)
	global_B_per_landmark=-10*ones(num_landmarks,max_counter);
	for i=1:size(global_bearings,2)
		global_bearing=global_bearings(1,i);
		current_landmark=associations(2,i);
		first_empty_index=find(global_B_per_landmark(current_landmark,:)==-10,1);
		global_B_per_landmark(current_landmark,first_empty_index)=global_bearing;
	endfor
endfunction;


% given the global bearings associated to each landmark and their number, returns their variances 
function V = compute_variances(counters,global_bearings_per_landmark)
	V=zeros(1,size(counters,2));
	for i=1:size(V,2)
		counter=counters(1,i);
		global_bearings=global_bearings_per_landmark(i,1:counter);
		V(1,i)=std(global_bearings);
	endfor
endfunction;


% given the full list of global bearings and poses, as well as the total number of landmarks and all the pose-landmark associations, returns:
% 	- the global bearings reordered per landmark (they are stored in a matrix having a row per landmark and as many coloumns as the maximum number of bearings associated to a landmark; 
%	  as always, -10 represents an empty element)
% 	- the poses associated to the global bearings ordered per landmark (they are stored in two matrices, one for x and another for y, having a row per landmark and as many coloumns as 
%         the maximum number of bearings associated to a landmark; as always, -10 represents an empty element)
function [global_B_per_landmark x_poses_per_landmark y_poses_per_landmark]= split_global_bearings_and_poses(global_bearings,poses,associations,num_landmarks,max_counter)
	global_B_per_landmark=-10*ones(num_landmarks,max_counter);
	x_poses_per_landmark=-10*ones(num_landmarks,max_counter);
	y_poses_per_landmark=-10*ones(num_landmarks,max_counter);
	for i=1:size(global_bearings,2)
		global_bearing=global_bearings(1,i);
		current_landmark=associations(2,i);
		current_pose=t2v(poses(:,:,associations(1,i)));
		first_empty_index=find(global_B_per_landmark(current_landmark,:)==-10,1);
		global_B_per_landmark(current_landmark,first_empty_index)=global_bearing;
		x_poses_per_landmark(current_landmark,first_empty_index)=current_pose(1,1);
		y_poses_per_landmark(current_landmark,first_empty_index)=current_pose(2,1);
	endfor
endfunction;


% triangulation function: given the global pose (x1,y1) associated to the bearing measure b1 and the global pose (x2,y2) associated to the bearing measure b2, returns the global coordinates of the intersection among the bearing measurements
function [x_w y_w] = triangulate(b1,x1,y1,b2,x2,y2)
	t=[x2 y2] - [x1 y1];
	tg=tan([b1 b2]);
	x=(-t(1,1)*tg(1,2)+t(1,2))/(tg(1,1)-tg(1,2));
	y=x*tg(1,1);
	x_w=x1+x;
	y_w=y1+y;
endfunction




