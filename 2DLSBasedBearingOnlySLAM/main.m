close all
clear
clc
source "./total_least_squares.m"
addpath '../tools/g2o_wrapper'



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOAD THE DATASET
[landmarks, poses, transitions, observations] = loadG2o('../dataset/slam-2d-bearing-only.g2o');

num_landmarks=size(landmarks,2);
num_poses=size(poses,2);
num_transitions=size(transitions,2);
num_observations=size(observations,2);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RETRIEVE GT

% poses
XR_true=zeros(3,3,num_poses);
pose_ids=zeros(1,num_poses);
for i=1:num_poses
	XR_true(:,:,i)=v2t([poses(i).x poses(i).y poses(i).theta]');
	pose_ids(:,i)=poses(i).id;
endfor

% landmarks
XL_true=zeros(2,num_landmarks);
landmark_ids=zeros(1,num_landmarks);
for i=1:num_landmarks
	XL_true(:,i)=[landmarks(i).x_pose landmarks(i).y_pose]';
	landmark_ids(:,i)=landmarks(i).id;
endfor



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RETRIEVE POSE MEASUREMENTS (ODOMETRY)
Zr=zeros(3,3,num_transitions);
pose_associations=zeros(2,num_transitions);

measurement_num=1;
for (trans_num=1:num_transitions)

    pose_from=find(pose_ids==transitions(trans_num).id_from);
    pose_to=find(pose_ids==transitions(trans_num).id_to);

    pose_associations(:,measurement_num)=[pose_from, pose_to]';
    Zr(:,:,measurement_num)=v2t(transitions(trans_num).v);

    measurement_num++;
endfor



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RETRIEVE LANDMARK MEASUREMENTS (BEARING ONLY)
num_landmark_measurements=0;
for i=1:num_observations
	num_landmark_measurements+=size(observations(i).observation,2);
endfor

BZl=zeros(1,num_landmark_measurements); % bearing measurements
landmark_associations=zeros(2,num_landmark_measurements);

measurement_num=1;
for (pose_num=1:num_poses-1)
    current_pose_id=find(pose_ids==observations(pose_num).pose_id);
    current_observation=observations(pose_num).observation;
    for (landmark_num=1:size(current_observation,2))
	current_landmark_id=find(landmark_ids==current_observation(landmark_num).id);
	landmark_associations(:,measurement_num)=[current_pose_id,current_landmark_id]';
	BZl(:,measurement_num)=current_observation(landmark_num).bearing;
	measurement_num++;
    endfor;
endfor



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RETRIEVE POSE INITIAL GUESS
XR_guess=zeros(3,3,num_poses);

odometry=zeros(num_transitions+1,3);
odometry(1,:)=[poses(1).x,poses(1).y,poses(1).theta];
for i=2:num_transitions+1
	odometry(i,:)=[transitions(i-1).v(1),transitions(i-1).v(2),transitions(i-1).v(3)];
endfor
OdomTrajectory=compute_odometry_trajectory(odometry);

for (pose_num=1:num_poses)
    XR_guess(:,:,pose_num)=v2t(OdomTrajectory(pose_num,:));
endfor



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FILTER LANDMARKS

% in the following, in order to represent an empty element a value of -10 is used (-1 could coincide with a bearing measurement, therefore -10 is preferred)

global_BZl=global_bearings(BZl,XR_guess,landmark_associations); % global bearings
global_BZl_counters=count_global_bearings(landmark_associations,num_landmarks); % counters storing how many global bearings are associated to each landmark
global_BZl_per_landmark=split_global_bearings(global_BZl,landmark_associations,num_landmarks,max(global_BZl_counters)); % reorder the global bearings per landmark
variances=compute_variances(global_BZl_counters,global_BZl_per_landmark); % variances of the global bearings associated to each landmark

% exclude the landmarks having global bearing variance less than a threshold
variance_threshold=0.1;
landmark_ids_to_be_excluded=-10;
for i=1:num_landmarks
	if variances(1,i)<variance_threshold
		if(landmark_ids_to_be_excluded(1,1)==-10)
			landmark_ids_to_be_excluded(:,1)=landmark_ids(:,i);	
		else
			landmark_ids_to_be_excluded(:,end+1)=landmark_ids(:,i);	
		endif
	endif
endfor
num_landmarks_to_be_excluded=size(landmark_ids_to_be_excluded,2);


% recompute the landmark-related variables after filtering

filtered_num_landmarks=num_landmarks-num_landmarks_to_be_excluded; % replaces num_landmarks
filtered_XL_true=zeros(2,filtered_num_landmarks); % replaces XL_true
filtered_landmark_ids=zeros(1,filtered_num_landmarks); % replaces landmark_ids

counter=1;
for i=1:num_landmarks
	current_landmark_id=landmark_ids(:,i);
	if (any(landmark_ids_to_be_excluded==current_landmark_id))
		continue
	else
		filtered_XL_true(:,counter)=XL_true(:,i);
		filtered_landmark_ids(:,counter)=current_landmark_id;
		counter+=1;
	endif
endfor

filtered_BZl=-10; % replaces BZl
filtered_landmark_associations=-10*ones(2,1); % replaces landmark_associations

for (pose_num=1:num_poses-1)
    current_pose_id=find(pose_ids==observations(pose_num).pose_id);
    current_observation=observations(pose_num).observation;
    for (landmark_num=1:size(current_observation,2))
	real_id=current_observation(landmark_num).id;
	current_landmark_id=find(filtered_landmark_ids==current_observation(landmark_num).id);
	if (isempty(current_landmark_id))
		continue
	else
		if (filtered_BZl(1,1)==-10)
			filtered_landmark_associations(:,1)=[current_pose_id,current_landmark_id]';
			filtered_BZl(:,1)=current_observation(landmark_num).bearing;
		
		else
			filtered_landmark_associations(:,end+1)=[current_pose_id,current_landmark_id]';
			filtered_BZl(:,end+1)=current_observation(landmark_num).bearing;
		endif
	endif
    endfor;
endfor

filtered_num_landmark_measurements=size(filtered_landmark_associations,2); % replaces num_landmark_measurements



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RETRIEVE INITIAL GUESS (LANDMARKS)
filtered_XL_guess=zeros(2,filtered_num_landmarks); 

filtered_global_BZl=global_bearings(filtered_BZl,XR_guess,filtered_landmark_associations); % filtered global bearings
filtered_global_BZl_counters=count_global_bearings(filtered_landmark_associations,filtered_num_landmarks); % counters storing how many filtered global bearings are associated to each landmark
[filtered_global_BZl_per_landmark x_poses_per_landmark y_poses_per_landmark]=split_global_bearings_and_poses(filtered_global_BZl,XR_guess,filtered_landmark_associations,filtered_num_landmarks,max(filtered_global_BZl_counters));% reorder the global bearings and the associated poses per landmark  

% compute, for each landmark, the intersections obtained considering each possible couple of bearing measurements related to that landmark
intersection_x=-10*ones(filtered_num_landmarks,max(filtered_global_BZl_counters)^2); % x coordinate of the intersections for each landmark (as always, -10 represents an empty element) 
intersection_y=-10*ones(filtered_num_landmarks,max(filtered_global_BZl_counters)^2); % y coordinate of the intersections for each landmark (as always, -10 represents an empty element)

for l=1:filtered_num_landmarks % for each landmark (after filtering)
	for i=1:filtered_global_BZl_counters(l) % for each bearing associated to that landmark
		for j=i+1:filtered_global_BZl_counters(l) % for each other bearing associated to that landmark (if the couple has not been already considered)
			p_i=[x_poses_per_landmark(l,i) y_poses_per_landmark(l,i)]; % global pose associated to the first bearing measurement
			p_j=[x_poses_per_landmark(l,j) y_poses_per_landmark(l,j)]; % global pose associated to the second bearing measurement
			b_i=filtered_global_BZl_per_landmark(l,i); % first bearing measurement
			b_j=filtered_global_BZl_per_landmark(l,j); % second bearing measurement
			[x_w y_w]=triangulate(b_i,p_i(1,1),p_i(1,2),b_j,p_j(1,1),p_j(1,2)); % triangulation function returning the global intersection among bearings
			first_empty_index=find(intersection_x(l,:)==-10,1);
			intersection_x(l,first_empty_index)=x_w;
			intersection_y(l,first_empty_index)=y_w;
		endfor 
	endfor
endfor

% estimate the position of each landmark as the mean of the intersections computed above 
for l=1:filtered_num_landmarks
	first_empty_index=find(intersection_x(l,:)==-10,1);
	x_mean=sum(intersection_x(l,1:first_empty_index-1))/(first_empty_index-1);
	y_mean=sum(intersection_y(l,1:first_empty_index-1))/(first_empty_index-1);
	filtered_XL_guess(:,l)=[x_mean y_mean]';
endfor



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SOLVE
damping=1e-10;
kernel_threshold_poses=1e2;
kernel_threshold_bearings=0.5;
num_iterations=30;

[XR, XL,chi_stats_l, num_inliers_l, chi_stats_r, num_inliers_r, H, b]=doTotalLS(XR_guess, filtered_XL_guess, 
									      	filtered_BZl, filtered_landmark_associations, 
									        Zr, pose_associations, 
										num_poses, 
										filtered_num_landmarks, 
										num_iterations, 
										damping, 
										kernel_threshold_poses,
										kernel_threshold_bearings);
												      

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOT (STATE)
figure(1);
hold on;
grid;

subplot(2,2,1);
title("Landmark Initial Guess");
plot(XL_true(1,:),XL_true(2,:),'g*',"linewidth",1);
hold on;
plot(filtered_XL_true(1,:),filtered_XL_true(2,:),'b*',"linewidth",1);
hold on;
plot(filtered_XL_guess(1,:),filtered_XL_guess(2,:),'ro',"linewidth",1);
legend("Filtered Landmarks","Landmark True", "Initial Guess");grid;

subplot(2,2,2);
title("Landmark After Optimization");
plot(XL_true(1,:),XL_true(2,:),'g*',"linewidth",1);
hold on;
plot(filtered_XL_true(1,:),filtered_XL_true(2,:),'b*',"linewidth",1);
hold on;
plot(XL(1,:),XL(2,:),'ro',"linewidth",1);
legend("Filtered Landmarks","Landmark True", "Final Guess");grid;
axis([-15 15 -5 15])

subplot(2,2,3);
title("Poses Initial Guess");
plot(reshape(XR_true(1,3,:),1,num_poses),reshape(XR_true(2,3,:),1,num_poses),'b*',"linewidth",1);
hold on;
plot(reshape(XR_guess(1,3,:),1,num_poses),reshape(XR_guess(2,3,:),1,num_poses),'ro',"linewidth",1);
legend("Poses True", "Initial Guess");grid;

subplot(2,2,4);
title("Poses After Optimization");
plot(reshape(XR_true(1,3,:),1,num_poses),reshape(XR_true(2,3,:),1,num_poses),'b*',"linewidth",1);
hold on;
plot(reshape(XR(1,3,:),1,num_poses),reshape(XR(2,3,:),1,num_poses),'ro',"linewidth",1);
legend("Poses True", "Final Guess"); grid;

#print('-dpng','-r500', 'state.png')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOT (ERROR)
figure(2);
hold on;
grid;

subplot(3,2,1);
plot(chi_stats_r, 'r-', "linewidth", 2);
legend("Chi Poses"); grid; xlabel("iterations");
subplot(3,2,2);
plot(num_inliers_r, 'b-', "linewidth", 2);
legend("#inliers"); grid; xlabel("iterations");

subplot(3,2,3);
plot(chi_stats_l, 'r-', "linewidth", 2);
legend("Chi Landmark"); grid; xlabel("iterations");
subplot(3,2,4);
plot(num_inliers_l, 'b-', "linewidth", 2);
legend("#inliers"); grid; xlabel("iterations");

#print('-dpng','-r500', 'error.png')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOT (H MATRIX)
figure(3);
H_ =  H./H;                      # NaN and 1 element
H_(isnan(H_))=0;                 # Nan to Zero
H_ = abs(ones(size(H_)) - H_);   # switch zero and one
H_ = flipud(H_);                 # switch rows
colormap(gray(64));
hold on;
image([0.5, size(H_,2)-0.5], [0.5, size(H_,1)-0.5], H_*64);
hold off;

#print('-dpng','-r500', 'H_matrix.png')
