close all
clear
clc

addpath '../tools/g2o_wrapper'
source '../tools/geometry_helpers_2d.m'


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOAD THE DATASET
[landmarks, poses, transitions, observations] = loadG2o('../dataset/slam-2d-bearing-only.g2o');
#VERTEX_XY landID land.x land.y ---------------------------> landmark positions (GT) 
#VERTEX_SE2 poseID pose.x pose.y pose.theta ---------------> robot poses (GT)
#EDGE_SE2 fromID toID x y theta ---------------------------> transitions 
#EDGE_BEARING_SE2_XY poseID landID bearing ----------------> labeled bearing observations


num_landmarks=size(landmarks,2);
num_poses=size(poses,2);
num_transitions=size(transitions,2);
num_observations=size(observations,2);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOT GT
h = figure(1);

% plot GT landmark positions
for i=1:num_landmarks
	hold on;
	plot(landmarks(i).x_pose,landmarks(i).y_pose, 'b*');
endfor

% plot ideal observations
for i=1:size(observations,2)
	observation_curr=observations(i);
	pose_curr=searchById(poses,observation_curr.pose_id);
	landmarks_curr=observation_curr.observation;
	for j=1:size(landmarks_curr,2)
		landmark_curr=searchById(landmarks,landmarks_curr(j).id);
		hold on;
		plot([pose_curr.x landmark_curr.x_pose]',[pose_curr.y landmark_curr.y_pose]', 'g-', 'linewidth', 0.2);
	endfor
endfor

% plot GT trajectory
TrueTrajectory=zeros(num_poses,3);
for i=1:num_poses
	TrueTrajectory(i,:)=[poses(i).x,poses(i).y,poses(i).theta];
endfor
hold on;
plot(TrueTrajectory(:,1),TrueTrajectory(:,2), 'r-', 'linewidth', 2);
waitfor(h)
#print('-dpng','-r500','GT.png')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOT OBSERVATIONS (ODOMETRY + BEARINGS) FOR SOME POSES

% choose the poses you want to inspect
% poses_to_be_compared=[1 20 40 60 80 100];
poses_to_be_compared=linspace(1,100);

% for each selected pose, generate a plot with the GT trajectory and landmark positions, the odometry trajectory, the ideal and the real bearing measurements related to the pose at issue
for index=1:size(poses_to_be_compared,2)
	pose_to_be_compared=poses_to_be_compared(1,index);
	close all;
	h = figure(1);

	% plot GT landmark positions
	for i=1:num_landmarks
		hold on;
		plot(landmarks(i).x_pose,landmarks(i).y_pose, 'b*');
	endfor

	% plot ideal observations
	observation_curr=observations(pose_to_be_compared);
	pose_curr=searchById(poses,observation_curr.pose_id);
	landmarks_curr=observation_curr.observation;
	for j=1:size(landmarks_curr,2)
		landmark_curr=searchById(landmarks,landmarks_curr(j).id);
		hold on;
		plot([pose_curr.x landmark_curr.x_pose]',[pose_curr.y landmark_curr.y_pose]', 'b-', 'linewidth', 0.2);
	endfor

	% plot GT trajectory
	TrueTrajectory=zeros(num_poses,3);
	for i=1:num_poses
		TrueTrajectory(i,:)=[poses(i).x,poses(i).y,poses(i).theta];
	endfor
	hold on;
	plot(TrueTrajectory(:,1),TrueTrajectory(:,2), 'r-', 'linewidth', 2);

	% plot odometry trajectory
	odometry=zeros(num_transitions+1,3);
	odometry(1,:)=[poses(1).x,poses(1).y,poses(1).theta];
	for i=2:num_transitions+1
		odometry(i,:)=[transitions(i-1).v(1),transitions(i-1).v(2),transitions(i-1).v(3)];
	endfor
	OdomTrajectory=compute_odometry_trajectory(odometry);
	hold on;
	plot(OdomTrajectory(:,1),OdomTrajectory(:,2), 'g-', 'linewidth', 2);
	
	% plot bearing measurements related to the pose at issue
	range=1;
	pose_curr=OdomTrajectory(pose_to_be_compared+1,:);
	for j=1:size(landmarks_curr,2)
		bearing=landmarks_curr(j).bearing;
		global_bearing=pose_curr(3)+bearing;	
		hold on;
		plot([pose_curr(1) pose_curr(1)+range*cos(global_bearing)]',[pose_curr(2) pose_curr(2)+range*sin(global_bearing)]', 'k-', 'linewidth', 0.8);
	endfor

	waitfor(h)
	#print('-dpng','-r500', strcat('pose',num2str(pose_to_be_compared),'.png'))
endfor

