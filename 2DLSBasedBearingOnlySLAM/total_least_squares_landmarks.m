source "../tools/geometry_helpers_2d.m"
source "./bearing_helpers.m"
source "./total_least_squares_indices.m"


# error and jacobian of a measured landmark
# input:
#   Xr: the robot pose in world frame (3x3 homogeneous matrix)
#   Xl: the landmark pose (2x1 vector, 2d pose in world frame)
#   bz:  measured bearing of landmark
# output:
#   e: 1x1 is the difference between prediction and measurement (bearing-only)
#   Jr: 1x3 derivative of the error w.r.t. a perturbation of the pose
#   Jl: 1x2 derivative of the error w.r.t. a perturbation on the landmark

function [e,Jr,Jl]=landmarkErrorAndJacobian(Xr,Xl,bz)
  # inverse transform
  iR=Xr(1:2,1:2)';
  it=-iR*Xr(1:2,3);
  
  #prediction
  p_hat=iR*Xl+it;
  z_hat=atan2(p_hat(2),p_hat(1));
  e=z_hat-bz;
  e=normalizeAngle(e); % normalization between -pi and pi 

  J_atan=(1/(p_hat(1)^2+p_hat(2)^2))*[-p_hat(2) p_hat(1)];

  Jr=zeros(2,3);
  Jr(1:2,1:2)=-iR;
  Jr(1:2,3)=iR*[Xl(2) -Xl(1)]';

  Jl=iR;

  Jr=J_atan*Jr; 
  Jl=J_atan*Jl; 


endfunction;


#linearizes the robot-landmark measurements
#   XR: the initial robot poses (3x3xnum_poses: array of homogeneous matrices)
#   XL: the initial landmark estimates (2xnum_landmarks matrix of landmarks)
#   BZl:  the measurements (1xnum_measurements)
#   associations: 2xnum_measurements. 
#                 associations(:,k)=[p_idx,l_idx]' means the kth measurement
#                 refers to an observation made from pose p_idx, that
#                 observed landmark l_idx
#   num_poses: number of poses in XR (added for consistency)
#   num_landmarks: number of landmarks in XL (added for consistency)
#   kernel_threshod: robust kernel threshold
# output:
#   XR: the robot poses after optimization
#   XL: the landmarks after optimization
#   chi_stats: array 1:num_iterations, containing evolution of chi2
#   num_inliers: array 1:num_iterations, containing evolution of inliers

function [H,b, chi_tot, num_inliers]=linearizeLandmarks(XR, XL, BZl, associations,num_poses, num_landmarks, kernel_threshold)
  global pose_dim;
  global landmark_dim;
  system_size=pose_dim*num_poses+landmark_dim*num_landmarks; 
  H=zeros(system_size, system_size);
  b=zeros(system_size,1);
  chi_tot=0;
  num_inliers=0;
  for (measurement_num=1:size(BZl,2))
    Omega=1;
    pose_index=associations(1,measurement_num);
    landmark_index=associations(2,measurement_num);
    bz=BZl(:,measurement_num);
    Xr=XR(:,:,pose_index);
    Xl=XL(:,landmark_index);
    [e,Jr,Jl] = landmarkErrorAndJacobian(Xr, Xl, bz);
    chi=e'*Omega*e;
    if (chi>kernel_threshold)
      Omega*=sqrt(kernel_threshold/chi);
      chi=kernel_threshold;
    else
      num_inliers++;
    endif;
    chi_tot+=chi;

    pose_matrix_index=poseMatrixIndex(pose_index, num_poses, num_landmarks);
    landmark_matrix_index=landmarkMatrixIndex(landmark_index, num_poses, num_landmarks);

    H(pose_matrix_index:pose_matrix_index+pose_dim-1,
      pose_matrix_index:pose_matrix_index+pose_dim-1)+=Jr'*Omega*Jr;

    H(pose_matrix_index:pose_matrix_index+pose_dim-1,
      landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=Jr'*Omega*Jl;

    H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
      landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=Jl'*Omega*Jl;

    H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
      pose_matrix_index:pose_matrix_index+pose_dim-1)+=Jl'*Omega*Jr;

    b(pose_matrix_index:pose_matrix_index+pose_dim-1)+=Jr'*Omega*e;
    b(landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=Jl'*Omega*e;
  endfor
endfunction

