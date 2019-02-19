source "../tools/geometry_helpers_2d.m"
source "./bearing_helpers.m"
source "./total_least_squares_indices.m"

# error and jacobian of a measured pose, all poses are in world frame
# input:
#   Xi: the observing robot pose (3x3 homogeneous matrix)
#   Xj: the observed robot pose (3x3 homogeneous matrix)
#   Z:   the relative transform measured between Xr1 and Xr2
#   e: 6x1 is the difference between prediction, and measurement, vectorized
#   Ji : 6x3 derivative w.r.t a the error and a perturbation of the
#       first pose
#   Jj : 6x3 derivative w.r.t a the error and a perturbation of the
#       second pose

function [e,Ji,Jj]=poseErrorAndJacobian(Xi,Xj,Z)
  global R0;
  Ri=Xi(1:2,1:2);
  Rj=Xj(1:2,1:2);
  ti=Xi(1:2,3);
  tj=Xj(1:2,3);
  tij=tj-ti;
  Ri_transpose=Ri';
  Ji=zeros(6,3);
  Jj=zeros(6,3);
  
  dR_da=Ri_transpose*R0*Rj;
  
  Jj(1:4,3)=reshape(dR_da, 4, 1);
  Jj(5:6,1:2)=Ri_transpose;
  
  Jj(5:6,3)=-Ri_transpose*[-tj(2) tj(1)]';
  Ji=-Jj;

  Z_hat=eye(3);
  Z_hat(1:2,1:2)=Ri_transpose*Rj;
  Z_hat(1:2,3)=Ri_transpose*tij;
  e=flattenIsometryByColumns(Z_hat-Z);
 endfunction;

#linearizes the robot-robot measurements
# inputs:
#   XR: the initial robot poses (3x3xnum_poses: array of homogeneous matrices)
#   XL: the initial landmark estimates (2xnum_landmarks matrix of landmarks)
#   ZR: the robot_robot measuremenrs (3x3xnum_measurements: array of homogeneous matrices)
#   associations: 2xnum_measurements. 
#                 associations(:,k)=[i_idx, j_idx]' means the kth measurement
#                 refers to an observation made from pose i_idx, that
#                 observed the pose j_idx
#   num_poses: number of poses in XR (added for consistency)
#   num_landmarks: number of landmarks in XL (added for consistency)
#   kernel_threshod: robust kernel threshold
# outputs:
#   H: the H matrix, filled
#   b: the b vector, filled
#   chi_tot: the total chi2 of the current round
#   num_inliers: number of measurements whose error is below kernel_threshold

function [H,b, chi_tot, num_inliers]=linearizePoses(XR, XL, Zr, associations,num_poses, num_landmarks, kernel_threshold)
  global pose_dim;
  global landmark_dim;
  system_size=pose_dim*num_poses+landmark_dim*num_landmarks; 
  H=zeros(system_size, system_size);
  b=zeros(system_size,1);
  chi_tot=0;
  num_inliers=0;
  for (measurement_num=1:size(Zr,3))
    Omega=eye(6);
    Omega(1:4,1:4)*=1e3; # we need to pimp the rotation  part a little
    pose_i_index=associations(1,measurement_num);
    pose_j_index=associations(2,measurement_num);
    Z=Zr(:,:,measurement_num);
    Xi=XR(:,:,pose_i_index);
    Xj=XR(:,:,pose_j_index);
    [e,Ji,Jj] = poseErrorAndJacobian(Xi, Xj, Z);
    chi=e'*Omega*e;
    if (chi>kernel_threshold)
      Omega*=sqrt(kernel_threshold/chi);
      chi=kernel_threshold;
    else
      num_inliers ++;
    endif;
    chi_tot+=chi;

    pose_i_matrix_index=poseMatrixIndex(pose_i_index, num_poses, num_landmarks);
    pose_j_matrix_index=poseMatrixIndex(pose_j_index, num_poses, num_landmarks);
    
    H(pose_i_matrix_index:pose_i_matrix_index+pose_dim-1,
      pose_i_matrix_index:pose_i_matrix_index+pose_dim-1)+=Ji'*Omega*Ji;

    H(pose_i_matrix_index:pose_i_matrix_index+pose_dim-1,
      pose_j_matrix_index:pose_j_matrix_index+pose_dim-1)+=Ji'*Omega*Jj;

    H(pose_j_matrix_index:pose_j_matrix_index+pose_dim-1,
      pose_i_matrix_index:pose_i_matrix_index+pose_dim-1)+=Jj'*Omega*Ji;

    H(pose_j_matrix_index:pose_j_matrix_index+pose_dim-1,
      pose_j_matrix_index:pose_j_matrix_index+pose_dim-1)+=Jj'*Omega*Jj;

    b(pose_i_matrix_index:pose_i_matrix_index+pose_dim-1)+=Ji'*Omega*e;
    b(pose_j_matrix_index:pose_j_matrix_index+pose_dim-1)+=Jj'*Omega*e;
  endfor
endfunction
