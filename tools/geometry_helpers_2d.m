% computes the pose 2d pose vector v from an homogeneous transform A
% A:[ R t ] 3x3 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector
function v=t2v(A)
	v(1:2, 1)=A(1:2,3);
	v(3,1)=atan2(A(2,1),A(1,1));
end


% computes the homogeneous transform matrix A of the pose vector v
% A:[ R t ] 3x3 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector
function A=v2t(v)
  	c=cos(v(3));
  	s=sin(v(3));
	A=[c, -s, v(1) ;
	s,  c, v(2) ;
	0   0  1  ];
end


% normalizes an angle between -pi and pi
% th: input angle
% o: output angle
function o = normalizeAngle(th)
	o = atan2(sin(th),cos(th));
end


% rotation matrix 
function R=R(alpha)
 c=cos(alpha);
 s=sin(alpha);
 R= [ c  -s;
      s  c];
endfunction


% derivative of rotation matrix
function R=Rx_prime(alpha)
 dc=-sin(alpha); % derivative of cos(alpha)
 ds=cos(alpha);  % derivative of sin(alpha)
 R= [dc  -ds;
     ds  dc];
endfunction


% derivative of rotation matrix in 0
global  R0=[0 -1;
	    1  0];


function v=flattenIsometry(T)
v=zeros(6,1);
v(1:4)=reshape(T(1:2,1:2)',4,1);
v(5:6)=T(1:2,3);
endfunction


function T=unflattenIsometry(v)
  T=eye(3);
  T(1:2,1:2)=reshape(v(1:4),2,2)';
  T(1:2,3)=v(5:6);
endfunction


function v=flattenIsometryByColumns(T)
v=zeros(6,1);
v(1:4)=reshape(T(1:2,1:2),4,1);
v(5:6)=T(1:2,3);
endfunction


function T=unflattenIsometryByColumns(v)
  T=eye(3);
  T(1:2,1:2)=reshape(v(1:4),2,2);
  T(1:2,3)=v(5:6);
endfunction


#computes the trajectory of the robot by chaining up
#the incremental movements of the odometry vector
#U:	a Nx3 matrix, each row contains the odometry ux, uy utheta
#T:	a Nx3 matrix, each row contains the robot pose (starting from 0,0,0)
function T=compute_odometry_trajectory(U)
	T=zeros(size(U,1),3);
	current_T=v2t(zeros(1,3));
	for i=1:size(U,1),
		u=U(i,1:3)';
		current_T*=v2t(u);
		T(i,1:3)=t2v(current_T)';
	end
	
end

