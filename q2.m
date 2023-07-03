%defining s and t as a variables
syms s t
%defining system matrix, input matrix, output matrix and feedforward matrix
%respectively
A = [0 1;-6 -5];
B = [0;1];
C = [1 0];  
D = [0];
%getting the numerator and denumerator of the transfer function
[num , denum] = ss2tf(A,B,C,D); 
TF_1 = tf(num,denum)
%calculating the transfer function using another method
TF_2 = C*inv(s.*eye(2)-A)*B + D

%Calculating the state transition matrix 𝛷(𝑡)
PHI_S = inv(s.*eye(2)-A)
PHI_t = ilaplace(PHI_S)

%Verifying that 𝛷˙(𝑡) = 𝐴𝛷(𝑡)
PHI_DOT_t_1 = diff(PHI_t, t)
PHI_DOT_t_2 = A*PHI_t

%checking the rank of Q_C and Q_o to check the controllability and 
%observability
sys=ss(A,B,C,D);
Q_C = rank(ctrb(sys))
Q_O = rank(obsv(sys))

%Calcating the unforced solution given an initial conditions x_o
x_o = [0;1];
x_unforced = PHI_t*x_o
y_unforced = C*x_unforced

%Calcating the forced solution given the same initial conditions x_o
x_forced = x_unforced + ilaplace(PHI_S*B*1/s)
y_forced = C*x_forced

%plotting the step response of the transfer function
figure
step(TF_1)
%checking the new transfer function and plotting its new step responce:
%1-Genereating the new transfer function from modifing the value of A by 
%using the feedback gain matrix (K) so that Ac = A - BK
Ac = [0 1;-32.65306122 -8];
[num_2,denum_2] = ss2tf(Ac,B,C,D);
TF_state_feedback = tf(num_2,denum_2)
%2-The new transfer function step response 
figure
step(TF_state_feedback)