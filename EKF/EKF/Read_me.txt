Shneka Muthu Kumara Swamy
5042889

Instructions to run the program 

To run both the files mentioned below the other files included must also be used that is to run the following two files please include
EKF_propagate.m , EKF_update_bear.m,EKF_update_dist,EKF_update_dist_bear.m,plot_error_ellipse.m,rws.m

1. TwoDim_EKF.m file

In line 11 of this file a variable input is given. Please change the value of this to change the landmark measurement used,
(i) Distance only measurement -- 1
(ii) Bearing only measurement -- 2
(iii) Distance and bearing measurement -- 3

2. Two_Dim_ec.m file

This file is included for the extracredit problem given.
The type of measurement used is changed in the same way as mentioned for TwoDim_EKF.m.
The number of landmarks is given as an input.

To run the function use: Two_Dim_ec(2)

Assumptions:

1. It is assumed that the input given is always correct that is the value of input is 1,2 or 3. If anyother input is given the output 
   will be the same as giving input 3.
2. The input given for extra credit is also assumed to be always correct.  