# WPG_polynomial_-_DE

Developped for Matlab 2012a and more recent

WPG based on 5th order polynomials interpolated between via-points and a deformation estimator to take into account soft sole deformation detailed in https://www.researchgate.net/publication/318412580_Optimized_Humanoid_Walking_with_Soft_Soles


Launch AA_constant => tune the WPG parameters to chose/describe the wanted trajectory

run script up to AD_QP_optimization => optimization criteria weight can be chosen

run up to AA_writing_in_txt => you can chose what has to be printed to control the robot


matlab compiler option for the deformation estimator:

mex '-IC:\Users\Giovanni\Documents\eigen-eigen-1306d75b4a21\eigen-eigen-1306d75b4a21' Simulation_3D\splineBasis.cpp

mex '-IC:\Users\Giovanni\Documents\eigen-eigen-1306d75b4a21\eigen-eigen-1306d75b4a21' GaussFtotZMP.cpp Gausstot.cpp


In main.m is the WPG built-in as a programming object in matlab.
However, it is note up-to-date with the previous method (notably the deformation estimator links)