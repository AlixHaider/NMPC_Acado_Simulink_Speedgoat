# NMPC_Acado_Simulink_Speedgoat
NMPC Design in MATLAB/Simulink for Speedgoat Target Machine Using Acado Toolkit 

Sussessfully tested on MATLAB-2021

Citation: Please cite this article:  https://doi.org/10.1049/rpg2.12257

"Application of real-time nonlinear model predictive control for wave energy conversion"
Ali S. Haider, Ted K.A. Brekken, Alan McCall

IET Renewable Power Generation, 2021



Instruction:

1>>Download and install community (free) version of Visual Studio 2019 from https://visualstudio.microsoft.com/downloads/ and Install MSBuild package.

2>> After step 1, launch Matlab, and  “mex -setup C++” command should show following the outcome:
“MEX configured to use 'Microsoft Visual C++ 2019' for C++ language compilation.”

3>>Download the zip Acado toolkit package from https://github.com/acado/acado

4>>Unzip the package into a folder named acadoMaster

5>> Copy acadoMaster folder in C drive. The path will look like this: C:\acadoMaster.
Now add acadoMaster folder and its sub folders to matlab path, and save this path for future matlab sessions too.

6>> In matlab, Run “make clean all” command with the current directory set to C:\acadoMaster\interfaces\matlab
It should start generating the mex files needed for later code generation. This process should complete 100%. This step needs to be done once while you are setting up Acado toolkit. One successful, you do not need to repeat it for the future matlab sessions.

7>> Now download and unzip our full directory “NMPC_Acado_Simulink_Speedgoat” and set it as current directory in Matlab.

8>> run “ACADO_NMPC_Main_File.m” file. It should successfully generate “acado_solver_sfun.mexw64”

9>> open the Simulink model “NLMPC_Acado_Deploy_Speedgoat”. This model should run successfully. You should be able to built it and deploy it on Speedgoat machine too. 
Enjoy!

Message me for any questions:
https://www.linkedin.com/in/ali-s-haider-eecs/

Best

-Ali S. Haider
