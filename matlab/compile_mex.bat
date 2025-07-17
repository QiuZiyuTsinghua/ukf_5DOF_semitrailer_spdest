@echo off
echo 编译UKF MEX模块...
echo Compiling UKF MEX module...

rem 设置MATLAB路径 (根据实际安装路径调整)
rem Set MATLAB path (adjust according to actual installation path)
set MATLAB_PATH="C:\Program Files\MATLAB\R2023a\bin\matlab.exe"

rem 编译MEX文件
rem Compile MEX file
%MATLAB_PATH% -batch "cd('matlab'); compile_mex; exit"

echo 编译完成!
echo Compilation completed!
pause
