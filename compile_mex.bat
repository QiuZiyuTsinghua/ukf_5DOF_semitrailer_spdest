@echo off
REM 编译UKF MEX模块的Windows批处理脚本
REM 在Windows命令行中运行此脚本

echo 正在编译UKF MEX模块...

REM 进入MATLAB并运行编译脚本
matlab -batch "compile_mex"

echo 编译完成!
pause
