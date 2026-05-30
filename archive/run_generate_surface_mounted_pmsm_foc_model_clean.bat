@echo off
cd /d d:\work\pmsm_control
"D:\Matlab2024\bin\matlab.exe" -nosplash -nodesktop -batch "try; addpath(pwd); generate_surface_mounted_pmsm_foc_model_clean; disp('MODEL_CLEAN_OK'); catch ME; disp(getReport(ME,'extended')); exit(1); end; exit(0);" > generate_clean_output.txt 2>&1
exit /b %errorlevel%
