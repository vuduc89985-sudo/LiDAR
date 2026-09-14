@echo off
setlocal enabledelayedexpansion
cd /d "%~dp0"

rem =========================================================
rem  Neu cua so nay CHUA minimized -> mo lai chinh no minimized
rem  roi thoat. Ket qua: khong thay cua so start_all nua.
rem =========================================================
if defined STARTALL_MIN goto :RUN
set STARTALL_MIN=1
start /min "LiDAR start_all (ngam)" "%~f0"
exit /b

:RUN
title LiDAR WEB VIEWER - start_all (ngam)
echo === [start_all] Khoi dong he thong (che do ngam) === > start_all_log.txt

echo [1/4] Don tien trinh cu (tranh xung dot cong 8080/9001)... >> start_all_log.txt
taskkill /F /IM LidarWebGateway.exe >nul 2>&1
for /f "tokens=5" %%p in ('netstat -ano ^| findstr ":9001" ^| findstr "LISTENING"') do taskkill /F /PID %%p >nul 2>&1
for /f "tokens=5" %%p in ('netstat -ano ^| findstr ":8080" ^| findstr "LISTENING"') do taskkill /F /PID %%p >nul 2>&1
timeout /t 1 /nobreak >nul

echo [2/4] Khoi dong backend (HIEN binh thuong)... >> start_all_log.txt
if exist "backend\LidarWebGateway\x64\Release\LidarWebGateway.exe" (
    start "LiDAR Backend" /D "backend\LidarWebGateway\x64\Release" LidarWebGateway.exe
) else if exist "backend\LidarWebGateway\LidarWebGateway.exe" (
    start "LiDAR Backend" /D "backend\LidarWebGateway" LidarWebGateway.exe
) else (
    echo [CANH BAO] Khong tim thay LidarWebGateway.exe! >> start_all_log.txt
)

echo [3/4] Khoi dong web server 8080 (MINIMIZED)... >> start_all_log.txt
where python >nul 2>&1 && (
    start "LiDAR Web Server" /min /D "frontend" python -m http.server 8080
) || (
    start "LiDAR Web Server" /min /D "frontend" py -3 -m http.server 8080
)
timeout /t 2 /nobreak >nul

echo [4/4] Mo trinh duyet (kem chong cache)... >> start_all_log.txt
set CB=%RANDOM%%RANDOM%
start "" "http://localhost:8080/?v=!CB!"

echo [OK] He thong da khoi dong xong. >> start_all_log.txt
endlocal
exit /b