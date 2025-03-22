@echo off
call gradlew.bat installRoboRIOToolchain
call gradlew.bat :spotlessApply
call gradlew.bat publish
call gradlew.bat test
call tests.bat
echo Current directory: %cd% starting copy of docs
Xcopy /E /y /i .\\GalvonizedLib\docs .\\docs\\GalvonizedLib