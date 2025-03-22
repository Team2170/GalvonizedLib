@echo off
call gradlew.bat installRoboRIOToolchain
call gradlew.bat :spotlessApply
call gradlew.bat publish
call gradlew.bat test
call tests.bat
echo Current directory: %cd% starting copy of docs
Xcopy /E /y /i .\\BobcatLib\docs .\\docs\\BobcatLib


PushD "Examples/SimpleSwerve"
echo Current directory: %cd% starting test
call ./gradlew.bat test
PushD "../../"
echo Current directory: %cd% starting to copy results
Xcopy /E /y /i .\\Examples\\SimpleSwerve\\build\\reports .\\Tests\\SimpleSwerve