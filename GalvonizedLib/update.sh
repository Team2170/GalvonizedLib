#!/bin/bash
./gradlew installRoboRIOToolchain
./gradlew :spotlessApply
./gradlew publish
