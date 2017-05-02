@echo off

mkdir ga5-mingw
cd ga5-mingw
cmake -G "MinGW Makefiles" -DVIDEO_OPENGLES=0 ../../src/engine
cd ..
