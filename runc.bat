@echo off
call cd C:\Users\Kunal\Desktop\CEWiT _Presentations\code
call vcvars32
if %1.==. (
	set /p input=Which file do you want to run? 1 for test1.cpp and 2 for test2.cpp: 
) else (
	set input=%1
)
call cl /EHsc test%input%.cpp
call test%input%
call python plot.py outmain%input%.dat
call cmd \k