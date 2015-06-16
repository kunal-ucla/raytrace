call vcvars32
call cl /EHsc test1.cpp
call test1
call python plot.py outmain.dat
cmd /k