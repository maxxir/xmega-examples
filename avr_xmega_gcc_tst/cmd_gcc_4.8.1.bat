echo OFF
echo -=Use modern AVR-GCC-4.8.1 from xmegaduino-beta5 (2014 builded)=-
echo -=PS. binutils (make.. etc..) used from old WinAVR2010=-
rem clear path
prompt #
rem set PATH=
rem add PATH to binutils (make etc..) from WinAVR
set PATH=%PATH%;E:\Portable\WinAVR-20090313\utils\bin\;E:\Working\arduino-1.5.6-xmegaduino-beta5\;E:\Working\arduino-1.5.6-xmegaduino-beta5\hardware\tools\avr\bin\;
echo *****MAKE*****
make -v
echo *****AVR-GCC*****
avr-gcc -v
echo *****DIRs*****
dir /AD
call C:\Windows\System32\cmd.exe
prompt


