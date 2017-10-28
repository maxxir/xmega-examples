echo OFF
echo -=Use WINAVR2010 AVR-GCC-4.3.3 =-
rem clear path
prompt #
rem set PATH=
rem add PATH to binutils (make etc..) from WinAVR
rem set PATH=%PATH%;E:\Portable\WinAVR-20090313\utils\bin\;E:\Working\arduino-1.5.6-xmegaduino-rem beta5\;E:\Working\arduino-1.5.6-xmegaduino-beta5\hardware\tools\avr\bin\;
set PATH=%PATH%;E:\WinAVR-20100110\bin;E:\WinAVR-20100110\utils\bin
echo *****MAKE*****
make -v
echo *****AVR-GCC*****
avr-gcc -v
echo *****DIRs*****
dir /AD
call C:\Windows\System32\cmd.exe
prompt


