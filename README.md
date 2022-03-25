Compile with gcc modb_sam.c -o modb_sam.

This is a POSIX (Linux) command-line tool to read or write Modbus registers via a serial port. It is called like this:

modb <tty> <addr> <reg> R|r|w|t ...
 
<tty>   is the device file suffix for the Modbus serial port, like USB0 for /dev/ttyUSB0, or ACM1 for /dev/ttyACM1
<addr>  is the Modbus address of the device we are accessing
<reg>   is the starting Modbus register we are reading or writing
 
R <#>                     reads <#> input reigsters starting at <addr>, and prints them, space-delimited
r <#>                     reads <#> holding registers starting at <addr>, and prints them, space-delimited
w <value1> <value2> ...   writes <values> to holding registers starting at <addr>
t "<text>"                converts <text> to ASCII values and writes those values starting at <addr>
 
All values are decimal: the values printed from an 'r' command, and the values used as parameters to the 'w' 
command.
 
<reg> is the on-the-wire register value, i.e. it is 0-based.
 
If the target device responds correctly, the letters "OK" are printed on a line by themselves as the last line 
the program writes.
 
Otherwise, the program returns a text description of what went wrong (timeout, bad CRC, incorrect target address 
responded, etc.) as the last line the program writes.
