    #include <stdio.h>
    #include <fcntl.h>       /* File Control Definitions           */
    #include <termios.h>     /* POSIX Terminal Control Definitions */
    #include <unistd.h>      /* UNIX Standard Definitions          */ 
    #include <errno.h>       /* ERROR Number Definitions           */
    #include <sys/ioctl.h>   /* ioctl()                            */
    #include <stdlib.h>
    #include <string.h>
   
	
    int main(void){
        int fd;     /*File Descriptor*/
        int status; 
	int wlen;
	
    	
        fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY ); //Opening the serial port

        ioctl(fd,TIOCMGET,&status); /* GET the State of MODEM bits in Status */
        status |= TIOCM_RTS;        // Set the RTS pin
        ioctl(fd, TIOCMSET, &status);	
    	
        getchar(); //To view the change in status pins before closing the port
        
        fd = open("/dev/ttyUSB1",O_RDWR | O_NOCTTY ); //Opening the serial port

        ioctl(fd,TIOCMGET,&status); /* GET the State of MODEM bits in Status */
        status |= TIOCM_RTS;        // Set the RTS pin
        ioctl(fd, TIOCMSET, &status);	
    	
        getchar();
        //close(fd);

     }
     
     /*  Compile over terminal command: gcc serial2usb.c -o serial2usb */
