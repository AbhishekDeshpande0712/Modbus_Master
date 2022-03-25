/* modb.c
 *
 * Simple utility to send a Modbus read or write command to a peripheral.
 * See usage() below for more details, and see https://www.simplymodbus.ca/FAQ.htm
 * for a primer on how Modbus works.
 *
 * compile with: cc modb.c -o modb
 *
 * Copyright (c) 2022 AssetLink Global LLC
 */
 
#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>  
#include <sys/time.h>


// Structure to capture the concept of "a Modbus transaction"
typedef struct modb_args{
	char *tty;
	int addr;
	int function;
	int reg;
	int length;
	char action;
	int timeout_ms;
} modb_t;


// Once we open the serial port, everyone can use it
int fd;


// Utility function to get time in ms
uint64_t getms() {
	struct timeval t;
	gettimeofday(&t, NULL);
	uint64_t ms = ((uint64_t)t.tv_sec * 1000) + ((uint64_t)t.tv_usec / 1000);
	return ms;
}

// Utility function for a simple strtol
long int strtol0(const char *str) {
	return strtol(str, NULL, 0);
}


int set_interface_attribs(int fd, int speed){
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error from tcgetattr while setting attribs: %s\n", strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

	tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;         /* 8-bit characters */
	tty.c_cflag &= ~PARENB;     /* no parity bit */
	tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		printf("Error from tcsetattr while setting attribs: %s\n", strerror(errno));
		return -1;
	}
	
	return 0;
}

void set_mincount(int fd, int mcount) {
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error tcgetattr while setting mincount: %s\n", strerror(errno));
		return;
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;
	tty.c_cc[VTIME] = 5;        /* half second timer */

	if (tcsetattr(fd, TCSANOW, &tty) < 0) 
	  printf("Error tcsetattr while setting mincount: %s\n", strerror(errno));
}


// See https://www.simplymodbus.ca/crc.xls for a nice step-by-step demonstration of how this
// calculation --- and CRC calculations in general --- work
uint16_t crc16(uint8_t *buf, int len) {
	uint16_t	crc = 0xFFFF;
	uint16_t	polynomial = 0xA001;
	
	while (len--) {
		crc ^= *buf++;
		for (int bit = 7; bit >= 0; bit--) {
			int do_convolution = (crc & 0x0001);
			crc >>= 1;
			if (do_convolution) crc ^= polynomial;
		}
	}
	
	return crc;
}

// All Modbus transactions are write (with a CRC), then read (checking CRC)
// buf is assumed to have enough space to add the CRC before writing, then contain the entire response
int exchange(modb_t req_params, uint8_t *buf, int numoutbytes, int numinbytes) {
	// Add the crc (little-endian, oddly)
	uint16_t crc = crc16(buf, numoutbytes);
	buf[numoutbytes++] = (crc & 0xFF);
	buf[numoutbytes++] = (crc >> 8);
	
	// Clear out anything currently in the tty receive buffer
	uint8_t inbyte;
	uint64_t start = getms();
	while (read(fd, &inbyte, 1) == 1) {
		if ((getms() - start) >= req_params.timeout_ms) {
			printf("Endless data arriving\n");
			return 0;
		}
	}

	// Push the bytes out
	write(fd, buf, numoutbytes);

	// Then, wait up until the timeout for a response to come in
	// Put the response into the buf we were given
	int index = 0;
	start = getms();
	do {
		ssize_t numread = read(fd, &inbyte, 1);
		//printf("%d\n",inbyte);
		
		if (numread == 1) {				// A byte has come in
			buf[index] = inbyte;
			//printf("index is %d and value is %d\n",index, buf[index]);
			// Check the bytes that we can anticipate (address, function code, CRC)
			if (index == 0) {
				// Should be the address we wrote to
				if (inbyte != req_params.addr) {
					printf("Mismatch in peripheral address: expected %d, got %d\n", req_params.addr, inbyte);
					return 0;
				}
			}
			else if (index == 1) {
				// Should be the function we asked for
				if (inbyte != req_params.function) {
					printf("Mismatch in function: expected %d, got %d\n", req_params.function, inbyte);
					return 0;
				}
			}
			else if (index == numinbytes-1) {
				
				// Last byte we expected
				crc = crc16(buf, numinbytes-2);
				//uint16_t crc_rcvd = ((uint16_t)buf[numinbytes-2] << 8) | buf[numinbytes-1];
				uint16_t crc_rcvd = buf[numinbytes-2] | ((uint16_t)buf[numinbytes-1] << 8);
				if (crc != crc_rcvd) {
					printf("Mismatch in received CRC: expected %04x, got %04x\n", crc, crc_rcvd);
					return 0;
				}
				else {
					// Success!
					return 1;
				}
				
			}
			switch(index++) {
			}
		}
		
	} while ((getms() - start) < req_params.timeout_ms);
	
	// If we got here, we timed out
	printf("No response from device %d after %d milliseconds\n", req_params.addr, req_params.timeout_ms);
	return 0;
}

// Functions 3 or 4
void read_request(modb_t req_params) {
	uint8_t	bytes[256];
	
	// Construct read header
	bytes[0] = req_params.addr;
	bytes[1] = req_params.function;
	bytes[2] = (req_params.reg >> 8);
	bytes[3] = (req_params.reg & 0xFF);
	bytes[4] = (req_params.length >> 8);
	bytes[5] = (req_params.length & 0xFF);

	// Go send this request; if it was successful, print out the registers received
	int success = exchange(req_params, bytes, 6, 2*req_params.length + 5);
	//printf("post exchnage\n");
	if (success) {
		for (int i=0; i<req_params.length; i++) {
			uint16_t regval = ((uint16_t)bytes[2*i+3] << 8) | bytes[2*i+4];
			printf("%u ", regval);
		}
		printf("\nOK\n");
	}
}

// Function 16
void write_request(modb_t req_params, int16_t *registers) {
	uint8_t	bytes[256];
	
	// Construct write header
	bytes[0] = req_params.addr;
	bytes[1] = req_params.function;
	bytes[2] = (req_params.reg >> 8);
	bytes[3] = (req_params.reg & 0xFF);
	bytes[4] = (req_params.length >> 8);
	bytes[5] = (req_params.length & 0xFF);
	bytes[6] = req_params.length * 2;

	// Add the values to write
	int numoutbytes = 7;
	for (int i=0; i<req_params.length; i++) {
		// Big-endian pack of 16 bit words into bytes
		bytes[numoutbytes++] = (registers[i] >> 8);
		bytes[numoutbytes++] = (registers[i] & 0xFF);
	}

	// Go send this request; all we care is that it was successful
	int success = exchange(req_params, bytes, 2*req_params.length + 7, 8);
	if (success) printf("OK\n");
}


void usage() {
	printf("\nmodb <tty> <addr> <reg> R|r|w|t ...\n\n");
	printf("<tty>   is the device file suffix for the Modbus serial port, like USB0 for /dev/ttyUSB0, or ACM1 for /dev/ttyACM1\n");
	printf("<addr>  is the Modbus address of the device we are accessing\n");
	printf("<reg>   is the starting Modbus register we are reading or writing\n\n");
	printf("R #                       reads # input reigsters starting at <addr>, and prints them, space-delimited\n");
	printf("r #                       reads # holding registers starting at <addr>, and prints them, space-delimited\n");
	printf("w <value1> <value2> ...   writes <values> to holding registers starting at <addr>\n");
	printf("t \"<text>\"                converts <text> to ASCII values and writes those values starting at <addr>\n\n");
	printf("All values are decimal: the values printed from an 'r' command, and the values used as parameters to the 'w' command.\n\n");
	printf("<reg> is the on-the-wire register value, i.e. it is 0-based.\n\n");
	printf("If the target device responds correctly, the letters \"OK\" are printed on a line by themselves as the last line the program writes.\n\n");
	printf("Otherwise, the program returns a text description of what went wrong (timeout, bad CRC, incorrect target address responded, etc.) as the last line the program writes.\n\n");
}

int main(int argc, char *argv[]) {
	char portname[1024];
	int	first_extra_param;
	int16_t buf[256];			// Modbus revolves around 16-bit register values
    modb_t req_params;
	
	if (argc < 6) {
		usage();
		return -1;
	}
	
	// parsing through the command line arguments
	req_params.tty = argv[1];
	req_params.addr = strtol0(argv[2]);
	req_params.reg = strtol0(argv[3]);
	req_params.action = *argv[4];
	first_extra_param = 5;
	req_params.timeout_ms = 2000;

	// opening the serial port
	sprintf(portname, "/dev/tty%s", req_params.tty);
	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		printf("Error opening %s: %s\n", portname, strerror(errno));   
		return -1;
	}

	/* baudrate 57600, 8 bits, no parity, 1 stop bit */
	set_interface_attribs(fd, B57600);
	set_mincount(fd, 0);					/* set to pure timed read */
	
	// dispatching to read/write routines
	if (req_params.action == 'R') {			// Read input registers
		req_params.function = 4;
		req_params.length = strtol0(argv[first_extra_param]);
		read_request(req_params);
	}
	else if (req_params.action == 'r') {	// Read holding registers
		req_params.function = 3;
		req_params.length = strtol0(argv[first_extra_param]);
		read_request(req_params);
	}
	else if (req_params.action == 'w') {	// Write holding registers
		req_params.function = 16;
		req_params.length = argc - first_extra_param;
		for (int i=0; i<req_params.length; i++) {
			int16_t	regval = strtol0(argv[i+first_extra_param]);
			buf[i] = strtol0(argv[i+first_extra_param]);
		}
		write_request(req_params, buf);
	}
	else if (req_params.action == 't') {	// Write text to holding registers (one character per register)
		req_params.function = 16;
		req_params.length = strlen(argv[first_extra_param]);
		for (int i=0; i<req_params.length; i++) {
			buf[i] = (uint16_t)argv[first_extra_param][i];
		}
		write_request(req_params, buf);
	}
	else {
		printf("Unknown action '%c'\n", req_params.action);
	}
	
	close(fd);
	
	return 0;
}


//>> fid = fopen('off03082022_knowngoodstate', 'rb');