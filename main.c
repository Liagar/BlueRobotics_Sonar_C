#include <stdio.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <stdlib.h>

#define NULL    ((void *)0)

unsigned int byteToint(unsigned char* bytes,int length){
    unsigned int num=0 ;
    for (int i=length-1;i>=0;i--){
        num=num|bytes[i]<<(8*(i));
        printf("\n%i\t%x\t%d",i,bytes[i]<<(8*(i)),num);
    }
    return num;
}
unsigned int checksum(unsigned char* msg, int msgLength){
    unsigned int sum=0;
    for(int i=0;i<(msgLength-2);i++){
        sum=sum+(int)msg[i];
    }
    unsigned char checkSumBytes[2]={msg[msgLength-2],msg[msgLength-1]};
   if(sum==byteToint(checkSumBytes,2)) return 0;
   return 1;
}

int main()
{
    // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
      int serial_port = open("/dev/ttyUSB0", O_RDWR);
      if(serial_port<0){
            printf("Error al abrir el puerto\n");
            return 0;
      }
      printf("Puerto abierto %i\n",serial_port);

      // Create new termios struc, we call it 'tty' for convention
      struct termios tty;

      // Read in existing settings, and handle any error

      if(tcgetattr(serial_port, &tty) != 0) {
          printf("Error from tcgetattr\n");
          return 1;
      }

      tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
      tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
      tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
      tty.c_cflag |= CS8; // 8 bits per byte (most common)
      tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
      tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

      tty.c_lflag &= ~ICANON;
      tty.c_lflag &= ~ECHO; // Disable echo
      tty.c_lflag &= ~ECHOE; // Disable erasure
      tty.c_lflag &= ~ECHONL; // Disable new-line echo
      tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
      tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
      tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

      tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
      tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
      // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
      // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

      tty.c_cc[VTIME] = 1;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
      tty.c_cc[VMIN] = 1;

      // Set in/out baud rate to be 115200
      cfsetispeed(&tty, B115200);
      cfsetospeed(&tty, B115200);

      // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
          printf("Error from tcsetattr\n");
          return 1;
      }

      // Write to serial port
      unsigned char msg[] = { 'B',
                              'R',
                              0x02, //Length
                              0x00,
                              0x06,
                              0x00,
                              0x00,
                              0x01,
                              0xBB,
                              0x04,
                              0x5C,
                              0x01
                            };
      write(serial_port, msg, sizeof(msg));
      for(int i=0;i<sizeof (msg);i++){
            printf  ("%x\t",msg[i]);
       }
      printf("\n");

      unsigned char read_buf [2];
      unsigned char *msg_buf=NULL;
      unsigned char readByte;
      int i=3;
      int msgLength=0;
      read(serial_port,&readByte,1);
      if (readByte=='B'){
        read_buf[0]=readByte;
        read(serial_port,&readByte,1);
        if(readByte=='R'){
            read_buf[1]=readByte;
            read(serial_port,&readByte,1);
            msgLength=10+(int)readByte;
            msg_buf=(unsigned char *) malloc(msgLength); // Allocate memory
            *(msg_buf)=read_buf[0];
            msg_buf[1]=read_buf[1];
            msg_buf[2]=readByte;
            while (i<msgLength){
                read(serial_port,&msg_buf[i],1);
                i++;
            }
        }
      }
       //Check the check sum
      if(checksum(msg_buf,msgLength)!=0){
          printf(("Error en el checksum"));
      }
      else{
          printf("Checksum ok\n");

      }

    // TODO: translate bytes to numbers
    //Check the message type (msg_buf[4] and msg_buf[5]
      // distance_simple
      if(msg_buf[4]==0xBB && msg_buf[5]==0x04){
          printf("distance_simple message\n");
           //Confidence
          int confidence=(int)msg_buf[msgLength-3];
          unsigned char distanceBytes[4]={
              msg_buf[8],
              msg_buf[9],
              msg_buf[10],
              msg_buf[11]
          };
          int distance=byteToint(distanceBytes,4);
          printf("Confidence: %x\t%d\n",msg_buf[msgLength-3],confidence);
          printf("Distance (mm) %d",distance);
    }



      // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
      if (msgLength <= 0) {
          printf("Error reading: %s");
          return 1;
      }

      // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
      // print it to the screen like this!)
      printf("Read %i bytes. Received message;", msgLength);
      //printf("Read %i bytes. Received message;", i);
      unsigned char *ptMsgBuf=&msg_buf[0];
      for(int j=0;j<msgLength;j++){
          printf("%x\t",msg_buf[j]);
      }
     // if(msg_buf!=NULL) free(msg_buf);
      close(serial_port);
      return 0; // success
}
