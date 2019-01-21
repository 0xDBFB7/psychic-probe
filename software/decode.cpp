#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>

/////////////////////////////////////////////////
//Settings

#define CHANNELS 1
#define ID 1 //must be < 256 and > 0 or else line termination will break
            // A maximum of 4080 channels!

#define SCALE_THRESHOLD 150

/////////////////////////////////////////////////
//Board calibration factors
/////////////////////////////////////////////////
#define PACKET_LENGTH (CHANNELS*2)+1+1+4 //channels + checksum + id + terminator
#define ID_INDEX (CHANNELS*2)+1
#define CHECKSUM_INDEX ID_INDEX+1
#define TERMINATOR_INDEX_1 CHECKSUM_INDEX+1
#define TERMINATOR_INDEX_2 CHECKSUM_INDEX+2
#define TERMINATOR_INDEX_3 CHECKSUM_INDEX+3
#define R1 1000000
#define R2 10000

#define CHANNEL_ZERO 2048
//
// float channel_zero_trim[2][CHANNELS] = {{-1.701416,-1.874023,-1.775391,-1.726074,-1.676758,-1.726074,-1.627441,-1.701416,-1.701416,-1.800049,-1.726074,-1.800049,-1.824707,-1.849365,-1.800049,-1.726074},
//                       {-1.701416,-1.874023,-1.775391,-1.726074,-1.676758,-1.726074,-1.627441,-1.701416,-1.701416,-1.800049,-1.726074,-1.800049,-1.824707,-1.849365,-1.800049,-1.726074}};
//


//thanks, wallyk!
int set_interface_attribs (int fd, int speed, int parity){
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block) {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 10;            // 0.5 seconds read timeout
}

uint8_t chksum8(const uint8_t *buff, size_t len){
    unsigned int sum;
    for ( sum = 0 ; len != 0 ; len-- )
        sum += *(buff++);
    return (uint8_t)sum;
}

uint16_t buffer_to_unsigned(const uint8_t *buff, int index){
  return (((buff[index+1] & 0xff) << 8) | (buff[index] & 0xff));
}

float compute_voltage(uint16_t input){
  return ((((((float)input)-CHANNEL_ZERO)/4096.0) * (R1 + R2))/R2);
}

int main(){
  int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
  set_interface_attribs(fd, B3000000, 0);
  set_blocking(fd, 1);

  uint8_t packet[PACKET_LENGTH];

  struct timespec start,timestamp;
  clock_gettime(CLOCK_REALTIME, &start);
  uint64_t start_time = (start.tv_sec * 1e6) + (start.tv_nsec/1000);
  uint64_t new_timestamp = 0;

  printf("t,");
  for(int i = 0; i < CHANNELS; i++){
    printf("CHANNEL%i,",i);
  }
  printf("\n");
  int cycle_count = 0;
  while(1){
    uint8_t current_byte[1];
    current_byte[0] = 0;
    while(!read(fd, current_byte, 1));  // read up to 100 characters if ready to read

    //shift and append the new byte to the end
    for (int i = 1; i < PACKET_LENGTH; i++){
      packet[i-1]=packet[i];
    }
    packet[PACKET_LENGTH-1] = current_byte[0];

    // for(int i =0 ; i < PACKET_LENGTH; i++){
    //   printf("%i,",packet[i]);
    // }
    //
    // printf("\r\n");
    //15037613,1112.676758,1087.796631,1131.885498,1138.148682,1138.099365,1138.148682,1138.050049,1106.561523,1119.186523,1081.410156,1093.961182,1094.035156,1112.997314,1113.021973,1138.222656,7704.702148,

    float scaled_channel_values[CHANNELS];

    if(packet[TERMINATOR_INDEX_1] == 0xff && packet[TERMINATOR_INDEX_2] == 0x00 &&
        packet[TERMINATOR_INDEX_3] == 0xff){
          //Now we've got a packet!
          //printf("%i,%i\r\n",chksum8(packet,CHECKSUM_INDEX),packet[CHECKSUM_INDEX]);
          if(chksum8(packet,CHECKSUM_INDEX) != packet[CHECKSUM_INDEX]){
            //printf("Bad checksum\r\n");
            continue;
          }

          for(int i = 0; i < CHANNELS;i++){
            uint16_t current_value = buffer_to_unsigned(packet,i*2);
            //save the value of the scale bit for later use
            uint8_t bit_six = ((current_value >> 14) & 1U);
            //clear bit six in either case
            current_value &= ~(1UL << 14);

            float input_voltage = compute_voltage(current_value);

            if(bit_six){ //x10
              input_voltage /= 10.0;
            }

            // input_voltage -= channel_zero_trim[bit_six][i];

            scaled_channel_values[i] = input_voltage;
            // if(input_voltage > 50.0){
            //   printf("%f,%i,%i\r\n",input_voltage,current_value,bit_six);
            // }
            printf("%i,%f\r\n",bit_six,input_voltage);
          }

          clock_gettime(CLOCK_REALTIME, &timestamp);
          new_timestamp = (timestamp.tv_sec * 1e6) + (timestamp.tv_nsec/1000);
          printf("%i,",new_timestamp-start_time);
          for(int i = 0; i < CHANNELS; i++){
            printf("%f,",scaled_channel_values[i]);
          }
          printf("\n");
          cycle_count ++;
          // printf("\r\nus per sample: %f\r\n", (new_timestamp-start_time)/(float)cycle_count);

    }
  }
}
