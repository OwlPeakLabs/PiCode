 #include <linux/i2c-dev.h>
/* Before include, make sure you have BITS_PER_LONG defined. This is a CPU
 * specific value which is used to generate bit masks. You can also use -D
 * to input definition to compiler via command line
 */
#define BITS_PER_LONG 64 
#include "mlx90632.h"
/* Declare and implement here functions you find in mlx90632_depends.h */

/* You can use global or local storage for EEPROM register values so declare
 * them whereever you want. Do not forget to declare ambient_new_raw,
 * ambient_old_raw, object_new_raw, object_old_raw
 * 
 */
#include <bcm2835.h>
#include <pthread.h>
#include <stdio.h>
#include <i2c/smbus.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdint.h>

#define MLX90632_ADDR 0x3A
#define TCA9548A_ADDR 0x70

int i2c_fd;
uint32_t P_R, P_G, P_T, P_O;
int32_t Ea, Eb, Ga, Fa, Fb, Ha, Hb; 
int16_t Ka, Gb;

// Raw temperature variables
int16_t ambient_new_raw, ambient_old_raw;
int16_t object_new_raw, object_old_raw;

//Registers
#define EE_VERSION 0x240B

//32 bit constants
#define EE_P_R 0x240C
#define EE_P_G 0x240E
#define EE_P_T 0x2410
#define EE_P_O 0x2412
#define EE_Aa 0x2414
#define EE_Ab 0x2416
#define EE_Ba 0x2418
#define EE_Bb 0x241A
#define EE_Ca 0x241C
#define EE_Cb 0x241E
#define EE_Da 0x2420
#define EE_Db 0x2422
#define EE_Ea 0x2424
#define EE_Eb 0x2426
#define EE_Fa 0x2428
#define EE_Fb 0x242A
#define EE_Ga 0x242C

//16 bit constants
#define EE_Ha 0x2481
#define EE_Hb 0x2482
#define EE_Gb 0x242E
#define EE_Ka 0x242F
#define EE_Kb 0x2430

//Control registers
#define EE_CONTROL 0x24D4
#define EE_I2C_ADDRESS 0x24D5
#define REG_I2C_ADDRESS 0x3000
#define REG_CONTROL 0x3001
#define REG_STATUS 0x3FFF

//User RAM
#define RAM_1 0x4000
#define RAM_2 0x4001
#define RAM_3 0x4002
#define RAM_4 0x4003
#define RAM_5 0x4004
#define RAM_6 0x4005
#define RAM_7 0x4006
#define RAM_8 0x4007
#define RAM_9 0x4008

//Three measurement modes available
#define MODE_SLEEP 0b01
#define MODE_STEP 0b10
#define MODE_CONTINUOUS 0b11

//REG_STATUS bits
#define BIT_DEVICE_BUSY 10
#define BIT_EEPROM_BUSY 9
#define BIT_BROWN_OUT 8
#define BIT_CYCLE_POS 2 //6:2 = 5 bits
#define BIT_NEW_DATA 0

//REG_CONTROL bits
#define BIT_SOC 3
#define BIT_MODE 1 //2:1 = 2 bits

#define I2C_SPEED_STANDARD        100000
#define I2C_SPEED_FAST            400000

#define SENSOR_SUCCESS 0
#define SENSOR_ID_ERROR 1
#define SENSOR_I2C_ERROR 2
#define SENSOR_INTERNAL_ERROR 3
#define SENSOR_GENERIC_ERROR 4
#define SENSOR_TIMEOUT_ERROR 5

pthread_mutex_t i2c_mutex = PTHREAD_MUTEX_INITIALIZER;


typedef struct {
    pthread_t tid;
    int i2cHandle;
} ThreadHandle;

ThreadHandle threadHandles[8];
int is_channel_taken[8] = {0};

// Declare an array to store the I2C handles for each channel
int channelHandles[8];

#define TCA_CHANNEL_0              (0x00)   ///< TCA channel0 selection
#define TCA_CHANNEL_1              (0x01)   ///< TCA channel1 selection
#define TCA_CHANNEL_2              (0x02)   ///< TCA channel2 selection
#define TCA_CHANNEL_3              (0x04)   ///< TCA channel3 selection
#define TCA_CHANNEL_4              (0x08)   ///< TCA channel4 selection
#define TCA_CHANNEL_5              (0x10)   ///< TCA channel5 selection
#define TCA_CHANNEL_6              (0x20)   ///< TCA channel6 selection
#define TCA_CHANNEL_7              (0x40)   ///< TCA channel7 selection
#define TCA_CHANNEL_8              (0x80)   ///< TCA channel8 selection



// Function prototypes
void* thread_func(void* arg);


// Replace i2cOpen with this
int open_i2c_bus(int bus, int addr) {
    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", bus);
    int fd = open(filename, O_RDWR);
    if (fd < 0) {
        // Handle error
        return -1;
    }
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        // Handle error
        close(fd);
        return -1;
    }
    return fd;
}

// Replace i2cWriteDevice with this
ssize_t write_i2c_data(int fd, const uint8_t *data, size_t length) {
    return write(fd, data, length);
}

// Replace i2c_smbus_open with this
int open_i2c_smbus(int bus) {
    return open_i2c_bus(bus, MLX90632_ADDR);  // Replace MLX90632_ADDR with the actual device address
}

uint16_t swap_endian_16(uint16_t value) {
    return (value << 8) | (value >> 8);
}

int32_t mlx90632_i2c_read(int16_t register_address, uint16_t *value) {
    /*
    pthread_mutex_lock(&i2c_mutex);
    pthread_t this_thread = pthread_self();
    
    int i2cHandle = -1;

    // Find the handle for this thread
    for (int i = 0; i < 8; ++i) {
        if (pthread_equal(this_thread, threadHandles[i].tid)) {
            i2cHandle = threadHandles[i].i2cHandle;
            break;
        }
    }
	
	printf("read thread handle: %d\n",i2cHandle);
	
    if (i2cHandle == -1) {
        pthread_mutex_unlock(&i2c_mutex);
        return -1; // Handle not found
    }
    */
    uint8_t reg[2];
    reg[0] = (register_address >> 8) & 0xFF;
    reg[1] = register_address & 0xFF;
	
	    // Open I2C connection to /dev/i2c-22 with slave address 0x3A
    int i2cHandle22 = open_i2c_bus(23, 0x3A); // Your existing open_i2c_bus function should work here
    if (i2cHandle22 < 0) {
        fprintf(stderr, "Failed to open I2C device on /dev/i2c-22: %d\n", i2cHandle22);
        return 1;
    }
	
    // Write the bytes one by one
    if (write(i2cHandle22, &reg[0], 1) != 1 || write(i2cHandle22, &reg[1], 1) != 1) {
        perror("Failed to write to the i2c bus");
        //pthread_mutex_unlock(&i2c_mutex);
        return -1;
    }

    usleep(10000);

    // Read the bytes into reg
    if (read(i2cHandle22, &reg, 2) != 2) {
        perror("Failed to read from the i2c bus");
        //pthread_mutex_unlock(&i2c_mutex);
        return -1;
    }

    *value = (reg[1] << 8) | reg[0];    
    *value = swap_endian_16(*value);
    //pthread_mutex_unlock(&i2c_mutex);
        // Close the I2C handle
    //close_i2c_bus(i2cHandle22);
    return 0;
}

int16_t mlx90632_i2c_write(int16_t register_address, uint16_t value) {
    pthread_mutex_lock(&i2c_mutex);
    pthread_t this_thread = pthread_self();
    
    int i2cHandle = -1;

    // Find the handle for this thread
    for (int i = 0; i < 8; ++i) {
        if (pthread_equal(this_thread, threadHandles[i].tid)) {
            i2cHandle = threadHandles[i].i2cHandle;
            break;
        }
    }

    if (i2cHandle == -1) {
        pthread_mutex_unlock(&i2c_mutex);
        return -1; // Handle not found
    }
    
    uint8_t reg[4];

    // Prepare register address
    reg[0] = (register_address >> 8) & 0xFF;
    reg[1] = register_address & 0xFF;
	
    // Prepare value to write (no endianness swap)
    reg[2] = (value >> 8) & 0xFF;
    reg[3] = value & 0xFF;

    // Write the bytes one by one
    if (write(i2cHandle, &reg[0], 1) != 1 || write(i2cHandle, &reg[1], 1) != 1 ||
        write(i2cHandle, &reg[2], 1) != 1 || write(i2cHandle, &reg[3], 1) != 1) {
        perror("Failed to write to the i2c bus");
        pthread_mutex_unlock(&i2c_mutex);
        return -1;
    }

    pthread_mutex_unlock(&i2c_mutex);
    return 0;
}



void msleep(int msecs) {
    usleep(msecs * 1000);
}

void custom_usleep(int min_range, int max_range) {
    // Sleeping for min_range microseconds
    // Implement more complex logic if needed
    usleep(min_range);
}

/*
void select_channel(int channel) {
	
	pthread_mutex_lock(&i2c_mutex);
	if (is_channel_taken[channel]) {
        pthread_mutex_unlock(&i2c_mutex);
        return;
    }
    is_channel_taken[channel] = 1;
	
    if (channel < 0 || channel > 7) {
        printf("Channel out of range (0-7)\n");
        return;
    }

    uint8_t channel_data;

    // Map channel number to predefined channel data
    switch(channel) {
        case 0: channel_data = TCA_CHANNEL_0; break;
        case 1: channel_data = TCA_CHANNEL_1; break;
        case 2: channel_data = TCA_CHANNEL_2; break;
        case 3: channel_data = TCA_CHANNEL_3; break;
        case 4: channel_data = TCA_CHANNEL_4; break;
        case 5: channel_data = TCA_CHANNEL_5; break;
        case 6: channel_data = TCA_CHANNEL_6; break;
        case 7: channel_data = TCA_CHANNEL_7; break;
        default: printf("Invalid channel number\n"); return;
    }
    
    printf("Channel data: 0x%x, Address of channel_data: %p\n", channel_data, &channel_data); // Print the address of channel_data

    ssize_t writeStatus = write_i2c_data(TCA9548A_ADDR, &channel_data, 1);
    usleep(100000);
    
    if(writeStatus != 1) {
        printf("TCA9548A write failed!\n");
        return;
    }
    if (writeStatus < 0) {
        printf("i2cWriteDevice returned %d while trying to select channel.\n", writeStatus);
    }

    pthread_mutex_unlock(&i2c_mutex);
}
*/

// EEPROM busy check
bool mlx90632_is_eeprom_busy() {
  uint16_t status;
  if (mlx90632_i2c_read(REG_STATUS, &status) != 0) {
    // Handle read error
    return false; 
  }
  return (status & (1 << BIT_EEPROM_BUSY)) != 0;
}


int32_t mlx90632_set_mode(uint8_t mode)
{
    uint16_t reg;
    int32_t ret = mlx90632_i2c_read(REG_CONTROL, &reg); // Get current bits
    if (ret < 0)
    {
        return ret; // Return the error code
    }

    reg &= ~(0x0003 << 1); // Clear the mode bits
    reg |= (mode << 1);   // Set the bits
    ret = mlx90632_i2c_write(REG_CONTROL, reg); // Set the mode bits
	
    return ret; // Return status (0 if successful, error code otherwise)
}


bool mlx90632_begin() {
  
  int32_t returnError = SENSOR_SUCCESS;
  
  
  //Wait for eeprom_busy to clear
  /*
  uint16_t counter = 0;
  while (mlx90632_is_eeprom_busy())
  {
    usleep(100000);
    counter++;
    if (counter == 750)
    {
      returnError = SENSOR_TIMEOUT_ERROR;
      return (false); //Error
    }
  }
  */
  
  /*
  int res = mlx90632_set_mode(MODE_SLEEP);
  
  if(res == 0)
	printf("Mode SET");
  */
  	
  uint16_t thisVersion;
  
  returnError = mlx90632_i2c_read(EE_I2C_ADDRESS, &thisVersion);
  
  
  if (returnError != SENSOR_SUCCESS) {
	printf("Failed to read sensor \n");
	return false;
  }

  printf("Successfully read. Value: 0x%04X\n", thisVersion);
  printf("_device. Value: 0x%04X\n", 0x3A >> 1);
  return true;
 
}

void* thread_func(void* arg) {
    int channel = *(int*)arg;
    printf("Thread for channel %d started. Thread ID: %lu\n", channel, (unsigned long)pthread_self());

    // Store the thread ID and initialize the i2cHandle to -1
    threadHandles[channel].tid = pthread_self();
    threadHandles[channel].i2cHandle = -1;

    // Select the channel
    //select_channel(channel);
	
	
    // Initialize the I2C handle for this channel after selecting it
    int handle = open_i2c_bus(1,0x3A);
    if (handle < 0) {
        fprintf(stderr, "Failed to open I2C device for channel %d: %d\n", channel, handle);
        return NULL;
    } else {
        printf("Successfully opened I2C device for channel %d with handle %d\n", channel, handle);
    }

    // Store the handle
    threadHandles[channel].i2cHandle = handle;
    channelHandles[channel] = handle; // You may still keep this if needed elsewhere

    // Perform calibration
    mlx90632_begin(); 
	
    return NULL;
}
int main(void) {
    pthread_t threads[8];
    int channel_ids[8];
    /*
    // Initialize threadHandles to known state
	for (int i = 0; i < 8; ++i) {
		threadHandles[i].tid = pthread_self();  // Current thread ID (main thread)
		threadHandles[i].i2cHandle = -1;
	}

    // Create threads for each channel
    for (int channel = 0; channel < 8; ++channel) {
        channel_ids[channel] = channel;
        if (pthread_create(&threads[channel], NULL, thread_func, &channel_ids[channel]) != 0) {
            fprintf(stderr, "Failed to create thread for channel %d\n", channel);
            return 1;
        }
    }

    // Wait for all threads to complete
    for (int channel = 0; channel < 8; ++channel) {
        pthread_join(threads[channel], NULL);
    }
    */
    


    // Read the EEPROM version register
    uint16_t eeprom_version;
    int32_t readStatus = mlx90632_i2c_read(REG_STATUS, &eeprom_version); // Your existing mlx90632_i2c_read function should work here
    if (readStatus < 0) {
        fprintf(stderr, "Failed to read EEPROM version: %d\n", readStatus);
        return 1;
    }

    // Print the result
    printf("EEPROM Version: 0x%04X\n", eeprom_version);



    
    return 0;
}
