#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>

#define MEM_SIZE 64

#define CPU_DATA_LEN_IN_BITS 32
#define UNIFIED_CACHE_BLOCK_SIZE_IN_BITS 4*8
#define UNIFIED_CACHE_PACKET_TYPE_WIDTH 4
#define UNIFIED_CACHE_PACKET_BYTE_MASK_LENGTH 4
#define UNIFIED_CACHE_PACKET_PORT_ID_WIDTH 5

#define UNIFIED_CACHE_PACKET_ADDR_POS_LO 0
#define UNIFIED_CACHE_PACKET_ADDR_POS_HI        (UNIFIED_CACHE_PACKET_ADDR_POS_LO  + CPU_DATA_LEN_IN_BITS - 1)

#define UNIFIED_CACHE_PACKET_DATA_POS_LO        (UNIFIED_CACHE_PACKET_ADDR_POS_HI  + 1)
#define UNIFIED_CACHE_PACKET_DATA_POS_HI        (UNIFIED_CACHE_PACKET_DATA_POS_LO  + UNIFIED_CACHE_BLOCK_SIZE_IN_BITS - 1)

#define UNIFIED_CACHE_PACKET_TYPE_POS_LO        (UNIFIED_CACHE_PACKET_DATA_POS_HI  + 1)
#define UNIFIED_CACHE_PACKET_TYPE_POS_HI        (UNIFIED_CACHE_PACKET_TYPE_POS_LO + UNIFIED_CACHE_PACKET_TYPE_WIDTH - 1)

#define UNIFIED_CACHE_PACKET_BYTE_MASK_POS_LO   (UNIFIED_CACHE_PACKET_TYPE_POS_HI + 1)
#define UNIFIED_CACHE_PACKET_BYTE_MASK_POS_HI   (UNIFIED_CACHE_PACKET_BYTE_MASK_POS_LO + UNIFIED_CACHE_PACKET_BYTE_MASK_LENGTH - 1)

#define UNIFIED_CACHE_PACKET_PORT_NUM_LO        (UNIFIED_CACHE_PACKET_BYTE_MASK_POS_HI + 1)
#define UNIFIED_CACHE_PACKET_PORT_NUM_HI        (UNIFIED_CACHE_PACKET_PORT_NUM_LO  + UNIFIED_CACHE_PACKET_PORT_ID_WIDTH - 1)

#define UNIFIED_CACHE_PACKET_VALID_POS          (UNIFIED_CACHE_PACKET_PORT_NUM_HI  + 1)
#define UNIFIED_CACHE_PACKET_IS_WRITE_POS       (UNIFIED_CACHE_PACKET_VALID_POS  + 1)
#define UNIFIED_CACHE_PACKET_CACHEABLE_POS      (UNIFIED_CACHE_PACKET_IS_WRITE_POS + 1)

#define UNIFIED_CACHE_PACKET_WIDTH_IN_BITS      (UNIFIED_CACHE_PACKET_CACHEABLE_POS - UNIFIED_CACHE_PACKET_ADDR_POS_LO + 1)

char mem[MEM_SIZE][UNIFIED_CACHE_BLOCK_SIZE_IN_BITS + 1];

void create_sim_main_memory(char* path);
void create_request_read(char* path_request, char* path_correct, int *addresses, int port);
void create_request_write(char *path_request_write, char *path_correct, int *addresses, int port);
int itoa_bin(unsigned int data, int length, char *str);

main()
{
    int i, j,k ,index;
    char *temp; 
    
    int addresses[MEM_SIZE];
    int random_addresses_1[MEM_SIZE / 2] = {0};
    //random_addresses

   for (i = 0; i < MEM_SIZE; i++)
    {
        addresses[i] = i;
    }

    srand((unsigned int) time(NULL));
    for (i = 0; i < MEM_SIZE; i++)
    {
        index = (int)((63.0 - i)*rand()/(RAND_MAX+1.0));
	    
	    
	    temp = addresses[i + index];
	    addresses[i + index] = addresses[i];	
	//printf("%d\n", i + index);    

	    if(i < MEM_SIZE / 2)
	    {
	        random_addresses_1[i] = temp;
	    }
	    /*else
	    {
	        random_addresses_2[i - MEM_SIZE / 2] = temp;
	    }*/

    }    
  

    
    create_sim_main_memory("sim_main_mem");
    

    printf("case_00\n"); 
    create_request_read("case_00/way1_request_pool", "case_00/way1_correct_result_mem", random_addresses_1, 0); 
    printf("case_01\n"); 
    create_request_write("case_01/way1_request_pool", "case_01/way1_correct_result_mem", random_addresses_1, 0);
     
    
    char buf[100];
    itoa_bin(0x1234, 64, buf);
    printf("%s\n", buf);
}

int itoa_bin(unsigned int data, int length, char *str)
{

    char *low, *high, temp;    
    int i = 0;
    
	if(str == NULL)
		return -1;
	
	char *start = str;

	while(data)
	{
		if(data & 0x1)
			*str++ = 0x31;
		else
			*str++ = 0x30;

		data >>= 1;
		i ++;
	}

    for (; i < length; i++)
    {
	    *str++ = 0x30;
	}
	*str = 0;
	
	//reverse the order
	low = start, high = str - 1;
	
	while(low < high)
	{
		temp = *low;
		*low = *high;
		*high = temp;

		++low; 
		--high;
	}

	return 0;
}


void create_sim_main_memory(char *path)
{
    int i, j, k = 1;
    int fd = open(path,O_WRONLY|O_CREAT|O_TRUNC,0644);
    char buffer[UNIFIED_CACHE_BLOCK_SIZE_IN_BITS + 1];
    if(fd < 0){
        perror("network open ");
        exit(1);
    }
    
    for (i = 0; i < MEM_SIZE; i++)
    {
        for (j = 0; j < UNIFIED_CACHE_BLOCK_SIZE_IN_BITS; j++)
        {
           if (rand() % 2 == 1)
           {
                buffer[j] = 49;
                mem[i][j] = '1';
           }
            else
            {
                buffer[j] = 48;
                mem[i][j] = '0';
            }
        }

        k =~ k;
        buffer[UNIFIED_CACHE_BLOCK_SIZE_IN_BITS] = 0;
        mem[i][UNIFIED_CACHE_BLOCK_SIZE_IN_BITS] = 0;

        write(fd, buffer, UNIFIED_CACHE_BLOCK_SIZE_IN_BITS);
	write(fd, "\n", 1);
        
    }

    close(fd);
}

void create_request_write(char *path_request_write, char *path_correct, int *addresses, int port)
{
    int i, j, k = 1;
    
    int fd_request_write = open(path_request_write, O_WRONLY|O_CREAT|O_TRUNC, 0644);
    int fd_correct = open(path_correct,O_WRONLY|O_CREAT|O_TRUNC,0644);

    char str[32];
    int address;
    long write_content = 0xffffffff;
    
    if(fd_request_write < 0){
        perror("network open ");
        exit(1);
    }

    for (i = 0; i < MEM_SIZE / 4; i ++)
	{
	
            

      	/*cacheable*/write(fd_request_write, "0", 1); 

      	/*write*/write(fd_request_write, "1", 1);
        /*valid*/write(fd_request_write, "1", 1);

        /*port num*/itoa_bin(port, UNIFIED_CACHE_PACKET_PORT_ID_WIDTH, str);
        write(fd_request_write, str, UNIFIED_CACHE_PACKET_PORT_ID_WIDTH);

        /*byte mask*/itoa_bin(0, UNIFIED_CACHE_PACKET_BYTE_MASK_LENGTH, str);
        write(fd_request_write, str, UNIFIED_CACHE_PACKET_BYTE_MASK_LENGTH);
	
     	/*type*/itoa_bin(0, UNIFIED_CACHE_PACKET_TYPE_WIDTH, str);
        write(fd_request_write, str, UNIFIED_CACHE_PACKET_TYPE_WIDTH);

     	/*data*/itoa_bin(write_content, UNIFIED_CACHE_BLOCK_SIZE_IN_BITS, str);
     	write(fd_request_write, str, UNIFIED_CACHE_BLOCK_SIZE_IN_BITS);

        /*addr*/itoa_bin(addresses[i] << 2, CPU_DATA_LEN_IN_BITS, str);
        write(fd_request_write, str, CPU_DATA_LEN_IN_BITS);

	    itoa_bin(write_content--, CPU_DATA_LEN_IN_BITS, str);	    

        write(fd_request_write, "\n", 1);
	    write(fd_correct, str, UNIFIED_CACHE_BLOCK_SIZE_IN_BITS);
	    write(fd_correct, "\n", 1);
        printf("1---%s\n", str);
	    
	    
	    /*cacheable*/write(fd_request_write, "0", 1); 

      	/*write*/write(fd_request_write, "0", 1);
        /*valid*/write(fd_request_write, "1", 1);

        /*port num*/itoa_bin(port, UNIFIED_CACHE_PACKET_PORT_ID_WIDTH, str);
        write(fd_request_write, str, UNIFIED_CACHE_PACKET_PORT_ID_WIDTH);

        /*byte mask*/itoa_bin(0, UNIFIED_CACHE_PACKET_BYTE_MASK_LENGTH, str);
        write(fd_request_write, str, UNIFIED_CACHE_PACKET_BYTE_MASK_LENGTH);
	
     	/*type*/itoa_bin(0, UNIFIED_CACHE_PACKET_TYPE_WIDTH, str);
        write(fd_request_write, str, UNIFIED_CACHE_PACKET_TYPE_WIDTH);

     	/*data*/itoa_bin(0, UNIFIED_CACHE_BLOCK_SIZE_IN_BITS, str);
     	write(fd_request_write, str, UNIFIED_CACHE_BLOCK_SIZE_IN_BITS);

        /*addr*/itoa_bin(addresses[i] << 2, CPU_DATA_LEN_IN_BITS, str);
        write(fd_request_write, str, CPU_DATA_LEN_IN_BITS);
        
        write(fd_request_write, "\n", 1);
	}
	
	for (i = 0; i < MEM_SIZE / 4; i ++)
	{
	    itoa_bin(0, UNIFIED_CACHE_BLOCK_SIZE_IN_BITS, str);

        write(fd_correct, str, UNIFIED_CACHE_BLOCK_SIZE_IN_BITS);
        
        write(fd_correct, "\n", 1);
        printf("2---%s\n", str);
	}
	
	
    close(fd_request_write);
    close(fd_correct);
}

void create_request_read(char *path_request, char *path_correct, int *addresses, int port)
{
    int i, j, k = 1;
    
    int fd_request = open(path_request,O_WRONLY|O_CREAT|O_TRUNC,0644);
    int fd_correct = open(path_correct,O_WRONLY|O_CREAT|O_TRUNC,0644);
    
    char buffer[UNIFIED_CACHE_PACKET_WIDTH_IN_BITS + 1];
    char str[32];
    int address;
    long write_content = 0;
    
    if(fd_request < 0 | fd_correct < 0){
        perror("network open ");
        exit(1);
    }

    
	for (i = 0; i < MEM_SIZE / 2; i ++)
	{
        for (j = 0; j < UNIFIED_CACHE_PACKET_WIDTH_IN_BITS + 1; j ++)
        {
            buffer[j] = 0;
        }
	
            
	    //requests
      	/*cacheable*/write(fd_request, "0", 1);

	/*write*/write(fd_request, "0", 1); 

	/*valid*/write(fd_request, "1", 1);

        /*port num*/itoa_bin(port, UNIFIED_CACHE_PACKET_PORT_ID_WIDTH, str);
	write(fd_request, str, UNIFIED_CACHE_PACKET_PORT_ID_WIDTH);

    	/*byte mask*/itoa_bin(0, UNIFIED_CACHE_PACKET_BYTE_MASK_LENGTH, str);
        write(fd_request, str, UNIFIED_CACHE_PACKET_BYTE_MASK_LENGTH);

     	/*type*/itoa_bin(0, UNIFIED_CACHE_PACKET_TYPE_WIDTH, str);
        write(fd_request, str, UNIFIED_CACHE_PACKET_TYPE_WIDTH);
     	/*data*/itoa_bin(write_content, UNIFIED_CACHE_BLOCK_SIZE_IN_BITS, str);
     	write(fd_request, str, UNIFIED_CACHE_BLOCK_SIZE_IN_BITS);

        /*addr*/itoa_bin(addresses[i] << 2, CPU_DATA_LEN_IN_BITS, str);
	write(fd_request, str, CPU_DATA_LEN_IN_BITS);

	write(fd_correct, mem[addresses[i]], UNIFIED_CACHE_BLOCK_SIZE_IN_BITS);

        write(fd_correct, "\n", 1);
        write(fd_request, "\n", 1);
	    //printf("%s\n", mem[addresses[i]]);
	}
	

	
    close(fd_request);
    close(fd_correct);

}
