#include "verilated.h"          // Defines common routines
#include <iostream>             // Need std::cout
#include "VTLSimpleL2Cache.h"               // From Verilating "top.v"
#define TOP VTLSimpleL2Cache
TOP* top;                     // Instantiation of module
vluint64_t main_time = 0;       // Current simulation time
#define RESET_TIME 10000
int state = 0;
#define TEST_END 70000
#define TLMESSAGE_GET 0x4
int readCntFromMemToCache = -1;
//0 <--> nop; 1 <--> request read addr; 2 <--> cache reply 1; 3 <--> cache reply data; 6 <--> cpu reply 3
//4 <--> cache request read data from mem(8 beats total); 5 <--> mem reply 4 to cache;

void printInfo(const char* str) {
	cout << "-------" << str << endl;
	cout << "main_time: " << main_time << endl;
	printf("DEBUG state %x\n", state);
	// printf("DEBUG auto_out_a_valid %x\n", top->auto_out_a_valid);
	// printf("DEBUG auto_out_a_ready %x\n", top->auto_out_a_ready); 
	// printf("DEBUG auto_out_a_bits_opcode %x\n", top->auto_out_a_bits_opcode); 
	//printf("DEBUG TLSimpleL2Cache__DOT___T_267 %x\n", top->TLSimpleL2Cache__DOT___T_267); //T_267 is the state machine
	//printf("io_deq_bits: %d\n", top->io_deq_bits);
	//printf("io_deq_ready: %d\n", top->io_deq_ready);
	//printf("io_deq_valid: %d\n", top->io_deq_valid);
}

void printOutA() {
	cout << "-------cache request read data from mem:" << endl;
	cout << "main_time: " << main_time << endl;
	printf("out.a.opcode: %x\n", top->auto_out_a_bits_opcode);
	printf("out.a.address: %lx\n", top->auto_out_a_bits_address);
	printf("out.a.data: %x\n", top->auto_out_a_bits_data);
}

void printInD() {
	cout << "-------cache reply read data to processor:" << endl;
	cout << "main_time: " << main_time << endl;
	printf("in.d.opcode: %x\n", top->auto_in_d_bits_opcode);
	printf("in.d.size: %x\n", top->auto_in_d_bits_size);
	printf("in.d.source: %x\n", top->auto_in_d_bits_source);
	printf("in.d.data: %x\n", top->auto_in_d_bits_data);
}

double sc_time_stamp () {       // Called by $time in Verilog
	return main_time;           // converts to double, to match
								// what SystemC does
}

int main(int argc, char** argv) {
	Verilated::commandArgs(argc, argv);   // Remember args
	top = new TOP;             // Create instance
	top->reset = 1;           
	//TODO: need handling write miss

	while (!Verilated::gotFinish()) {
		if (main_time == RESET_TIME + 1) {
			top->reset = 0;
			state = 0;
		}
		if ((main_time % 10) == 1) {
			top->clock = 1;       // Toggle clock
		}
		else if ((main_time % 10) == 6) {
			top->clock = 0;
		}
		
		if ((main_time > RESET_TIME) && (main_time % 10 == 1)) {
			if (main_time >= 60971 && main_time <= 60991) {
				printInfo("");
			}

			//request1: read miss
			if (main_time == RESET_TIME + 1) { //init
				top->auto_in_a_valid = 0x0;
				top->auto_in_a_bits_opcode = TLMESSAGE_GET;
				top->auto_in_a_bits_dsid = 0x01;
				top->auto_in_a_bits_param = 0x0;
				top->auto_in_a_bits_size = 0x6;
				top->auto_in_a_bits_source = 0x010;
				top->auto_in_a_bits_address = 0x100800000;
				top->auto_in_a_bits_mask = 0xff;
				top->auto_in_a_bits_data = 0x0000000000000000;

				top->auto_out_a_ready = 0x1;
			}
			else if (main_time == RESET_TIME + 10 + 1) {
				state = 1;
				top->auto_in_a_valid = 0x1; //weird problem happens if put valid in previous cycle
				printInfo("cpu: cpu(read req) ---> cache");
				printf("in.a.addr: %lx\n", top->auto_in_a_bits_address);
				printf("in.a.size: %x\n", top->auto_in_a_bits_size);
				printf("in.a.mask: %x\n", top->auto_in_a_bits_mask);
			}
			else if (state == 1 && top->auto_in_a_ready == 0x1) {
				printInfo("cache: cpu ---> cache(read req)");
				state = 2;
				top->auto_in_a_valid = 0x0;
			}

			else if (state == 2 && (top->auto_out_a_valid&top->auto_out_a_ready == 0x1) && top->auto_out_a_bits_opcode == TLMESSAGE_GET) {
				printInfo("mem: cache ---> mem(read req1)");
				state = 4;
				top->auto_out_a_ready = 0x0;
				readCntFromMemToCache = 0;
				printOutA();
			}

			else if (state == 4) {
				state = 5;
				top->auto_out_d_valid = 0x1;
				top->auto_out_d_bits_data = 0xdeadbeef2040406f + readCntFromMemToCache; //TODO: mem array
				printInfo("mem: cache <--- mem(data beat)");
				printf("out.d.data: %lx\n", top->auto_out_d_bits_data);
			}
			else if (state == 5 && top->auto_out_d_ready == 0x1) {
				readCntFromMemToCache++;
				printInfo("cache: cache(data beat) <--- mem");
				printf("readCntFromMemToCache: %lx\n", readCntFromMemToCache);
				if (readCntFromMemToCache == 4) {
					state = 2;
					top->auto_out_d_valid = 0x0;
					top->auto_out_a_ready = 0x1;
					printInfo("cache: has recvd all data beats from mem");
				} else 
					state = 4;
			}

			else if (state == 2 && (top->auto_in_d_valid&top->auto_in_d_ready == 0x1)) {
				printInfo("cache: cpu <--- cache(read req)");
				printInD();
				state = 6;
			}

			else if (state == 6) {
				printInfo("cpu: cpu(read req) <--- cache");
				top->auto_in_d_ready = 0x1;
				state = 0;
			}
			else {
				//printInfo("");
			}
		}

		top->eval();            // Evaluate model
		main_time++;            // Time passes...

		if (main_time > TEST_END) {
			printf("simulator: Test end.\n");
			break;
		}
	}
	top->final();               // Done simulating
	//    // (Though this example doesn't get here)
	delete top;
}
