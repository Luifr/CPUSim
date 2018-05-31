#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>


//################################################################################################################################ 
// Bit manipulation




	#define INT sizeof(int)*8

	void SetBit( int* B, int pos){
		B[ pos / INT ] |= 1 << (pos % INT);
	}

	void ClearBit( int* B, int pos){
		B[ pos / INT ] &= ~(1 << (pos % INT));
	}

	int TestBit( int* B, int pos){
		return ((B[ pos / INT ] & (1 << (pos % INT))) != 0 ) ;
	}

	void SetBits( int* B, int pos, int* A, int size){
		for(int i = 0; i < size ; i ++){
			if(A[i] == 1){
				//SetBit(B,i);
				B[ i / INT ] |= 1 << (i % INT);
			}
			else{
				//ClearBit(B,i);
				B[ i / INT ] &= ~(1 << (i % INT));
			}
		}
	}

	int TestBits( int* B, int pos, int size){ 
		int num = 0;
		size += pos;
		for(int i = pos, j = 0; i < size ; i++, j++){
			num |= TestBit(B,i) << j ;
		}
		return num;
	}




// Bit Manipulation
//################################################################################################################################
// Defines




	// Fixed sizes
	#define RAM_SIZE 128

	// all these defines have (int*) as arguments

	// INSTRUCTION GETTERS
	#define OPCODE(instruction_adress) TestBits(instruction_adress,26,6)
	#define RS(instruction_adress) TestBits(instruction_adress,21,5)
	#define RT(instruction_adress) TestBits(instruction_adress,16,5)
	#define RD(instruction_adress) TestBits(instruction_adress,11,5)
	#define IMMEDIATE(instruction_adress) TestBits(instruction_adress,0,16)
	#define ADDRESS(instruction_adress) TestBits(instruction_adress,0,26)
	#define FUNCTION_FIELD(instruction_adress) TestBits(instruction_adress,0,6)
	#define SHAMT(instruction_adress) TestBits(instruction_adress,6,5)

	// CU Read Signals
	#define getRegDst() TestBits(&cu_signals,0,2)
	#define getRegWrite() TestBits(&cu_signals,2,1)
	#define getALUSrcA() TestBits(&cu_signals,3,1)
	#define getALUSrcB() TestBits(&cu_signals,4,2)
	#define getALUOp() TestBits(&cu_signals,6,2)
	#define getPCSource() TestBits(&cu_signals,8,2)
	#define getPCWriteCond() TestBits(&cu_signals,10,1)
	#define getPCWrite() TestBits(&cu_signals,11,1)
	#define getIorD() TestBits(&cu_signals,12,1)
	#define getMemRead() TestBits(&cu_signals,13,1)
	#define getMemWrite() TestBits(&cu_signals,14,1)
	#define getBNE() TestBits(&cu_signals,15,1)
	#define getIRWrite() TestBits(&cu_signals,16,1)
	#define getMemtoReg() TestBits(&cu_signals,17,2)

	// CU Set Signals
	#define setRegDst(vec_num) SetBits(&cu_signals,0,vec_num,2)
	#define setRegWrite(vec_num) SetBits(&cu_signals,2,vec_num,1)
	#define setALUSrcA(vec_num) SetBits(&cu_signals,3,vec_num,1)
	#define setALUSrcB(vec_num) SetBits(&cu_signals,4,vec_num,2)
	#define setALUOp(vec_num) SetBits(&cu_signals,6,vec_num,2)
	#define setPCSource(vec_num) SetBits(&cu_signals,8,vec_num,2)
	#define setPCWriteCond(vec_num) SetBits(&cu_signals,10,vec_num,1)
	#define setPCWrite(vec_num) SetBits(&cu_signals,11,vec_num,1)
	#define setIorD(vec_num) SetBits(&cu_signals,12,vec_num,1)
	#define setMemRead(vec_num) SetBits(&cu_signals,13,vec_num,1)
	#define setMemWrite(vec_num) SetBits(&cu_signals,14,vec_num,1)
	#define setBNE(vec_num) SetBits(&cu_signals,15,vec_num,1)
	#define setIRWrite(vec_num) SetBits(&cu_signals,16,vec_num,1)
	#define setMemtoReg(vec_num) SetBits(&cu_signals,17,vec_num,2)

	// Registers
	#define $zero 0
	#define $at 1
	#define $v0 2
	#define $v1 3
	#define $a0 4
	#define $a1 5
	#define $a2 6
	#define $a3 7
	#define $t0 8
	#define $t1 9
	#define $t2 10
	#define $t3 11
	#define $t4 12
	#define $t5 13
	#define $t6 14
	#define $t7 15
	#define $s0 16
	#define $s1 17
	#define $s2 18
	#define $s3 19
	#define $s4 20
	#define $s5 21
	#define $s6 22
	#define $s7 23
	#define $t8 24
	#define $t9 25
	#define $k0 26
	#define $k1 27
	#define $gp 28
	#define $sp 29
	#define $fp 30
	#define $ra 31


// Defines
//################################################################################################################################
// Global variables




	// semaphores to control threads
	sem_t clock_sem, main_sem, pc_sem, ram_sem, ir_sem, alu_sem, cu_sem, mbr_sem ;
	sem_t regDst_mux_sem, memToReg_mux_sem, iord_mux_sem, a_sem, b_sem, regBank_sem, aluControl_sem;
	sem_t signExtend_sem, aluSrcA_mux_sem, aluSrcB_mux_sem, signExtend_sem, shiftLeftMuxALU_sem;
	sem_t pcSrc_mux_sem, aluOut_sem, bne_mux_sem, and_sem, or_sem, shiftLeftPCSrc_sem;

	// has all the signals from CU
	int cu_signals = 0;

	// connections
	int pc, muxAddressResult, memData, writeData, memDataRegister;
	int instruction_15_0, instruction_20_16, instruction_25_21, instruction_31_26, instruction_15_11, instruction_6_10, instruction_5_0, instruction_25_0;
	int signExtendOut, shiftLeftMuxALU, shiftLeftMuxPCSource;
	int outMuxBNE, andToOr, orToPc, muxToPc, outMuxRegDst, outMuxMemToReg, ALUResult;
	int instruction;
		
	//value of registers
	int a_reg, b_reg, ALUOutResult, ALUA, ALUB, mbr;


// Global variables
//################################################################################################################################
//Threads/Modules

	// CONTROL UNIT ------------------------------------------------------------------------
	void* CU(void* arg){ 
		// initialize this thread before while(1)
		
		while(1){
			sem_wait(&cu_sem);

			sem_post(&and_sem);
		}
	}

	//PROGRAM COUNTER ------------------------------------------------------------------------
	void* PC(void* arg){ 
		// initialize this thread before while(1)
		int pc_local = 0; // current value of pc

		while(1){
			sem_wait(&pc_sem); // it waits for the semaphore to allow it to run
			pc_local += 4; // points to next instruction
			if(orToPc == 1){ // if condition to write in pc
				pc_local = muxToPc; // overwrite pc with adress location
			}
			sem_post(&iord_mux_sem); // start next modules
			sem_wait(&clock_sem);
			pc = pc_local; // update global pc value after clock
		}
	}

	//RANDOM ACCESS MEMORY ----------------------------------------------------------------------
	void* RAM(void* arg){ 
		// initialize this thread before while(1)
		unsigned int ram[RAM_SIZE]; // o conteudo da ram
		int iord; // get the iord signal from cu

		// load program from input file to ram
		FILE* code = fopen((char*)arg,"r");
		int counter = 0;

		memset(ram,0,RAM_SIZE*sizeof(int));

		while(fgetc(code) != EOF && fgetc(code) != EOF){
			fseek(code,-2,SEEK_CUR);
			fscanf(code,"%d",ram+counter);
			counter++;
		}

		while(1){
			sem_wait(&ram_sem);
			if(getMemRead() == 1){
				instruction = ram[muxAddressResult/4];
			}
			else if(getMemWrite() == 1){
				ram[muxAddressResult/4] = b_reg;
			}
			sem_post(&ir_sem);
			sem_post(&mbr_sem);
		}
	}

	//INSTRUCTION REGISTER ------------------------------------------------------------------------
	void* IR(void* arg){ 
		// initialize this thread before while(1)
	
		while(1){
			
			sem_wait(&ir_sem);    
			//sem_wait(&clock_sem); 

			if(getIRWrite() == 1){

				instruction_31_26 = OPCODE(&instruction); 
				instruction_25_21 = RS(&instruction);    
				instruction_20_16 = RT(&instruction);
				instruction_15_11 = RD(&instruction);
				instruction_15_0  = IMMEDIATE(&instruction);
				instruction_25_0  = ADDRESS(&instruction);
				instruction_5_0   = FUNCTION_FIELD(&instruction);
				instruction_6_10  = SHAMT(&instruction);
			}
			sem_post(&memToReg_mux_sem);   //Now, the mux controlled by the UC signal MemToReg is unlocked
			sem_post(&regDst_mux_sem);     //Now, the mux controlled by the UC signal RegDst is unlocked
			sem_post(&regBank_sem);        //...
			sem_post(&aluControl_sem);     //...
			sem_post(&shiftLeftPCSrc_sem); //...
		}
	}

	//MEMORY BUFFER REGISTER ------------------------------------------------------------------------
	void* MBR(void* arg){ 
		// initialize this thread before while(1)
		mbr = 0;

		while(1){
			//verifies if bmr function is allowed to run
			sem_wait(&mbr_sem);
			
			//mbr receives the content of the new instruction execution at this moment
			mbr = instruction;

			//now memToReg mux knows that one of its inputs were already seted
			sem_post(&memToReg_mux_sem);
		}
	}

	//REGISTER BANK ------------------------------------------------------------------------
	void* RegisterBank(void* arg){ //Contains register from 0 to 31
		
		// initialize this thread before while(1)
		int regs[32]; // all registers
		
		memset(regs,0,32*sizeof(int));

		while(1){

			sem_wait(&regBank_sem); //IR function  has to run first
			sem_wait(&regBank_sem); //resDst_mux has to be seted first
			sem_wait(&regBank_sem); //memToReg_mux has to be seted first

			//setting A and B registers:
			a_reg = regs[instruction_25_21]; 
			b_reg = regs[instruction_20_16];

			//Verifies if control unit allows to write on register bank:
			if (getRegWrite() == 1) {
				
			}

			//now that 
			sem_post(&a_sem); 
			sem_post(&b_sem);
			

		}
	}

	void* ALU(void* arg){ // Arithmetic Logic Unit
		// initialize this thread before while(1)

		while(1){
			sem_wait(&alu_sem);
			sem_wait(&alu_sem);
			sem_wait(&alu_sem);

			sem_post(&aluOut_sem);
		}
	}

	void* MuxIorD(void* arg){
		// initialize this thread before while(1)

		while(1){
			sem_wait(&iord_mux_sem);
			if (getIorD() == 0) {
				muxAddressResult = pc;
			} 
			else if (getIorD() == 1) {
				muxAddressResult = ALUOutResult;
			}
			sem_post(&ram_sem);
		}
	}


	void* MuxALUA(void* arg){
		// initialize this thread before while(1)

		while(1){
			sem_wait(&aluSrcA_mux_sem);
			if (getALUSrcA() == 0) {
				ALUA = pc;
			}
			else if (getALUSrcA() == 1) {
				ALUA = a_reg;
			}
			sem_post(&alu_sem);
		}
	}

	void* MuxALUB(void* arg){
		// initialize this thread before while(1)

		while(1){
			sem_wait(&aluSrcB_mux_sem);
			sem_wait(&aluSrcB_mux_sem);
			if (getALUSrcB() == 0) {
				ALUB = b_reg;
			}
			else if (getALUSrcB() == 1) {
				ALUB = 4;
			}
			else if (getALUSrcB() == 2) {
				ALUB = signExtendOut;
			}
			else if (getALUSrcB() == 3) { 
				ALUB = shiftLeftMuxALU;
			}
			sem_post(&alu_sem);
		}
	}

	void* MuxBNE(void* arg){
		// initialize this thread before while(1)

		while(1){
			sem_wait(&bne_mux_sem);
			if (getBNE() == 0) {
				outMuxBNE = 0;
			} 
			else if (getBNE() == 1) {
				outMuxBNE = 1;
			}
			sem_post(&cu_sem);
		}
	}

	void* MuxPCSource(void* arg){
		// initialize this thread before while(1)

		while(1){
			sem_wait(&pcSrc_mux_sem);
			sem_wait(&pcSrc_mux_sem);
			if (getPCSource() == 0) {
				muxToPc = ALUResult;
			}
			else if (getPCSource() == 1) {
				muxToPc = ALUOutResult;
			}
			else if (getPCSource() == 2) {
				muxToPc = shiftLeftMuxPCSource;
			}
			else if (getPCSource() == 3) {
				muxToPc = a_reg;
			}
			sem_wait(&bne_mux_sem);
		}
	}

	void* ALUControl(void* arg){
		// initialize this thread before while(1)
		while(1){
		sem_wait(&aluControl_sem);
			
		sem_post(&alu_sem);
		}
	}

	void* MuxRegDst(void* arg){
		// initialize this thread before while(1)

		while(1){
			sem_wait(&regDst_mux_sem);
			if (getRegDst() == 0) {
				outMuxRegDst = instruction_20_16;
			}
			else if (getRegDst() == 1) {
				outMuxRegDst = instruction_15_11;
			}
			else if (getRegDst() == 2) {
				outMuxRegDst = 31;
			}
			sem_post(&regBank_sem);
		}
	}

	void* MuxMemtoReg(void* arg){
		// initialize this thread before while(1)

		while(1){
			sem_wait(&memToReg_mux_sem);
			sem_wait(&memToReg_mux_sem);
			if (getMemtoReg() == 0) {
				sem_wait(&clock_sem);
				outMuxMemToReg = ALUOutResult;
			} else if (getMemtoReg() == 1) {
				sem_wait(&clock_sem);
				//outMuxMemToReg = mdr;
			} else if (getMemtoReg() == 2) {
				sem_wait(&clock_sem);
				outMuxMemToReg = pc;
			}
			sem_post(&regBank_sem);
		}
	}

	void* A(void* arg){
		// initialize this thread before while(1)

		while(1){
			sem_wait(&a_sem);

			sem_post(&aluSrcA_mux_sem);
		}
	}

	void* B(void* arg){
		// initialize this thread before while(1)

		while(1){
			sem_wait(&b_sem);

			sem_post(&aluSrcB_mux_sem);
		}
	}

	void* ALUOut(void* arg){
		// initialize this thread before while(1)

		while(1){
			sem_wait(&aluOut_sem);

			sem_post(&pcSrc_mux_sem);
		}
	}

	void* SignExtend(void* arg){
		// initialize this thread before while(1)

		while(1){
			sem_wait(&signExtend_sem);

			sem_post(&shiftLeftMuxALU_sem);
		}	
	}

	void* Shiftleft2ALU(void* arg){ // Shift block before ALU
		// initialize this thread before while(1)

		while(1){
			sem_wait(&shiftLeftMuxALU_sem);
			shiftLeftMuxALU = signExtendOut << 2;
			sem_post(aluSrcB_mux_sem);
		}
	}

	void* Shiftleft2PCSource(void* arg){ // Shift block before PCSource mux
		// initialize this thread before while(1)

		while(1){
			sem_wait(&shiftLeftPCSrc_sem);
			shiftLeftMuxPCSource = instruction_25_0 << 2;
			//concatenar com PC[31-28]
			sem_wait(&pcSrc_mux_sem);
		}
	}

	void* AND(void* arg){
		// initialize this thread before while(1)

		while(1){
			sem_wait(&and_sem);
			if (getPCWriteCond() && outMuxBNE) {
				andToOr = 1;
			} else {
				andToOr = 0;
			}
			sem_post(&or_sem);
		}
	}

	void* OR(void* arg){
		// initialize this thread before while(1)

		while(1){
			sem_wait(&or_sem);
			if (getPCWrite() && andToOr) {
				orToPc = 1;
			} else {
				orToPc = 0;
			}
			sem_post(&pc_sem);
		}
	}




// Threads/Modules
//################################################################################################################################
// Main


int main(int argc, char* argv[]){

	// thread handles
	pthread_t pc_th,ir_th,ram_th,mbr_th,registerbank_th,alu_th,muxiord_th,muxalua_th,muxalub_th,muxbne_th,and_th,or_th;
	pthread_t muxpcsource_th,cu_th,alucontrol_th,muxregdst_th,muxmemtoreg_th,a_th,b_th,aluout_th,signextend_th,shiftleft2alu_th,shiftleft2pcsource_th;

	sem_init(&clock_sem,0,0);

	// initialises all modules
	pthread_create(&pc_th,NULL,PC,NULL);
	pthread_create(&ir_th,NULL,IR,NULL);
	pthread_create(&ram_th,NULL,RAM,(void*)argv[1]);
	pthread_create(&mbr_th,NULL,MBR,NULL);
	pthread_create(&registerbank_th,NULL,RegisterBank,NULL);
	pthread_create(&alu_th,NULL,ALU,NULL);
	pthread_create(&muxiord_th,NULL,MuxIorD,NULL);
	pthread_create(&muxalua_th,NULL,MuxALUA,NULL);
	pthread_create(&muxalub_th,NULL,MuxALUB,NULL);
	pthread_create(&muxbne_th,NULL,MuxBNE,NULL);
	pthread_create(&muxpcsource_th,NULL,MuxPCSource,NULL);
	pthread_create(&cu_th,NULL,CU,NULL);
	pthread_create(&alucontrol_th,NULL,ALUControl,NULL);
	pthread_create(&muxregdst_th,NULL,MuxRegDst,NULL);
	pthread_create(&muxmemtoreg_th,NULL,MuxMemtoReg,NULL);
	pthread_create(&a_th,NULL,A,NULL);
	pthread_create(&b_th,NULL,B,NULL);
	pthread_create(&aluout_th,NULL,ALUOut,NULL);
	pthread_create(&signextend_th,NULL,SignExtend,NULL);
	pthread_create(&shiftleft2alu_th,NULL,Shiftleft2ALU,NULL);
	pthread_create(&shiftleft2pcsource_th,NULL,Shiftleft2PCSource,NULL);
	pthread_create(&and_th,NULL,AND,NULL);
	pthread_create(&or_th,NULL,OR,NULL);

	while(1){
		// aqui fic ao controle dos modulos, ou nos proprios modulos
		// provavelmente por meio do clock
		sem_post(clock_sem);

	}


	return 0;
}


// Main
//################################################################################################################################