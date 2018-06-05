/*  Trabalho 2 - Organizacao de computadores
ICMC/USP - 2018
Turma 2

Lui Franco Rocha - 10295558
Ana Carolina Fainelo de Oliveira - 10284542
Bruno Del Monde - 10262818
Carolina Arenas Okawa - 10258876

OBSERVACOES:
	#COMPILAR O PROGRAMA COM: gcc cpu_multi_code.c -Wall -pthread
	#RODAR O PROGRAMA NO TERMINAL COM O COMANDO:  ./a.out code.bin

*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>


//################################################################################################################################ 
// Bit manipulation


#define INT (int)(sizeof(int)*8)

// -----------------------------------------------------------------------------------------------
void SetBit( int* B, int pos){
	B[ pos / INT ] |= 1 << (pos % INT);
}

// -----------------------------------------------------------------------------------------------
void ClearBit( int* B, int pos){
	B[ pos / INT ] &= ~(1 << (pos % INT));
}

int TestBit( int* B, int pos){
	return ((B[ pos / INT ] & (1 << (pos % INT))) != 0);
}


// -----------------------------------------------------------------------------------------------
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


void output();

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
#define getALUOp0() TestBits(&cu_signals,6,1)  
#define getALUOp1() TestBits(&cu_signals,7,1)  
#define getPCSource() TestBits(&cu_signals,8,2)
#define getPCWriteCond() TestBits(&cu_signals,10,1)
#define getPCWrite() TestBits(&cu_signals,11,1)
#define getIorD() TestBits(&cu_signals,12,1)
#define getMemRead() TestBits(&cu_signals,13,1)
#define getMemWrite() TestBits(&cu_signals,14,1)
#define getBNE() TestBits(&cu_signals,15,1)
#define getIRWrite() TestBits(&cu_signals,16,1)
#define getMemtoReg() TestBits(&cu_signals,17,2)


// Defines
//################################################################################################################################
// Global variables




// semaphores to control threads
sem_t clock_sem, clock_done, main_sem, pc_sem, ram_sem, ir_sem, alu_sem, cu_sem, mbr_sem ;
sem_t regDst_mux_sem, memToReg_mux_sem, iord_mux_sem, a_sem, b_sem, regBank_sem, aluControl_sem;
sem_t aluSrcA_mux_sem, aluSrcB_mux_sem, signExtend_sem, shiftLeftMuxALU_sem;
sem_t pcSrc_mux_sem, aluOut_sem, bne_mux_sem, and_sem, or_sem, shiftLeftPCSrc_sem;

// has all the signals from CU
int cu_signals = 0;

// connections
int pc=0, muxAddressResult=0, memData=0, writeData=0, memDataRegister=0;
int instruction_15_0=0, instruction_20_16=0, instruction_25_21=0, instruction_31_26=0, instruction_15_11=0, instruction_6_10=0, instruction_5_0=0, instruction_25_0=0;
int signExtendOut=0, shiftLeftMuxALU=0, shiftLeftMuxPCSource=0;
int outMuxBNE=0, andToOr=0, orToPc=0, muxToPc=0, outMuxRegDst=0, outMuxMemToReg=0;
int instruction,ALUResult=0,ZERO=0,a_reg_signal=0,b_reg_signal=0, ALUControlOut=0,UCState=0;
	
//value of registers
int a_reg=0, b_reg=0, ALUOutResult=0, ALUA=0, ALUB=0, mbr=0;

int regs[32]; // all registers

int ERRO;

// initialize this thread before while(1)
unsigned int ram[RAM_SIZE]; // o conteudo da ram


// Global variables
//################################################################################################################################
//Threads/Modules

// CONTROL UNIT ------------------------------------------------------------------------
void* CU(void* arg){ // Control Unit
	union {
		struct {
			unsigned char S0 : 1;
			unsigned char S1 : 1;
			unsigned char S2 : 1;
			unsigned char S3 : 1;
			unsigned char S4 : 1;
		} sinais;
		unsigned char inteiro;
	} UC_State, auxState;
	auxState.inteiro = 0;
	UC_State.inteiro = 0;

	union {
		struct {
			unsigned char RegDst0 : 1;
			unsigned char RegDst1 : 1;
			unsigned char RegWrite : 1;
			unsigned char ALUSrcA : 1;
			unsigned char ALUSrcB0 : 1;
			unsigned char ALUSrcB1 : 1;
			unsigned char ALUOp0 : 1;
			unsigned char ALUOp1 : 1;
			unsigned char PCSource0 : 1;
			unsigned char PCSource1 : 1;
			unsigned char PCWriteCond : 1;
			unsigned char PCWrite : 1;
			unsigned char IorD : 1;
			unsigned char MemRead : 1;
			unsigned char MemWrite : 1;
			unsigned char BNE : 1;
			unsigned char IRWrite : 1;
			unsigned char MemtoReg0 : 1;
			unsigned char MemtoReg1 : 1;
		} sinais;
		int inteiro;
	} local;
	local.inteiro = 0;

	while(1){
		sem_wait(&cu_sem);

//jal:
//Estado 10 faz "j" e armazena PC em $ra
//jr:
//Estado 11 escreve em pc de A
//jarl:
//Estado 12 escreve em pc de A e armazena PC em $ra
//addi:
//Estado 13 soma A com o imediato e manda para estado 16
//andi:
//Estado 14 and A com o imediato e manda para estado 16
//bne:
//Estado 15 faz "beq" + sinal BNE

//Estado 16 salva ALUout em rt

		local.sinais.RegDst0 = UC_State.inteiro == 7;
		local.sinais.RegDst1 = UC_State.inteiro == 10;
		local.sinais.RegWrite = UC_State.inteiro == 4 || UC_State.inteiro == 7 || UC_State.inteiro == 10 || UC_State.inteiro == 12 || UC_State.inteiro == 16;
		local.sinais.ALUSrcA = UC_State.inteiro == 2 || UC_State.inteiro == 6 || UC_State.inteiro == 8 || UC_State.inteiro == 13 || UC_State.inteiro == 14 || UC_State.inteiro == 15;
		local.sinais.ALUSrcB0 = UC_State.inteiro == 0 || UC_State.inteiro == 1;
		local.sinais.ALUSrcB1 = UC_State.inteiro == 1 || UC_State.inteiro == 2 || UC_State.inteiro == 13 || UC_State.inteiro == 14;
		local.sinais.ALUOp0 = UC_State.inteiro == 8 || UC_State.inteiro == 14 || UC_State.inteiro == 15;
		local.sinais.ALUOp1 = UC_State.inteiro == 6 || UC_State.inteiro == 14;
		local.sinais.PCSource0 = UC_State.inteiro == 8 || UC_State.inteiro == 11 || UC_State.inteiro == 12 || UC_State.inteiro == 15;
		local.sinais.PCSource1 = UC_State.inteiro == 9 || UC_State.inteiro == 10 || UC_State.inteiro == 11 || UC_State.inteiro == 12;
		local.sinais.PCWriteCond = UC_State.inteiro == 8 || UC_State.inteiro == 15;
		local.sinais.PCWrite = UC_State.inteiro == 0 || UC_State.inteiro == 9 || UC_State.inteiro == 10 || UC_State.inteiro == 11 || UC_State.inteiro == 12;
		local.sinais.IorD = UC_State.inteiro == 3 || UC_State.inteiro == 5;
		local.sinais.MemRead = UC_State.inteiro == 0 || UC_State.inteiro == 3;
		local.sinais.MemWrite = UC_State.inteiro == 5;
		local.sinais.BNE = UC_State.inteiro == 15;
		local.sinais.IRWrite = UC_State.inteiro == 0;
		local.sinais.MemtoReg0 = UC_State.inteiro == 4;
		local.sinais.MemtoReg1 = UC_State.inteiro == 10 || UC_State.inteiro == 12;

		cu_signals = local.inteiro;

		auxState.sinais.S0 = UC_State.inteiro == 0 || UC_State.inteiro == 6 || (UC_State.inteiro == 1 && instruction_31_26 == 2) || (UC_State.inteiro == 2 && instruction_31_26 == 43) || (UC_State.inteiro == 2 && instruction_31_26 == 35) || (UC_State.inteiro == 1 && instruction_31_26 == 20) || (UC_State.inteiro == 1 && instruction_31_26 == 8) || (UC_State.inteiro == 1 && instruction_31_26 == 5);
		auxState.sinais.S1 = UC_State.inteiro == 6 || (UC_State.inteiro == 1 && instruction_31_26 == 0) || (UC_State.inteiro == 1 && instruction_31_26 == 35) || (UC_State.inteiro == 1 && instruction_31_26 == 43) || (UC_State.inteiro == 2 && instruction_31_26 == 35) || (UC_State.inteiro == 1 && instruction_31_26 == 3) || (UC_State.inteiro == 1 && instruction_31_26 == 20) || (UC_State.inteiro == 1 && instruction_31_26 == 12) || (UC_State.inteiro == 1 && instruction_31_26 == 5);
		auxState.sinais.S2 = UC_State.inteiro == 3 || UC_State.inteiro == 6 || (UC_State.inteiro == 1 && instruction_31_26 == 0) || (UC_State.inteiro == 2 && instruction_31_26 == 43) || (UC_State.inteiro == 1 && instruction_31_26 == 21) || (UC_State.inteiro == 1 && instruction_31_26 == 8) || (UC_State.inteiro == 1 && instruction_31_26 == 12) || (UC_State.inteiro == 1 && instruction_31_26 == 5);
		auxState.sinais.S3 = (UC_State.inteiro == 1 && instruction_31_26 == 2) || (UC_State.inteiro == 1 && instruction_31_26 == 3) || (UC_State.inteiro == 1 && instruction_31_26 == 20) || (UC_State.inteiro == 1 && instruction_31_26 == 21) || (UC_State.inteiro == 1 && instruction_31_26 == 8) || (UC_State.inteiro == 1 && instruction_31_26 == 12) || (UC_State.inteiro == 1 && instruction_31_26 == 5) || (UC_State.inteiro == 1 && instruction_31_26 == 4) ;
		auxState.sinais.S4 = UC_State.inteiro == 13 || UC_State.inteiro == 14;

		UC_State.inteiro = auxState.inteiro;

		sem_post(&iord_mux_sem);
	}
}

//PROGRAM COUNTER ------------------------------------------------------------------------
void* PC(void* arg){ 
	// initialize this thread before while(1)
	int pc_local = 0; // current value of pc

	while(1){

		sem_wait(&pc_sem); // it waits for the semaphore to allow it to run
		if(orToPc == 1){ // if condition to write in pc
			pc_local = muxToPc; // overwrite pc with adress location
		}

		sem_post(&main_sem); // start next modules
		
		sem_wait(&clock_sem);
		pc = pc_local; // update global pc value after clock
		sem_post(&clock_done);
	
	}
}

//RANDOM ACCESS MEMORY ----------------------------------------------------------------------
void* RAM(void* arg){ 

	// load program from input file to ram
	FILE* code = fopen((char*)arg,"r");
	int counter = 0;

	memset(ram,0,RAM_SIZE*sizeof(int));

	while(fgetc(code) != EOF && fgetc(code) != EOF){
		fseek(code,-2,SEEK_CUR);
		fscanf(code,"%d",ram+counter);
		counter++;
	}

	fclose(code);

	while(1){

		sem_wait(&ram_sem);

		if(muxAddressResult / 4 > RAM_SIZE)
			ERRO = 1;

		else if (getMemRead() == 1)
			instruction = ram[muxAddressResult / 4];

		else if (getMemWrite() == 1)
			ram[muxAddressResult / 4] = b_reg;

		sem_post(&ir_sem);
		sem_post(&mbr_sem);
	}
}

//INSTRUCTION REGISTER ------------------------------------------------------------------------
void* IR(void* arg){ 
	// initialize this thread before while(1)

	while(1){
		
		sem_wait(&ir_sem);    

		if(instruction == 0 || instruction == -1 ){
			ERRO = 0;
		}

		sem_post(&regDst_mux_sem);     //Now, the mux controlled by the UC signal RegDst is unlocked
		sem_post(&regBank_sem);        //...
		sem_post(&aluControl_sem);     //...
		sem_post(&signExtend_sem);     //...
		sem_post(&shiftLeftPCSrc_sem);

		sem_wait(&clock_sem);
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
		sem_post(&clock_done);
	}
}

//MEMORY BUFFER REGISTER ------------------------------------------------------------------------
void* MBR(void* arg){ 
	// initialize this thread before while(1)
	mbr = 0;

	while(1){
		//verifies if bmr function is allowed to run
		sem_wait(&mbr_sem);
		
		//now memToReg mux knows that one of its inputs were already seted
		sem_post(&memToReg_mux_sem);

		//mbr receives the content of the new instruction execution at this moment
		sem_wait(&clock_sem);
		mbr = instruction;
		sem_post(&clock_done);
	}
}

//REGISTER BANK ------------------------------------------------------------------------
void* RegisterBank(void* arg){ //Contains register from 0 to 31
	
	// initialize this thread before while(1)
	
	memset(regs,0,32*sizeof(int));

	while(1){

		sem_wait(&regBank_sem); //IR function  has to run first
		sem_wait(&regBank_sem); //resDst_mux has to be seted first
		sem_wait(&regBank_sem); //memToReg_mux has to be seted first

		//setting A and B registers:
		a_reg_signal = regs[instruction_25_21]; 
		b_reg_signal = regs[instruction_20_16];

		//now that 
		sem_post(&a_sem); 
		sem_post(&b_sem);
		
		//Verifies if control unit allows to write on register bank:
		sem_wait(&clock_sem);
		if (getRegWrite() == 1) {
			if(outMuxRegDst == 0 || outMuxRegDst == 1 ||  outMuxRegDst == 26 ||  outMuxRegDst == 27){
				ERRO = 3;	
			}
			else{
				regs[outMuxRegDst] = outMuxMemToReg;
			}
		}
		sem_post(&clock_done);

	}
}

//ARITHMETIC LOGIC UNIT ----------------------------------------------------------------------
void* ALU(void* arg){ // Arithmetic Logic Unit
	// initialize this thread before while(1)

	while(1){

		for (int i = 0; i < 3; ++i)
			sem_wait(&alu_sem);

		//ULA OPERATIONS
		ZERO = 0; // talvez mudar isso
		if (ALUControlOut == 0)      // --> 000/AND <--
			ALUResult = ALUA & ALUB;

		else if(ALUControlOut == 1)  // --> 001/OR  <--
			ALUResult = ALUA | ALUB;

		else if(ALUControlOut == 2)  // --> 010/ADD <--
			ALUResult = ALUA + ALUB;

		else if(ALUControlOut == 6)  // --> 110/SUB <--
			ALUResult = ALUA - ALUB;

		else if(ALUControlOut == 7)  // --> 111/SLT <--
			ALUResult = ALUA < ALUB ? 1 : 0;
	

		if(ALUResult == 0)
			ZERO = 1;
		else
			ZERO = 0;

	
		sem_post(&aluOut_sem); //now the it already ran, aluOut can execute
	}
}

//MUX CONTROLLED BY THE CU SIGNAL IorD -----------------------------------------------------------
void* MuxIorD(void* arg){
	// initialize this thread before while(1)

	while(1){

		sem_wait(&iord_mux_sem);
		if (getIorD() == 0) {
			muxAddressResult = pc;
		} 
		else{
			muxAddressResult = ALUOutResult;
		}
		sem_post(&ram_sem); //Now that the mux selected its output, ram function can run
	}
}

//MUX CONTROLLED BY THE UC SIGNAL ALUSrcA --------------------------------------------------------
void* MuxALUA(void* arg){
	// initialize this thread before while(1)
	while(1){

		sem_wait(&aluSrcA_mux_sem); //waits until A register allows it to run

		if (getALUSrcA() == 0) {
			ALUA = pc;
		}
		else {
			ALUA = a_reg;
		}

		sem_post(&alu_sem); //Now that the mux selected an output, ALU can run its operations
	}
}

//MUX CONTROLLED BY THE CU SIGNAL ALUSrcB --------------------------------------------------------
void* MuxALUB(void* arg){
	// initialize this thread before while(1)
	int alub;
	while(1){

		sem_wait(&aluSrcB_mux_sem); //waits till B register allows it to run
		sem_wait(&aluSrcB_mux_sem); //waits  Shiftleft2ALU alloes it to run
		alub = getALUSrcB();

		if (alub == 0) {
			ALUB = b_reg;
		}
		else if (alub == 1) {
			ALUB = 4;
		}
		else if (alub == 2) {
			ALUB = signExtendOut;
		}
		else if (alub == 3) { 
			ALUB = shiftLeftMuxALU;
		}
		sem_post(&alu_sem); //Now that the mux selected an output, ALU can run its operations

	}
}

//MUX CONTROLLED BY THE CU SIGNAL BNE ------------------------------------------------------------
void* MuxBNE(void* arg){
	// initialize this thread before while(1)

	while(1){

		sem_wait(&bne_mux_sem); //waits until  mux controlled by the signal PCSource aloWes it to run

		if (getBNE() == 0) {
			outMuxBNE = ZERO;
		} 
		else { 
			outMuxBNE = !ZERO;
		}

		sem_post(&and_sem); //allows AND to execute now
	}
}

//MUX CONTROLLED BY THE CU SIGNAL PCSource -------------------------------------------------------
void* MuxPCSource(void* arg){
	// initialize this thread before while(1)
	int pcsrc=0;
	while(1){
		sem_wait(&pcSrc_mux_sem); //waits ALUOuts allows it to execute
		sem_wait(&pcSrc_mux_sem); //waits Shiftleft2PCSource allows it to execute
		pcsrc = getPCSource();

		if (pcsrc == 0) {
			muxToPc = ALUResult;
		}
		else if (pcsrc == 1) {
			muxToPc = ALUOutResult;
		}
		else if (pcsrc == 2) {
			muxToPc = shiftLeftMuxPCSource;
		}
		else if (pcsrc == 3) {
			muxToPc = a_reg;
		}

		sem_post(&bne_mux_sem);

	}
}

//ALU CONTROL ------------------------------------------------------------------------------------
void* ALUControl(void* arg){
	// initialize this thread before while(1)
	while(1){

		sem_wait(&aluControl_sem); //waits until IR allows it to run

		if(getALUOp1() == 1){ // 10: R-type operation (add, sub, slt, and, or)
			
			if (getALUOp0() == 0) {
				if(instruction_5_0 == 0x20)       // --> 010/ADD <--
					ALUControlOut = 2;

				else if(instruction_5_0 == 0x22)  // --> 110/SUB <--
					ALUControlOut = 6;

				else if(instruction_5_0 == 0x2a)  // --> 111/SLT <--
					ALUControlOut = 7;

				else if(instruction_5_0 == 0x24)  // --> 000/AND <--
					ALUControlOut = 0;

				else if(instruction_5_0 == 0x25)  // --> 001/OR  <--
					ALUControlOut = 1;

				else 
					ERRO = 2;
			}

			else //11: andi
				ALUControlOut = 0; // (000) and
		}

		else if(getALUOp0() == 1) //01: beq
			ALUControlOut = 6; // (110) sub

		else  // 00: 2º ciclo/load/store
			ALUControlOut = 2; // (010) add
		
		sem_post(&alu_sem); //now that this function executed, the ALU can run

	}
}

//MUX CONTROLLED BY THE CU SIGNAL RegDst ---------------------------------------------------------
void* MuxRegDst(void* arg){
	// initialize this thread before while(1)
	int regdst=0;
	while(1){

		sem_wait(&regDst_mux_sem); //waits till IR function allows it to run

		regdst = getRegDst();

		if (regdst == 0)
			outMuxRegDst = instruction_20_16;

		else if (regdst == 1)
			outMuxRegDst = instruction_15_11;

		else if (regdst == 2)
			outMuxRegDst = 31;

		sem_post(&regBank_sem); //Allows the Register Bank to execute
	}
}

//MUX CONTROLLED BY THE CU SIGNAL MemToReg -------------------------------------------------------
void* MuxMemtoReg(void* arg){
	// initialize this thread before while(1)
	int memtoreg=0;
	while(1){

		sem_wait(&memToReg_mux_sem); //waiting MBR allows it to execute
		memtoreg = getMemtoReg();
		if (memtoreg == 0) {
			outMuxMemToReg = ALUOutResult;
		} 
		else if (memtoreg == 1) {
			outMuxMemToReg = mbr;
		} 
		else if (memtoreg == 2) {
			outMuxMemToReg = pc;
		}

		sem_post(&regBank_sem); //allows Register Bank to execute
	}
}

//REGISTER A -------------------------------------------------------------------------------------
void* A(void* arg){
	// initialize this thread before while(1)

	while(1){

		sem_wait(&a_sem); //waits till the register bank allows it to run

		sem_post(&aluSrcA_mux_sem); //allows the mux controlled by the signal ALUSrcA to run

		sem_wait(&clock_sem); //clock control
		a_reg = a_reg_signal;
		sem_post(&clock_done);
	}
}

//REGISTER B -------------------------------------------------------------------------------------
void* B(void* arg){
	// initialize this thread before while(1)

	while(1){

		sem_wait(&b_sem); //waits till the register bank allows it to run

		sem_post(&aluSrcB_mux_sem); //allows the mux controlled by the signal ALUSrcB to run

		sem_wait(&clock_sem); //clock control
		b_reg = b_reg_signal;
		sem_post(&clock_done);
	}
}

//ALUOut REGISTER -------------------------------------------------------------------------------- 
void* ALUOut(void* arg){
	// initialize this thread before while(1)

	while(1){

		sem_wait(&aluOut_sem); //Waits ALU alloew it to run

		sem_post(&pcSrc_mux_sem); //allows mux controlled by the cu signal PCSource to run

		sem_wait(&clock_sem); //clock control
		ALUOutResult = ALUResult;
		sem_post(&clock_done);
	}
}

//SIGN EXTEND ------------------------------------------------------------------------------------
void* SignExtend(void* arg){
	// initialize this thread before while(1)

	while(1){

		sem_wait(&signExtend_sem); //waits till IR function allows it to run

		signExtendOut = instruction_15_0;
		if(TestBit(&signExtendOut,15) == 1){
			signExtendOut |= 0b11111111111111110000000000000000; 
		}

		sem_post(&shiftLeftMuxALU_sem); //Allows shift function to execute

	}	
}

//SHIFT LEFT (2 BITS) GOING TO ALU ---------------------------------------------------------------
void* Shiftleft2ALU(void* arg){ // Shift block before ALU
	// initialize this thread before while(1)

	while(1){

		sem_wait(&shiftLeftMuxALU_sem); //waits SignExtend allows it to run
		shiftLeftMuxALU = signExtendOut << 2;
		sem_post(&aluSrcB_mux_sem); //allows the mux controlled by the CU signal ALUSrcB to execute
	}
}

//SHIFT LEFT (2 BITS) GOING TO PCSource MUX ------------------------------------------------------
void* Shiftleft2PCSource(void* arg){ // Shift block before PCSource mux
	// initialize this thread before while(1)
	int auxSignal;

	while(1){

		sem_wait(&shiftLeftPCSrc_sem);  //waits till IR allows it to execute
		shiftLeftMuxPCSource = instruction_25_0 << 2;
		//concatenar com PC[31-28]

		auxSignal= pc & 0b11110000000000000000000000000000;
		shiftLeftMuxPCSource |= auxSignal;

		sem_post(&pcSrc_mux_sem); //allows mux controlled by the UC signal PCSource to execute
	}
}

//AND (PCWriteCond && outMuxBNE) -----------------------------------------------------------------
void* AND(void* arg){
	// initialize this thread before while(1)

	while(1){

		sem_wait(&and_sem); // waits UC allows it to run

		andToOr = getPCWriteCond() & outMuxBNE;

		sem_post(&or_sem); //allows OR functin to run
	}
}

// OR (andToOr || PCWrite) -----------------------------------------------------------------------
void* OR(void* arg){
	// initialize this thread before while(1)

	while(1){

		sem_wait(&or_sem); //AND has to run first

		orToPc = getPCWrite() | andToOr;

		sem_post(&pc_sem); //Allows PC funtion to run
	}
}




// Threads/Modules
//################################################################################################################################
// Main


int main(int argc, char* argv[]){

	// thread handles
	pthread_t pc_th,ir_th,ram_th,mbr_th,registerbank_th,alu_th,muxiord_th,muxalua_th,muxalub_th,muxbne_th,and_th,or_th;
	pthread_t muxpcsource_th,cu_th,alucontrol_th,muxregdst_th,muxmemtoreg_th,a_th,b_th,aluout_th,signextend_th,shiftleft2alu_th,shiftleft2pcsource_th;

	ERRO = -1;

	// initialises all semaphores
	sem_init(&clock_sem,0,0);
	sem_init(&main_sem,0,0);
	sem_init(&pc_sem,0,0);
	sem_init(&ram_sem,0,0);
	sem_init(&ir_sem,0,0);
	sem_init(&alu_sem,0,0);
	sem_init(&cu_sem,0,0);
	sem_init(&mbr_sem,0,0);
	sem_init(&regDst_mux_sem,0,0);
	sem_init(&memToReg_mux_sem,0,0);
	sem_init(&iord_mux_sem,0,0);
	sem_init(&a_sem,0,0);
	sem_init(&b_sem,0,0);
	sem_init(&regBank_sem,0,0);
	sem_init(&aluControl_sem,0,0);
	sem_init(&signExtend_sem,0,0);
	sem_init(&aluSrcA_mux_sem,0,0);
	sem_init(&aluSrcB_mux_sem,0,0);
	sem_init(&shiftLeftMuxALU_sem,0,0);
	sem_init(&shiftLeftPCSrc_sem,0,0);
	sem_init(&pcSrc_mux_sem,0,0);
	sem_init(&aluOut_sem,0,0);
	sem_init(&bne_mux_sem,0,0);
	sem_init(&and_sem,0,0);
	sem_init(&or_sem,0,0);
	sem_init(&clock_done,0,0);

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

	sem_post(&cu_sem);

	while(ERRO == -1){
		
		sem_wait(&main_sem);
		
		for(int i=0; i < 7 ; i++){
			sem_post(&clock_sem);
		}
		for(int i=0; i < 7 ; i++){
			sem_wait(&clock_done);
		}

		sem_post(&cu_sem);

	}

	// OUTPUTS ----------------------------------------------------------------
	output();

	return 0;
}


// Main
//################################################################################################################################

void output(){
	printf("STATUS DE SAIDA: ");
	if(ERRO == 0)       printf("Termino devido a tentativa de execucao de instrucao invalida.\n");
	else if(ERRO == 1)  printf("Termino devido a acesso ivalido de memoria.\n");
	else if(ERRO == 2)  printf("Termino devido a operacao ivalida na ULA.\n");
	else if(ERRO == 3)  printf("Termino devido a acesso invalido no Banco de Registradores.\n");
	printf("PC=%d  IR=%u  MDR=%u  A=%d  B=%d AluOut=%d\n", pc, instruction, mbr, a_reg, b_reg, ALUOutResult);

	printf("\nCONTROLE: \n");

	printf("RegDst %d, ",getRegDst());
	printf("RegWrite %d, ",getRegWrite());
	printf("ALUSrcA %d, ",getALUSrcA());
	printf("ALUSrcB %d, ",getALUSrcB());
	printf("ALUOp0 %d, ",getALUOp0());
	printf("ALUOp1 %d, ",getALUOp1());
	printf("PCSource %d, ",getPCSource());
	printf("PCWriteCond %d, ",getPCWriteCond());
	printf("PCWrite %d, ",getPCWrite());
	printf("IorD %d, ",getIorD());
	printf("MemRead %d, ",getMemRead());
	printf("MemWrite %d, ",getMemWrite());
	printf("BNE %d, ",getBNE());
	printf("IRWrite %d, ",getIRWrite());
	printf("MemtoReg %d\n",getMemtoReg());

	printf("\nBANCO DE REGISTRADORES: \n");


	printf("R00(r0)=%d\t", regs[0]);
	printf("R08(t0)=%d\t", regs[8]);
	printf("R16(s0)=%d\t", regs[16]);
	printf("R24(t8)=%d\n", regs[24]);

	printf("R01(at)=%d\t", regs[1]);
	printf("R09(t1)=%d\t", regs[9]);
	printf("R17(s1)=%d\t", regs[17]);
	printf("R25(t9)=%d\n", regs[25]);


	printf("R02(v0)=%d\t", regs[2]);
	printf("R10(t2)=%d\t", regs[10]);
	printf("R18(s2)=%d\t", regs[18]);
	printf("R26(k0)=%d\n", regs[26]);

	printf("R03(v1)=%d\t", regs[3]);
	printf("R11(t3)=%d\t", regs[11]);
	printf("R19(s3)=%d\t", regs[19]);
	printf("R27(k1)=%d\n", regs[27]);

	printf("R04(a0)=%d\t", regs[4]);
	printf("R12(t4)=%d\t", regs[12]);
	printf("R20(s4)=%d\t", regs[20]);
	printf("R28(gp)=%d\n", regs[28]);

	printf("R05(a1)=%d\t", regs[5]);
	printf("R13(t5)=%d\t", regs[13]);
	printf("R21(s5)=%d\t", regs[21]);
	printf("R29(sp)=%d\n", regs[29]);

	printf("R06(a2)=%d\t", regs[6]);
	printf("R14(t6)=%d\t", regs[14]);
	printf("R22(s6)=%d\t", regs[22]);
	printf("R30(fp)=%d\n", regs[30]);

	printf("R07(a3)=%d\t", regs[7]);
	printf("R15(t7)=%d\t", regs[15]);
	printf("R23(s7)=%d\t", regs[23]);
	printf("R31(ra)=%d\n", regs[31]);

	printf("\nMEMORIA (ENDERECOS A BYTE)\n");
	for(int i = 0; i <= 28; i+=4){
		printf("[%3d]=%u\t   [%3d]=%u\t   [%3d]=%u\t   [%3d]=%u\n", i, ram[i/4], 32+i, ram[(32+i)/4], 64+i, ram[(64+i)/4], 96+i, ram[(96+i)/4]);
	}
}