/* 046267 Computer Architecture - Spring 2019 - HW #1 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"

#define MAX_HISTORY_SIZE 8
#define MAX_NUMBER_OF_STATE_MACHINES 256
#define MAX_BTB_TABLE_SIZE 32
#define MAX_STATE 3
#define MAX_TAG_SIZE 32



//enum state{SNT = 0 , WNT = 1 , WT = 2 , ST = 3};

struct entry_line {
	int tag;
	uint32_t target;
};


class btb {
private:
	// all size's are fixed so no need for allocation
	entry_line btbTable[MAX_BTB_TABLE_SIZE]; //this array is an array of structs that hold the tag and the targetPc for each entry
	unsigned stateMachineTable[MAX_BTB_TABLE_SIZE][MAX_NUMBER_OF_STATE_MACHINES]; // 2d array of state machines
	unsigned historyRegTable[MAX_BTB_TABLE_SIZE];
	int xorShiftAmount; //if using shared mid should be 16 if using shared lsb should be 2 o.w. 0
	int historyBitsMask; //mask any unnecessary bits according to history register size in any calculation regarding to history reg.
	int tagMaskBits; //mask any unnecessary bits according to tag size in any calculation regarding to the tag.
	int addressBitsMask; //mask any unnecessary bits according to btbsize
	int isLGShared;
	bool isGlobalHist;
	bool isGlobalTable;
	bool lastPrediction;
	int btbSize;
	int tagSize;
	int historyRegisterSize;
	unsigned defaultState;
	unsigned numOfFlushes;
	unsigned numOfBranches;

public:
	void initBtb(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
				 bool isGlobalHist, bool isGlobalTable, int Shared);
	void initStateMachineTable(int tableNum);
	bool isTagInTable(unsigned pc) const;
	void getEntryIndexes(unsigned pc, unsigned &entryIndex, unsigned &historyRegIndex, unsigned &stateMachineTableIndex,
						 unsigned &stateMachineIndex) const;
	bool getBranchPrediction(uint32_t pc, uint32_t *dst);
	void updateHistoryReg(unsigned historyRegTableIndex, bool taken);
	void updateStats(uint32_t pc, uint32_t targetPc, uint32_t dst, bool taken, bool prediction);
	void updateStateMachine(unsigned stateMachineTableIndex, unsigned stateMachineIndex, bool taken);
	void setEntry(uint32_t pc, uint32_t targetPc, bool taken);
	bool getLastPrediction() const;
	unsigned getBtbSize()const;
	unsigned int getNumOfBranches() const;
	unsigned int getNumOfFlushes() const;
};

void btb::initBtb(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState, bool isGlobalHist,
				  bool isGlobalTable, int Shared) {
	this->numOfBranches = 0;
	this->numOfFlushes = 0;
	this->btbSize = btbSize;
	this->tagSize= tagSize;
	this->historyRegisterSize = historySize;
	this->isLGShared = Shared;
	this->tagMaskBits = static_cast<uint32_t> (1 << tagSize) - 1; //e.g. tagSize==3 then ((1 << 3) - 1) :: ((0001 << 3) - 1) => 0111
	this->historyBitsMask = (1 << historySize ) - 1; // same claculation as tagMaskBits
	this->addressBitsMask = btbSize - 1;// e.g. btbSize==16 then 16 - 1 = 15 which is 01111 in binary and would mask any bits above bit number 3 (or 4 depends on how we count)
	this->isGlobalHist = isGlobalHist;
	this->isGlobalTable = isGlobalTable;
	this->lastPrediction = false;
	this->defaultState = fsmState;
	if (Shared == 1) this->xorShiftAmount = 2; //if using lsb shared then we should shift 2 bits to the right before doing xor with the history reg
	else if (Shared == 2) this->xorShiftAmount =  16; //if using lsb shared then we should shift 16 bits to the right before doing xor with the history reg
	else this->xorShiftAmount = 0;

	for(int i = 0 ; i < MAX_BTB_TABLE_SIZE ; i++){
		this->btbTable[i].tag = 1;
		this->btbTable[i].target = 0;
		this->historyRegTable[i] = 0;
		this->initStateMachineTable(i);
	}


}

void btb::initStateMachineTable(int tableNum) {
	for(int i = 0 ; i < (1 << MAX_HISTORY_SIZE)  ; i++ ){
		this->stateMachineTable[tableNum][i] = this->defaultState;
	}
}

//this method checks in the corresponding entry if the pc's tag is saved inside the entry
bool btb::isTagInTable(unsigned pc) const {
	int pc_shifted = pc >> 2;
	int entry_address = pc_shifted & this->addressBitsMask; //because this is direct mapping - mask the unwanted bits.
	int tag = pc_shifted & this->tagMaskBits; // mask the unwanted bits from the tag
	return this->btbTable[entry_address].tag == tag;
}


// this method determines each index according to btb configuration
void btb::getEntryIndexes(unsigned pc, unsigned &entryIndex, unsigned &historyRegIndex, unsigned &stateMachineTableIndex,
					 unsigned &stateMachineIndex) const {

	unsigned pc_shifted = pc >> 2; // ignore 2 lsb's
	unsigned pc_xor_shifted = pc >> this->xorShiftAmount; //if using Gshare or Lshare - shift the relevant bits to lsb as explained in init method
	unsigned pc_xor_bits;
	entryIndex = pc_shifted & this->addressBitsMask; //mask unneeded bits for direct mapping
	historyRegIndex = this->isGlobalHist ? 0 : entryIndex; //if global history we only use the index 0 in the historyRegTable array
	stateMachineTableIndex = this->isGlobalTable ? 0 : entryIndex; //if global table we only use the index 0 in the stateMachineTable array
	pc_xor_bits = pc_xor_shifted & this->historyBitsMask; //masking because we need only as much bits as in the register
	stateMachineIndex = isLGShared == 0 ? this->historyRegTable[historyRegIndex] : this->historyRegTable[historyRegIndex] ^ pc_xor_bits;
}

// predict next pc
bool btb::getBranchPrediction(uint32_t pc, uint32_t *dst) {

	unsigned entry_num = 0 , history_reg_num = 0 , table_num = 0 , st_machine_num = 0; //indexes for the btb
	this->getEntryIndexes(pc, entry_num, history_reg_num, table_num, st_machine_num); //indexes are passed by reference
	if(this->isTagInTable(pc) && this->stateMachineTable[table_num][st_machine_num] >= 2){ //if the pc's tag is save inside the btb and the corresponding state machine is ST or WT
		*dst = this->btbTable[entry_num].target;
		this->lastPrediction = true; // save this value so we wont have to call prediction again inside the update function
		return true;
	}
	this->lastPrediction = false; // save this value so we wont have to call prediction again inside the update function
	*dst = pc + 4;
	return false;
}

void btb::updateHistoryReg(unsigned historyRegTableIndex, bool taken) {
	this->historyRegTable[historyRegTableIndex] = (( this->historyRegTable[historyRegTableIndex] << 1)+ taken) & this->historyBitsMask;
	// shift the history register to the left add the value of taken and then mask with the history's register's size mask
}

void btb::updateStats(uint32_t pc, uint32_t targetPc, uint32_t dst, bool taken, bool prediction) {
	//first lets check if our prediction was correct
	bool prediction_correct = ((taken == prediction) and (((targetPc == dst) and taken) or (!taken and targetPc != dst))) or (prediction and !taken and dst == pc + 4);
	this->numOfBranches++;
	this->numOfFlushes += !prediction_correct; // if our prediction was incorrect, Flush!
}

void btb::updateStateMachine(unsigned stateMachineTableIndex, unsigned stateMachineIndex, bool taken) {
	if(taken && this->stateMachineTable[stateMachineTableIndex][stateMachineIndex] < 3) {
		this->stateMachineTable[stateMachineTableIndex][stateMachineIndex]++;
	}
	else if(!taken && this->stateMachineTable[stateMachineTableIndex][stateMachineIndex] > 0) {
		this->stateMachineTable[stateMachineTableIndex][stateMachineIndex]--;
	}
}

void btb::setEntry(uint32_t pc, uint32_t targetPc, bool taken) {
	unsigned entry_num = 0 , history_reg_num = 0 , table_num = 0 , st_machine_num = 0; //indexes for the btb
	this->getEntryIndexes(pc, entry_num, history_reg_num, table_num, st_machine_num); //indexes are passed by reference
	if(!this->isTagInTable(pc)){ //if we dont have the tag in the table we will need to add it and according to global/local reset the state machines and history reg
		this->btbTable[entry_num].tag = (pc >> 2) & this->tagMaskBits; //set the new tag
		if(!this->isGlobalTable ) this->initStateMachineTable(table_num);
		if(!this->isGlobalHist) this->historyRegTable[history_reg_num] = 0;
		this->getEntryIndexes(pc, entry_num, history_reg_num, table_num, st_machine_num); // our state machine index might be incorrect
	}
	this->updateStateMachine(table_num, st_machine_num, taken);
	this->updateHistoryReg(history_reg_num, taken);
	this->btbTable[entry_num].target = targetPc;

}

unsigned btb::getBtbSize() const {
	int numOfStateMachineTables = this->isGlobalTable ? 1 : this->btbSize;
	int numOfHistoryRegs = this->isGlobalHist ? 1 : this->btbSize;
	int btbSizeBits = this->btbSize * (this->tagSize + 30);
	int historyTableSizeBits = numOfHistoryRegs * this->historyRegisterSize;
	int stateMachineTablesSizeBits = numOfStateMachineTables * 2 * (1 << this->historyRegisterSize);

	return static_cast<unsigned int>(btbSizeBits + historyTableSizeBits + stateMachineTablesSizeBits);
}

unsigned int btb::getNumOfBranches() const {
	return this->numOfBranches;
}

unsigned int btb::getNumOfFlushes() const {
	return this->numOfFlushes;
}

bool btb::getLastPrediction() const {
	return this->lastPrediction;
}


btb table;


int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
	if(btbSize > MAX_BTB_TABLE_SIZE || historySize > MAX_HISTORY_SIZE || tagSize > MAX_TAG_SIZE || fsmState > MAX_STATE) return -1;
	table.initBtb(btbSize, historySize, tagSize, fsmState, isGlobalHist, isGlobalTable, Shared);
	return 0;

}




bool BP_predict(uint32_t pc, uint32_t *dst){
	return table.getBranchPrediction(pc, dst);
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	bool isTaken = table.getLastPrediction();
	table.updateStats(pc, targetPc, pred_dst, taken, isTaken);
	table.setEntry(pc, targetPc, taken);
}

void BP_GetStats(SIM_stats *curStats){
	curStats->size = table.getBtbSize();
	curStats->br_num = table.getNumOfBranches();
	curStats->flush_num = table.getNumOfFlushes();
}

