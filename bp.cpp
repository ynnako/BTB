/* 046267 Computer Architecture - Spring 2019 - HW #1 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"

#define MAX_HISTORY_SIZE 8
#define MAX_BTB_TABLE_SIZE 32



//enum state{SNT = 0 , WNT = 1 , WT = 2 , ST = 3};

struct entry_line {
	int tag;
	uint32_t target;
};


class btb {
private:
	entry_line btbTable[MAX_BTB_TABLE_SIZE];
	unsigned stateMachineTable[MAX_BTB_TABLE_SIZE][2 << (MAX_HISTORY_SIZE - 1)];
	unsigned historyRegTable[MAX_BTB_TABLE_SIZE];
	int xorShiftAmount;
	int historyBitsMask;
	int tagMaskBits;
	int addressBitsMask;
	int isLGShared;
	bool isGlobalHist;
	bool isGlobalTable;
	int btbSize;
	int tagSize;
	int historyRegisterSize;
	unsigned defaultState;
	int numOfFlushes;
	int numOfBranches;



public:
	void initBtb(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
				 bool isGlobalHist, bool isGlobalTable, int Shared);
	void initStateMachineTable(int tableNum);
	bool isTagInTable(unsigned pc) const;
	void getEntryIndexes(unsigned pc, unsigned &entryIndex, unsigned &historyRegIndex, unsigned &stateMachineTableIndex,
						 unsigned &stateMachineIndex) const;
	bool getBranchPrediction(uint32_t pc, uint32_t *dst) const;
	void updateHistoryReg(unsigned historyRegTableIndex, bool taken);
	void updateStats(uint32_t targetPc, uint32_t dst, bool taken, bool prediction);
	void updateStateMachine(unsigned stateMachineTableIndex, unsigned stateMachineIndex, bool taken);
	void setEntry(uint32_t pc, uint32_t targetPc, bool taken, uint32_t predictedDest);
	unsigned getBtbSize()const;
	unsigned getNumOfBranches() const;
	unsigned getNumOfFlushes() const;



};

void btb::initBtb(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState, bool isGlobalHist,
				  bool isGlobalTable, int Shared) {


	this->numOfBranches = 0;
	this->numOfFlushes = 0;
	this->btbSize = btbSize;
	this->tagSize= tagSize;
	this->historyRegisterSize = historySize;
	this->isLGShared = Shared;
	this->tagMaskBits = static_cast<uint32_t> (1 << tagSize) - 1;
	this->historyBitsMask = (1 << historySize ) - 1;
	this->addressBitsMask = btbSize - 1;
	this->isGlobalHist = isGlobalHist;
	this->isGlobalTable = isGlobalTable;
	this->defaultState = fsmState;
	if (Shared == 1) this->xorShiftAmount = 2;
	else if (Shared == 2) this->xorShiftAmount =  16;
	else this->xorShiftAmount = 0;

	for(int i = 0 ; i < MAX_BTB_TABLE_SIZE ; i++){
		this->btbTable[i].tag = -1;
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

bool btb::isTagInTable(unsigned pc) const {
	int pc_shifted = pc >> 2;
	int entry_address = pc_shifted & this->addressBitsMask;
	int tag = pc_shifted & this->tagMaskBits;
	return this->btbTable[entry_address].tag == tag;
}

void
btb::getEntryIndexes(unsigned pc, unsigned &entryIndex, unsigned &historyRegIndex, unsigned &stateMachineTableIndex,
					 unsigned &stateMachineIndex) const {

	unsigned pc_shifted = pc >> 2;
	unsigned pc_xor_shifted = pc >> this->xorShiftAmount; //if using Gshare or Lshare - shift the relevant bits to lsb
	unsigned pc_xor_bits;
	entryIndex = pc_shifted & this->addressBitsMask;
	historyRegIndex = this->isGlobalHist ? 0 : entryIndex; //if global history we only use the index 0 in the historyRegTable array
	stateMachineTableIndex = this->isGlobalTable ? 0 : entryIndex; //if global table we only use the index 0 in the stateMachineTable array
	pc_xor_bits = pc_xor_shifted & this->historyBitsMask; //masking because we need only as much bits as in the register
	stateMachineIndex = isLGShared == 0 ? this->historyRegTable[historyRegIndex] : this->historyRegTable[historyRegIndex] ^ pc_xor_bits;
}

bool btb::getBranchPrediction(uint32_t pc, uint32_t *dst) const {

	unsigned entry_num = 0 , history_reg_num = 0 , table_num = 0 , st_machine_num = 0;
	this->getEntryIndexes(pc, entry_num, history_reg_num, table_num, st_machine_num);
	if(this->isTagInTable(pc) && this->stateMachineTable[table_num][st_machine_num] >= 2){
		*dst = this->btbTable[entry_num].target;
		return true;
	}
	*dst = pc + 4;
	return false;
}

void btb::updateHistoryReg(unsigned historyRegTableIndex, bool taken) {
	this->historyRegTable[historyRegTableIndex] = (( this->historyRegTable[historyRegTableIndex] << 1)+ taken) & this->historyBitsMask;
}

void btb::updateStats(uint32_t targetPc, uint32_t dst, bool taken, bool prediction) {
	bool prediction_correct = (taken == prediction) and (((targetPc == dst) and taken) or (!taken and targetPc != dst));
	this->numOfBranches++;
	this->numOfFlushes += !prediction_correct;
}

void btb::updateStateMachine(unsigned stateMachineTableIndex, unsigned stateMachineIndex, bool taken) {
	if(taken && this->stateMachineTable[stateMachineTableIndex][stateMachineIndex] < 3) this->stateMachineTable[stateMachineTableIndex][stateMachineIndex]++;
	else if(!taken && this->stateMachineTable[stateMachineTableIndex][stateMachineIndex] > 0) this->stateMachineTable[stateMachineTableIndex][stateMachineIndex]--;
}

void btb::setEntry(uint32_t pc, uint32_t targetPc, bool taken, uint32_t predictedDest) {
	unsigned entry_num = 0 , history_reg_num = 0 , table_num = 0 , st_machine_num = 0;
	this->getEntryIndexes(pc, entry_num, history_reg_num, table_num, st_machine_num);
	if(!this->isTagInTable(pc)){
		this->btbTable[entry_num].tag = (pc >> 2) & this->tagMaskBits;
		if(!this->isGlobalTable ) this->initStateMachineTable(table_num);
		if(!this->isGlobalHist) this->historyRegTable[history_reg_num] = 0;
		this->getEntryIndexes(pc, entry_num, history_reg_num, table_num, st_machine_num);
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

unsigned btb::getNumOfBranches() const {
	return this->numOfBranches;
}

unsigned btb::getNumOfFlushes() const {
	return this->numOfFlushes;
}




btb table;


int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
	if(btbSize > MAX_BTB_TABLE_SIZE || historySize > MAX_HISTORY_SIZE || tagSize > 32 || fsmState > 3) return -1;
	table.initBtb(btbSize, historySize, tagSize, fsmState, isGlobalHist, isGlobalTable, Shared);
	return 0;

}




bool BP_predict(uint32_t pc, uint32_t *dst){
	return table.getBranchPrediction(pc, dst);
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	uint32_t dest;
	bool isTaken = table.getBranchPrediction(pc, &dest);
	table.updateStats(targetPc, pred_dst, taken, isTaken);
	table.setEntry(pc, targetPc, taken, pred_dst);
}

void BP_GetStats(SIM_stats *curStats){
	curStats->size = table.getBtbSize();
	curStats->br_num = table.getNumOfBranches();
	curStats->flush_num = table.getNumOfFlushes();
	return;
}

