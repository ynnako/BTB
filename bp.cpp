/* 046267 Computer Architecture - Spring 2019 - HW #1 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"

#define MAX_HISTORY_SIZE 8
#define MAX_BTB_TABLE_SIZE 32



enum state{SNT = 0 , WNT = 1 , WT = 2 , ST = 3};

struct entry_line {
	uint32_t tag;
	uint32_t target;
};


class btb{
private:
	entry_line btb_table[MAX_BTB_TABLE_SIZE];
	unsigned st_table[MAX_BTB_TABLE_SIZE][2 << MAX_HISTORY_SIZE];
	uint8_t history_reg[MAX_BTB_TABLE_SIZE];
	unsigned m_btbSize;
	unsigned m_historySize;
	unsigned m_tagSize;
	unsigned m_default_State;
	unsigned m_table_current_size;
	bool m_isGlobalHist;
	bool m_isGlobalTable;
	int m_Shared;
	uint32_t m_xor_mask;
	uint32_t m_tag_mask;


public:
	void init_btb(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
				  bool isGlobalHist, bool isGlobalTable, int Shared);
	void init_state_table();
	void set_entry(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst);
	bool get_prediction(uint32_t pc, uint32_t *dst) const;


};

void btb::init_btb(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState, bool isGlobalHist,
				   bool isGlobalTable, int Shared) {
	this->m_historySize = historySize;
	this->m_table_current_size = 0;
	this->m_btbSize = btbSize;
	this->m_tagSize = tagSize;
	this->m_tag_mask = static_cast<uint32_t>((2 << tagSize ) - 1) << 2;
	this->m_xor_mask = this->m_tag_mask;
	this->m_isGlobalHist = isGlobalHist;
	this->m_isGlobalTable = isGlobalTable;
	this->m_Shared = Shared;
	this->m_default_State = fsmState;
	if(Shared == 2){
		this->m_xor_mask = this->m_tag_mask << 14;
	}

	for(int i = 0 ; i < MAX_BTB_TABLE_SIZE ; i++){
		this->btb_table[i].tag = 1;
		this->btb_table[i].target = 1;
		this->history_reg[i] = 0;
		for(int j = 0 ; j < (2 << MAX_HISTORY_SIZE) ; j++){
			this->st_table[i][j] = m_default_State;
		}
	}

}

void btb::set_entry(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {


}

bool btb::get_prediction(uint32_t pc, uint32_t *dst) const {
	int i , line_num = 0 , state_machine_num = 0;
	unsigned tag = (pc & this->m_tag_mask) >> 2;
	bool match_found = false;

	for(i = 0 ; i < this->m_btbSize ; i++) {
		if (tag == this->btb_table[i].tag) {
			match_found = true;
			if (this->m_isGlobalHist) {  //global history and global state_table
				state_machine_num = history_reg[0] ^ ((pc & this->m_xor_mask) >> 16);
			}
			else if(this->m_isGlobalTable){
				state_machine_num = history_reg[i] ^ ((pc & this->m_xor_mask) >> 2);
			}
			else {
				state_machine_num = this->history_reg[i];
				if (!this->m_isGlobalTable) line_num = i;
			}
			break;
		}
	}


	if(match_found && this->st_table[line_num][state_machine_num] >= 2) {
		*dst = this->btb_table[line_num].target;
	}
	else{
		*dst = pc +4;
	}
	return match_found;

}

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
	if(btbSize > MAX_BTB_TABLE_SIZE || historySize > MAX_HISTORY_SIZE || tagSize > 32 || fsmState > 3) return -1;

}

bool BP_predict(uint32_t pc, uint32_t *dst){
	return false;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	return;
}

void BP_GetStats(SIM_stats *curStats){
	return;
}

