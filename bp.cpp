/* 046267 Computer Architecture - Spring 2019 - HW #1 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"

#define MAX_HISTORY_SIZE 8
#define MAX_BTB_TABLE_SIZE 32



//enum state{SNT = 0 , WNT = 1 , WT = 2 , ST = 3};

struct entry_line {
	uint32_t tag;
	uint32_t target;
};


class btb{
private:
	entry_line btb_table[MAX_BTB_TABLE_SIZE];
	unsigned st_table[MAX_BTB_TABLE_SIZE][2 << (MAX_HISTORY_SIZE - 1)];
	uint8_t history_reg[MAX_BTB_TABLE_SIZE];
	uint8_t m_history_mask;
	unsigned m_btbSize;
	unsigned m_default_State;
	unsigned m_table_current_size;
	unsigned m_shared;
	bool m_isGlobalHist;
	bool m_isGlobalTable;
	uint32_t m_xor_mask;
	uint32_t m_tag_mask;


public:
	void init_btb(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
				  bool isGlobalHist, bool isGlobalTable, int Shared);
	void init_state_table(unsigned line_num);
	void set_entry(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst);
	bool get_prediction(uint32_t pc, uint32_t *dst) const;


};

void btb::init_btb(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState, bool isGlobalHist,
				   bool isGlobalTable, int Shared) {
	this->m_table_current_size = 0;
	this->m_btbSize = btbSize;
	this->m_shared = Shared;
	this->m_tag_mask = static_cast<uint32_t>(((2 << (tagSize - 1 )) - 1) << 2);
	this->m_history_mask = static_cast<uint8_t>((2 << (historySize - 1 )) - 1);
	this->m_xor_mask = this->m_history_mask;
	this->m_isGlobalHist = isGlobalHist;
	this->m_isGlobalTable = isGlobalTable;
	this->m_default_State = fsmState;
	if(Shared == 2){
		this->m_xor_mask = this->m_tag_mask << 14;
	}

	for(int i = 0 ; i < MAX_BTB_TABLE_SIZE ; i++){
		this->btb_table[i].tag = 0;
		this->btb_table[i].target = 0;
		this->history_reg[i] = 0;
		for(int j = 0 ; j < (2 << (MAX_HISTORY_SIZE - 1)) ; j++){
			this->st_table[i][j] = m_default_State;
		}
	}

}

void btb::set_entry(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
	int i , line_num = 0 , table_num = 0 , history_line_num = 0 , state_machine_num = 0;
	int ip = (this->m_xor_mask & pc) >> 2; // initial to this value to use less if statements
	if(this->m_shared == 2) ip = ip  >> 14;
	unsigned tag = (pc & this->m_tag_mask) >> 2;
	bool match_found = false;
	bool prediction_correct = false;

	if((taken && pred_dst == targetPc) || (!taken && pred_dst == pc + 4)){ //correct prediction
		prediction_correct = true;
	}
	else if((!taken && pred_dst == targetPc) || (taken && pred_dst != targetPc)){ //wrong prediction
		prediction_correct = false;
	}

	for(i = 0 ; i < this->m_btbSize ; i++){
		if(tag == this->btb_table[i].tag){
			line_num = i;
			match_found = true;
			if (this->m_isGlobalHist) {  //global history and global state_table
				state_machine_num = (history_reg[0] & m_history_mask) ^ ((pc & this->m_xor_mask) >> 16);
				table_num = 0;
				history_line_num = 0;
			}
			else if(this->m_isGlobalTable){
				state_machine_num = (history_reg[i] & m_history_mask) ^ ((pc & this->m_xor_mask) >> 2);
				table_num = 0;
				history_line_num = i;
			}
			else {
				state_machine_num = this->history_reg[i] & m_history_mask;
				table_num = i;
				history_line_num = i;
			}
			break;
		}
	}
	if(match_found){

		this->btb_table[line_num].target = targetPc;
		this->history_reg[history_line_num] = static_cast<uint8_t>(((this->history_reg[history_line_num] << 1) + prediction_correct) & m_history_mask);
		if(taken && this->st_table[table_num][state_machine_num] < 3){
			this->st_table[table_num][state_machine_num]++;
		}
		else if(!taken && this->st_table[table_num][state_machine_num] > 0){
			this->st_table[table_num][state_machine_num]--;
		}

	}
	else{
		state_machine_num = this->history_reg[0] ^ ip;
		table_num = 0;
		if(!this->m_isGlobalHist) {
			this->history_reg[m_table_current_size] = static_cast<uint8_t>((0 + prediction_correct) & m_history_mask);
			state_machine_num = this->history_reg[m_table_current_size] ^ ip;

		}
		if(!this->m_isGlobalTable){
			init_state_table(m_table_current_size);
			state_machine_num = history_reg[m_table_current_size];
			table_num = m_table_current_size;
		}
		if(taken && this->st_table[table_num][state_machine_num] < 3){
			this->st_table[table_num][state_machine_num]++;
		}
		else if(!taken && this->st_table[table_num][state_machine_num] > 0){
			this->st_table[table_num][state_machine_num]--;
		}
		this->btb_table[this->m_table_current_size].tag = tag;
		this->btb_table[this->m_table_current_size].target = targetPc;
		this->m_table_current_size ++;

		if(this->m_table_current_size == this->m_btbSize) this->m_table_current_size = 0;

	}


}

bool btb::get_prediction(uint32_t pc, uint32_t *dst) const {
	int i , line_num = 0 , state_machine_num = 0 , table_num = 0;
	unsigned tag = (pc & this->m_tag_mask) >> 2;
	bool match_found = false , taken = false;

	for(i = 0 ; i < this->m_btbSize ; i++) { //checks if the tag is in the table
		if (tag == this->btb_table[i].tag) {
			match_found = true;
			line_num = i;
			if (this->m_isGlobalHist) {  //global history and global state_table
				state_machine_num = ((history_reg[0] & m_history_mask) ^ ((pc & this->m_xor_mask) >> 16)) ;
				table_num = 0;
			}
			else if(this->m_isGlobalTable){
				state_machine_num = ((history_reg[i] & m_history_mask) ^ ((pc & this->m_xor_mask) >> 2));
				table_num = 0;
			}
			else {
				state_machine_num = ((this->history_reg[i] & m_history_mask));
				table_num = i;
			}
			break;
		}
	}

	if(match_found && this->st_table[table_num][state_machine_num] >= 2) { //if found and msb is 1 then jump
		*dst = this->btb_table[line_num].target;
		taken = true;
	}
	else{
		*dst = pc +4;
		taken = false;
	}
	return taken;

}

void btb::init_state_table(unsigned line_num) {
	for(int i = 0 ; i < (1 << MAX_HISTORY_SIZE)  ; i++ ){
		this->st_table[line_num][i] = this->m_default_State;
	}

}


btb table;


int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
	if(btbSize > MAX_BTB_TABLE_SIZE || historySize > MAX_HISTORY_SIZE || tagSize > 32 || fsmState > 3) return -1;
	table.init_btb(btbSize, historySize, tagSize, fsmState, isGlobalHist, isGlobalTable, Shared);
	return 0;

}

bool BP_predict(uint32_t pc, uint32_t *dst){
	if(dst == nullptr) return false;
	return table.get_prediction(pc , dst);
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	table.set_entry(pc , targetPc , taken , pred_dst);
	return;
}

void BP_GetStats(SIM_stats *curStats){
	return;
}

