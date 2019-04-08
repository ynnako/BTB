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


class btb {
private:
	entry_line btb_table[MAX_BTB_TABLE_SIZE];
	unsigned st_table[MAX_BTB_TABLE_SIZE][2 << (MAX_HISTORY_SIZE - 1)];
	uint32_t history_reg[MAX_BTB_TABLE_SIZE];
	int m_history_mask;
	unsigned m_btbSize;
	unsigned m_default_State;
	unsigned m_table_current_size;
	int m_shared;
	bool m_isGlobalHist;
	bool m_isGlobalTable;
	int m_xor_mask;
	int m_tag_mask;

public:
	void init_btb(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
				  bool isGlobalHist, bool isGlobalTable, int Shared);
	void init_state_table(unsigned line_num);
	bool get_entry_indexes(unsigned pc , unsigned &entry_num , unsigned &history_reg_num , unsigned &table_num , unsigned &st_machine_num) const;
	bool get_prediction(uint32_t pc, uint32_t *dst) const;
	void update_bhr(unsigned history_reg_num, uint32_t target_pc, uint32_t dst , bool taken);
	void update_st_machine(unsigned table_num, uint32_t st_machine_num, bool taken);
	void set_entry(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst);



};

void btb::init_btb(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState, bool isGlobalHist,
				   bool isGlobalTable, int Shared) {


	this->m_table_current_size = 0;
	this->m_btbSize = btbSize;
	this->m_shared = Shared;
	this->m_tag_mask = static_cast<uint32_t> (((1 << tagSize) - 1) << 2);
	this->m_history_mask = ((1 << historySize ) - 1);
	this->m_isGlobalHist = isGlobalHist;
	this->m_isGlobalTable = isGlobalTable;
	this->m_default_State = fsmState;
	if (Shared == 1) this->m_xor_mask = this->m_history_mask << 2;
	else if (Shared == 2) this->m_xor_mask = this->m_history_mask << 16;
	else this->m_xor_mask = 0xFFFFFFFC;

	for(unsigned i = 0 ; i < MAX_BTB_TABLE_SIZE ; i++){
		this->btb_table[i].tag = 0;
		this->btb_table[i].target = 0;
		this->history_reg[i] = 0;
		this->init_state_table(i);
	}


}

void btb::init_state_table(unsigned line_num) {
	for(int i = 0 ; i < (1 << MAX_HISTORY_SIZE)  ; i++ ){
		this->st_table[line_num][i] = this->m_default_State;
	}
}

bool
btb::get_entry_indexes(unsigned pc, unsigned &entry_num ,unsigned &history_reg_num, unsigned &table_num, unsigned &st_machine_num) const {

	uint32_t tag = (pc & this->m_tag_mask) >> 2;

	for(unsigned i = 0 ; i < this->m_btbSize ; i++){
		if(tag == this->btb_table[i].tag){ // line exists in table
			entry_num = i;
			if(m_isGlobalTable){
				table_num = 0; //we have only one state machine table

				if(m_isGlobalHist) {
					history_reg_num = 0; //we have only one history reg
					st_machine_num = this->history_reg[0];
				}
				else {
					history_reg_num = i;
					st_machine_num = this->history_reg[i];
				}

				if(m_shared == 1){
					st_machine_num = ((pc & this->m_xor_mask) >> 2) ^ this->history_reg[history_reg_num]; //get only the relevant bits form pc and xor with history reg
				}
				else if(m_shared ==2){
					st_machine_num = ((pc & this->m_xor_mask) >> 16) ^ this->history_reg[history_reg_num];//get only the relevant bits form pc and xor with history reg
				}
			}
			else{
				table_num = i;
				history_reg_num = i;
				st_machine_num = this->history_reg[i];
			}
			return true;
		}
	}

	//we didnt find the tag in the btb table
	entry_num = this->m_table_current_size;
	if(m_isGlobalTable){
		table_num = 0;

		if(m_isGlobalHist) {
			history_reg_num = 0;
			st_machine_num = this->history_reg[0];
		}
		else {
			history_reg_num = this->m_table_current_size;
			st_machine_num = this->history_reg[history_reg_num];
		}

		if(m_shared == 1){
			st_machine_num = ((pc & this->m_xor_mask) >> 2) ^ this->history_reg[history_reg_num];
		}
		else if(m_shared ==2){
			st_machine_num = ((pc & this->m_xor_mask) >> 16) ^ this->history_reg[history_reg_num];
		}
	}
	else{
		table_num = this->m_table_current_size;
		history_reg_num = this->m_table_current_size;
		st_machine_num = this->history_reg[m_table_current_size];
	}

	return false;
}

bool btb::get_prediction(uint32_t pc, uint32_t *dst) const {

	unsigned entry_num = 0 , history_reg_num = 0 , table_num = 0 , st_machine_num = 0;
	if(this->get_entry_indexes(pc , entry_num , history_reg_num , table_num , st_machine_num) && this->st_table[table_num][st_machine_num] >= 2){
		*dst = this->btb_table[entry_num].target;
		return true;
	}
	*dst = pc + 4;
	return false;
}

void btb::update_bhr(unsigned history_reg_num, uint32_t target_pc, uint32_t dst ,bool taken ) {
	bool prediction_correct = ((target_pc == dst) and taken) or (!taken and target_pc != dst);
	this->history_reg[history_reg_num] = (( this->history_reg[history_reg_num] << 1)+ prediction_correct) & this->m_history_mask;
}

void btb::update_st_machine(unsigned table_num, uint32_t st_machine_num, bool taken) {
	if(taken && this->st_table[table_num][st_machine_num] < 3) this->st_table[table_num][st_machine_num]++;
	else if(!taken && this->st_table[table_num][st_machine_num] > 0) this->st_table[table_num][st_machine_num]--;
}

void btb::set_entry(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
	unsigned entry_num = 0 , history_reg_num = 0 , table_num = 0;
	unsigned st_machine_num = 0;
	if(!this->get_entry_indexes(pc , entry_num , history_reg_num , table_num , st_machine_num)){
		this->btb_table[entry_num].tag = (pc & this->m_tag_mask) >> 2;
		this->btb_table[entry_num].target = targetPc;
		if(!this->m_isGlobalTable && !this->m_isGlobalHist) this->init_state_table(table_num);
		if(!this->m_isGlobalHist) this->history_reg[history_reg_num] = 0;
		this->m_table_current_size = (this->m_table_current_size + 1) % this->m_btbSize;
	}
	this->update_st_machine(table_num , st_machine_num ,taken);
	this->update_bhr(history_reg_num , targetPc , pred_dst , taken);

}


btb table;


int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
	if(btbSize > MAX_BTB_TABLE_SIZE || historySize > MAX_HISTORY_SIZE || tagSize > 32 || fsmState > 3) return -1;
	table.init_btb(btbSize, historySize, tagSize, fsmState, isGlobalHist, isGlobalTable, Shared);
	return 0;

}

bool BP_predict(uint32_t pc, uint32_t *dst){
	return table.get_prediction(pc , dst);
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	table.set_entry(pc , targetPc , taken , pred_dst);
}

void BP_GetStats(SIM_stats *curStats){
	return;
}

