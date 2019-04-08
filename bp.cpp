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
	unsigned m_tagSize;
	unsigned m_historySize;
	unsigned m_default_State;
	unsigned m_num_of_flushes;
	unsigned m_num_of_branches;
	unsigned m_address_mask;
	int m_shared;
	bool m_isGlobalHist;
	bool m_isGlobalTable;
	int m_xor_shift;
	int m_tag_mask;

public:
	void init_btb(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
				  bool isGlobalHist, bool isGlobalTable, int Shared);
	void init_state_table(int line_num);
	bool check_if_tag_exists(unsigned pc) const;
	void get_entry_indexes(unsigned pc, unsigned &entry_num ,unsigned &history_reg_num, unsigned &table_num, unsigned &st_machine_num) const;
	bool get_prediction(uint32_t pc, uint32_t *dst) const;
	void update_bhr(unsigned history_reg_num, uint32_t target_pc, uint32_t dst , bool taken);
	void update_st_machine(unsigned table_num, unsigned st_machine_num, bool taken);
	void set_entry(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst);
	unsigned get_size()const;
	unsigned get_num_of_branches() const;
	unsigned get_num_of_flushes() const;



};

void btb::init_btb(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState, bool isGlobalHist,
				   bool isGlobalTable, int Shared) {


	this->m_num_of_branches = 0;
	this->m_num_of_flushes = 0;
	this->m_btbSize = btbSize;
	this->m_tagSize= tagSize;
	this->m_historySize = historySize;
	this->m_shared = Shared;
	this->m_tag_mask = static_cast<uint32_t> (1 << tagSize) - 1;
	this->m_history_mask = (1 << historySize ) - 1;
	this->m_address_mask = btbSize - 1;
	this->m_isGlobalHist = isGlobalHist;
	this->m_isGlobalTable = isGlobalTable;
	this->m_default_State = fsmState;
	if (Shared == 1) this->m_xor_shift = 2;
	else if (Shared == 2) this->m_xor_shift =  16;
	else this->m_xor_shift = 0;

	for(int i = 0 ; i < MAX_BTB_TABLE_SIZE ; i++){
		this->btb_table[i].tag = 0;
		this->btb_table[i].target = 0;
		this->history_reg[i] = 0;
		this->init_state_table(i);
	}


}

void btb::init_state_table(int line_num) {
	for(int i = 0 ; i < (1 << MAX_HISTORY_SIZE)  ; i++ ){
		this->st_table[line_num][i] = this->m_default_State;
	}
}

bool btb::check_if_tag_exists(unsigned pc) const {
	unsigned pc_shifted = pc >> 2;
	unsigned entry_address = pc_shifted & this->m_address_mask;
	unsigned tag = pc_shifted & this->m_tag_mask;
	return this->btb_table[entry_address].tag == tag;
}

void
btb::get_entry_indexes(unsigned pc, unsigned &entry_num ,unsigned &history_reg_num, unsigned &table_num, unsigned &st_machine_num) const {

	unsigned pc_shifted = pc >> 2;
	unsigned pc_xor_shifted = pc >> this->m_xor_shift; //if using Gshare or Lshare - shift the relevant bits to lsb
	unsigned pc_xor_bits;
	entry_num = pc_shifted & this->m_address_mask;
	history_reg_num = this->m_isGlobalHist ? 0 : entry_num; //if global history we only use the index 0 in the history_reg array
	table_num = this->m_isGlobalTable ? 0 : entry_num; //if global table we only use the index 0 in the st_table array
	pc_xor_bits = pc_xor_shifted & this->m_history_mask; //masking because we need only as much bits as in the register
	st_machine_num = m_shared == 0 ? this->history_reg[history_reg_num] : this->history_reg[history_reg_num] ^ pc_xor_bits;
}

bool btb::get_prediction(uint32_t pc, uint32_t *dst) const {

	unsigned entry_num = 0 , history_reg_num = 0 , table_num = 0 , st_machine_num = 0;
	this->get_entry_indexes(pc , entry_num , history_reg_num , table_num , st_machine_num);
	if(this->check_if_tag_exists(pc) && this->st_table[table_num][st_machine_num] >= 2){
		*dst = this->btb_table[entry_num].target;
		return true;
	}
	*dst = pc + 4;
	return false;
}

void btb::update_bhr(unsigned history_reg_num, uint32_t target_pc, uint32_t dst ,bool taken ) {
	bool prediction_correct = ((target_pc == dst) and taken) or (!taken and target_pc != dst);
	this->m_num_of_branches++;
	this->m_num_of_flushes += !prediction_correct;
	this->history_reg[history_reg_num] = (( this->history_reg[history_reg_num] << 1)+ prediction_correct) & this->m_history_mask;
}

void btb::update_st_machine(unsigned table_num, unsigned st_machine_num, bool taken) {
	if(taken && this->st_table[table_num][st_machine_num] < 3) this->st_table[table_num][st_machine_num]++;
	else if(!taken && this->st_table[table_num][st_machine_num] > 0) this->st_table[table_num][st_machine_num]--;
}

void btb::set_entry(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
	unsigned entry_num = 0 , history_reg_num = 0 , table_num = 0 , st_machine_num = 0;
	this->get_entry_indexes(pc , entry_num , history_reg_num , table_num , st_machine_num);
	if(!this->check_if_tag_exists(pc)){
		this->btb_table[entry_num].tag = (pc & this->m_tag_mask) >> 2;
		if(!this->m_isGlobalTable && !this->m_isGlobalHist) this->init_state_table(table_num);
		if(!this->m_isGlobalHist) this->history_reg[history_reg_num] = 0;
	}
	this->btb_table[entry_num].target = targetPc;
	this->update_st_machine(table_num , st_machine_num ,taken);
	this->update_bhr(history_reg_num , targetPc , pred_dst , taken);

}

unsigned btb::get_size() const {
	int num_of_tables = this->m_isGlobalTable ? 1 : this->m_btbSize;
	int num_of_history_regs = this->m_isGlobalHist ? 1 : this->m_btbSize;
	int btb_size = this->m_btbSize * (this->m_tagSize + 32); // i need to ask if this includes the two lsb's
	int history_size = num_of_history_regs * this->m_historySize;
	int table_size = num_of_tables * 2 * (1 << this->m_historySize);

	return btb_size + history_size + table_size;
}

unsigned btb::get_num_of_branches() const {
	return this->m_num_of_branches;
}

unsigned btb::get_num_of_flushes() const {
	return this->m_num_of_flushes;
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
	curStats->size = table.get_size();
	curStats->br_num = table.get_num_of_branches();
	curStats->flush_num = table.get_num_of_flushes();
	return;
}

