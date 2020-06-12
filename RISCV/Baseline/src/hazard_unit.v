module hazard_detect(
				inst_rs1,
				inst_rs2,
				inst_op,
				inst_funct3,
				ex_rd,
				wb_rd,
				id_memread,
				ex_memread,
				wb_regwrite,
				branch_compare,
				//branch_true,
				//jal_true,
				//jalr_true,
				pcwrite,
				if_id_write,
				if_flush,
				control_zero
				);
				
	input [4:0] inst_rs1, inst_rs2;
	input [6:0] inst_op;
	input [2;0] inst_funct3;
	input [4:0] ex_rd, wb_rd; //rd in ex stage or wb stage
	input id_memread, ex_memread, wb_regwrite, branch_compare; // branch_compare means if rs1 = rs2 in branch instruction
	//output branch_true, jal_true, jalr_true; // if we need to take branch step or jump step
	output pcwrite, if_id_write, if_flush, control_zero; //control_zero = 1 when all control signals need to be zero
	
	//reg branch_true, jal_true, jalr_true;
	reg pcwrite, if_id_write, if_flush, control_zero;
	//Load-use hazard and write-read register hazard//
	always @(*) begin
		pcwrite = 1'b0;
		if_id_write = 1'b0;
		control_zero = 1'b0;
		if(ex_memread & ((ex_rd == inst_rs1) | (ex_rd == inst_rs2))) begin // load-use hazard
			pcwrite = 1'b1;
			if_id_write = 1'b1;
			control_zero = 1'b1;
		end
		else if(wb_regwrite & ((wb_rd == inst_rs1) | (wb_rd == inst_rs2))) begin // write-read hazard
			pcwrite = 1'b1;
			if_id_write = 1'b1;
			control_zero = 1'b1;
		end
		else if(inst_op == 7'b1100111 & wb_regwrite & (wb_rd == inst_rs1)) begin
			pcwrite = 1'b1;
			if_id_write = 1'b1;
			control_zero = 1'b1;
		end
	end
	////////////////////////////////////////////
	//branch or jump hazard
	always @(*) begin
		//branch_true = 1'b0;
		//jal_true = 1'b0;
		//jalr_true = 1'b0;
		if_flush = 1'b0;
		if((inst_op == 7'b1100011) & (inst_funct3 == 3'b000) & branch_compare) begin
			//branch_true = 1'b1;
			if_flush = 1'b1;
		end
		else if((inst_op == 7'b1100011) & (inst_funct3 == 3'b001) & !branch_compare) begin
			//branch_true = 1'b1;
			if_flush = 1'b1;
		end
		else if(inst_op == 7'b1101111) begin
			//jal_true = 1'b1;
			if_flush = 1'b1;
		end
		else if(inst_op == 7'b1100111) begin
			//jalr_true = 1'b1;
			if_flush = 1'b1;
		end
	end
	/////////////////////////////////////////////
endmodule
/*				
module reg_compare(
				reg1_value,
				reg2_value,
				equal
				);
	input [31:0] reg1_value, reg2_value;
	output equal;
	
	assign equal = (reg1_value == reg2_value) ? 1'b1 : 1'b0;
endmodule
	*/			