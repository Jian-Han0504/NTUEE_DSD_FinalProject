*************************
Date:2020/06/30
Author:周光照
*************************
This readme is for BPU execution.
To successfully execute the program:

RTL:
1.Make sure you are in the rtl folder
2.Put following files in the same folder:

  For BrPred test patterns:
  D_mem
  Final_tb.v
  I_mem_BrPred
  TestBed_BrPred
  slow_memory.v

  For hasHazard test patterns:
  D_mem
  Final_tb.v
  I_mem_hasHazard
  TestBed_hasHazard
  slow_memory.v

3. Run the following commands:

  For BrPred:
  source /usr/cad/cadence/cshrc
  source /usr/spring_soft/CIC/verdi.cshrc
  ncverilog Final_tb.v CHIP.v slow_memory.v +define+BrPred +access+r

  For hasHazard:
  source /usr/cad/cadence/cshrc
  source /usr/spring_soft/CIC/verdi.cshrc
  ncverilog Final_tb.v CHIP.v slow_memory.v +define+hasHazard +access+r

SYN:
1.Make sure you are in the syn folder
2.Put following files in the same folder:

  For BrPred test patterns:
  D_mem
  Final_tb.v
  I_mem_BrPred
  TestBed_BrPred
  slow_memory.v
  tsmc13.v

  For hasHazard test patterns:
  D_mem
  Final_tb.v
  I_mem_hasHazard
  TestBed_hasHazard
  slow_memory.v
  tsmc13.v

3. Run the following commands:

  For BrPred:
  source /usr/cad/cadence/cshrc
  source /usr/spring_soft/CIC/verdi.cshrc
  ncverilog Final_tb.v CHIP_syn.v slow_memory.v tsmc13.v +define+BrPred +define+SDF +access+r

  For hasHazard:
  source /usr/cad/cadence/cshrc
  source /usr/spring_soft/CIC/verdi.cshrc
  ncverilog Final_tb.v CHIP_syn.v slow_memory.v tsmc13.v +define+hasHazard +define+SDF +access+r
 




