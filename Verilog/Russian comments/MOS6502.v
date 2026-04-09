/*
 ===============================================================================================
 *                           Copyright (C) 2023-2026 andkorzh
 *
 *
 *                This program is free software; you can redistribute it and/or
 *                modify it under the terms of the GNU General Public License
 *                as published by the Free Software Foundation; either version 2
 *                of the License, or (at your option) any later version.
 *
 *                This program is distributed in the hope that it will be useful,
 *                but WITHOUT ANY WARRANTY; without even the implied warranty of
 *                MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *                GNU General Public License for more details.
 *
 *                                6502 SYNCHRONOUS CORE
 *
 *   This design is inspired by Wiki BREAKNES. I tried to replicate the design of the real
 * NMOS processor MOS 6502 as much as possible. The Logsim 6502 model was taken as the basis
 * for the design of the circuit diagram
 *
 *  author andkorzh
 *  Thanks:
 *      HardWareMan: author of the concept of synchronously core NES PPU, help & support.
 *
 *      Org (ogamespec): help & support, C++ Cycle accurate model NES, Author: Wiki BREAKNES
 *
 *      Nukeykt: help & support
 *
 ===============================================================================================
*/

// MOS6502
module MOS6502 (
// Такты
input Clk,               // Тактовый сигнал
input PHI0,              // Такт PHI0
// Входы
input BCD_OFF,           // BCD коррекция отключена
input SO,                // Вход сихронизации
input nNMI,              // Вход немаскируемого прерывания
input nIRQ,              // Вход маскируемого прерывания
input nRES,              // Сигнал сброса
input RDY,               // Сигнал готовности процессора
input [7:0]DIN,          // Шина данных (вход)
// Выходы
output PHI1,             // Такт PHI1 (выход)
output PHI2,             // Такт PHI2 (выход)
output RW,               // Сигнал записи процессора
output [7:0]DOUT,        // Шина данных (выход)
output [15:0]AB,         // Шина Адреса
output SYNC              // Выход синхронизации (Т1)
);
// Связи модулей
wire [128:0]X;           // Выходы декодера инструкций
wire [7:0]DB;            // Шина DB
wire [7:0]SB;            // Шина SB
wire [7:0]DL;            // Шина Input DатаLatch
wire [7:0]ADL;           // Шина ADL
wire [7:0]ADH;           // Шина ADH
wire [7:0]ADD;           // Шина ADD (выход АЛУ)
wire [7:0]ACC;           // Шина ADD (выход аккумулятора)
wire [7:0]FLAG;          // Шина данных флагов
wire [7:0]PCL;           // Шина младших разрядов PC
wire [7:0]PCH;           // Шина старших разрядов PC
wire [7:0]IR;            // Шина регистра инструкций
wire RESP;
wire nPRDY;
wire nTWOCYCLE;             // Инструкция 2 такта
wire IMPLIED;               // Инструкция 1 такт
wire nREADY;                // Сигнал "Процессор не готов"
wire TRES2;                 // Очистка выходов EXTENDED_COUNTER
wire nT0, nT1X, T1, nT2, nT3, nT4, nT5;   // Циклы процессора
wire T1_LATCH;              // Цикл 1 задержанный латчем PHI2
wire AVR, ACR;              // Переполнение АЛУ, выходной перенос АЛУ
wire Z_ADH0;                // Обнулить бит 0 шины ADH
wire Z_ADH17;               // Обнулить биты 1-7 шины ADH
wire SB_AC;                 // Шину SB на аккумулятор
wire ADL_ABL;               // Шину ADL на регистр адресной шины ABL
wire AC_SB;                 // Значение аккумулятора на шину SB
wire SB_DB;                 // Проброс данных между шинами DB <-> SB
wire AC_DB;                 // Аккумулятор на шину DB
wire SB_ADH;                // Проброс данных между шинами SB <-> ADH
wire DL_ADH;                // Значение защелки DL на шину ADH
wire DL_ADL;                // Значение защелки DL на шину ADL
wire ADH_ABH;               // Шину ADH на регистр адресной шины ABH
wire DL_DB;                 // Значение защелки DL на шину DB
wire Y_SB;                  // Регистр Y на шину SB
wire X_SB;                  // Регистр X на шину SB
wire S_SB;                  // Регистр S на шину SB
wire SB_Y;                  // Шина SB на регистр Y
wire SB_X;                  // Шина SB на регистр X
wire SB_S;                  // Шина SB на регистр S
wire S_S;                   // Хранение значение в регистре S
wire S_ADL;                 // Регистр S на шину ADL
wire [2:0]Z_ADL;            // Сигналы управления генератором констант
wire WRPHI1;                // Сигнал записи процессора PHI1
wire B_OUT;                 // B Флаг
wire P_DB;                  // Данные флагов на шину DB
wire PCL_DB;                // Данные PCL на шину DB
wire PCH_DB;                // Данные PCH на шину DB
wire PCL_ADL;               // Данные PCL на шину ADL
wire PCH_ADH;               // Данные PCH на шину ADH
wire PCL_PCL;               // Режим хранения данных в разрядах счетчика PCL
wire ADL_PCL;               // Загрузка данных из шины ADL
wire ADH_PCH;               // Загрузка данных из шины ADH
wire PCH_PCH;               // Режим хранения данных в разрядах счетчика PCH
wire NDB_ADD;               // Шину ~DB на вход АЛУ
wire DB_ADD;                // Шину DB на вход АЛУ
wire Z_ADD;                 // Обнулить вход А АЛУ
wire SB_ADD;                // Шину SB на вход АЛУ
wire ADL_ADD;               // Шину ADL на вход АЛУ
wire ANDS;                  // Логическое И
wire EORS;                  // Исключающее ИЛИ
wire ORS;                   // Логическое ИЛИ
wire ACIN;                  // Входной перенос АЛУ
wire SRS;                   // Сдвиг вправо
wire SUMS;                  // Сумма А + В
wire nDAA;                  // Выполнить коррекцию после сложения
wire ADD_SB7;               // Бит    7 на шину SB
wire ADD_SB06;              // Биты 0-6 на шину SB
wire ADD_ADL;               // Выход АЛУ на шину ADL
wire nDSA;                  // Выполнить коррекцию после вычитания
wire I_PC;                  // Входной перенос PC

// Переменные
reg nNMIP, nIRQPR1, nIRQP, RESPR1, RESPR2;
reg nPRDYR1, nPRDYR2;
reg [7:0]DL_LATCH;                                    // INPUT  DATA LATCH
reg [7:0]DOR_LATCH;                                   // OUTPUT DATA LATCH
reg [7:0]ABL_LATCH;
reg [7:0]ABH_LATCH;
reg [7:0]X_REG;                                       // Регистр X
reg [7:0]Y_REG;                                       // Регистр Y
reg [7:0]S_REG_LATCH1;                                // Входная защелка указателя стека
reg [7:0]S_REG;                                       // Указатель стека
reg [2:0]PHI2_DELAY;
// Комбинаторика
assign PHI1  = ~PHI0;
assign PHI2  =  PHI0;
assign RESP  = ~RESPR2;
assign nPRDY = ~nPRDYR2;
assign AB[15:0] = { ABH_LATCH[7:0], ABL_LATCH[7:0] };
//Выход данных из входной защелки
assign DL[7:0] = DL_LATCH[7:0] & {8{ PHI1 }};
assign SYNC = T1;
assign RW = ~WRPHI1;                            // Чтение/запись
wire OE;
assign OE =  ~RW & PHI2;
assign DOUT[7:0] = OE ? DOR_LATCH[7:0] : 8'hZZ; // Управление выходом шины данных
// Логика
always @(posedge Clk) begin
       PHI2_DELAY[2:0] <= { PHI2_DELAY[1:0], PHI2 };
       if (PHI1) begin
       nIRQP   <= nIRQPR1;
       RESPR2  <= RESPR1;
       nPRDYR2 <= nPRDYR1;
       DOR_LATCH[7:0] <= DB[7:0];
                 end
       if (PHI2) begin
       nNMIP         <= nNMI;
       nIRQPR1       <= nIRQ;
       RESPR1        <= nRES;
       nPRDYR1       <= RDY;
       DL_LATCH[7:0] <= DIN[7:0];
       S_REG[7:0]    <= S_REG_LATCH1[7:0];
                 end
       if ( SB_S | ( S_S & S_SB )) S_REG_LATCH1[7:0] <= SB[7:0];  // BB Hack
       if ( ADL_ABL & PHI1 ) ABL_LATCH[7:0] <= ADL[7:0];
       if ( ADH_ABH & PHI1 ) ABH_LATCH[7:0] <= ADH[7:0];
       if ( SB_X ) X_REG[7:0] <= SB[7:0];
       if ( SB_Y ) Y_REG[7:0] <= SB[7:0];
                      end

// Вложенные модули
PREDECODE_IR MOD_PREDECODE_IR(
Clk,
PHI1,
nREADY,
B_OUT,
DL_LATCH[7:0],
T1_LATCH,
IMPLIED,
nTWOCYCLE,
IR[7:0]
);

EXTENDED_COUNTER MOD_EXTENDED_COUNTER(
Clk,
PHI1,
PHI2,
T1,
nREADY,
TRES2,
T1_LATCH,
nT2,
nT3,
nT4,
nT5
);

DECODER MOD_DECODER(
nT0,
nT1X,
nT2,
nT3,
nT4,
nT5,
nPRDY,
IR[7:0],
X[128:0]
);

RANDOM_LOGIC MOD_RANDOM_LOGIC(
Clk,
PHI1,
PHI2,
X[128:0],
DB[7:0],
SO,
nIRQP,
nNMIP,
RESP,
AVR,
ACR,
IR[5],
RDY,
nTWOCYCLE,
IMPLIED,
FLAG[7:0],
Z_ADH0,
Z_ADH17,
SB_AC,
ADL_ABL,
AC_SB,
SB_DB,
AC_DB,
SB_ADH,
DL_ADH,
DL_ADL,
ADH_ABH,
DL_DB,
Y_SB,
X_SB,
S_SB,
SB_Y,
SB_X,
SB_S,
S_S,
S_ADL,
Z_ADL[2:0],
WRPHI1,
nT0,
T1,
nT1X,
TRES2,
nREADY,
B_OUT,
P_DB,
PCL_DB,
PCH_DB,
PCL_ADL,
PCH_ADH,
PCL_PCL,
ADL_PCL,
ADH_PCH,
PCH_PCH,
NDB_ADD,
DB_ADD,
Z_ADD,
SB_ADD,
ADL_ADD,
ANDS,
ORS,
SRS,
EORS,
SUMS,
ACIN,
nDAA,
ADD_SB7,
ADD_SB06,
ADD_ADL,
nDSA,
I_PC
);

BUS_MUX MOD_BUS_MUX(
Z_ADL[2:0],
Z_ADH0,
Z_ADH17,
SB_DB,
PCL_DB,
PCH_DB,
P_DB,
AC_DB,
AC_SB,
SB_AC,
ADD_ADL,
ADD_SB06,
ADD_SB7,
Y_SB,
X_SB,
S_SB,
SB_ADH,
S_ADL,
DL_ADL,
DL_ADH,
DL_DB,
PCL_ADL,
PCH_ADH,
DL[7:0],
PCL[7:0],
PCH[7:0],
FLAG[7:0],
ADD[7:0],
ACC[7:0],
Y_REG[7:0],
X_REG[7:0],
S_REG[7:0],
DB[7:0],
SB[7:0],
ADL[7:0],
ADH[7:0]
);

ALU MOD_ALU(
Clk,
PHI2,
Z_ADD,
SB[7:0],
SB_ADD,
DB[7:0],
NDB_ADD,
DB_ADD,
ADL[7:0],
ADL_ADD,
ACIN,
ANDS,
ORS,
EORS,
SRS,
SUMS,
SB_AC,
BCD_OFF ? 1'b1 : nDAA,
BCD_OFF ? 1'b1 : nDSA,
ACC[7:0],
ADD[7:0],
ACR,
AVR
);

PC MOD_PC(
Clk,
PHI2,
I_PC,
PCL_PCL,
ADL_PCL,
ADL[7:0],
PCH_PCH,
ADH_PCH,
ADH[7:0],
PCL[7:0],
PCH[7:0]
);
// Конец модуля MOS6502
endmodule

//===============================================================================================
// Модуль предварительного декодирования инструкций и регистра инструкций
//===============================================================================================
module PREDECODE_IR(
// Такты
input Clk,               // Тактовый сигнал
input PHI1,              // Такт PHI1
// Входы
input nREADY,            // Сигнал "Процессор не готов"
input B_OUT,             // B Флаг
input [7:0]DL_LATCH,     // Данные входной защелки
input T1_LATCH,          // Цикл 1 задержанный латчем PHI2
// Выходы
output IMPLIED,          // Инструкция 1 такт
output nTWOCYCLE,        // Инструкция 2 такта
output reg [7:0]IR       // Выход регистра инструкций
);
// Комбинаторика
wire Z_IR;
wire [7:0]PD;
assign Z_IR = ~( ~T1_LATCH | nREADY ) & B_OUT;
assign PD[7:0]   =  DL_LATCH[7:0] & { 8 { Z_IR }};
assign IMPLIED   = ~( PD[0] | PD[2] | ~PD[3] );
assign nTWOCYCLE = ~(( IMPLIED & ( PD[1] | PD[4] | PD[7] )) | ~( ~PD[0] | PD[2] | ~PD[3] | PD[4] ) | ~( PD[0] | PD[2] | PD[3] | PD[4] | ~PD[7] ));
// Логика
always @(posedge Clk) begin
       if (PHI1 & ~( ~T1_LATCH | nREADY )) IR[7:0] <= PD[7:0];
                      end
// Конец модуля предварительного декодирования инструкций и регистра инструкций
endmodule

//===============================================================================================
// Модуль расширенного счетчика циклов
//===============================================================================================
module EXTENDED_COUNTER(
// Такты
input Clk,             // Тактовый сигнал
input PHI1,            // Такт PHI1
input PHI2,            // Такт PHI2
// Входы
input T1,              // Цикл 1
input nREADY,          // Сигнал "Процессор не готов"
input TRES2,           // Очистка выходов EXTENDED_COUNTER
// Выходы
output reg T1_LATCH,   // Выход Цикл 1 латч PHI2
output nT2,            // Выход Цикл 2
output nT3,            // Выход Цикл 3
output nT4,            // Выход Цикл 4
output nT5             // Выход Цикл 5
);
// Переменные
reg [3:0]LATCH1;        // 1-й латч бита сдвигового регистра
reg [3:0]LATCH2;        // 2-й латч бита сдвигового регистра
// Комбинаторика
assign {nT5, nT4, nT3, nT2} = {4{ TRES2 }} | LATCH1[3:0];
// Логика
always @(posedge Clk) begin
       if (PHI1)  LATCH1[3:0] <= nREADY ? LATCH2[3:0] : {LATCH2[2:0], ~T1_LATCH};
       if (PHI2) {LATCH2[3:0], T1_LATCH} <= { nT5, nT4, nT3, nT2, T1 };
                      end
// Конец модуля расширенного счетчика циклов
endmodule

//===============================================================================================
// Модуль декодера инструкций
//===============================================================================================
module DECODER(
input nT0,        // Вход Цикл 0
input nT1X,       // Вход Цикл 1X
input nT2,        // Вход Цикл 2
input nT3,        // Вход Цикл 3
input nT4,        // Вход Цикл 4
input nT5,        // Вход Цикл 5
input nPRDY,      // Вход Процессор не готов задержанный на 1 цикл
input  [7:0]IR,   // Данные регистра инструкций
output [128:0]X   // Выход декодера инструкций
);
// Комбинаторика
wire IR01;
assign IR01 = IR[0] | IR[1];
wire PUSHP;
assign X[0]   = ~( ~IR[7] |  IR[6] |  IR[5] | ~IR[2] |  IR01  );                               // Tx  STY
assign X[1]   = ~(  nT3   | ~IR[4] |  IR[3] |  IR[2] | ~IR[0] );                               // T3 IND_Y
assign X[2]   = ~(  nT2   | ~IR[4] | ~IR[3] |  IR[2] | ~IR[0] );                               // T2 ABS_Y
assign X[3]   = ~(  nT0   | ~IR[7] |  IR[5] |  IR[4] | ~IR[3] |  IR[2] |  IR01  );             // T0 DEY_INY
assign X[4]   = ~(  nT0   | ~IR[7] |  IR[6] |  IR[5] | ~IR[4] | ~IR[3] |  IR[2] |  IR01  );    // T0 TYA
assign X[5]   = ~(  nT0   | ~IR[7] | ~IR[6] |  IR[5] |  IR[4] |  IR01  );                      // T0 CPY_INY
assign X[6]   = ~(  nT2   | ~IR[4] | ~IR[2] );                                                 // T2 ANY_X
assign X[7]   = ~( ~IR[7] |  IR[6] | ~IR[1] );                                                 // Tx LDX_STX
assign X[8]   = ~(  nT2   |  IR[4] |  IR[3] |  IR[2] | ~IR[0] );                               // T2 X_IND
assign X[9]   = ~(  nT0   | ~IR[7] |  IR[6] |  IR[5] |  IR[4] | ~IR[3] |  IR[2] | ~IR[1] );    // T0 TXA
assign X[10]  = ~(  nT0   | ~IR[7] | ~IR[6] |  IR[5] |  IR[4] | ~IR[3] |  IR[2] | ~IR[1] );    // T0 DEX
assign X[11]  = ~(  nT0   | ~IR[7] | ~IR[6] | ~IR[5] |  IR[4] |  IR01  );                      // T0 CPX_INX
assign X[12]  = ~( ~IR[7] |  IR[6] |  IR[5] | ~IR[1] );                                        // Tx STX
assign X[13]  = ~(  nT0   | ~IR[7] |  IR[6] |  IR[5] | ~IR[4] | ~IR[3] |  IR[2] | ~IR[1] );    // T0 TXS
assign X[14]  = ~(  nT0   | ~IR[7] |  IR[6] | ~IR[5] | ~IR[1] );                               // T0 TAX_LDX_TSX
assign X[15]  = ~(  nT1X  | ~IR[7] | ~IR[6] |  IR[5] |  IR[4] | ~IR[3] |  IR[2] | ~IR[1] );    // T1 DEX
assign X[16]  = ~(  nT1X  | ~IR[7] | ~IR[6] | ~IR[5] |  IR[4] | ~IR[3] |  IR[2] |  IR01  );    // T1 INX
assign X[17]  = ~(  nT0   | ~IR[7] |  IR[6] | ~IR[5] | ~IR[4] | ~IR[3] |  IR[2] | ~IR[1] );    // T0 TSX
assign X[18]  = ~(  nT1X  | ~IR[7] |  IR[5] |  IR[4] | ~IR[3] |  IR[2] |  IR01  );             // T1 DEY_INY
assign X[19]  = ~(  nT0   | ~IR[7] |  IR[6] | ~IR[5] | ~IR[2] |  IR01  );                      // T0 LDY1
assign X[20]  = ~(  nT0   | ~IR[7] |  IR[6] | ~IR[5] |  IR[4] |  IR01  );                      // T0 LDY2_TAY
assign X[21]  = ~(  nT0   |  IR[7] |  IR[6] | ~IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01  );    // T0 JSR
assign X[22]  = ~(  nT5   |  IR[7] |  IR[6] |  IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01  );    // T5 INT (BRK5)
assign X[23]  = ~(  nT0   |  IR[7] |  IR[5] |  IR[4] | ~IR[3] |  IR[2] |  IR01  );             // T0 PSHA_PSHP
assign X[24]  = ~(  nT4   |  IR[7] | ~IR[6] | ~IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01  );    // T4 RTS
assign X[25]  = ~(  nT3   |  IR[7] | ~IR[5] |  IR[4] | ~IR[3] |  IR[2] |  IR01  );             // T3 PULA_PULP
assign X[26]  = ~(  nT5   |  IR[7] | ~IR[6] |  IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01  );    // T5 RTI
assign X[27]  = ~(  IR[7] | ~IR[6] | ~IR[5] | ~IR[1] );                                        // Tx ROR_RORA
assign X[28]  = ~   nT2;                                                                       // T2
assign X[29]  = ~(  nT0   |  IR[7] | ~IR[6] |  IR[5] | ~IR[0] );                               // T0 EOR
assign X[30]  = ~(  IR[7] | ~IR[6] |  IR[4] | ~IR[3] | ~IR[2] |  IR01  );                      // Tx JMP
assign X[31]  = ~(  nT2   |  IR[4] | ~IR[3] | ~IR[2] );                                        // T2 EXT
assign X[32]  = ~(  nT0   |  IR[7] |  IR[6] |  IR[5] | ~IR[0] );                               // T0 ORA
assign X[33]  = ~(  nT2   |  IR[3] );                                                          // T2 ANY_ABS_N
assign X[34]  = ~   nT0;                                                                       // T0
assign X[35]  = ~(  nT2   |  IR[7] |  IR[4] |  IR[2] |  IR01  );                               // T2 STACK
assign X[36]  = ~(  nT3   |  IR[7] |  IR[4] |  IR01  );                                        // T3 STACK
assign X[37]  = ~(  nT4   |  IR[7] |  IR[6] |  IR[4] |  IR[3] |  IR[2] |  IR01  );             // T4 JSR_INT
assign X[38]  = ~(  nT4   |  IR[7] | ~IR[6] |  IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01  );    // T4 RTI
assign X[39]  = ~(  nT3   |  IR[4] |  IR[3] |  IR[2] | ~IR[0] );                               // T3 X_IND
assign X[40]  = ~(  nT4   | ~IR[4] |  IR[3] |  IR[2] | ~IR[0] );                               // T4 IND_Y
assign X[41]  = ~(  nT2   | ~IR[4] |  IR[3] |  IR[2] | ~IR[0] );                               // T2 IND_Y
assign X[42]  = ~(  nT3   | ~IR[4] | ~IR[3] );                                                 // T3 ABS_X_Y
assign X[43]  = ~(  IR[7] | ~IR[5] |  IR[4] | ~IR[3] |  IR[2] |  IR01  );                      // Tx PULA_PULP
assign X[44]  = ~( ~IR[7] | ~IR[6] | ~IR[5] | ~IR[1] );                                        // Tx INC 
assign X[45]  = ~(  nT4   |  IR[4] |  IR[3] |  IR[2] | ~IR[0] );                               // T4 X_IND
assign X[46]  = ~(  nT3   | ~IR[4] |  IR[3] |  IR[2] | ~IR[0] );                               // T3 IND_Y
assign X[47]  = ~(  IR[7] | ~IR[6] |  IR[4] |  IR[3] |  IR[2] |  IR01  );                      // Tx RTI_RTS
assign X[48]  = ~(  nT2   |  IR[7] |  IR[6] | ~IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01  );    // T2 JSR
assign X[49]  = ~(  nT0   | ~IR[7] | ~IR[6] |  IR[4] |  IR01  );                               // T0 CPX_Y_INXD_Y
assign X[50]  = ~(  nT0   | ~IR[7] | ~IR[6] |  IR[5] | ~IR[0] );                               // T0 CMP
assign X[51]  = ~(  nT0   | ~IR[7] | ~IR[6] | ~IR[5] | ~IR[0] );                               // T0 SBC
assign X[52]  = ~(  nT0   | ~IR[6] | ~IR[5] | ~IR[0] );                                        // T0 ADC_SBC
assign X[53]  = ~(  IR[7] |  IR[6] | ~IR[5] | ~IR[1] );                                        // Tx ROL_ROLA
assign X[54]  = ~(  nT3   |  IR[7] | ~IR[6] |  IR[4] | ~IR[3] | ~IR[2] |  IR01  );             // T3 JMP
assign X[55]  = ~(  IR[7] |  IR[6] | ~IR[1] );                                                 // Tx ROLx_ASLx
assign X[56]  = ~(  nT5   |  IR[7] |  IR[6] | ~IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01  );    // T5 JSR
assign X[57]  = ~(  nT2   |  IR[7] |  IR[4] |  IR[2] |  IR01  );                               // T2 RTI_INT_PHx
assign X[58]  = ~(  nT0   | ~IR[7] |  IR[6] |  IR[5] | ~IR[4] | ~IR[3] |  IR[2] |  IR01  );    // T0 TYA
assign X[59]  = ~(  nT1X  |  IR[7] | ~IR[0] );                                                 // T1 AND_EOR_OR_ADC
assign X[60]  = ~(  nT1X  | ~IR[6] | ~IR[5] | ~IR[0] );                                        // T1 ADC_SBC
assign X[61]  = ~(  nT1X  |  IR[7] |  IR[4] | ~IR[3] |  IR[2] | ~IR[1] );                      // T1 ASLA_ROLx_A_LSRA
assign X[62]  = ~(  nT0   | ~IR[7] |  IR[6] |  IR[5] |  IR[4] | ~IR[3] |  IR[2] | ~IR[1] );    // T0 TXA
assign X[63]  = ~(  nT0   |  IR[7] | ~IR[6] | ~IR[5] |  IR[4] | ~IR[3] |  IR[2] |  IR01  );    // T0 PLA
assign X[64]  = ~(  nT0   | ~IR[7] |  IR[6] | ~IR[5] | ~IR[0] );                               // T0 LDA
assign X[65]  = ~(  nT0   | ~IR[0] );                                                          // T0 G1
assign X[66]  = ~(  nT0   | ~IR[7] |  IR[6] | ~IR[5] |  IR[4] | ~IR[3] |  IR[2] |  IR01  );    // T0 TAY
assign X[67]  = ~(  nT0   |  IR[7] |  IR[4] | ~IR[3] |  IR[2] | ~IR[1] );                      // T0 ASLA_ROLx_A_LSRA
assign X[68]  = ~(  nT0   | ~IR[7] |  IR[6] | ~IR[5] |  IR[4] | ~IR[3] |  IR[2] | ~IR[1] );    // T0 TAX
assign X[69]  = ~(  nT0   |  IR[7] |  IR[6] | ~IR[5] |  IR[4] | ~IR[2] |  IR01  );             // T0 BIT
assign X[70]  = ~(  nT0   |  IR[7] |  IR[6] | ~IR[5] | ~IR[0] );                               // T0 AND
assign X[71]  = ~(  nT4   | ~IR[4] | ~IR[3] );                                                 // T4 ABS_X_Y
assign X[72]  = ~(  nT5   | ~IR[4] |  IR[3] |  IR[2] | ~IR[0] );                               // T5 IND_Y
assign X[73]  = ~(  nT0   | ~IR[4] |  IR[3] |  IR[2] |  IR01  |  nPRDY );                      // T0 BR
assign X[74]  = ~(  nT2   |  IR[7] | ~IR[6] |  IR[5] |  IR[4] | ~IR[3] |  IR[2] |  IR01 );     // T2 PSHA
assign X[75]  = ~(  nT0   |  IR[7] | ~IR[6] |  IR[4] | ~IR[3] |  IR[2] | ~IR[1] );             // T0 LSRA
assign X[76]  = ~(  IR[7] | ~IR[6] | ~IR[1] );                                                 // Tx LSR_LSRA
assign X[77]  = ~(  nT2   |  IR[7] |  IR[6] |  IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01 );     // T2 INT
assign X[78]  = ~(  nT3   |  IR[7] |  IR[6] | ~IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01 );     // T3 JSR
assign X[79]  = ~( ~IR[7] |  IR[6] |  IR[5] | ~IR[0] );                                        // Tx STA
assign X[80]  = ~(  nT2   | ~IR[4] |  IR[3] |  IR[2] |  IR01  );                               // T2 BR
assign X[81]  = ~(  nT2   |  IR[3] | ~IR[2] );                                                 // T2 ANY_ZP
assign X[82]  = ~(  nT2   |  IR[3] |  IR[2] | ~IR[0] );                                        // T2 ANY_IND
assign X[83]  = ~(  nT2   | ~IR[3] |  PUSHP );                                                 // T2 ANY_ABS
assign X[84]  = ~(  nT5   |  IR[7] | ~IR[6] | ~IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01 );     // T5 RTS
assign X[85]  = ~   nT4;                                                                       // T4
assign X[86]  = ~   nT3;                                                                       // T3
assign X[87]  = ~(  nT0   |  IR[7] |  IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01 );              // T0 RTI_INT
assign X[88]  = ~(  nT0   |  IR[7] | ~IR[6] |  IR[4] | ~IR[3] | ~IR[2] |  IR01 );              // T0 JMP
assign X[89]  = ~(  nT5   |  IR[4] |  IR[3] |  IR[2] | ~IR[0] );                               // T5 X_IND
assign X[90]  = ~(  nT3   | ~IR[3] |  PUSHP );                                                 // T3 ANY_ABS
assign X[91]  = ~(  nT4   | ~IR[4] |  IR[3] |  IR[2] | ~IR[0] );                               // T4 IND_Y
assign X[92]  = ~(  nT3   | ~IR[4] | ~IR[3] );                                                 // T3 ABS_X_Y
assign X[93]  = ~(  nT3   | ~IR[4] |  IR[3] |  IR[2] |  IR01  );                               // T3 BR
assign X[94]  = ~(  IR[7] |  IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01  );                      // Tx RTI_INT
assign X[95]  = ~(  IR[7] |  IR[6] | ~IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01 );              // Tx JSR
assign X[96]  = ~(  IR[7] | ~IR[6] |  IR[4] | ~IR[3] | ~IR[2] |  IR01  );                      // Tx JMP
assign X[97]  = ~( ~IR[7] |  IR[6] |  IR[5] );                                                 // Tx STA_STY_STX
assign X[98]  = ~(  nT4   |  IR[7] |  IR[6] |  IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01 );     // T4 INT
assign X[99]  = ~(  nT2   |  IR[7] |  IR[6] |  IR[5] |  IR[4] | ~IR[3] |  IR[2] |  IR01 );     // T2 PSHP
assign X[100] = ~(  nT2   |  IR[7] |  IR[5] |  IR[4] | ~IR[3] |  IR[2] |  IR01  );             // T2 PSHA_PSHP
assign X[101] = ~(  nT4   |  IR[7] | ~IR[6] |  IR[4] | ~IR[3] | ~IR[2] |  IR01  );             // T4 JMP
assign X[102] = ~(  nT5   |  IR[7] | ~IR[6] |  IR[4] |  IR[3] |  IR[2] |  IR01  );             // T5 RTI_RTS
assign X[103] = ~(  nT5   |  IR[7] |  IR[6] | ~IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01 );     // T5 JSR
assign X[104] = ~(  nT2   |  IR[7] | ~IR[6] |  IR[5] |  IR[4] | ~IR[3] | ~IR[2] |  IR01 );     // T2 JMP_ABS
assign X[105] = ~(  nT3   |  IR[7] | ~IR[5] |  IR[4] | ~IR[3] |  IR[2] |  IR01  );             // T3 PULA_PULP
assign X[106] = ~( ~IR[6] | ~IR[1] );                                                          // Tx LSRx_DEC_INC
assign X[107] = ~(  IR[7] |  IR[6] | ~IR[1] );                                                 // Tx ROLx_ASLx
assign X[108] = ~(  nT0   |  IR[7] | ~IR[6] | ~IR[4] | ~IR[3] |  IR[2] |  IR01  );             // T0 CLI_SEI
assign X[109] = ~(  nT1X  |  IR[7] |  IR[6] | ~IR[5] |  IR[4] | ~IR[2] |  IR01  );             // T1 BIT
assign X[110] = ~(  nT0   |  IR[7] |  IR[6] | ~IR[4] | ~IR[3] |  IR[2] |  IR01  );             // T0 CLC_SEC
assign X[111] = ~(  nT3   | ~IR[4] |  IR[3] | ~IR[2] );                                        // T3 ZP_X
assign X[112] = ~(  nT1X  | ~IR[6] | ~IR[5] | ~IR[0] );                                        // T1 ADC_SBC
assign X[113] = ~(  nT0   |  IR[7] |  IR[6] | ~IR[5] |  IR[4] | ~IR[2] |  IR01  );             // T0 BIT
assign X[114] = ~(  nT0   |  IR[7] |  IR[6] | ~IR[5] |  IR[4] | ~IR[3] |  IR[2] |  IR01 );     // T0 PULP
assign X[115] = ~(  nT4   |  IR[7] | ~IR[6] |  IR[5] |  IR[4] |  IR[3] |  IR[2] |  IR01 );     // T4 RTI
assign X[116] = ~(  nT1X  | ~IR[7] | ~IR[6] |  IR[5] | ~IR[0] );                               // T1 CMP
assign X[117] = ~(  nT1X  | ~IR[7] | ~IR[6] |  IR[4] | ~IR[3] | ~IR[2] |  IR01  );             // T1 CPX1_CPY1
assign X[118] = ~(  nT1X  |  IR[7] |  IR[6] |  IR[4] | ~IR[3] |  IR[2] | ~IR[1] );             // T1 ASLA_ROLA
assign X[119] = ~(  nT1X  | ~IR[7] | ~IR[6] |  IR[4] |  IR[3] |  IR01  );                      // T1 CPX2_CPY2
assign X[120] = ~(  nT0   | ~IR[7] | ~IR[6] | ~IR[4] | ~IR[3] |  IR[2] |  IR01  );             // T0 CLD_SED
assign X[121] = ~   IR[6];                                                                     // Tx I6
assign X[122] = ~(  nT3   |  IR[4] | ~IR[3] | ~IR[2] );                                        // T3 ABS
assign X[123] = ~(  nT2   |  IR[4] |  IR[3] | ~IR[2] );                                        // T2 ZP
assign X[124] = ~(  nT5   |  IR[3] |  IR[2] | ~IR[0] );                                        // T5 ANY_IND
assign X[125] = ~(  nT4   | ~IR[4] | ~IR[3] );                                                 // T4 ABS_X_Y
assign X[126] = ~   IR[7];                                                                     // Tx I7
assign X[127] = ~( ~IR[7] |  IR[6] | ~IR[5] | ~IR[4] | ~IR[3] |  IR[2] |  IR01  );             // Tx CLV
assign X[128] = ~( ~IR[3] |  IR[2] |  IR[0] |  PUSHP );                                        // Tx IMPL
assign PUSHP  = ~(  IR[7] |  IR[4] | ~IR[3] |  IR[2] |  IR01  );                               // Tx PSH_PUL
endmodule // Конец модуля Decoder

//===============================================================================================
// Модуль рандомной логики
//===============================================================================================
module RANDOM_LOGIC (
// Такты
input Clk,         // Тактовый сигнал
input PHI1,        // Такт PHI1
input PHI2,        // Такт PHI2
// Входы
input [128:0]X,    // Шина Декодера
input [7:0]DB,     // Шина DB
input SO,          // Вход синхронизации
input nIRQP,       //
input nNMIP,       //
input RESP,        //
input AVR,         //
input ACR,         //
input IR5,         // Бит 5 регистра инструкций
input RDY,         //
input nTWOCYCLE,   //
input IMPLIED,     //
// Выходы
output [7:0]FLAG,  // Выход шины данных флагов
output Z_ADH0,     // Обнулить бит 0 шины ADH
output Z_ADH17,    // Обнулить биты 1-7 шины ADH
output SB_AC,      // Шину SB на аккумулятор
output ADL_ABL,    // Шину ADL на регистр адресной шины ABL
output AC_SB,      // Значение аккумулятора на шину SB
output SB_DB,      // Проброс данных между шинами DB <-> SB
output AC_DB,      // Аккумулятор на шину DB
output SB_ADH,     // Проброс данных между шинами SB <-> ADH
output DL_ADH,     // Значение защелки DL на шину ADH
output DL_ADL,     // Значение защелки DL на шину ADL
output ADH_ABH,    // Шину ADH на регистр адресной шины ABH
output DL_DB,      // Значение защелки DL на шину DB
output Y_SB,       // Регистр Y на шину SB
output X_SB,       // Регистр X на шину SB
output S_SB,       // Регистр S на шину SB
output SB_Y,       // Шина SB на регистр Y
output SB_X,       // Шина SB на регистр X
output SB_S,       // Шина SB на регистр S
output S_S,        // Хранение значение в регистре S
output S_ADL,      // Регистр S на шину ADL
output [2:0]Z_ADL, //
output WRPHI1,     // Сигнал записи процессора
output nT0,        // Цикл ~0
output T1,         // Цикл  1
output nT1X,       // Цикл ~T1X
output TRES2,      //
output nREADY,     // Сигнал "Процессор не готов"
output B_OUT,      // Флаг B
output P_DB,       // Данные флагов на шину DB
output PCL_DB,     // Данные PCL на шину DB
output PCH_DB,     // Данные PCH на шину DB
output PCL_ADL,    // Данные PCL на шину ADL
output PCH_ADH,    // Данные PCH на шину ADH
output PCL_PCL,    // Режим хранения данных в разрядах счетчика PCL
output ADL_PCL,    // Загрузка данных из шины ADL
output ADH_PCH,    // Загрузка данных из шины ADH
output PCH_PCH,    // Режим хранения данных в разрядах счетчика PCH
output NDB_ADD,    // Шину ~DB на вход АЛУ
output DB_ADD,     // Шину DB на вход АЛУ
output Z_ADD,      // Обнулить вход АЛУ
output SB_ADD,     // Шину SB на вход АЛУ
output ADL_ADD,    // Шину ADL на вход АЛУ
output ANDS,       // Логическое  И
output ORS,        // Логическое  ИЛИ
output SRS,        // Сдвиг вправо
output EORS,       // Исключающее ИЛИ
output SUMS,       // Сумма А и В
output ACIN,       // Входной перенос АЛУ
output nDAA,       // Выполнить коррекцию после сложения
output ADD_SB7,    // Бит    7 на шину SB
output ADD_SB06,   // Биты 0-6 на шину SB
output ADD_ADL,    // Выход АЛУ на шину ADL
output nDSA,       // Выполнить коррекцию после вычитания
output I_PC        // Входной перенос PC
);
// Связи модулей
wire BRK6E;
wire BRFW;
wire nBRTAKEN;
wire nC_OUT;
wire nI_OUT;
wire nD_OUT;
wire T0;
wire T6;
wire T7;
wire SR;
wire ZTST;
wire PGX;
wire DORES;
wire STXY;
wire nSBXY;
wire STKOP;
wire PC_DB;
wire nADL_PCL;
wire DL_PCH;
wire nPCH_PCH;
wire AND;
wire INC_SB;
wire ACRL2;
wire STOR;
wire nREADYR;

// Модуль контроля регистров
REGS_CONTROL MOD_REGS_CONTROL(
Clk,
PHI2,
X[128:0],
STOR,
nREADY,
nREADYR,
Y_SB,
STXY,
X_SB,
S_SB,
SB_X,
nSBXY,
SB_Y,
STKOP,
S_ADL,
SB_S,
S_S
);

// Модуль упрравления шинами
BUS_CONTROL MOD_BUS_CONTROL(
Clk,
PHI2,
X[128:0],
T0,
T1,
T6,
T7,
nSBXY,
AND,
STXY,
STOR,
DL_PCH,
Z_ADL[0],
nPCH_PCH,
ACRL2,
nREADY,
nREADYR,
BRK6E,
INC_SB,
Z_ADH0,
DL_ADL,
Z_ADH17,
SB_AC,
ZTST,
ADL_ABL,
AC_SB,
SB_DB,
AC_DB,
PGX,
SB_ADH,
DL_ADH,
ADH_ABH,
DL_DB
);

// Модуль управления программным счетчиком
PC_SETUP MOD_PC_SETUP(
Clk,
PHI1,
PHI2,
X[128:0],
T0,
T1,
nREADY,
nREADYR,
PCL_DB,
PC_DB,
PCH_DB,
PCL_ADL,
PCH_ADH,
PCL_PCL,
ADL_PCL,
nADL_PCL,
DL_PCH,
ADH_PCH,
PCH_PCH,
nPCH_PCH
);

// Модуль управления АЛУ
ALU_SETUP MOD_ALU_SETUP(
Clk,
PHI1,
PHI2,
X[128:0],
nREADY,
nREADYR,
PGX,
nD_OUT,
nC_OUT,
T0,
T1,
T6,
T7,
BRK6E,
STKOP,
BRFW,
Z_ADD,
SB_ADD,
NDB_ADD,
DB_ADD,
ADL_ADD,
AND,
ANDS,
ORS,
SRS,
EORS,
SUMS,
ACIN,
ADD_SB06,
ADD_SB7,
ADD_ADL,
nDSA,
nDAA,
INC_SB,
SR
);

// Модуль диспетчера
DISPATCH MOD_DISPATCH(
Clk,
PHI1,
PHI2,
X[128:0],
ACR,
BRFW,
RDY,
BRK6E,
RESP,
DORES,
PC_DB,
nBRTAKEN,
nTWOCYCLE,
nADL_PCL,
IMPLIED,
B_OUT,
ACRL2,
T6,
T7,
STOR,
TRES2,
nREADY,
nREADYR,
WRPHI1,
T1,
nT0,
T0,
nT1X,
I_PC
);

// Модуль ФЛАГОВ
FLAGS MOD_FLAGS(
Clk,
PHI1,
PHI2,
X[128:0],
DB[7:0],
IR5,
nREADY,
T7,
SR,
ZTST,
B_OUT,
ACR,
BRK6E,
AVR,
SO,
FLAG[7:0],
P_DB,
nD_OUT,
nI_OUT,
nC_OUT,
nBRTAKEN,
BRFW
);

// Модуль обработки прерываний
INT_RESET_CONTROL MOD_INT_RESET_CONTROL(
Clk,
PHI1,
PHI2,
X[128:0],
nREADY,
RESP,
nNMIP,
nI_OUT,
nIRQP,
T0,
BRK6E,
Z_ADL[2:0],
DORES,
B_OUT
);

// Конец модуля RANDOM_LOGIC
endmodule

//===============================================================================================
// Модуль управления регистрами
//===============================================================================================
module REGS_CONTROL(
// Такты
input Clk,         // Тактовый сигнал
input PHI2,        // Такт PHI2
// Входы
input [128:0]X,    // Шина Декодера
input STOR,        // Stor operation
input nREADY,      // Сигнал "Процессор не готов"
input nREADYR,     // Сигнал "Процессор не готов" латч PHI1
// Выходы
output Y_SB,       // Регистр Y на шину SB
output STXY,       //
output X_SB,       // Регистр X на шину SB
output reg S_SB,   // Регистр S на шину SB
output SB_X,       // Шина SB на регистр X
output nSBXY,      //
output SB_Y,       // Шина SB на регистр Y
output STKOP,      // Stack operation
output S_ADL,      // Регистр S на шину ADL
output SB_S,       // Шина SB на регистр S
output S_S         // Хранение значение в регистре S
);
// Переменные
reg Y_SBR, X_SBR, SB_XR, SB_YR, S_ADLR, SB_SR;       // Выходные латчи модуля
// Комбинаторика
assign STXY  = ~(( X[0] & STOR ) | ( X[12] & STOR ));
assign nSBXY = ~( ~( X[14] | X[15] | X[16] ) & ~( X[18] | X[19] | X[20] ));
assign STKOP = ~( nREADYR | ~( X[21] | X[22] | X[23] | X[24] | X[25] | X[26] ));
//Выходные сигналы модуля
assign Y_SB  = ~( Y_SBR | PHI2 );
assign X_SB  = ~( X_SBR | PHI2 );
assign SB_X  = ~( SB_XR | PHI2 );
assign SB_Y  = ~( SB_YR | PHI2 );
assign S_ADL = ~S_ADLR;
assign SB_S  = ~(  SB_SR | PHI2 );
assign S_S   = ~( ~SB_SR | PHI2 );
// Логика
always @(posedge Clk) begin
       if (PHI2) begin
       Y_SBR  <= ~( X[1] | X[2] | X[3]  | X[4]  | X[5]  | ( X[0]  &  STOR ) | ( X[6] &  X[7] ));
       X_SBR  <= ~( X[8] | X[9] | X[10] | X[11] | X[13] | ( X[12] &  STOR ) | ( X[6] & ~X[7] ));
       S_SB   <=    X[17];
       SB_XR  <= ~( X[14] | X[15] | X[16] );
       SB_YR  <= ~( X[18] | X[19] | X[20] );
       S_ADLR <= ~(( ~nREADYR & X[21] ) | X[35] );
       SB_SR  <= ~( STKOP | X[13] | ~( ~X[48] | nREADY ));
                 end
                      end
// Конец модуля REGS_CONTROL
endmodule

//===============================================================================================
// Модуль управления шинами
//===============================================================================================
module BUS_CONTROL(
// Такты
input Clk,        // Тактовый сигнал
input PHI2,       // Такт PHI2
// Входы
input [128:0]X,   // Шина Декодера
input T0,         // Цикл 0
input T1,         // Цикл 1
input T6,         // Цикл 6
input T7,         // Цикл 7
input nSBXY,      //
input AND,        // Операция логического И
input STXY,       //
input STOR,       //
input DL_PCH,     // Значение защелки DL на шину PCH
input Z_ADL0,     // Обнуление разряда 0 шины ADL
input nPCH_PCH,   //
input ACRL2,      // Вход регистра переноса ALU
input nREADY,     // Сигнал "Процессор не готов"
input nREADYR,    // Сигнал "Процессор не готов" латч PHI1
input BRK6E,      // Break Cycle 6 End
input INC_SB,     //
// Выходы
output Z_ADH0,    // Обнулить бит 0 шины ADH
output DL_ADL,    // Значение защелки DL на шину ADL
output Z_ADH17,   // Обнулить биты 1-7 шины ADH
output SB_AC,     // Шину SB на аккумулятор
output ZTST,      //
output ADL_ABL,   // Шину ADL на регистр адресной шины ABL
output AC_SB,     // Значение аккумулятора на шину SB
output SB_DB,     // Проброс данных между шинами DB <-> SB
output AC_DB,     // Аккумулятор на шину DB
output PGX,       //
output SB_ADH,    // Проброс данных между шинами SB <-> ADH
output DL_ADH,    // Значение защелки DL на шину ADH
output ADH_ABH,   // Шину ADH на регистр адресной шины ABH
output DL_DB      // Значение защелки DL на шину DB
);
// Переменные
reg Z_ADH0R, Z_ADH17R, SB_ACR, ADL_ABLR, AC_SBR, SB_DBR;
reg  AC_DBR, SB_ADHR, DL_ADHR, ADH_ABHR, DL_DBR;
// Комбинаторика
wire nSB_AC, IND, a, b, SBA;
assign nSB_AC =  ~( X[58] | X[59] | X[60] | X[61] | X[62] | X[63] | X[64] );
assign ZTST = T7 | AND | nSBXY | ~nSB_AC ;
assign PGX = ~( ~X[73] & ~( X[71] | X[72] ));
assign IND =  X[89] | X[90] | X[91] | X[84];
assign a = ~( nREADY | ~( IND | X[28] | X[56] | nPCH_PCH ));
assign b = X[45] | X[46] | X[47] | X[48] | BRK6E | INC_SB;
assign SBA = ~( ~( ~nREADYR & ACRL2 ) | ~( X[93] | PGX ));
//Выходные сигналы 
assign Z_ADH0  = ~Z_ADH0R;
assign DL_ADL  = ~Z_ADH0R;
assign Z_ADH17 = ~Z_ADH17R;
assign SB_AC   = ~( PHI2 | SB_ACR );
assign ADL_ABL = ~ADL_ABLR;
assign AC_SB   = ~( PHI2 | AC_SBR );
assign SB_DB   = ~SB_DBR;
assign AC_DB   = ~( PHI2 | AC_DBR );
assign SB_ADH  = ~SB_ADHR;
assign DL_ADH  = ~DL_ADHR;
assign ADH_ABH = ~ADH_ABHR;
assign DL_DB   = ~DL_DBR;
// Логика
always @(posedge Clk) begin
       if (PHI2) begin
       Z_ADH0R  <= ~( X[81] | X[82] );
       Z_ADH17R <= ~( X[57] | ( X[81] | X[82] ));
       SB_ACR   <=   nSB_AC;
       ADL_ABLR <= ~( ~( T6 | T7 ) & ~( nREADY | ( X[71] | X[72] ))); 
       AC_SBR   <= ~( AND | X[66] | X[67] | X[68] | ( ~X[64] & X[65] ));
       SB_DBR   <= ~( T1 | X[80] | X[67] | ~( AND | ~ZTST ) | ( T6 & X[55] ) | ~( STXY & ~X[48] ));
       AC_DBR   <= ~( X[74] | ( X[79] & STOR ));
       SB_ADHR  <= ~( X[93] | PGX );
       DL_ADHR  <= ~( DL_PCH | IND );
       ADH_ABHR <= ~( Z_ADL0 | ( ~X[93] & ( a | SBA )));
       DL_DBR   <= ~( b | T6 | X[80] | X[101] | ~( X[128] | ~( T0 | X[83] )));
                 end
                      end
// Конец модуля BUS_CONTROL
endmodule

//===============================================================================================
// Модуль управления программным счетчиком
//===============================================================================================
module PC_SETUP(
// Такты
input Clk,        // Тактовый сигнал
input PHI1,       // Такт PHI1
input PHI2,       // Такт PHI2
// Входы
input [128:0]X,   // Шина Декодера
input T0,         // Цикл 0
input T1,         // Цикл 1
input nREADY,     // Сигнал "Процессор не готов"
input nREADYR,    // Сигнал "Процессор не готов" латч PHI1
// Выходы
output PCL_DB,    // Данные PCL на шину DB
output PC_DB,     // Данные PC на шину DB
output PCH_DB,    // Данные PCH на шину DB
output PCL_ADL,   // Данные PCL на шину ADL
output PCH_ADH,   // Данные PCH на шину ADH
output PCL_PCL,   // Режим хранения данных в разрядах счетчика PCL
output ADL_PCL,   // Загрузка данных из шины ADL
output nADL_PCL,  //
output DL_PCH,    // Данные DL на шину PCH
output ADH_PCH,   // Загрузка данных из шины ADH
output PCH_PCH,   // Режим хранения данных в разрядах счетчиках PCH
output nPCH_PCH   // Режим хранения данных в разрядах счетчиках PCH
);
// Переменные
reg PCL_DBR, PCH_DBR, PCL_ADLR, PCH_ADHR;  // Выходные защелки модуля
reg ADL_PCLR, ADH_PCHR;                    // Выходные защелки модуля
reg PCL_DBR1;
// Комбинаторика
wire JB, nPCL_ADL, nADH_PCH;
assign JB = ~( X[94] | X[95] | X[96] );
assign DL_PCH = ~( ~T0 | JB );
assign nPCL_ADL = ~( T1 | X[80] | X[56] | X[83] | ~( ~T0 | ~( nREADYR | JB )));
assign nADL_PCL = ~( T0 | X[84] | ( X[93] & ~nREADYR ) | ~nPCL_ADL );
assign nADH_PCH = ~( T0 | T1 | X[80] | X[93] | X[83] | X[84] );
assign nPCH_PCH =    T0 | T1 | X[80] | X[93] | X[83] | X[84];
assign PC_DB = ~( ~PCL_DBR1 & ~( X[77] | X[78] ));
//Выходные сигналы 
assign PCL_DB  = ~PCL_DBR;
assign PCH_DB  = ~PCH_DBR;
assign PCL_ADL = ~PCL_ADLR;
assign PCH_ADH = ~PCH_ADHR;
assign PCL_PCL = ~( PHI2 | ~ADL_PCLR );
assign ADL_PCL = ~( PHI2 |  ADL_PCLR );
assign PCH_PCH = ~( PHI2 | ~ADH_PCHR );
assign ADH_PCH = ~( PHI2 |  ADH_PCHR );
// Логика
always @(posedge Clk) begin
       if (PHI1) PCL_DBR1 <= ~( nREADY | PCH_DBR );
       if (PHI2) begin
       PCL_DBR  <= ~PCL_DBR1;
       PCH_DBR  <= ~( X[77] | X[78] );
       PCL_ADLR <= nPCL_ADL;
       PCH_ADHR <= ~( X[93] | ~( X[73] | DL_PCH | nPCL_ADL ));
       ADL_PCLR <= nADL_PCL;
       ADH_PCHR <= nADH_PCH;
                 end
                      end
// Конец модуля PC_CONTROL
endmodule

//===============================================================================================
// Модуль управления АЛУ
//===============================================================================================
// Модуль ALU_SETUP
module ALU_SETUP(
// Такты
input Clk,            // Тактовый сигнал
input PHI1,           // Такт PHI1
input PHI2,           // Такт PHI2
// Входы
input [128:0]X,       // Шина Декодера
input nREADY,         // Сигнал "Процессор не готов"
input nREADYR,        // Сигнал "Процессор не готов" латч PHI1
input PGX,            //
input nD_OUT,         // Вход флага D
input nC_OUT,         // Вход флага C
input T0,             // Цикл 0
input T1,             // Цикл 1
input T6,             // Цикл 6
input T7,             // Цикл 7
input BRK6E,          // Break Cycle 6 End
input STKOP,          // Stack operation
input BRFW,           // BranchForward
// Выходы
output Z_ADD,         // Обнулить
output SB_ADD,        // Шину SB  на вход АЛУ
output NDB_ADD,       // Шину ~DB на вход АЛУ
output DB_ADD,        // Шину DB  на вход АЛУ
output ADL_ADD,       // Шину ADL на вход АЛУ
output AND,           // Логическое И
output reg ANDS,      // Логическое И
output reg ORS,       // Логическое ИЛИ
output reg SRS,       // Сдвиг вправо
output reg EORS,      // Исключающее ИЛИ
output reg SUMS,      // Сумма А + В
output reg ACIN,      // Входной перенос АЛУ
output ADD_SB06,      // Биты 0-6 на шину SB
output reg ADD_SB7,   // Бит    7 на шину SB
output ADD_ADL,       // Выход АЛУ на шину ADL
output reg nDSA,      // Выполнить коррекцию после вычитания
output reg nDAA,      // Выполнить коррекцию после сложения
output INC_SB,        // Инкремент SB
output SR             // Сдвиг вправо
);
// Переменные
reg Z_ADDR, NDB_ADDR, DB_ADDR, ADL_ADDR;           // Защелки сигналов управления входными защелками АЛУ
reg ADD_SB06R, ADD_ADLR;                           // Защелки выходных сигналов модуля
reg ANDS1, SUMS1, ORS1, SRS1, EORS1, nDSA1, nDAA1; // Защелки сигналов управления режимами АЛУ
reg ACIN_IN;                                       // Защелка схемы входного переноса АЛУ
reg MUX_LATCH, COUT_LATCH;                         // Защелки схемы ADDSB7
reg FFLATCH1, FFLATCH2;                            // Защелки регистра схемы ADDSB7
// Комбинаторика
wire BRX, nADL_ADD, OR;
assign BRX =  X[49] | X[50] | ~( ~X[93] | BRFW );
assign nADL_ADD = ~( nREADY | X[35] | X[36] | X[37] | X[38] | X[39] | ( X[33] & ~X[34] ));
assign AND =  X[69] | X[70];
assign OR  =  X[32] | nREADY;
assign SR  =  X[75] | ( T6 & X[76] );
assign INC_SB = X[39] | X[40] | X[41] | X[42] | X[43] | ( T6 & X[44] );
//Выходные сигналы
assign Z_ADD    = ~( PHI2 | Z_ADDR );
assign SB_ADD   = ~( PHI2 | ~Z_ADDR );
assign NDB_ADD  = ~( PHI2 | NDB_ADDR );
assign DB_ADD   = ~( PHI2 | DB_ADDR );
assign ADL_ADD  = ~( PHI2 | ADL_ADDR );
assign ADD_SB06 = ~ADD_SB06R;
assign ADD_ADL  = ~ADD_ADLR;
// Логика
always @(posedge Clk) begin
       if (PHI1) begin
       ACIN       <= ACIN_IN;
       FFLATCH1   <= ( MUX_LATCH & COUT_LATCH )|( ~MUX_LATCH & FFLATCH2 );
       ANDS       <= ANDS1;
       SUMS       <= SUMS1;
       ORS        <= ORS1;
       SRS        <= SRS1;
       EORS       <= EORS1;
       nDSA       <= nDSA1;
       nDAA       <= nDAA1;
                 end
       if (PHI2) begin
       ACIN_IN    <= ~( nADL_ADD | ~X[47] ) | INC_SB | BRX | ~( ~X[54] & ~(( X[52] | X[53] ) & ~( nC_OUT | ~( T0 | T6 ))));
       Z_ADDR     <= ~( STKOP | X[30] | X[31] | X[45] | X[47] | X[48] | nREADY | BRK6E | INC_SB );
       NDB_ADDR   <= ~( ~nREADY & ( X[51] | X[56] | BRX ));
       DB_ADDR    <= ~( nADL_ADD & ~( ~nREADY & ( X[51] | X[56] | BRX )));
       ADL_ADDR   <= nADL_ADD;
       ANDS1      <= AND;
       SUMS1      <= ~( X[29] | SR | AND | OR );
       ORS1       <= OR;
       SRS1       <= SR;
       EORS1      <= X[29];
       ADD_SB06R  <= ~( T1 | T7 | PGX | STKOP | X[56] );
       ADD_SB7    <= ~( ~( T1 | T7 | PGX | STKOP | X[56] ) | ~( ~X[27] | ~SRS | FFLATCH1 ));
       ADD_ADLR   <= PGX | ~( X[84] | X[85] | X[86] | X[87] | X[88] | X[89] | X[26] );
       nDSA1      <= ~( X[51] & ~nD_OUT );
       nDAA1      <= X[51] | ~( X[52] & ~nD_OUT );
       COUT_LATCH <= nC_OUT;
       MUX_LATCH  <= ~( ~SR | nREADYR );
       FFLATCH2   <= FFLATCH1;
                 end
                      end
// Конец модуля ALU_SETUP
endmodule

//===============================================================================================
// Модуль ДИСПЕТЧЕРА
//===============================================================================================
module DISPATCH(
// Такты
input Clk,               // Тактовый сигнал
input PHI1,              // Такт PHI1
input PHI2,              // Такт PHI2
// Входы
input [128:0]X,          // Шина Декодера
input ACR,               //
input BRFW,              //
input RDY,               //
input BRK6E,             // Break Cycle 6 End
input RESP,              //
input DORES,             //
input PC_DB,             // Флаги на шину DB
input nBRTAKEN,          //
input nTWOCYCLE,         //
input nADL_PCL,          //
input IMPLIED,           //
input B_OUT,             // Вход флага B
// Выходы
output reg ACRL2,        //
output T6,               // Цикл  6
output reg T7,           // Цикл  7
output STOR,             //
output TRES2,            //
output reg nREADY,       // Сигнал "Процессор не готов"
output nREADYR,          // Сигнал "Процессор не готов" латч PHI1
output WRPHI1,           // Сигнал записи процессора
output T1,               // Цикл  1
output nT0,              // Цикл ~0
output T0,               // Цикл  0
output nT1X,             // Цикл ~T1X
output I_PC              // Входной перенос PC
);
// Переменные
reg RDYDELAY1, RDYDELAY2, nREADY2, WRLATCH;
reg ACRL_LATCH1;
reg T61, T62, T67, T71;
reg TRESXLATCH1, TRESXLATCH2, TRES2LATCH;
reg ENDS1LATCH, ENDS2LATCH;
reg STEPLATCH1, STEPLATCH2;
reg BRLATCH1, BRLATCH2;
reg COMPLATCH2;
reg T0LATCH, T1LATCH, T1XLATCH;
reg [2:0]IPC;
// Комбинаторика
wire nMEMOP, nSHIFT, ENDX, ENDS, REST, ACRL1, nTRESX, BRA;
assign nMEMOP = ~( X[111] | X[122] | X[123] | X[124] | X[125] );
assign nSHIFT = ~( X[106] | X[107] ); 
assign ENDX = ~( X[93] | T7 | ( X[100] | X[101] | X[102] | X[103] | X[104] | X[105] ) | ~( X[96] | nMEMOP | ~nSHIFT ));
assign ENDS = ~( ENDS1LATCH | ENDS2LATCH );
assign REST = ~( nSHIFT & ~X[97] );
assign ACRL1 = ( ~RDYDELAY2 & ACR )|( RDYDELAY2 & ACRL2 );
assign nTRESX = ~( BRK6E | ~TRESXLATCH2 | ~( ACRL1 | nREADY | REST | TRESXLATCH1 ));
assign BRA = BRLATCH2 & ( ~ACR ^ BRFW );
//Выходные сигналы
assign STOR = ~( ~X[97] | nMEMOP );
assign WRPHI1 = nREADY2;
assign T6 = ~T61;
assign TRES2 = ~TRES2LATCH;
assign T0 = ~( T0LATCH | T1XLATCH ) | ~( T1 | ( COMPLATCH2 & ~TRES2 ));
assign nT0 = ~T0;
assign T1 = ~T1LATCH;
assign nT1X = ~T1XLATCH;
assign I_PC = IPC[0] & ( IPC[1] | IPC[2] );
assign nREADYR = RDYDELAY1;
// Логика
always @(posedge Clk) begin
       if (PHI1) begin
       RDYDELAY1   <= nREADY;
       ACRL_LATCH1 <= ACRL1;
       T61         <= ~( T67 | ( nREADY & T62 ));
       T7          <= ~T71;
       TRES2LATCH  <= nTRESX;
       nREADY2     <= ~( DORES | nREADY | WRLATCH );
       STEPLATCH2  <= ~( BRA | STEPLATCH1 );
       COMPLATCH2  <= nTWOCYCLE;
       T1LATCH     <= ~( ENDS | ~( nREADY | ~( BRA | STEPLATCH1 )));
       T1XLATCH    <= ~( nREADY | T0LATCH );
       IPC[2:0]    <= { ~( nREADY | BRLATCH1 | IMPLIED ), BRA, B_OUT };
                 end
       if (PHI2) begin
       RDYDELAY2   <= RDYDELAY1;
       ACRL2       <= ACRL_LATCH1;
       T67         <= ~( nSHIFT | nMEMOP | nREADY );
       T62         <= T6;
       T71         <= ~( ~nREADY & T6 );
       TRESXLATCH1 <= ~( X[91] | X[92] );
       TRESXLATCH2 <= ~( RESP | ENDS | ~( nREADY | ENDX ));
       nREADY      <= ~( RDY | nREADY2 );
       WRLATCH     <= ~( T6 | T7 | STOR | X[98] | X[100] | PC_DB );
       ENDS1LATCH  <= ( ~T1 & nREADY ) | ( ~( T0 | ( X[80] & nBRTAKEN )) & ~nREADY );
       ENDS2LATCH  <= RESP;
       STEPLATCH1  <= ~( RESP | ~nREADYR | STEPLATCH2 ); 
       BRLATCH1    <= ~( ~( nADL_PCL | ( X[80] | X[93] )) | ( X[80] & nBRTAKEN ));
       BRLATCH2    <= ~( ~X[93] | RDYDELAY2 );
       T0LATCH     <= nT0;
                 end
                      end
// Конец модуля DISPATCH
endmodule

//===============================================================================================
// Модуль ФЛАГОВ
//===============================================================================================
module FLAGS(
// Такты
input Clk,           // Тактовый сигнал
input PHI1,          // Такт PHI1
input PHI2,          // Такт PHI2
// Входы
input [128:0]X,      // Шина Декодера
input [7:0]DB,       // Шина DB
input IR5,           // Бит 5 регистра инструкций
input nREADY,        // Сигнал "Процессор не готов"
input T7,            // Цикл 7
input SR,            // Сдвиг вправо
input ZTST,          //
input B_OUT,         // Вход флага B
input ACR,           // Сигнал переноса АЛУ
input BRK6E,         // Break Cycle 6 End
input AVR,           // Сигнал переполнения АЛУ
input SO,            // Вход синхронизации
// Выходы
output [7:0]FLAG,    // Выход шины данных флагов
output P_DB,         // Данные флагов на шину DB
output reg nD_OUT,   // Выход флага D
output     nI_OUT,   // Выход флага I
output reg nC_OUT,   // Выход флага C
output nBRTAKEN,     //
output reg BRFW      //
);
// Переменные
reg P_DBR, IR5_I, IR5_C, IR5_D, Z_V, ARC_CR;                    // Защелки схемы управления флагами
reg DBZ_ZR, DB_CR, DB_NR, DB_VR, DB_VR2;                        // Защелки схемы управления флагами
reg nN_OUT, nZ_OUT, nV_OUT, I_LATCH1;                           // Первая защелка флага
reg C_LATCH2, Z_LATCH2, I_LATCH2, D_LATCH2, V_LATCH2, N_LATCH2; // Вторая защелка флага
reg [2:0]SO_LATCH;                                              // Защелки детектора фронта синхронизации
reg VSET, AVR_LATCH, BRFW2, BR2_LATCH;
// Комбинаторика
// Управление флагами
wire DB_V, DB_P, DB_N, nDBZ;
assign DB_N = ~(( DBZ_ZR & DB_VR ) | DB_NR );
assign DB_P = ~( DB_VR | nREADY );
assign DB_V = ~( DB_VR & DB_VR2 );
assign nDBZ = DB[7] | DB[6] | DB[5] | DB[4] | DB[3] | DB[2] | DB[1] | DB[0];
// Мультиплексор флагов перехода
wire FLAG_MUX;
assign FLAG_MUX = ( nZ_OUT & ~X[121] & ~X[126] )|( nC_OUT & X[121] & ~X[126] )|( nV_OUT & ~X[121] & X[126] )|( nN_OUT & X[121] & X[126] );
// Выход данных флагов
assign FLAG[7:0] = { ~nN_OUT, ~nV_OUT, 1'b1, B_OUT, ~nD_OUT, ~nI_OUT, ~nZ_OUT, ~nC_OUT};
assign P_DB = ~P_DBR;
assign nI_OUT = I_LATCH1 & ~BRK6E;
assign nBRTAKEN = ~IR5 ^ FLAG_MUX; // Сигнал готовности перехода
// Логика
always @(posedge Clk) begin
       if (PHI1) begin
       // Первичные защелки флагов
       nC_OUT   <= ( ~DB[0] & ~DB_CR ) | ( ~ACR & ~ARC_CR  ) | ( ~IR5   & IR5_C ) | ( ~( IR5_C | ~ARC_CR | ~DB_CR ) & C_LATCH2 );
       nZ_OUT   <= ( ~DB[1] & DB_P   ) | ( nDBZ & ~DBZ_ZR  ) | ( ~( DB_P | ~DBZ_ZR ) & Z_LATCH2 );
       I_LATCH1 <= ( ~DB[2] & DB_P   ) | ( ~IR5   & IR5_I  ) | ( ~( IR5_I | DB_P ) & I_LATCH2 );
       nD_OUT   <= ( ~DB[3] & DB_P   ) | ( ~IR5   & IR5_D  ) | ( ~( IR5_D | DB_P ) & D_LATCH2 );
       nV_OUT   <= ( ~DB[6] & DB_V   ) | ( AVR & AVR_LATCH ) | ( ~( AVR_LATCH | DB_V | VSET ) & V_LATCH2 ) | Z_V;
       nN_OUT   <= ( ~DB[7] & DB_N   ) | ( ~DB_N & N_LATCH2 );
       // Защелки V флага
       SO_LATCH[0] <= SO;
       SO_LATCH[2] <= ~SO_LATCH[1];
       // Защелка BRFW
       BRFW <= ( ~BR2_LATCH & BRFW2 ) | ( BR2_LATCH & DB[7] );
                  end
       if (PHI2) begin
       // Защелки системы управления флагами
       P_DBR  <= ~( X[98] | X[99] );
       IR5_I  <= X[108];
       IR5_C  <= X[110];
       IR5_D  <= X[120];
       Z_V    <= X[127];
       ARC_CR <= ~( X[112] | X[116] | X[117] | X[118] | X[119] | ( X[107] & T7 ));
       DBZ_ZR <= ~( ~ARC_CR | ZTST | X[109] );
       DB_NR  <= X[109];
       DB_CR  <= ~( SR | DB_P );
       DB_VR  <= ~( X[114] | X[115] );
       DB_VR2 <= ~  X[113];
       // Вторичные защелки флагов
       C_LATCH2 <= nC_OUT;
       Z_LATCH2 <= nZ_OUT;
       I_LATCH2 <= I_LATCH1 & ~BRK6E;
       D_LATCH2 <= nD_OUT;
       V_LATCH2 <= nV_OUT;
       N_LATCH2 <= nN_OUT;
       // Защелки V флага
       AVR_LATCH <= X[112];
       SO_LATCH[1] <= SO_LATCH[0];
       VSET <= ~( SO_LATCH[0] | SO_LATCH[2] );
       // Защелки BRANCH LOGIC
       BR2_LATCH <= X[80];
       BRFW2 <= BRFW;
                 end
                      end
// Конец модуля FLAGS
endmodule

//===============================================================================================
// Модуль обработки прерываний
//===============================================================================================
module INT_RESET_CONTROL(
// Такты
input Clk,             // Тактовый сигнал
input PHI1,            // Такт PHI1
input PHI2,            // Такт PHI2
// Входы
input [128:0]X,        // Шина Декодера
input nREADY,          // Сигнал "Процессор не готов"
input RESP,            //
input nNMIP,           //
input nI_OUT,          // Вход флага I
input nIRQP,           //
input T0,              // Цикл 0
// Выходы
output BRK6E,          // Break Cycle 6 End
output reg[2:0]Z_ADL,  //
output DORES,          //
output B_OUT           // Выход флага B
);
// Переменные
reg BRK6LATCH1, BRK6LATCH2;
reg RESLATCH1, RESLATCH2;
reg BRK7LATCH, NMIPLATCH, FF2LATCH, DELAYLATCH2;
reg DONMILATCH, FF1LATCH, BRK6ELATCH;
reg BLATCH1, BLATCH2;
// Комбинаторика
wire BRK7, a, b, nDONMI;
assign BRK6E = ~( nREADY | ~BRK6LATCH2 );
assign BRK7 = ~(( X[22] & ~nREADY ) | ~BRK6LATCH1 );
assign a = ~( NMIPLATCH | ~( DELAYLATCH2 | FF2LATCH ));
assign b = ~(( nIRQP | ~nI_OUT ) & nDONMI ) & ( X[80] | T0 );
assign nDONMI = ~( DONMILATCH | ~( BRK6ELATCH | FF1LATCH ));
assign DORES  = RESLATCH1 | RESLATCH2;
assign B_OUT  = ~( ~( BRK6E | BLATCH2 ) | DORES );
// Логика
always @(posedge Clk) begin
       if (PHI1) begin
       BRK6LATCH1  <= ~( Z_ADL[0] | ( nREADY & ~BRK6LATCH1 ));
       RESLATCH2   <= ~( BRK6E | ~( RESLATCH1 | RESLATCH2 ));
       NMIPLATCH   <= nNMIP;
       DELAYLATCH2 <= ~FF1LATCH;
       DONMILATCH  <= ~( nNMIP | ~BRK7LATCH | a );
       BRK6ELATCH  <= BRK6E;
       BLATCH1     <= ~( BRK6E | BLATCH2 );
                 end
       if (PHI2) begin
       BRK6LATCH2 <= ~BRK6LATCH1;
       RESLATCH1  <= RESP;
       BRK7LATCH  <= BRK7;
       FF2LATCH   <= a;
       FF1LATCH   <= nDONMI;
       BLATCH2    <= ~( b | BLATCH1 );
       Z_ADL[2:0] <= { ~( BRK7 | DORES | nDONMI ), ~( BRK7 | ~DORES ), X[22] & ~nREADY };
                 end
                      end
// Конец модуля INT_RESET_CONTROL
endmodule

//===============================================================================================
// Модуль мультиплексирования шин
//===============================================================================================
module BUS_MUX(
// Секция входов
// Управление генератором констант
input [2:0]Z_ADL,        // Обнулить биты 2:0 шины ADL
input Z_ADH0,            // Обнулить бит 0 шины ADH
input Z_ADH17,           // Обнулить биты 1-7 шины ADH
// Управление мультиплексорами шин
input SB_DB,             // Данные SB на шину DB
input PCL_DB,            // Данные PCL на шину DB
input PCH_DB,            // Данные PCH на шину DB
input P_DB,              // Данные флагов на шину DB
input AC_DB,             // Данные аккумулятора на шину DB
input AC_SB,             // Данные аккумулятора на шину SB
input SB_AC,             // Данные SB на аккумулятор
input ADD_ADL,           // Данные АЛУ на шину ADL
input ADD_SB06,          // Данные АЛУ биты 0-6 на шину SB
input ADD_SB7,           // Данные АЛУ бит  7 на шину SB
input Y_SB,              // Данные регистра Y на шину SB
input X_SB,              // Данные регистра X на шину SB
input S_SB,              // Данные регистра S на шину SB
input SB_ADH,            // Данные ADH на шину SB
input S_ADL,             // Данные регистра S на шину ADL
input DL_ADL,            // Данные Data Latch на шину ADL
input DL_ADH,            // Данные Data Latch на шину ADH
input DL_DB,             // Данные Data Latch на шину DB
input PCL_ADL,           // Данные PCL на шину ADL
input PCH_ADH,           // Данные PCH на шину ADH
// Входные шины
input [7:0]DL,           // Шина DL
input [7:0]PCL,          // Шина PCL
input [7:0]PCH,          // Шина PCH
input [7:0]FLAG,         // Данные флагов
input [7:0]ADD,          // Данные АЛУ
input [7:0]ACC,          // Данные аккумулятора
input [7:0]Y_REG,        // Данные регистра Y
input [7:0]X_REG,        // Данные регистра X
input [7:0]S_REG,        // Данные регистра S
// Секция выходов
output [7:0]DB,          // Выход шины DB
output [7:0]SB,          // Выход шины SB
output [7:0]ADL,         // Выход шины ADL
output [7:0]ADH          // Выход шины ADH
);
// Комбинаторика
wire AC_SB2;
assign AC_SB2 = ~AC_SB | SB_AC;  // AB Hack
// Промежуточные шины
wire [7:0]SBT, ADHT;
// Мультиплексор шины DB
assign DB[7:0]  = (~{8{SB_DB}}  | SBT[7:0] ) & (~{8{AC_DB}} | ACC[7:0] ) & (~{8{P_DB}} | FLAG[7:0] ) & (~{8{DL_DB}} | DL[7:0]   ) & (~{8{PCL_DB}} | PCL[7:0]) & (~{8{PCH_DB}} | PCH[7:0] );
// Мультиплексор шины SBT
assign SBT[7:0] = (~{8{SB_ADH}} | ADHT[7:0]) & (~{8{X_SB}} | X_REG[7:0]) & (~{8{Y_SB}} | Y_REG[7:0]) & (~{8{S_SB}}  | S_REG[7:0]) & ( {8{AC_SB2}} | ACC[7:0]) & {~ADD_SB7 | ADD[7], ~{7{ADD_SB06}} | ADD[6:0]};
// Мультиплексор шины SB
assign SB[7:0]   =  SB_DB ? DB[7:0] : SBT[7:0];
// Мультиплексор промежуточной шины ADHT
assign ADHT[7:0] = ( ~{8{PCH_ADH}} | PCH[7:0] ) & ( ~{8{DL_ADH}} | DL[7:0] ) & { {7{ ~Z_ADH17 }}, ~Z_ADH0 };
// Мультиплексор шины ADH
assign ADH[7:0]  =  SB_ADH ? SBT[7:0] : ADHT[7:0];
// Мультиплексор шины ADL
assign ADL[7:0]  = ( ~{8{S_ADL}} | S_REG[7:0] ) & ( ~{8{ADD_ADL}} | ADD[7:0] ) & ( ~{8{PCL_ADL}} | PCL[7:0] ) & ( ~{8{DL_ADL}} | DL[7:0] ) & { 5'h1f, ~Z_ADL[2:0] };
// Конец модуля BUS_MUX
endmodule

//===============================================================================================
// Модуль АЛУ
//===============================================================================================
module ALU (
// Такты
input Clk,               // Тактовый сигнал
input PHI2,              // Такт PHI2
// Входы
input Z_ADD,             // Обнуление входа AI АЛУ
input [7:0]SB,           // Шина SB
input SB_ADD,            // Загрузка данных из шины SB
input [7:0]DB,           // Шина DB
input NDB_ADD,           // Загрузка инвертированных данных шины DB
input DB_ADD,            // Загрузка данных из шины DB
input [7:0]ADL,          // Шина ADL
input ADL_ADD,           // Загрузка данных из шины ADL
input ACIN,              // Входной перенос АЛУ
input ANDS,              // Результат логического И
input ORS,               // Результат логического ИЛИ
input EORS,              // Результат логического исключающего ИЛИ
input SRS,               // Результат сдвига вправо
input SUMS,              // Результат суммирования A + B
input SB_AC,             // Шина SB на аккумулятор
input nDAA,              // Выполнить коррекцию после сложения
input nDSA,              // Выполнить коррекцию после вычитания
// Выходы
output reg [7:0]ACC,     // Выход аккумулятора
output reg [7:0]ADD,     // Выход результата операций
output ACR,              // Выход переноса АЛУ
output reg AVR           // Выход переполнения АЛУ
);
// Переменные
reg [7:0]AI, BI;                // Входные латчи АЛУ
reg LATCH_C7;                   // Латчи схемы переполнения АЛУ
reg LATCH_DC7;                  // Латчи схемы переполнения АЛУ
reg DAAL, DAAHR, DSAL, DSAHR;   // Латчи управления десятичной коррекцией
// Комбинаторика логических операций
wire [7:0]ANDo, ORo, XORo, SUMo;
assign ANDo[7:0] =   AI[7:0] &  BI[7:0];  // Логическое И
assign  ORo[7:0] =   AI[7:0] |  BI[7:0];  // Логическое ИЛИ
assign XORo[7:0] =   AI[7:0] ^  BI[7:0];  // Логическое исключающее ИЛИ
assign SUMo[7:0] = XORo[7:0] ^ CIN[7:0];  // Сумма А и В
wire [7:0]RESULT;                         // Результат АЛУ
assign RESULT[7:0] = ({8{ANDS}} & ANDo[7:0]) | ({8{ORS}} & ORo [7:0]) | ({8{EORS}} & XORo[7:0]) | ({8{SRS}} & {1'b0 ,ANDo[7:1]}) | ({8{SUMS}} & SUMo[7:0]);
// Комбинаторика переполнения АЛУ
wire [7:0]CIN, COUT;
assign CIN[7:0] = { COUT[6:4], DCOUT3, COUT[2:0], ACIN };
assign COUT[7:0] = ( CIN[7:0] & XORo[7:0] ) | ANDo[7:0];
wire DCOUT3;
assign DCOUT3 = COUT[3] | DC3;
assign ACR = LATCH_C7 | LATCH_DC7;
//BCD
wire DAAH, DSAH;
assign DAAH =    ACR & DAAHR;
assign DSAH = ~( ACR | DSAHR );
wire [5:0]bcd;       // промежуточные сигналы BCD
assign bcd[0] = DAAL | DSAL;
assign bcd[1] = (( DAAL &  ~ADD[1] )           | ( DSAL & ADD[1] ));
assign bcd[2] = (( DAAL & ( ADD[1] | ADD[2] )) | ( DSAL & ~( ADD[1] & ADD[2] )));
assign bcd[3] = DAAH | DSAH;
assign bcd[4] = (( DAAH &  ~ADD[5] )           | ( DSAH & ADD[5] ));
assign bcd[5] = (( DAAH & ( ADD[5] | ADD[6] )) | ( DSAH & ~( ADD[5] & ADD[6] )));
wire [7:0]BCDRES;       // Выход схемы десятичной коррекции
assign BCDRES[7:0] = { SB[7:5] ^ bcd[5:3], SB[4], SB[3:1] ^ bcd[2:0], SB[0] };
// BCD CARRY
wire DC3,DC7; 
wire a,b,c,d,e,f,g; // промежуточные сигналы BCD CARRY
assign a   = ~( ~ORo[0] | ( ~ACIN & ~ANDo[0] ));
assign b   = ~( a & ANDo[1] );
assign c   = ~( ANDo[2] | XORo[3] );
assign d   = ~( a | ~( ANDo[2] | ~ORo[2] ) | ANDo[1] | XORo[1] );
assign e   = ~( ANDo[5] & COUT[4] );
assign f   = ~( ANDo[6] | XORo[7] );
assign g   = ~( XORo[5] | XORo[6] | ANDo[5] | COUT[4] );
assign DC3 = ~( nDAA | (( b | ~ORo[2]  ) & ( c | d )) );
assign DC7 = ~( nDAA | (( e | ~XORo[6] ) & ( f | g )) );
// Логика
always @(posedge Clk) begin
            if (Z_ADD | SB_ADD )             AI[7:0] <= Z_ADD ? 8'h00 : SB[7:0];
            if (DB_ADD | NDB_ADD | ADL_ADD ) BI[7:0] <= NDB_ADD ? ~DB[7:0] : ADL_ADD ? ADL[7:0] : DB[7:0];
            if (SB_AC)  ACC[7:0]  <= BCDRES[7:0];
            if (PHI2) begin
            ADD[7:0]  <= RESULT[7:0];
            LATCH_C7  <= COUT[7];
            LATCH_DC7 <= DC7;
            AVR <=  ( COUT[6] & ORo[7] ) | ( ~COUT[6] & ~ANDo[7] );
            DAAL  <=  ( DCOUT3 & ~nDAA );
            DAAHR <= ~nDAA;
            DSAL  <= ~( DCOUT3 |  nDSA );
            DSAHR <=  nDSA;
                      end
                      end
// Конец модуля ALU
endmodule

//===============================================================================================
// Модуль Программного счетчика (PC)
//===============================================================================================
module PC (
// Такты
input Clk,               // Тактовый сигнал
input PHI2,              // Такт PHI2
// Входы
input I_PC,              // Входной перенос счетчика
input PCL_PCL,           // Режим хранения данных в разрядах счетчика PCL
input ADL_PCL,           // Загрузка данных из шины ADL
input [7:0]ADL,          // Шина ADL
input PCH_PCH,           // Режим хранения данных в разрядах счетчика PCH
input ADH_PCH,           // Загрузка данных из шины ADH
input [7:0]ADH,          // Шина ADH
// Выходы
output reg [7:0]PCL,     // Выход младших 8 битов PC
output reg [7:0]PCH      // Выход старших 8 битов PC
);
// Переменные
reg [7:0]PCLS, PCHS;     // Промежуточные регистры
// Комбинаторика
wire [7:0]ADL_COUT, ADH_COUT;
assign ADL_COUT[7:0] = PCLS[7:0] & {ADL_COUT[6:0], I_PC};
assign ADH_COUT[7:0] = PCHS[7:0] & {ADH_COUT[6:4], PCH_03, ADH_COUT[2:0], PCH_IN};
wire PCH_IN, PCH_03;
assign PCH_IN = PCLS[7] & PCLS[6] & PCLS[5] & PCLS[4] & PCLS[3] & PCLS[2] & PCLS[1] & PCLS[0] & I_PC;
assign PCH_03 = PCHS[3] & PCHS[2] & PCHS[1] & PCHS[0] & I_PC & PCH_IN;
// Логика
always @(posedge Clk) begin
   if ( PCL_PCL | ADL_PCL ) PCLS[7:0] <= ( {8{ PCL_PCL }} & PCL[7:0] )|( {8{ ADL_PCL }} & ADL[7:0] );
   if ( PCH_PCH | ADH_PCH ) PCHS[7:0] <= ( {8{ PCH_PCH }} & PCH[7:0] )|( {8{ ADH_PCH }} & ADH[7:0] );
   if (PHI2) begin
   PCL[7:0] <= ( PCLS[7:0] ^ {ADL_COUT[6:0], I_PC} );
   PCH[7:0] <= ( PCHS[7:0] ^ {ADH_COUT[6:4], PCH_03, ADH_COUT[2:0], PCH_IN} );
              end
                      end
// Конец модуля Program counter
endmodule
