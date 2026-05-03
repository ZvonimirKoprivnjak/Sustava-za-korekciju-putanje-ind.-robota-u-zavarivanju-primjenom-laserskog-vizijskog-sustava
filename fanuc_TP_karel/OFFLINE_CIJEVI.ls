/PROG  OFFLINE_CIJEVI
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "PRIHVAT";
PROG_SIZE	= 4882;
CREATE		= DATE 26-04-14  TIME 02:07:14;
MODIFIED	= DATE 26-04-15  TIME 18:28:54;
FILE_NAME	= ZK_014_C;
VERSION		= 0;
LINE_COUNT	= 236;
MEMORY_SIZE	= 5362;
PROTECT		= READ_WRITE;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,1,*,*,*;
CONTROL_CODE	= 00000000 00000000;
LOCAL_REGISTERS	= 0,0,0;
/APPL
/MN
   1:  !bez tool offseta ;
   2:  UFRAME_NUM=4 ;
   3:  UTOOL_NUM=7 ;
   4:  R[52:Kraj_snimanja]=0    ;
   5:  //IF R[48:Data_set]=1,JMP LBL[10] ;
   6:  R[48:Data_set]=1    ;
   7:  CALL ZK_READ_CSV_XYZWPR    ;
   8:  LBL[10] ;
   9:   ;
  10:J PR[150:ZK_diplomski] 100% FINE    ;
  11:  WAIT   1.00(sec) ;
  12:  RUN OFFLINE_1_CIJEV ;
  13:  R[52:Kraj_snimanja]=1    ;
  14:  WAIT R[54]=11111 TIMEOUT,LBL[99] ;
  15:A PR[151] 2mm/sec CNT100    ;
  16:A PR[152] 2mm/sec CNT100    ;
  17:A PR[153] 2mm/sec CNT100    ;
  18:A PR[154] 2mm/sec CNT100    ;
  19:A PR[155] 2mm/sec CNT100    ;
  20:A PR[156] 2mm/sec CNT100    ;
  21:A PR[157] 2mm/sec CNT100    ;
  22:A PR[158] 2mm/sec CNT100    ;
  23:A PR[159] 2mm/sec CNT100    ;
  24:A PR[160] 2mm/sec CNT100    ;
  25:A PR[161] 2mm/sec CNT100    ;
  26:A PR[162] 2mm/sec CNT100    ;
  27:A PR[163] 2mm/sec CNT100    ;
  28:A PR[164] 2mm/sec CNT100    ;
  29:A PR[165] 2mm/sec CNT100    ;
  30:A PR[166] 2mm/sec CNT100    ;
  31:A PR[167] 2mm/sec CNT100    ;
  32:A PR[168] 2mm/sec CNT100    ;
  33:A PR[169] 2mm/sec CNT100    ;
  34:A PR[170] 2mm/sec CNT100    ;
  35:A PR[171] 2mm/sec CNT100    ;
  36:A PR[172] 2mm/sec CNT100    ;
  37:A PR[173] 2mm/sec CNT100    ;
  38:A PR[174] 2mm/sec CNT100    ;
  39:A PR[175] 2mm/sec CNT100    ;
  40:A PR[176] 2mm/sec CNT100    ;
  41:A PR[177] 2mm/sec CNT100    ;
  42:A PR[178] 2mm/sec CNT100    ;
  43:A PR[179] 2mm/sec CNT100    ;
  44:A PR[180] 2mm/sec CNT100    ;
  45:A PR[181] 2mm/sec CNT100    ;
  46:A PR[182] 2mm/sec CNT100    ;
  47:A PR[183] 2mm/sec CNT100    ;
  48:A PR[184] 2mm/sec CNT100    ;
  49:A PR[185] 2mm/sec CNT100    ;
  50:A PR[186] 2mm/sec CNT100    ;
  51:A PR[187] 2mm/sec CNT100    ;
  52:A PR[188] 2mm/sec CNT100    ;
  53:A PR[189] 2mm/sec CNT100    ;
  54:A PR[190] 2mm/sec CNT100    ;
  55:A PR[191] 2mm/sec CNT100    ;
  56:A PR[192] 2mm/sec CNT100    ;
  57:A PR[193] 2mm/sec CNT100    ;
  58:A PR[194] 2mm/sec CNT100    ;
  59:A PR[195] 2mm/sec CNT100    ;
  60:A PR[196] 2mm/sec CNT100    ;
  61:A PR[197] 2mm/sec CNT100    ;
  62:A PR[198] 2mm/sec CNT100    ;
  63:A PR[199] 2mm/sec CNT100    ;
  64:A PR[200] 2mm/sec CNT100    ;
  65:A PR[201] 2mm/sec CNT100    ;
  66:A PR[202] 2mm/sec CNT100    ;
  67:A PR[203] 2mm/sec CNT100    ;
  68:A PR[204] 2mm/sec CNT100    ;
  69:A PR[205] 2mm/sec CNT100    ;
  70:A PR[206] 2mm/sec CNT100    ;
  71:A PR[207] 2mm/sec CNT100    ;
  72:A PR[208] 2mm/sec CNT100    ;
  73:A PR[209] 2mm/sec CNT100    ;
  74:A PR[210] 2mm/sec CNT100    ;
  75:A PR[211] 2mm/sec CNT100    ;
  76:A PR[212] 2mm/sec CNT100    ;
  77:A PR[213] 2mm/sec CNT100    ;
  78:A PR[214] 2mm/sec CNT100    ;
  79:A PR[215] 2mm/sec CNT100    ;
  80:A PR[216] 2mm/sec CNT100    ;
  81:A PR[217] 2mm/sec CNT100    ;
  82:A PR[218] 2mm/sec CNT100    ;
  83:A PR[219] 2mm/sec CNT100    ;
  84:A PR[220] 2mm/sec CNT100    ;
  85:A PR[221] 2mm/sec CNT100    ;
  86:A PR[222] 2mm/sec CNT100    ;
  87:A PR[223] 2mm/sec CNT100    ;
  88:A PR[224] 2mm/sec CNT100    ;
  89:A PR[225] 2mm/sec CNT100    ;
  90:A PR[226] 2mm/sec CNT100    ;
  91:A PR[227] 2mm/sec CNT100    ;
  92:A PR[228] 2mm/sec CNT100    ;
  93:A PR[229] 2mm/sec CNT100    ;
  94:A PR[230] 2mm/sec CNT100    ;
  95:A PR[231] 2mm/sec CNT100    ;
  96:A PR[232] 2mm/sec CNT100    ;
  97:A PR[233] 2mm/sec CNT100    ;
  98:A PR[234] 2mm/sec CNT100    ;
  99:A PR[235] 2mm/sec CNT100    ;
 100:A PR[236] 2mm/sec CNT100    ;
 101:A PR[237] 2mm/sec CNT100    ;
 102:A PR[238] 2mm/sec CNT100    ;
 103:A PR[239] 2mm/sec CNT100    ;
 104:A PR[240] 2mm/sec CNT100    ;
 105:A PR[241] 2mm/sec CNT100    ;
 106:A PR[242] 2mm/sec CNT100    ;
 107:A PR[243] 2mm/sec CNT100    ;
 108:A PR[244] 2mm/sec CNT100    ;
 109:A PR[245] 2mm/sec CNT100    ;
 110:A PR[246] 2mm/sec CNT100    ;
 111:A PR[247] 2mm/sec CNT100    ;
 112:A PR[248] 2mm/sec CNT100    ;
 113:A PR[249] 2mm/sec CNT100    ;
 114:A PR[250] 2mm/sec CNT100    ;
 115:   ;
 116:  R[52:Kraj_snimanja]=99000    ;
 117:  WAIT R[52:Kraj_snimanja]=99000 TIMEOUT,LBL[99] ;
 118:  CALL OFFLINE_2_CIJEV    ;
 119:   ;
 120:  LBL[99] ;
 121:   ;
 122:L P[1] 4000mm/sec CNT100    ;
 123:J P[2] 100% CNT100    ;
 124:J P[3] 100% CNT100    ;
 125:J P[5] 100% CNT100    ;
 126:  //UTOOL_NUM=8 ;
 127:J PR[260] 100% FINE    ;
 128:A PR[261] 5mm/sec CNT100    ;
 129:A PR[262] 5mm/sec CNT100    ;
 130:A PR[263] 5mm/sec CNT100    ;
 131:A PR[264] 5mm/sec CNT100    ;
 132:A PR[265] 5mm/sec CNT100    ;
 133:A PR[266] 5mm/sec CNT100    ;
 134:A PR[267] 5mm/sec CNT100    ;
 135:A PR[268] 5mm/sec CNT100    ;
 136:A PR[269] 5mm/sec CNT100    ;
 137:A PR[270] 5mm/sec CNT100    ;
 138:A PR[271] 5mm/sec CNT100    ;
 139:A PR[272] 5mm/sec CNT100    ;
 140:A PR[273] 5mm/sec CNT100    ;
 141:A PR[274] 5mm/sec CNT100    ;
 142:A PR[275] 5mm/sec CNT100    ;
 143:A PR[276] 5mm/sec CNT100    ;
 144:A PR[277] 5mm/sec CNT100    ;
 145:A PR[278] 5mm/sec CNT100    ;
 146:A PR[279] 5mm/sec CNT100    ;
 147:A PR[280] 5mm/sec CNT100    ;
 148:A PR[281] 5mm/sec CNT100    ;
 149:A PR[282] 5mm/sec CNT100    ;
 150:A PR[283] 5mm/sec CNT100    ;
 151:A PR[284] 5mm/sec CNT100    ;
 152:A PR[285] 5mm/sec CNT100    ;
 153:A PR[286] 5mm/sec CNT100    ;
 154:A PR[287] 5mm/sec CNT100    ;
 155:A PR[288] 5mm/sec CNT100    ;
 156:A PR[289] 5mm/sec CNT100    ;
 157:A PR[290] 5mm/sec CNT100    ;
 158:A PR[291] 5mm/sec CNT100    ;
 159:A PR[292] 5mm/sec CNT100    ;
 160:A PR[293] 5mm/sec CNT100    ;
 161:A PR[294] 5mm/sec CNT100    ;
 162:A PR[295] 5mm/sec CNT100    ;
 163:A PR[296] 5mm/sec CNT100    ;
 164:A PR[297] 5mm/sec CNT100    ;
 165:A PR[298] 5mm/sec CNT100    ;
 166:A PR[299] 5mm/sec CNT100    ;
 167:A PR[300] 5mm/sec CNT100    ;
 168:A PR[301] 5mm/sec CNT100    ;
 169:A PR[302] 5mm/sec CNT100    ;
 170:A PR[303] 5mm/sec CNT100    ;
 171:A PR[304] 5mm/sec CNT100    ;
 172:A PR[305] 5mm/sec CNT100    ;
 173:A PR[306] 5mm/sec CNT100    ;
 174:A PR[307] 5mm/sec CNT100    ;
 175:A PR[308] 5mm/sec CNT100    ;
 176:A PR[309] 5mm/sec CNT100    ;
 177:A PR[310] 5mm/sec CNT100    ;
 178:A PR[311] 5mm/sec CNT100    ;
 179:A PR[312] 5mm/sec CNT100    ;
 180:A PR[313] 5mm/sec CNT100    ;
 181:A PR[314] 5mm/sec CNT100    ;
 182:A PR[315] 5mm/sec CNT100    ;
 183:A PR[316] 5mm/sec CNT100    ;
 184:A PR[317] 5mm/sec CNT100    ;
 185:A PR[318] 5mm/sec CNT100    ;
 186:A PR[319] 5mm/sec CNT100    ;
 187:A PR[320] 5mm/sec CNT100    ;
 188:A PR[321] 5mm/sec CNT100    ;
 189:A PR[322] 5mm/sec CNT100    ;
 190:A PR[323] 5mm/sec CNT100    ;
 191:A PR[324] 5mm/sec CNT100    ;
 192:A PR[325] 5mm/sec CNT100    ;
 193:A PR[326] 5mm/sec CNT100    ;
 194:A PR[327] 5mm/sec CNT100    ;
 195:A PR[328] 5mm/sec CNT100    ;
 196:A PR[329] 5mm/sec CNT100    ;
 197:A PR[330] 5mm/sec CNT100    ;
 198:A PR[331] 5mm/sec CNT100    ;
 199:A PR[332] 5mm/sec CNT100    ;
 200:A PR[333] 5mm/sec CNT100    ;
 201:A PR[334] 5mm/sec CNT100    ;
 202:A PR[335] 5mm/sec CNT100    ;
 203:A PR[336] 5mm/sec CNT100    ;
 204:A PR[337] 5mm/sec CNT100    ;
 205:A PR[338] 5mm/sec CNT100    ;
 206:A PR[339] 5mm/sec CNT100    ;
 207:A PR[340] 5mm/sec CNT100    ;
 208:A PR[341] 5mm/sec CNT100    ;
 209:A PR[342] 5mm/sec CNT100    ;
 210:A PR[343] 5mm/sec CNT100    ;
 211:A PR[344] 5mm/sec CNT100    ;
 212:A PR[345] 5mm/sec CNT100    ;
 213:A PR[346] 5mm/sec CNT100    ;
 214:A PR[347] 5mm/sec CNT100    ;
 215:A PR[348] 5mm/sec CNT100    ;
 216:A PR[349] 5mm/sec CNT100    ;
 217:A PR[350] 5mm/sec CNT100    ;
 218:A PR[351] 5mm/sec CNT100    ;
 219:A PR[352] 5mm/sec CNT100    ;
 220:A PR[353] 5mm/sec CNT100    ;
 221:A PR[354] 5mm/sec CNT100    ;
 222:A PR[355] 5mm/sec CNT100    ;
 223:A PR[356] 5mm/sec CNT100    ;
 224:A PR[357] 5mm/sec CNT100    ;
 225:A PR[358] 5mm/sec CNT100    ;
 226:A PR[359] 5mm/sec CNT100    ;
 227:A PR[360] 5mm/sec CNT100    ;
 228:  LBL[100] ;
 229:   ;
 230:  PAUSE ;
 231:  UTOOL_NUM=7 ;
 232:L P[1] 4000mm/sec CNT100    ;
 233:J P[2] 100% CNT100    ;
 234:J P[3] 100% CNT100    ;
 235:J P[5] 100% CNT100    ;
 236:   ;
/POS
P[1]{
   GP1:
	UF : 4, UT : 7,		CONFIG : 'N U T, 0, 0, 0',
	X =   137.376  mm,	Y =   125.120  mm,	Z =   380.072  mm,
	W =  -142.292 deg,	P =    22.795 deg,	R =   -90.000 deg
   GP2:
	UF : 4, UT : 7,	
	J1=  2953.175  mm
};
P[2]{
   GP1:
	UF : 4, UT : 7,		CONFIG : 'N U T, 0, 0, 0',
	X =   137.404  mm,	Y =   125.113  mm,	Z =   380.051  mm,
	W =   -83.931 deg,	P =    12.256 deg,	R =   -70.414 deg
   GP2:
	UF : 4, UT : 7,	
	J1=  2953.175  mm
};
P[3]{
   GP1:
	UF : 4, UT : 7,		CONFIG : 'N U T, 0, 0, 0',
	X =  -139.008  mm,	Y =   125.055  mm,	Z =   380.717  mm,
	W =   -83.928 deg,	P =    12.260 deg,	R =   -70.413 deg
   GP2:
	UF : 4, UT : 7,	
	J1=  2953.176  mm
};
P[5]{
   GP1:
	UF : 4, UT : 7,		CONFIG : 'N U T, 0, 0, 0',
	X =  -115.854  mm,	Y =   118.210  mm,	Z =   324.712  mm,
	W =   -59.190 deg,	P =    20.783 deg,	R =   -90.476 deg
   GP2:
	UF : 4, UT : 7,	
	J1=  2953.176  mm
};
/END
