/PROG  ONLINE_TP
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "online_TP";
PROG_SIZE	= 810;
CREATE		= DATE 26-04-20  TIME 22:20:28;
MODIFIED	= DATE 26-04-20  TIME 22:23:24;
FILE_NAME	= ZK_013_O;
VERSION		= 0;
LINE_COUNT	= 18;
MEMORY_SIZE	= 1250;
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
   1:  UTOOL_NUM=7 ;
   2:  UFRAME_NUM=7 ;
   3:   ;
   4:L P[4] 100mm/sec FINE    ;
   5:  //PAUSE ;
   6:  R[52:Kraj_snimanja]=0    ;
   7:   ;
   8:  RUN ONLINE_KAREL ;
   9:  WAIT    .50(sec) ;
  10:  LBL[5] ;
  11:  R[57:Help_var]=160    ;
  12:   ;
  13:  FOR R[57:Help_var]=160 TO 175 ;
  14:L PR[R[57]] 4mm/sec CNT100    ;
  15:  ENDFOR ;
  16:  JMP LBL[5] ;
  17:   ;
  18:L P[3] 100mm/sec FINE    ;
/POS
P[3]{
   GP1:
	UF : 7, UT : 7,		CONFIG : 'N U T, 0, 0, 0',
	X =   176.876  mm,	Y =  -117.114  mm,	Z =   373.362  mm,
	W =     -.090 deg,	P =      .004 deg,	R =    -1.767 deg
   GP2:
	UF : 0, UT : 7,	
	J1=  2953.184  mm
};
P[4]{
   GP1:
	UF : 7, UT : 7,		CONFIG : 'N U T, 0, 0, 0',
	X =    73.842  mm,	Y =      .200  mm,	Z =    17.290  mm,
	W =   -90.007 deg,	P =     -.010 deg,	R =      .007 deg
   GP2:
	UF : 7, UT : 7,	
	J1=  2953.160  mm
};
/END
