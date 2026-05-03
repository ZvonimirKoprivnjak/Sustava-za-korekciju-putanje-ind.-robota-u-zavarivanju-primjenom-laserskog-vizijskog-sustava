/PROG  OFFLINE_LOMLJENI
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "offline_TP";
PROG_SIZE	= 1000;
CREATE		= DATE 26-04-20  TIME 22:14:44;
MODIFIED	= DATE 26-04-20  TIME 22:25:06;
FILE_NAME	= ZK_012_O;
VERSION		= 0;
LINE_COUNT	= 27;
MEMORY_SIZE	= 1412;
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
   1:  UFRAME_NUM=7 ;
   2:  UTOOL_NUM=7 ;
   3:  R[49:Odstupanje]=1.5    ;
   4:  R[52:Kraj_snimanja]=0    ;
   5:   ;
   6:L P[3] 100mm/sec FINE    ;
   7:  RUN OFFLINE_1 ;
   8:  R[52:Kraj_snimanja]=1    ;
   9:  WAIT R[54]=11111 TIMEOUT,LBL[99] ;
  10:   ;
  11:L P[3] 10mm/sec FINE    ;
  12:L P[1] 10mm/sec FINE    ;
  13:   ;
  14:   ;
  15:  R[52:Kraj_snimanja]=99000    ;
  16:  WAIT R[52:Kraj_snimanja]=99000 TIMEOUT,LBL[99] ;
  17:  //PAUSE ;
  18:  CALL OFFLINE_2    ;
  19:  LBL[99] ;
  20:  WAIT R[55:Broj_korekcija]<>0    ;
  21:  //PAUSE ;
  22:  R[56:Brojac]=0    ;
  23:  R[57:Help_var]=160    ;
  24:  FOR R[56:Brojac]=1 TO R[55:Broj_korekcija] ;
  25:L PR[R[57]] 10mm/sec CNT100    ;
  26:  R[57:Help_var]=R[57:Help_var]+1    ;
  27:  ENDFOR ;
/POS
P[1]{
   GP1:
	UF : 7, UT : 7,		CONFIG : 'N U T, 0, 0, 0',
	X =    85.906  mm,	Y =   149.000  mm,	Z =    -8.874  mm,
	W =   -90.004 deg,	P =     -.006 deg,	R =      .003 deg
   GP2:
	UF : 7, UT : 7,	
	J1=  2953.160  mm
};
P[3]{
   GP1:
	UF : 7, UT : 7,		CONFIG : 'N U T, 0, 0, 0',
	X =    85.906  mm,	Y =      .200  mm,	Z =    -8.874  mm,
	W =   -90.001 deg,	P =     -.002 deg,	R =      .000 deg
   GP2:
	UF : 7, UT : 7,	
	J1=  2953.160  mm
};
/END
