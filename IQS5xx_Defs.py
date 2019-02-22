#//*****************************************************************************
#//
#//! ----------------------    IQS5xx-B000 BIT DEFINITIONS     -----------------
#//
#//*****************************************************************************
#
#//
#//! GestureEvents0 bit definitions
#//
SWIPE_Y_NEG = 			0x20
SWIPE_Y_POS = 			0x10
SWIPE_X_POS = 			0x08
SWIPE_X_NEG = 			0x04
TAP_AND_HOLD = 			0x02
SINGLE_TAP = 			0x01
#//
#//! GesturesEvents1 bit definitions
#//
ZOOM = 					0x04
SCROLL = 				0x02
TWO_FINGER_TAP = 		0x01
#//
#//! SystemInfo0 bit definitions
#//
SHOW_RESET = 			0x80
ALP_REATI_OCCURRED = 	0x40
ALP_ATI_ERROR = 		0x20
REATI_OCCURRED = 		0x10
ATI_ERROR = 			0x08
CHARGING_MODE_2 = 		0x04
CHARGING_MODE_1 = 		0x02
CHARGING_MODE_0 = 		0x01
#//
#//! SystemInfo1 bit definitions
#//
SNAP_TOGGLE = 			0x10
RR_MISSED = 			0x08
TOO_MANY_FINGERS = 		0x04
PALM_DETECT = 			0x02
TP_MOVEMENT = 			0x01
#//
#//! SystemControl0 bit definitions
#//
ACK_RESET = 			0x80
AUTO_ATI = 				0x20
ALP_RESEED = 			0x10
RESEED = 				0x08
MODE_SELECT_2 = 		0x04
MODE_SELECT_1 = 		0x02
MODE_SELECT_0 = 		0x01
#//
#//! SystemControl1 bit definitions
#//
RESET = 				0x02
SUSPEND = 				0x01
#//
#//! SystemConfig0 bit definitions
#//
MANUAL_CONTROL = 		0x80
SETUP_COMPLETE = 		0x40
WDT_ENABLE = 			0x20
ALP_REATI = 			0x08
REATI = 				0x04
IO_WAKEUP_SELECT = 		0x02
IO_WAKE = 				0x01
#//
#//! SystemConfig1 bit definitions
#//
PROX_EVENT = 			0x80
TOUCH_EVENT = 			0x40
SNAP_EVENT = 			0x20
ALP_PROX_EVENT = 		0x10
REATI_EVENT = 			0x08
TP_EVENT = 				0x04
GESTURE_EVENT = 		0x02
EVENT_MODE = 			0x01
#//
#//! FilterSettings0 bit definitions
#//
ALP_COUNT_FILTER = 		0x08
IIR_SELECT = 			0x04
MAV_FILTER = 			0x02
IIR_FILTER = 			0x01
#//
#//! ALPChannelSetup0 bit definitions
#//
CHARGE_TYPE = 			0x80
RX_GROUP = 				0x40
PROX_REV = 				0x20
ALP_ENABLE = 			0x10
#//
#//! IQS525RxToTx bit definitions
#//
RX7_TX2 = 				0x80
RX6_TX3 = 				0x40
RX5_TX4 = 				0x20
RX4_TX5 = 				0x10
RX3_TX6 = 				0x08
RX2_TX7 = 				0x04
RX1_TX8 = 				0x02
RX0_TX9 = 				0x01
#//
#//! HardwareSettingsA bit definitions
#//
ND_ENABLE = 			0x20
RX_FLOAT = 				0x04
#//
#//! HardwareSettingsB bit definitions
#//
CK_FREQ_2 = 			0x40
CK_FREQ_1 = 			0x20
CK_FREQ_0 = 			0x10
ANA_DEAD_TIME = 		0x02
INCR_PHASE = 			0x01
#//
#//! HardwareSettingsC bit definitions
#//
STAB_TIME_1 = 			0x80
STAB_TIME_0 = 			0x40
OPAMP_BIAS_1 = 			0x20
OPAMP_BIAS_0 = 			0x10
VTRIP_3 = 				0x08
VTRIP_2 = 				0x04
VTRIP_1 = 				0x02
VTRIP_0 = 				0x01
#//
#//! HardwareSettingsD bit definitions
#//
UPLEN_2 = 				0x40
UPLEN_1 = 				0x20
UPLEN_0 = 				0x10
PASSLEN_2 = 			0x04
PASSLEN_1 = 			0x02
PASSLEN_0 = 			0x01
#//
#//! XYConfig0 bit definitions
#//
PALM_REJECT = 			0x08
SWITCH_XY_AXIS = 		0x04
FLIP_Y = 				0x02
FLIP_X = 				0x01
#//
#//! SFGestureEnable bit definitions
#//
SWIPE_Y_MINUS_EN = 		0x20
SWIPE_Y_PLUS_EN = 		0x10
SWIPE_X_PLUS_EN = 		0x08
SWIPE_X_MINUS_EN = 		0x04
TAP_AND_HOLD_EN = 		0x02
SINGLE_TAP_EN = 		0x01
#//
#//! MFGestureEnable bit definitions
#//
ZOOM_EN = 				0x04
SCROLL_EN = 			0x02
TWO_FINGER_TAP_EN = 	0x01
#
#//*****************************************************************************
#//
#//! ------------------    IQS5xx-B00 MEMORY MAP REGISTERS    ------------------
#//
#//*****************************************************************************
#
#/******************** DEVICE INFO REGISTERS ***************************/
ProductNumber_adr = 	0x0000	#(READ)			#2 BYTES;
ProjectNumber_adr = 	0x0002	#(READ)			#2 BYTES;
MajorVersion_adr = 		0x0004	#(READ)
MinorVersion_adr = 		0x0005	#(READ)
BLStatus_adr = 			0x0006	#(READ)
#/******************** ************************* ***************************/
MaxTouch_adr = 			0x000B	#(READ)
PrevCycleTime_adr = 	0x000C	#(READ)
#/******************** GESTURES AND EVENT STATUS REGISTERS ***************************/
GestureEvents0_adr = 	0x000D	#(READ)
GestureEvents1_adr = 	0x000E	#(READ)
SystemInfo0_adr = 		0x000F	#(READ)
SystemInfo1_adr = 		0x0010	#(READ)
#/******************** XY DATA REGISTERS ***************************/
NoOfFingers_adr = 		0x0011	#(READ)
RelativeX_adr = 		0x0012	#(READ)			#2 BYTES;
RelativeY_adr = 		0x0014	#(READ)		   	#2 BYTES;
#/******************** INDIVIDUAL FINGER DATA ***************************/
AbsoluteX_adr = 		0x0016	#(READ) 2 BYTES	#ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
AbsoluteY_adr = 		0x0018	#(READ) 2 BYTES	#ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
TouchStrength_adr = 	0x001A	#(READ) 2 BYTES	#ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
Area_adr = 				0x001C	#(READ)			#ADD 0x0007 FOR FINGER 2; 0x000E FOR FINGER 3; 0x0015 FOR FINGER 4 AND 0x001C FOR FINGER 5
#/******************** CHANNEL STATUS REGISTERS ***************************/
ProxStatus_adr = 		0x0039	#(READ)	  		#32 BYTES;
TouchStatus_adr = 		0x0059	#(READ)	 	    #30 BYTES;
SnapStatus_adr = 		0x0077	#(READ)		    #30 BYTES;
#/******************** DATA STREAMING REGISTERS ***************************/
Counts_adr = 			0x0095	#(READ)	  		#300 BYTES;
Delta_adr = 			0x01C1	#(READ)	 		#300 BYTES;
ALPCount_adr = 			0x02ED	#(READ)	 		#2 BYTES;
ALPIndivCounts_adr = 	0x02EF	#(READ)	 		#20 BYTES;
References_adr = 		0x0303	#(READ/WRITE)		#300 BYTES;
ALPLTA_adr = 			0x042F	#(READ/WRITE)		#2 BYTES;
#/******************** SYSTEM CONTROL REGISTERS ***************************/
SystemControl0_adr = 	0x0431	#(READ/WRITE)
SystemControl1_adr = 	0x0432	#(READ/WRITE)
#/******************** ATI SETTINGS REGISTERS ***************************/
ALPATIComp_adr = 		0x0435	#(READ/WRITE)  	#10 BYTES;
ATICompensation_adr = 	0x043F	#(READ/WRITE)  	#150 BYTES;
ATICAdjust_adr = 		0x04D5	#(READ/WRITE/E2)	#150 BYTES;
GlobalATIC_adr = 		0x056B	#(READ/WRITE/E2)
ALPATIC_adr = 			0x056C	#(READ/WRITE/E2)
ATITarget_adr = 		0x056D	#(READ/WRITE/E2)	#2 BYTES;
ALPATITarget_adr = 		0x056F	#(READ/WRITE/E2)	#2 BYTES;
RefDriftLimit_adr = 	0x0571	#(READ/WRITE/E2)
ALPLTADriftLimit_adr = 	0x0572	#(READ/WRITE/E2)
ReATILowerLimit_adr = 	0x0573	#(READ/WRITE/E2)
ReATIUpperLimit_adr = 	0x0574	#(READ/WRITE/E2)
MaxCountLimit_adr = 	0x0575	#(READ/WRITE/E2)	#2 BYTES;
ReATIRetryTime_adr = 	0x0577	#(READ/WRITE/E2)
#/******************** TIMING SETTINGS REGISTERS ***************************/
ActiveRR_adr = 			0x057A	#(READ/WRITE/E2)   #2 BYTES;
IdleTouchRR_adr = 		0x057C	#(READ/WRITE/E2)	#2 BYTES;
IdleRR_adr = 			0x057E	#(READ/WRITE/E2)	#2 BYTES;
LP1RR_adr = 			0x0580	#(READ/WRITE/E2)	#2 BYTES;
LP2RR_adr = 			0x0582	#(READ/WRITE/E2)	#2 BYTES;
ActiveTimeout_adr = 	0x0584	#(READ/WRITE/E2)
IdleTouchTimeout_adr = 	0x0585	#(READ/WRITE/E2)
IdleTimeout_adr = 		0x0586	#(READ/WRITE/E2)
LP1Timeout_adr = 		0x0587	#(READ/WRITE/E2)
RefUpdateTime_adr = 	0x0588	#(READ/WRITE/E2)
SnapTimeout_adr = 		0x0589	#(READ/WRITE/E2)
I2CTimeout_adr = 		0x058A	#(READ/WRITE/E2)
#/******************** SYSTEM CONFIG REGISTERS ***************************/
SystemConfig0_adr = 	0x058E	#(READ/WRITE/E2)
SystemConfig1_adr = 	0x058F	#(READ/WRITE/E2)
#/******************** THRESHOLD SETTINGS REGISTERS ***************************/
SnapThreshold_adr = 	0x0592	#(READ/WRITE/E2)   #2 BYTES;
ProxThreshold_adr = 	0x0594	#(READ/WRITE/E2)
ALPProxThreshold_adr = 	0x0595	#(READ/WRITE/E2)
GlobalTouchSet_adr = 	0x0596	#(READ/WRITE/E2)
GlobalTouchClear_adr = 	0x0597	#(READ/WRITE/E2)
IndivTouchAdjust_adr = 	0x0598	#(READ/WRITE/E2)	#150 BYTES;
#/******************** FILTER SETTINGS REGISTERS ***************************/
FilterSettings0_adr = 	0x0632	#(READ/WRITE/E2)
XYStaticBeta_adr = 		0x0633	#(READ/WRITE/E2)
ALPCountBeta_adr = 		0x0634	#(READ/WRITE/E2)
ALP1LTABeta_adr = 		0x0635	#(READ/WRITE/E2)
ALP2LTABeta_adr = 		0x0636	#(READ/WRITE/E2)
DynamicBottomBeta_adr = 0x0637	#(READ/WRITE/E2)
DynamicLowerSpeed_adr = 0x0638	#(READ/WRITE/E2)
DynamicUpperSpeed_adr = 0x0639	#(READ/WRITE/E2)   #2 BYTES;
#/******************** CHANNEL SET UP (RX-TX MAPPING) REGISTERS ***************************/
TotalRx_adr = 			0x063D	#(READ/WRITE/E2)
TotalTx_adr = 			0x063E	#(READ/WRITE/E2)
RxMapping_adr = 		0x063F	#(READ/WRITE/E2)	#10 BYTES;
TxMapping_adr = 		0x0649	#(READ/WRITE/E2)	#15 BYTES;
ALPChannelSetup0_adr = 	0x0658	#(READ/WRITE/E2)
ALPRxSelect_adr = 		0x0659	#(READ/WRITE/E2)	#2 BYTES;
ALPTxSelect_adr = 		0x065B	#(READ/WRITE/E2)	#2 BYTES;
IQS525RxToTx_adr = 		0x065D  #(READ/WRITE/E2)
#/******************** HARDWARE SETTINGS REGISTERS ***************************/
HardwareSettingsA_adr = 0x065F	#(READ/WRITE/E2)
HardwareSettingsB1_adr = 0x0660	#(READ/WRITE/E2)
HardwareSettingsB2_adr = 0x0661	#(READ/WRITE/E2)
HardwareSettingsC1_adr = 0x0662	#(READ/WRITE/E2)
HardwareSettingsC2_adr = 0x0663	#(READ/WRITE/E2)
HardwareSettingsD1_adr = 0x0664	#(READ/WRITE/E2)
HardwareSettingsD2_adr = 0x0665	#(READ/WRITE/E2)
#/******************** XY CONFIG REGISTERS ***************************/
XYConfig0_adr = 		0x0669	#(READ/WRITE/E2)
MaxMultitouches_adr = 	0x066A	#(READ/WRITE/E2)
FingerSplitFactor_adr = 0x066B	#(READ/WRITE/E2)
PalmRejectThreshold_adr = 0x066C	#(READ/WRITE/E2)
PalmRejectTimeout_adr = 0x066D	#(READ/WRITE/E2)
XResolution_adr = 		0x066E	#(READ/WRITE/E2)	#2 BYTES;
YResolution_adr = 		0x0670	#(READ/WRITE/E2)	#2 BYTES;
StationaryTouchThr_adr = 0x0672	#(READ/WRITE/E2)
#/*********************************************************************/
DefaultReadAdr_adr = 	0x0675	#(READ/WRITE/E2)
#/******************** DEBOUNCE SETTING REGISTERS ***************************/
ProxDb_adr = 			0x0679	#(READ/WRITE/E2)
TouchSnapDb_adr = 		0x067A	#(READ/WRITE/E2)
#/******************** CHANNEL CONFIG REGISTERS ***************************/
ActiveChannels_adr = 	0x067B	#(READ/WRITE/E2)	#30 BYTES;
SnapChannels_adr = 		0x0699	#(READ/WRITE/E2)   #30 BYTES;
#/******************** GESTURE SETTING REGISTERS ***************************/
SFGestureEnable_adr = 	0x06B7	#(READ/WRITE/E2)
MFGestureEnable_adr = 	0x06B8	#(READ/WRITE/E2)
TapTime_adr = 			0x06B9	#(READ/WRITE/E2)	#2 BYTES;
TapDistance_adr = 		0x06BB	#(READ/WRITE/E2)	#2 BYTES;
HoldTime_adr = 			0x06BD	#(READ/WRITE/E2)   #2 BYTES;
SwipeInitTime_adr = 	0x06BF	#(READ/WRITE/E2)	#2 BYTES;
SwipeInitDistance_adr = 0x06C1	#(READ/WRITE/E2)	#2 BYTES;
SwipeConsTime_adr = 	0x06C2	#(READ/WRITE/E2)	#2 BYTES;
SwipeConsDistance_adr = 0x06C5	#(READ/WRITE/E2)	#2 BYTES;
SwipeAngle_adr = 		0x06C7	#(READ/WRITE/E2)
ScrollInitDistance_adr = 0x06C8	#(READ/WRITE/E2)	#2 BYTES;
ScrollAngle_adr = 		0x06CA	#(READ/WRITE/E2)
ZoomInitDistance_adr = 	0x06CB	#(READ/WRITE/E2)	#2 BYTES;
ZoomConsDistance_adr = 	0x06CD	#(READ/WRITE/E2)	#2 BYTES;
#
#
#
EndWindow_adr = 		0xEEEE
