#include "C826_analog.h"
#include <stdio.h>
#include <process.h>
#include <math.h>
unsigned __stdcall IOcardThread(void* arg);

#define X826(FUNC)	if ((errcode = FUNC) != S826_ERR_OK) { printf("\nERROR: %d\n", errcode); return errcode; }
// Counter mode: count down at 1MHz, preload upon start or when 0 reached, assert ExtOut when not 0
#define TMR_MODE  (S826_CM_K_1MHZ | S826_CM_UD_REVERSE | S826_CM_PX_ZERO | S826_CM_PX_START | S826_CM_OM_NOTZERO)

#define PI  3.1415926535

glm::vec2  m_act_pos[]= {glm::vec2(-1, 1), glm::vec2(0, 1), glm::vec2(1, 1), glm::vec2(-1, 0), glm::vec2(0, 0), glm::vec2(1, 0), glm::vec2(-1, -1), glm::vec2(0, -1), glm::vec2(1, -1)};

c826_analog::c826_analog() : m_cardInit(false), m_IO_enabled(false), m_hEvent(NULL), m_signalEnabled(false), m_sineTest_4ch(false)
{
	m_board = 0;
	m_outSingalType = 0;
	m_signalDuration = 0.f;
	m_hCustomEvent = NULL;
	m_nPlayCycle = 5;
	m_signalChange = false;
	m_signalType = 0;
	for(int i=0;i<8;i++) {
		m_channelEnable[i] = false;
		m_aout[i] = i;
	}
	m_n_ch=0;
}
c826_analog::c826_analog(uint board) : m_cardInit(false), m_IO_enabled(false), m_hEvent(NULL), m_signalEnabled(false), m_sineTest_4ch(false)
{
	m_board = board;
	m_outSingalType = 0;
	m_signalDuration = 0.f;
	m_hCustomEvent = NULL;
	m_nPlayCycle = 5;
	m_signalChange = false;
	for(int i=0;i<8;i++) {
		m_channelEnable[i] = false;
		m_aout[i] = i;
	}
	m_n_ch = 0;
}
c826_analog::~c826_analog()
{
	if(m_cardInit) {
		m_IO_enabled = false;
		SetEvent(m_hEvent);
		S826_SystemClose();
	}
	if(m_actuator) delete m_actuator;
}

int c826_analog::init(char* ret_string)
{
	int ret = 0;
	uint board		= 0;
	int errcode		= S826_ERR_OK;
//// device initializing routine
	int boardflags	= S826_SystemOpen();	// open 826 driver and find all 826 boards
	if(boardflags < 0) {
		errcode = boardflags;	// problem during open
		getErrorString(errcode, ret_string);
	}
	else if((boardflags & (1 << board)) == 0) {
		sprintf(ret_string, "Target board not found");
		ret = -2;
	}
	else {
		m_hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
		m_IO_enabled = true;
		m_h_IOcardThread = (HANDLE)_beginthreadex(NULL, 0, IOcardThread, (void*)this, 0, NULL);
	}
///// physical/phantom actuator setup
////
	double A = 1.0, sum_SOA = 0.f, SOA;
	double duration = 0.070;
	m_n_src = 3;
	m_actuator = new actuator_info[m_n_src];
	/// 0
	m_actuator[0].num_actuator = 1;
	m_actuator[0].act_idx[0] = 1;
	m_actuator[0].amplitude[0] = A;
	m_actuator[0].duration = duration;
	SOA = 0.32*duration+0.0473;
	m_actuator[0].SOA = SOA;
	sum_SOA += SOA;
	/// 1
	m_actuator[1].num_actuator = 3;
	m_actuator[1].act_idx[0] = 0; m_actuator[1].act_idx[1] = 1; m_actuator[1].act_idx[2] = 2;
	m_actuator[1].amplitude[0]=0.802*A; m_actuator[1].amplitude[1]=0.422*A; m_actuator[1].amplitude[2]=0.422*A;
	////
//	m_actuator[1].num_actuator = 3;
//	m_actuator[1].act_idx[0] = 1; m_actuator[1].act_idx[1] = 2;
//	m_actuator[1].amplitude
	//m_actuator[1].amplitude[0]=0.802*A; m_actuator[1].amplitude[1]=0.422*A; m_actuator[1].amplitude[2]=0.422*A;
	m_actuator[1].duration = duration;
	SOA = 0.32*duration+0.0473;
	m_actuator[1].SOA = SOA;
	sum_SOA += SOA;
	/// 2
	m_actuator[2].num_actuator = 2;
	m_actuator[2].act_idx[0] = 0; m_actuator[2].act_idx[1] = 2;
	m_actuator[2].amplitude[0] = 0.707*A; m_actuator[2].amplitude[1] = 0.707*A;
	m_actuator[2].duration = duration;
	SOA = 0.32*duration + 0.0473;
	m_actuator[2].SOA = SOA;
	sum_SOA += SOA;
	///
	m_unit_period = sum_SOA+ m_actuator[m_n_src-1].duration + 0.2;//1.0;
	printf("unit_perid = %f\n", m_unit_period);
	return ret;
}

void c826_analog::getErrorString(int errcode, char* ret_string)
{
	switch(errcode) {
    case S826_ERR_OK:    
		break;
    case S826_ERR_BOARD:
		sprintf(ret_string, "Illegal board number");
		break;
    case S826_ERR_VALUE:
		sprintf(ret_string, "Illegal argument");
		break;
    case S826_ERR_NOTREADY:
		sprintf(ret_string, "Device not ready or timeout");
		break;
    case S826_ERR_CANCELLED:
		sprintf(ret_string, "Wait cancelled");
		break;
    case S826_ERR_DRIVER:
		sprintf(ret_string, "Driver call failed");
		break;
    case S826_ERR_MISSEDTRIG:
		sprintf(ret_string, "Missed adc trigger");
		break;
    case S826_ERR_DUPADDR:
		sprintf(ret_string, "Two boards have same number");
		S826_SafeWrenWrite(m_board, 0x02);
		break;
    case S826_ERR_BOARDCLOSED:
		sprintf(ret_string, "Board not open");
		break;
	case S826_ERR_CREATEMUTEX:
		sprintf(ret_string, "Can't create mutex");
		break;
    case S826_ERR_MEMORYMAP:
		sprintf(ret_string, "Can't map board");
		break;
    default:
		sprintf(ret_string, "Unknown error");
		break;
	}
}

int c826_analog::periodicTimerStart(uint counter, uint period)
{
	int errcode;
	X826(S826_CounterStateWrite(m_board, counter, 0)			);	// halt channel if it is running
	X826(S826_CounterModeWrite(m_board, counter, TMR_MODE)	);	// configure counter as a periodic timer
	X826(S826_CounterSnapshotConfigWrite(m_board, counter, 4, S826_BITWRITE));	// capture snapshots at zero counts
	X826(S826_CounterPreloadWrite(m_board, counter, 0, period));	// timer period in microseconds
	X826(S826_CounterStateWrite(m_board, counter, 1)			);	// start timer
	return errcode;
}
// Halt channel operating as periodic timer
int c826_analog::periodicTimerStop(uint counter)
{
	return S826_CounterStateWrite(m_board, counter, 0);
}

// Wait for periodic timer event.
int c826_analog::periodicTimerWait(uint counter, uint *timestamp)
{
	uint counts, tstamp, reason;		// counter snapsho
	int errcode = S826_CounterSnapshotRead(m_board, counter, &counts, &tstamp, &reason, S826_WAIT_INFINITE);	// wait for timer snapshot
	if(timestamp != NULL)
		*timestamp = tstamp;
	return errcode;
}


void c826_analog::triggerTestSignal(int signal_type, HANDLE trigger_event, bool *flag)
{
	m_signalType = signal_type;
	m_hCustomEvent = trigger_event;
	m_signalEnabled = true;
	m_test_flag = flag;
	SetEvent(m_hEvent);
}

/////////////////////////////
// triggerStimulus_4ch :  triggers analog output for the input duration time
//
void c826_analog::triggerSinewave_4ch(const bool channel_enable[4], double amplitude, double freq, double offset, double duration, HANDLE trigger_event, bool test)
{
	m_sineTest_4ch = test;
	for(int i=0;i<4;i++) {
		m_channelEnable[i] = channel_enable[i];
	}
	m_signalDuration = duration;
	m_signalEnabled = true;
	m_sineFreq = freq;
	m_sineAmplitude = amplitude;
	m_sineOffset = offset;
	m_hCustomEvent = trigger_event;
	SetEvent(m_hEvent);
}
void c826_analog::triggerSignal(bool enable)
{
	if(m_signalEnabled != enable) {
		if(enable) {
			m_signalEnabled = enable;
			SetEvent(m_hEvent);
		}
		else m_signalEnabled = enable;
	}
}

void c826_analog::triggerSignal_ch(bool enable, int channel)
{
	if(m_signalEnabled != enable) {
		if(enable) {
			m_signalEnabled = enable;
			m_enabled_channel = channel;
			SetEvent(m_hEvent);
		}
		else m_signalEnabled = enable;
	}
}

void c826_analog::setActuatorParams(int n_actuator, int *act_idx, glm::vec2 act_pos, double A, actuator_info *pActuator)
{
	int i;
	double sum_d_inv, dist_buf, d_inv[3];
	pActuator->num_actuator = n_actuator;
	sum_d_inv = 0.f;
	for(i=0;i<n_actuator;i++) {
		pActuator->act_idx[i] = act_idx[i];
		dist_buf = glm::distance(m_act_pos[act_idx[i]], act_pos);
		if(dist_buf != 0.f) d_inv[i] = 1/dist_buf;
		else d_inv[i] = 0.f;
		sum_d_inv += d_inv[i];
	}
	for(i=0;i<3;i++)
		pActuator->amplitude[i] = sqrt(d_inv[i]/sum_d_inv)*A;
}

void c826_analog::playSignal_test()
{
	int errcode, i, j, n_src =3, n_ch = 4, curr_ch;
	double runtime, period_begin, sig_time_offset, t_signal_start;//sig_begin_time;
	double A = 1.0, duration = 0.140, sum_SOA = 0.f;
	double unit_period = 0.5;
	double t0 = 0.5;//, t_k;
///////
	uint counter = 0;	// Use this counter channel as DAC sample timer
	uint t_sample = 1;	// sample time in microseconds. Must be fast enough to meet Nyquist. 
	uint tstamp;	// timestamp in microseconds
	uint tbegin;	// sampling begin time in microseconds
	uint DAC_val;
//////
	bool *ch_enable;
	uint *amplitude, *offset; 
	amplitude = new uint[n_ch], offset = new uint[n_ch];
	ch_enable = new bool[n_ch];
//	actuator_info *pActuator = new actuator_info[n_src];
	for(i=0;i<n_ch;i++) {
		amplitude[i] = (uint)(32268.0*m_sineAmps[i]/10.0);
		offset[i] = (uint)(32268.0 + 32268.0*m_sineOffsets[i]/10.0);
		errcode = S826_DacRangeWrite(m_board, m_aout[i], S826_DAC_SPAN_10_10, 0);
		ch_enable[i] = false;
	}
	errcode = periodicTimerStart(counter, t_sample);
	errcode = periodicTimerWait(counter, &tstamp);
	tbegin = tstamp;
	period_begin = 0.f;
	sig_time_offset = 0.f;
/////
	ch_enable[0] = true;
	curr_ch =0;
	t_signal_start = 0.f;
	m_signalEnabled = true;
	*m_test_flag = false;
	while(m_signalEnabled) {
		runtime = (double) (tstamp - tbegin) / 1000000.0;	// compute time elapsed since t0 in seconds
		if(runtime >= (t0*(double)n_ch)) break;
		else {
			////
			if(runtime >= t0*((double)curr_ch+1.0)) {
				curr_ch++;
			//	printf("curr_ch=%d\n", curr_ch);
				t_signal_start = t0*(double)curr_ch;
				SetEvent(m_hCustomEvent);
			}
			DAC_val = (uint)(offset[curr_ch]+amplitude[curr_ch]*1.0*sin(2.0*PI*m_sineFreqs[curr_ch]*(runtime - t_signal_start)));
			// writes value
			S826_DacDataWrite(m_board, m_aout[curr_ch], DAC_val, 0);
		}
		errcode = periodicTimerWait(counter, &tstamp);
	}
	errcode = periodicTimerStop(counter);
	if(m_hCustomEvent != NULL) {
		*m_test_flag = true;
		SetEvent(m_hCustomEvent);
		m_hCustomEvent = NULL;
	}
	delete amplitude; delete offset;
}

void c826_analog::run_TBrush_signal(int n_src, double unit_period, actuator_info *pActuator)
{
	int errcode, i, j, act_idx;
	double runtime, period_begin, sig_time_offset, sig_begin_time, act_amplitude;
///////////////
	uint counter = 0;	// Use this counter channel as DAC sample timer
	uint t_sample = 1;	// sample time in microseconds. Must be fast enough to meet Nyquist. 
	uint tstamp;	// timestamp in microseconds
	uint tbegin;	// sampling begin time in microseconds
	uint DAC_val;
	uint *amplitude, *offset; 
	amplitude = new uint[m_n_ch], offset = new uint[m_n_ch];
	m_signalEnabled = true;
	for(i=0;i<m_n_ch;i++) {
		amplitude[i] = (uint)(32268.0*m_sineAmps[i]/10.0);
		offset[i] = (uint)(32268.0 + 32268.0*m_sineOffsets[i]/10.0);
///		printf("idx=%d amplitude=%d offset=%d\n", i, amplitude[i], offset[i]); 
	}
	for(i=0;i<n_src;i++) {
		pActuator[i].enabled = false;
		pActuator[i].t_signal_start = -100;
	}
	for(i=0;i<m_n_ch;i++)
		errcode = S826_DacRangeWrite(m_board, m_aout[i], S826_DAC_SPAN_10_10, 0);
	errcode = periodicTimerStart(counter, t_sample);
	errcode = periodicTimerWait(counter, &tstamp);
	tbegin = tstamp;
	period_begin = 0.f;
	sig_time_offset = 0.f;
	printf("unit period = %f\n", unit_period);
	while(m_signalEnabled) {
		runtime = (double) (tstamp - tbegin) / 1000000.0;	// compute time elapsed since t0 in seconds
		//
		if(runtime >= (period_begin+unit_period)) {
		//	printf("breaks\n");
			break;
		//	period_begin += unit_period;	// assuming that the timer is fast & accurate enough,
		}
		if(!pActuator[0].enabled) {
			if(runtime >=(period_begin+sig_time_offset) && runtime <=(period_begin+pActuator[0].duration + sig_time_offset)) {
			//	printf("act 0 enabled ---runtime =%f\n", runtime);
				pActuator[0].enabled = true;
				pActuator[0].t_signal_start = period_begin + sig_time_offset;
			}
		}
		else {
			if(runtime > (pActuator[0].t_signal_start+pActuator[0].duration)) {
			//	printf("act 0 disabled\n");
				pActuator[0].enabled = false;
				pActuator[0].t_signal_end = pActuator[0].t_signal_start+pActuator[0].duration;
			}
		}
	/////// enable/disable actuators 
		for(i=1;i<n_src;i++) {
			if(!pActuator[i].enabled){// && pActuator[i].t_signal_start >= 0.0) {
				sig_begin_time = pActuator[i-1].t_signal_start+pActuator[i-1].SOA;
				if(runtime >= sig_begin_time && runtime <= (sig_begin_time+pActuator[i].duration)) {
				//	printf("[%d]%f ", i, sig_begin_time);
				//	printf("act %d enabled\n", i);
					pActuator[i].enabled = true;
					pActuator[i].t_signal_start = sig_begin_time;
				}
			}
			else {
				if(runtime >= (pActuator[i].t_signal_start+pActuator[i].duration)) {
				//	printf("act %d disabled\n", i);
					pActuator[i].enabled = false;
					pActuator[i].t_signal_end = pActuator[i].t_signal_start+pActuator[i].duration;
				}
			}
		}
	//	printf("\n");
	//////  Writing output values
		for(i=0;i<n_src;i++) {
			if(pActuator[i].enabled) {
			//	printf("%d (", i);
				for(j=0;j<pActuator[i].num_actuator;j++) {
					act_idx = pActuator[i].act_idx[j];
					act_amplitude = pActuator[i].amplitude[j];
					// converts value
					DAC_val = (uint)(offset[act_idx]+amplitude[act_idx]*act_amplitude*sin(2.0*PI*m_sineFreqs[act_idx]*(runtime-pActuator[i].t_signal_start)));
					// writes value
					S826_DacDataWrite(m_board, m_aout[act_idx], DAC_val, 0);
				}
			}
		}
		errcode = periodicTimerWait(counter, &tstamp);
	}	
	errcode = periodicTimerStop(counter);
	if(m_hCustomEvent != NULL) {
		SetEvent(m_hCustomEvent);
		m_hCustomEvent = NULL;
	}
}

void c826_analog::playBruteForce()
{

}

void c826_analog::playSignal_TBrush1()
{
	int n_src =3;
	double A = 1.0, duration = 0.140, SOA, sum_SOA = 0.f;
	double unit_period, d0, d1;
	double t0 = 0.7, t_k;
	actuator_info *pActuator = new actuator_info[n_src];
	double end_time[2];
//////////////
	/// 0
	pActuator[0].num_actuator = 2;
	pActuator[0].act_idx[0] = 1;
	pActuator[0].act_idx[1] = 4;
	pActuator[0].amplitude[0] = 0.894*A;
	pActuator[0].amplitude[1] = 0.447*A;
	d0 = 0.76*t0 - 0.0358;
	pActuator[0].duration = d0;//duration;
	SOA = 0.32*pActuator[0].duration+0.0473;
	pActuator[0].SOA = SOA;
//	printf("d0=%f SOA=%f, t(0, %f)\n", d0, SOA, pActuator[0].duration);
	sum_SOA += SOA;
	/// 1
	pActuator[1].num_actuator = 2;
	pActuator[1].act_idx[0] = 3; pActuator[1].act_idx[1] = 4;
	pActuator[1].amplitude[0]=0.894*A; pActuator[1].amplitude[1]=0.447*A; //m_actuator[1].amplitude[2]=0.422*A;
	t_k = t0*2;
	d1 = 0.76*t_k-0.24*d0-0.76*sum_SOA-0.0358;
	pActuator[1].duration = d0+d1;//duration;
	SOA = 0.32*pActuator[1].duration+0.0473;
	pActuator[1].SOA = SOA;
//	printf("d0=%f d1=%f duration=%f SOA=%f t(%f, %f)\n", d0, d1, d0+d1, SOA, sum_SOA, sum_SOA+pActuator[1].duration);
	end_time[0] = sum_SOA + pActuator[1].duration;
	sum_SOA += SOA;
	/// 2
	pActuator[2].num_actuator = 2;
	pActuator[2].act_idx[0] = 4; pActuator[2].act_idx[1] = 7;
	pActuator[2].amplitude[0] = 0.447*A; pActuator[2].amplitude[1] = 0.894*A;
	SOA = 0.32*d1 + 0.0473;
	pActuator[2].SOA = SOA;
	pActuator[2].duration = d1;
	end_time[1] = sum_SOA+pActuator[2].duration;
//	printf("d1=%f SOA=%f t(%f, %f)\n", d1, SOA, sum_SOA, sum_SOA+pActuator[2].duration);
	if(end_time[0] <= end_time[1]) unit_period = end_time[1] + 0.2;
	else unit_period = end_time[0]+0.2;
	//unit_period = sum_SOA + pActuator[n_src-1].duration + 0.2;
//	printf("bbbb: %f %f %f\n", pActuator[0].duration, pActuator[1].duration, pActuator[2].duration);
	run_TBrush_signal(n_src, unit_period, pActuator);
	delete pActuator;
}

void c826_analog::playSignal_TBrush2()
{
	int errcode, i, j, act_idx, n_src =3;
	double A = 1.0, duration = 0.140, SOA, sum_SOA = 0.f;
	double unit_period, d0, d1;
	double t0 = 0.7, t_k;
	actuator_info *pActuator = new actuator_info[n_src];
	double end_time[2];
//////////////
	/// 0
	pActuator[0].num_actuator = 2;
	pActuator[0].act_idx[0] = 0;
	pActuator[0].act_idx[1] = 3;
	pActuator[0].amplitude[0] = 0.894*A;
	pActuator[0].amplitude[1] = 0.447*A;
	d0 = 0.76*t0 - 0.0358;
	pActuator[0].duration = d0;//duration;
	SOA = 0.32*pActuator[0].duration+0.0473;
	pActuator[0].SOA = SOA;
//	printf("d0=%f SOA=%f, t(0, %f)\n", d0, SOA, pActuator[0].duration);
	sum_SOA += SOA;
	/// 1
	pActuator[1].num_actuator = 2;
	pActuator[1].act_idx[0] = 4; pActuator[1].act_idx[1] = 3;
	pActuator[1].amplitude[0]=0.894*A; pActuator[1].amplitude[1]=0.447*A; //m_actuator[1].amplitude[2]=0.422*A;
	t_k = t0*2;
	d1 = 0.76*t_k-0.24*d0-0.76*sum_SOA-0.0358;
	pActuator[1].duration = d0+d1;//duration;
	SOA = 0.32*pActuator[1].duration+0.0473;
	pActuator[1].SOA = SOA;
//	printf("d0=%f d1=%f duration=%f SOA=%f t(%f, %f)\n", d0, d1, d0+d1, SOA, sum_SOA, sum_SOA+pActuator[1].duration);
	end_time[0] = sum_SOA + pActuator[1].duration;
	sum_SOA += SOA;
	/// 2
	pActuator[2].num_actuator = 2;
	pActuator[2].act_idx[0] = 3; pActuator[2].act_idx[1] = 6;
	pActuator[2].amplitude[0] = 0.447*A; pActuator[2].amplitude[1] = 0.894*A;
	SOA = 0.32*d1 + 0.0473;
	pActuator[2].SOA = SOA;
	pActuator[2].duration = d1;
	end_time[1] = sum_SOA+pActuator[2].duration;
//	printf("d1=%f SOA=%f t(%f, %f)\n", d1, SOA, sum_SOA, sum_SOA+pActuator[2].duration);
	if(end_time[0] <= end_time[1]) unit_period = end_time[1] + 0.2;
	else unit_period = end_time[0]+0.2;
//	unit_period = sum_SOA + pActuator[n_src-1].duration + 0.2;
//	printf("bbbb: %f %f %f\n", pActuator[0].duration, pActuator[1].duration, pActuator[2].duration);
	run_TBrush_signal(n_src, unit_period, pActuator);
	delete pActuator;
}

void c826_analog::playSignal_TBrush3()
{
	int errcode, i, j, act_idx, n_src =4;
	int idx_arr[][2] = {{4, 7}, {3, 4}, {1, 4}, {4, 5}};
	double A = 1.0, duration = 0.140, SOA, sum_SOA = 0.f;
	double unit_period, d0, d1;
	double t0 = 0.7, t_k;
	actuator_info *pActuator = new actuator_info[n_src];
	double end_time[2];
//////////////

//// check variables
	/// 0
	pActuator[0].num_actuator = 2;
	memcpy((void*)pActuator[0].act_idx, (void*) idx_arr[0], sizeof(int)*2);
//	pActuator[0].act_idx[0] = 4; pActuator[0].act_idx[1] = 7;
	pActuator[0].amplitude[0] = 0.447*A; pActuator[0].amplitude[1] = 0.894*A;
	d0 = 0.76*t0 - 0.0358;
	pActuator[0].duration = d0;//duration;
	SOA = 0.32*pActuator[0].duration+0.0473;
	pActuator[0].SOA = SOA;
//	printf("d0=%f SOA=%f, t(0, %f)\n", d0, SOA, pActuator[0].duration);
	sum_SOA += SOA;
	/// 1
	pActuator[1].num_actuator = 2;
	memcpy((void*)pActuator[1].act_idx, (void*) idx_arr[1], sizeof(int)*2);
//	pActuator[1].act_idx[0] = 4; pActuator[1].act_idx[1] = 3;
	pActuator[1].amplitude[0]=0.894*A; pActuator[1].amplitude[1]=0.447*A; //m_actuator[1].amplitude[2]=0.422*A;
	t_k = t0*2;
	d1 = 0.76*t_k-0.24*d0-0.76*sum_SOA-0.0358;
	pActuator[1].duration = d0+d1;//duration;
	SOA = 0.32*pActuator[1].duration+0.0473;
	pActuator[1].SOA = SOA;
//	printf("d0=%f d1=%f duration=%f SOA=%f t(%f, %f)\n", d0, d1, d0+d1, SOA, sum_SOA, sum_SOA+pActuator[1].duration);
	sum_SOA += SOA;
	d0 = d1;
	/// 2
	pActuator[2].num_actuator = 2;
	memcpy((void*)pActuator[2].act_idx, (void*) idx_arr[2], sizeof(int)*2);
//	pActuator[2].act_idx[0] = 3; pActuator[2].act_idx[1] = 6;
	pActuator[2].amplitude[0] = 0.894*A; pActuator[2].amplitude[1] = 0.447*A;
	t_k = t0*3;
	d1 = 0.75*t_k-0.24*d0-0.76*sum_SOA-0.0358;
	pActuator[2].duration = d0+d1;
//	pActuator[2].duration = d1*2.0;
	SOA = 0.32*pActuator[2].duration + 0.0473;
	pActuator[2].SOA = SOA;
//	printf("d1=%f SOA=%f t(%f, %f)\n", d1, SOA, sum_SOA, sum_SOA+pActuator[2].duration);
	end_time[0] = sum_SOA + pActuator[2].duration;
	sum_SOA += SOA;
	/// 3
	pActuator[3].num_actuator = 2;
	memcpy((void*)pActuator[3].act_idx, (void*) idx_arr[3], sizeof(int)*2);
	pActuator[3].amplitude[0] = 0.447*A; pActuator[3].amplitude[1] = 0.894*A;
	SOA = 0.32*d1 + 0.0473;
	pActuator[3].SOA = SOA;
	pActuator[3].duration = d1;
	end_time[1] = sum_SOA+pActuator[3].duration;
//	printf("d1=%f SOA=%f t(%f, %f)\n", d1, SOA, sum_SOA, sum_SOA+pActuator[2].duration);
	if(end_time[0] <= end_time[1]) unit_period = end_time[1] + 0.2;
	else unit_period = end_time[0]+0.2;
//	unit_period = sum_SOA + pActuator[n_src-1].duration + 0.2;
//	printf("bbbb: %f %f %f\n", pActuator[0].duration, pActuator[1].duration, pActuator[2].duration);
	run_TBrush_signal(n_src, unit_period, pActuator);
	delete pActuator;
}

void c826_analog::playSignal_TBrush4()
{
	int errcode, i, j, act_idx, n_src =4;
	int idx_arr[][2] = {{4,5}, {1,4}, {3,4}, {4,7}};//{4, 7}, {3, 4}, {1, 4}, {4, 5}};
	double A = 1.0, duration = 0.140, SOA, sum_SOA = 0.f;
	double unit_period, d0, d1;
	double t0 = 0.7, t_k;
	actuator_info *pActuator = new actuator_info[n_src];
	double end_time[2];
//////////////

//// check variables
	/// 0
	pActuator[0].num_actuator = 2;
	memcpy((void*)pActuator[0].act_idx, (void*) idx_arr[0], sizeof(int)*2);
//	pActuator[0].act_idx[0] = 4; pActuator[0].act_idx[1] = 7;
	pActuator[0].amplitude[0] = 0.447*A; pActuator[0].amplitude[1] = 0.894*A;
	d0 = 0.76*t0 - 0.0358;
	pActuator[0].duration = d0;//duration;
	SOA = 0.32*pActuator[0].duration+0.0473;
	pActuator[0].SOA = SOA;
//	printf("d0=%f SOA=%f, t(0, %f)\n", d0, SOA, pActuator[0].duration);
	sum_SOA += SOA;
	/// 1
	pActuator[1].num_actuator = 2;
	memcpy((void*)pActuator[1].act_idx, (void*) idx_arr[1], sizeof(int)*2);
//	pActuator[1].act_idx[0] = 4; pActuator[1].act_idx[1] = 3;
	pActuator[1].amplitude[0]=0.894*A; pActuator[1].amplitude[1]=0.447*A; //m_actuator[1].amplitude[2]=0.422*A;
	t_k = t0*2;
	d1 = 0.76*t_k-0.24*d0-0.76*sum_SOA-0.0358;
	pActuator[1].duration = d0+d1;//duration;
	SOA = 0.32*pActuator[1].duration+0.0473;
	pActuator[1].SOA = SOA;
//	printf("d0=%f d1=%f duration=%f SOA=%f t(%f, %f)\n", d0, d1, d0+d1, SOA, sum_SOA, sum_SOA+pActuator[1].duration);
	sum_SOA += SOA;
	d0 = d1;
	/// 2
	pActuator[2].num_actuator = 2;
	memcpy((void*)pActuator[2].act_idx, (void*) idx_arr[2], sizeof(int)*2);
//	pActuator[2].act_idx[0] = 3; pActuator[2].act_idx[1] = 6;
	pActuator[2].amplitude[0] = 0.894*A; pActuator[2].amplitude[1] = 0.447*A;
	t_k = t0*3;
	d1 = 0.75*t_k-0.24*d0-0.76*sum_SOA-0.0358;
	pActuator[2].duration = d0+d1;
//	pActuator[2].duration = d1*2.0;
	SOA = 0.32*pActuator[2].duration + 0.0473;
	pActuator[2].SOA = SOA;
//	printf("d1=%f SOA=%f t(%f, %f)\n", d1, SOA, sum_SOA, sum_SOA+pActuator[2].duration);
	end_time[0] = sum_SOA + pActuator[2].duration;
	sum_SOA += SOA;
	/// 3
	pActuator[3].num_actuator = 2;
	memcpy((void*)pActuator[3].act_idx, (void*) idx_arr[3], sizeof(int)*2);
	pActuator[3].amplitude[0] = 0.447*A; pActuator[3].amplitude[1] = 0.894*A;
	SOA = 0.32*d1 + 0.0473;
	pActuator[3].SOA = SOA;
	pActuator[3].duration = d1;
	end_time[1] = sum_SOA+pActuator[3].duration;
//	printf("d1=%f SOA=%f t(%f, %f)\n", d1, SOA, sum_SOA, sum_SOA+pActuator[2].duration);
	if(end_time[0] <= end_time[1]) unit_period = end_time[1] + 0.2;
	else unit_period = end_time[0]+0.2;
//	unit_period = sum_SOA + pActuator[n_src-1].duration + 0.2;
//	printf("bbbb: %f %f %f\n", pActuator[0].duration, pActuator[1].duration, pActuator[2].duration);
	run_TBrush_signal(n_src, unit_period, pActuator);
	delete pActuator;
}

void c826_analog::playSignal_PTBrush1()
{
	int errcode, i, j, n_src = 21, act_idx[3], n_act;
	int idx_arr2[][2] = {{1,4}, {0,4}, {3,4}, {4,6}, {4,7}};
	int idx_arr3[][3] = {{0,1,4}, {0,3,4}, {0,3,6}, {4,6,7}};
	double runtime, period_begin, sig_time_offset, sig_begin_time;
	double A = 1.0, duration = 0.140, SOA, sum_SOA = 0.f;
	double unit_period, d0, d1;
	double d_inv[3], sum_d_inv, dist_buf;
	double t0 = 0.7, t_k, dist[3], t_i, th_i;
	const double mult = 0.5*PI/t0;	// rad/sec
	glm::vec2 act_pos;
	// decision of n_src by the speed of the actuator movement
	actuator_info *pActuator = new actuator_info[n_src];
	for(i=0;i<21;i++) {
		if(i%5 == 0) {
			n_act = 2;
			j=i/5;
			act_pos = glm::vec2(-0.8*sin(0.25*PI*(double)j), 0.8*cos(0.25*PI*(double)j));
		//	for(j=0;j<2;j++) act_idx[j] = idx_arr2[(int)(i/5)][j];
			memcpy((void*)act_idx, (void*)idx_arr2[i/5], sizeof(int)*2);
		}
		else {
			n_act = 3;
			t_i = 0.1*t0*(double)i;	// signal start time
			th_i = mult*t_i;
			act_pos = glm::vec2(-0.8*sin(th_i), 0.8*cos(th_i));
			memcpy((void*)act_idx, (void*)idx_arr3[i/5], sizeof(int)*3);
			//for(j=0;j<3;j++) act_idx[j] = idx_arr
		}
		setActuatorParams(n_act, act_idx, act_pos, A, &pActuator[i]);
	}
	////
	sum_SOA = 0.f;
	for(i=0;i<21;i++) {
		pActuator[i].duration = 0.07;
		pActuator[i].SOA = 0.32*pActuator[i].duration + 0.0473;
		if(i<20) sum_SOA += pActuator[i].SOA;
	//	printf("%d pos(%.3f, %.3f) ", i, pActuator[i].pos.x, pActuator[i].pos.y);
	//	if(pActuator[i].num_actuator == 2) printf("amp([%d] %f [%d] %f)\n", pActuator[i].act_idx[0], pActuator[i].amplitude[0], pActuator[i].act_idx[1], pActuator[i].amplitude[1]);
	//	else printf("amp([%d] %f [%d] %f [%d] %f)\n", pActuator[i].act_idx[0], pActuator[i].amplitude[0], pActuator[i].act_idx[1], pActuator[i].amplitude[1], pActuator[i].act_idx[2], pActuator[i].amplitude[2]);
	}	
	sum_SOA += pActuator[20].duration + 0.2;
	run_TBrush_signal(n_src, sum_SOA, pActuator);
	delete pActuator;
}

void c826_analog::playSignal_PTBrush2()
{
	int errcode, i, j, n_src = 21, act_idx[3], n_act;
	int idx_arr2[][2] = {{0,3}, {1,3}, {3,4}, {4,6}, {3,6}};
	int idx_arr3[][3] = {{0,1,3}, {1,3,4}, {3,4,7}, {3,6,7}};

	double runtime, period_begin, sig_time_offset, sig_begin_time;
	double A = 1.0, duration = 0.140, SOA, sum_SOA = 0.f;
	double unit_period, d0, d1;
	double d_inv[3], sum_d_inv, dist_buf;
	double t0 = 0.7, t_k, dist[3], t_i, th_i;
	const double mult = 0.5*PI/t0;	// rad/sec
	glm::vec2 act_pos;
	// decision of n_src by the speed of the actuator movement
	actuator_info *pActuator = new actuator_info[n_src];
	for(i=0;i<21;i++) {
		if(i%5 == 0) {
			n_act = 2;
			j=i/5;
			act_pos = glm::vec2(0.8*sin(0.25*PI*(double)j)-1.0, 0.8*cos(0.25*PI*(double)j));
			//act_pos = glm::vec2(-0.8*sin(0.25*PI*(double)j), 0.8*cos(0.25*PI*(double)j));
			memcpy((void*)act_idx, (void*)idx_arr2[i/5], sizeof(int)*2);
		}
		else {
			n_act = 3;
			t_i = 0.1*t0*(double)i;	// signal start time
			th_i = mult*t_i;
			act_pos = glm::vec2(0.8*sin(th_i)-1.0, 0.8*cos(th_i));
			//act_pos = glm::vec2(-0.8*sin(th_i), 0.8*cos(th_i));
			memcpy((void*)act_idx, (void*)idx_arr3[i/5], sizeof(int)*3);
		}
		setActuatorParams(n_act, act_idx, act_pos, A, &pActuator[i]);
	}
	sum_SOA = 0.f;
	for(i=0;i<21;i++) {
		pActuator[i].duration = 0.07;
		pActuator[i].SOA = 0.32*pActuator[i].duration + 0.0473;
		if(i<20) sum_SOA += pActuator[i].SOA;
	//	printf("%d pos(%.3f, %.3f) ", i, pActuator[i].pos.x, pActuator[i].pos.y);
	//	if(pActuator[i].num_actuator == 2) printf("amp([%d] %f [%d] %f)\n", pActuator[i].act_idx[0], pActuator[i].amplitude[0], pActuator[i].act_idx[1], pActuator[i].amplitude[1]);
	//	else printf("amp([%d] %f [%d] %f [%d] %f)\n", pActuator[i].act_idx[0], pActuator[i].amplitude[0], pActuator[i].act_idx[1], pActuator[i].amplitude[1], pActuator[i].act_idx[2], pActuator[i].amplitude[2]);
	}	
	sum_SOA += pActuator[20].duration + 0.2;
	run_TBrush_signal(n_src, sum_SOA, pActuator);
	delete pActuator;
}

void c826_analog::playSignal_PTBrush3()
{
	int errcode, i, j, n_src = 31, act_idx[3], n_act;
	int idx_arr2[][2] = {{4,7}, {4,6}, {3,4}, {0,4}, {1,4}, {2, 4}, {4, 5}};
	int idx_arr3[][3] = {{4,6,7}, {3,4,6}, {0,3,4}, {0,1,4}, {1, 2, 4}, {2, 4, 5}};

	double runtime, period_begin, sig_time_offset, sig_begin_time;
	double A = 1.0, duration = 0.140, SOA, sum_SOA = 0.f;
	double unit_period, d0, d1;
	double d_inv[3], sum_d_inv, dist_buf;
	double t0 = 0.7, t_k, dist[3], t_i, th_i;
	const double mult = 0.5*PI/t0;	// rad/sec
	glm::vec2 act_pos;
	// decision of n_src by the speed of the actuator movement
	actuator_info *pActuator = new actuator_info[n_src];
	for(i=0;i<n_src;i++) {
		if(i%5 == 0) {
			n_act = 2;
			j=i/5;
			act_pos = glm::vec2(-0.8*sin(0.25*PI*(double)j), -0.8*cos(0.25*PI*(double)j));
			memcpy((void*)act_idx, (void*)idx_arr2[i/5], sizeof(int)*2);
		}
		else {
			n_act = 3;
			t_i = 0.1*t0*(double)i;	// signal start time
			th_i = mult*t_i;
			act_pos = glm::vec2(-0.8*sin(th_i), -0.8*cos(th_i));
			memcpy((void*)act_idx, (void*)idx_arr3[i/5], sizeof(int)*3);
		}
		setActuatorParams(n_act, act_idx, act_pos, A, &pActuator[i]);
	}
	sum_SOA = 0.f;
	for(i=0;i<n_src;i++) {
		pActuator[i].duration = 0.07;
		pActuator[i].SOA = 0.32*pActuator[i].duration + 0.0473;
		if(i<(n_src-1)) sum_SOA += pActuator[i].SOA;
	//	printf("%d pos(%.3f, %.3f) ", i, pActuator[i].pos.x, pActuator[i].pos.y);
	//	if(pActuator[i].num_actuator == 2) printf("amp([%d] %f [%d] %f)\n", pActuator[i].act_idx[0], pActuator[i].amplitude[0], pActuator[i].act_idx[1], pActuator[i].amplitude[1]);
	//	else printf("amp([%d] %f [%d] %f [%d] %f)\n", pActuator[i].act_idx[0], pActuator[i].amplitude[0], pActuator[i].act_idx[1], pActuator[i].amplitude[1], pActuator[i].act_idx[2], pActuator[i].amplitude[2]);
	}	
	sum_SOA += pActuator[n_src-1].duration + 0.2;
///	printf("sum_SOA=%f\n", sum_SOA);
	run_TBrush_signal(n_src, sum_SOA, pActuator);
	delete pActuator;
}

void c826_analog::playSignal_PTBrush4()
{
	int errcode, i, j, n_src = 31, act_idx[3], n_act;
//	int idx_arr2[][2] = {{4,7}, {4,6}, {3,4}, {0,4}, {1,4}, {2, 4}, {4, 5}};
//	int idx_arr3[][3] = {{4,6,7}, {3,4,6}, {0,3,4}, {0,1,4}, {1, 2, 4}, {2, 4, 5}};
	int idx_arr2[][2] = {{4,5}, {2, 4}, {1, 4}, {0,4}, {3,4}, {4,6}, {4,7}};
	int idx_arr3[][3] = {{2,4,5}, {1,2,4}, {0,1,4}, {0,3,4}, {3,4,6}, {4,6,7}};
	double runtime, period_begin, sig_time_offset, sig_begin_time;
	double A = 1.0, duration = 0.140, SOA, sum_SOA = 0.f;
	double unit_period, d0, d1;
	double d_inv[3], sum_d_inv, dist_buf;
	double t0 = 0.7, t_k, dist[3], t_i, th_i;
	const double mult = 0.5*PI/t0;	// rad/sec
	glm::vec2 act_pos;
	// decision of n_src by the speed of the actuator movement
	actuator_info *pActuator = new actuator_info[n_src];
	for(i=0;i<n_src;i++) {
		if(i%5 == 0) {
			n_act = 2;
			j=i/5;
			//act_pos = glm::vec2(-0.8*sin(0.25*PI*(double)i), -0.8*cos(0.25*PI*(double)i));
			act_pos = glm::vec2(0.8*cos(0.25*PI*(double)j), 0.8*sin(0.25*PI*(double)j));
			memcpy((void*)act_idx, (void*)idx_arr2[j], sizeof(int)*2);
		}
		else {
			n_act = 3;
			t_i = 0.1*t0*(double)i;	// signal start time
			th_i = mult*t_i;
		//	act_pos = glm::vec2(-0.8*sin(th_i), -0.8*cos(th_i));
			act_pos = glm::vec2(0.8*cos(th_i), 0.8*sin(th_i));
			memcpy((void*)act_idx, (void*)idx_arr3[i/5], sizeof(int)*3);
		}
		setActuatorParams(n_act, act_idx, act_pos, A, &pActuator[i]);
	}
	sum_SOA = 0.f;
	for(i=0;i<n_src;i++) {
		pActuator[i].duration = 0.07;
		pActuator[i].SOA = 0.32*pActuator[i].duration + 0.0473;
		if(i<(n_src-1)) sum_SOA += pActuator[i].SOA;
	//	printf("%d pos(%.3f, %.3f) ", i, pActuator[i].pos.x, pActuator[i].pos.y);
	//	if(pActuator[i].num_actuator == 2) printf("amp([%d] %f [%d] %f)\n", pActuator[i].act_idx[0], pActuator[i].amplitude[0], pActuator[i].act_idx[1], pActuator[i].amplitude[1]);
	//	else printf("amp([%d] %f [%d] %f [%d] %f)\n", pActuator[i].act_idx[0], pActuator[i].amplitude[0], pActuator[i].act_idx[1], pActuator[i].amplitude[1], pActuator[i].act_idx[2], pActuator[i].amplitude[2]);
	}	
	sum_SOA += pActuator[n_src-1].duration + 0.2;
//	printf("sum_SOA=%f\n", sum_SOA);
	run_TBrush_signal(n_src, sum_SOA, pActuator);
	delete pActuator;
}

void c826_analog::setEnvelopParams(int channel, double amplitude, double frequency,  double offset)
{
	m_enabled_channel = channel; 
	m_sineFreq = frequency;
	m_sineAmplitude = amplitude;
	m_sineOffset = offset;
//	if(m_signalEnabled) m_signalChange = true;
}
void c826_analog::setSignalVals_2ch(double amplitude[], double frequency[], double offset[])
{
	for(int i=0;i<2;i++) {
		m_sineFreqs[i]= frequency[i];
		m_sineAmps[i] = amplitude[i];
		m_sineOffsets[i]=offset[i];
	}
}

void c826_analog::setEnvelopParams(double amplitude[], double frequency[], double offset[], int n_ch, int channel[])
{
	m_n_ch = n_ch;
	for(int i=0;i<m_n_ch;i++) {
		m_sineFreqs[i] = frequency[i];
		m_sineAmps[i] = amplitude[i];
		m_sineOffsets[i] = offset[i];
		m_aout[i] = channel[i];
	}
}

unsigned __stdcall IOcardThread(void* arg)
{
//	int channel[2] = {0, 1}, channel3[] = {0, 1, 2}, channel4[]={0,1,2,4};
	c826_analog *p826 = (c826_analog*) arg;
	do {
		//////
		if(p826->m_signalEnabled) {
		//	p826->playSignal_multi_channel(channel3, 3);
			switch(p826->m_signalType) {
			case 0:
				p826->playSignal_test();
				break;
			case 1:

				break;
			//case 1:
			//	p826->playSignal_TBrush1();
			//	break;
			//case 2:
			//	p826->playSignal_TBrush2();
			//	break;
			//case 3:
			//	p826->playSignal_TBrush3();
			//	break;
			//case 4:
			//	p826->playSignal_TBrush4();
			//	break;
			//case 5:
			//	p826->playSignal_PTBrush1();
			//	break;
			//case 6:
			//	p826->playSignal_PTBrush2();
			//	break;
			//case 7:
			//	p826->playSignal_PTBrush3();
			//	break;
			//case 8:
			//	p826->playSignal_PTBrush4();
			//	break;

			}
		}
		p826->m_signalEnabled = false;
		WaitForSingleObject(p826->m_hEvent, INFINITE);
	//	printf("breaks\n");
	} while(p826->m_IO_enabled);
	return 0;
}





















