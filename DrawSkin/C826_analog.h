#ifndef _C826_ANALOG_H_
#define _C826_ANALOG_H_
#include <windows.h>
#include <826api.h>
#include <glm\glm.hpp>

typedef struct actuator_info {
	bool enabled;
	int num_actuator;
	int act_idx[3];
	double t_signal_start;
	double t_signal_end;
	double amplitude[3];
	double duration;
	double SOA;
//	glm::vec2 pos;
	actuator_info() {
		enabled = false;
		num_actuator = 1;
		t_signal_start = 0.f;
		t_signal_end = 0.f;
		amplitude[0] = amplitude[1] = amplitude[2] = 1.0;
		duration = 0.f;
		SOA = 0.f;
	}
};
class c826_analog {
public:
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
	c826_analog();
	c826_analog(uint board);
	~c826_analog();
	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------
	int init(char* ret_string =NULL);
	void getErrorString(int errcode, char* ret_string);
	void triggerSinewave_4ch(const bool channel_enable[4], double amplitude, double freq, double offset, double duration, HANDLE trigger_event, bool test=false);
	void triggerSignal(bool enable);
	void triggerSignal_ch(bool enable, int channel=0); 

	void triggerTestSignal(int signal_type, HANDLE trigger_event, bool *flag = NULL);

	void setEnvelopParams(int channel, double amplitude, double frequency, double offset=0.f);
	void setSignalVals_2ch(double amplitude[2], double frequency[2], double offset[2]);
	void setEnvelopParams(double amplitude[], double frequency[], double offset[], int n_ch, int channel[]);
	inline void setUnitPlayCycle(int unitPlayCycle) {m_nPlayCycle = unitPlayCycle;}
	void playSignal_test();

	void playSignal_TBrush1();
	void playSignal_PTBrush1();
	///
	void playSignal_TBrush2();
	void playSignal_PTBrush2();
	///
	void playSignal_TBrush3();
	void playSignal_PTBrush3();
	///
	void playSignal_TBrush4();
	void playSignal_PTBrush4();
	void playBruteForce();
	inline void setValue4(float val1, float val2, float val3, float val4) {	m_act_val[0] = val1; m_act_val[1] = val2; m_act_val[2] = val3; m_act_val[3] = val4;}
//////
	void setActuatorParams(int n_actuator, int *act_idx, glm::vec2 act_pos, double A, actuator_info *pActuator);
	void run_TBrush_signal(int n_src, double unit_period, actuator_info *pActuator);
	int periodicTimerStart(uint counter, uint period);
	int periodicTimerStop(uint counter);
	int periodicTimerWait(uint counter, uint *timestamp);
public:
	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------
	int m_outSingalType;
	int m_aout[8];
	int m_n_ch;
	bool m_channelEnable[8];
	bool m_cardInit;
	bool m_IO_enabled;
	bool m_signalEnabled;
	double m_signalDuration;
	int m_nPlayCycle;
/// sine wave variables
	double m_sineFreq;
	double m_sineAmplitude;
	double m_sineOffset;
	int m_signalType;

	double m_sineFreqs[8];
	double m_sineAmps[8];
	double m_sineOffsets[8];
////
	uint m_board;
	HANDLE m_h_IOcardThread;
	HANDLE m_hEvent;
	HANDLE m_hCustomEvent;
	int m_enabled_channel;
	bool *m_test_flag;
private:
	bool m_sineTest_4ch;
	bool m_signalChange;
	int m_n_src;
	double m_unit_period;
	double m_act_val[4];
	actuator_info *m_actuator;
};

#endif //_C826_ANALOG_H_
