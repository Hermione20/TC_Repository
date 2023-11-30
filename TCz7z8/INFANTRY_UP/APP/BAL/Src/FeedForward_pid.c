#include "FeedForward_pid.h"



void Feedforward_pid_init(Feedforward_pid_t *FF,float k1,float k2,float max_out)
{
	FF->k1=k1;
	FF->k2=k2;
	FF->max_out=max_out;
}


float Feedforward_pid_calc(Feedforward_pid_t *FF)
{
		FF->out=FF->k1*FF->set+FF->k2*(FF->set-FF->last_set);
		FF->last_set=FF->set;
		return((FF->out <= -FF->max_out)?-FF->max_out:((FF->out >= FF->max_out)?FF->max_out:FF->out));
}

