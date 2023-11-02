# Message containing the current state of the module

Header header

string ip

bool connection
uint8 attitude_state
bool attitude_fixed
uint8 rtk_state
bool rtk_fixed

float64 accuracy

float64 gnssReception
uint32   numSats

float64 latency
float64 timingElapsedGNSS
float64 timingElapsedIMU
float64 timingElapsedODO
float64 timingElapsedBARO