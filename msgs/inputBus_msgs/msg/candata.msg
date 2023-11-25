# This represents a simple Message for CAN-Signals
# signal_name:  Name of the signal according to dbc-file
# value:        Value of the message (already converted to corresponding unit as given in dbc)
# unit:         The unit of the signal value (optional)

Header header

string signal_name
float64 value
time timestamp
string unit
