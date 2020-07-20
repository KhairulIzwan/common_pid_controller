# import necessary packages

# https://www.arduino.cc/reference/en/language/functions/math/map/
# Re-maps a number from one range to another. That is, a value of fromLow would 
# get mapped to toLow, a value of fromHigh to toHigh, values in-between to values 
# in-between, etc.

def map(x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
