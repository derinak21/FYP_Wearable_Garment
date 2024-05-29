/*
 FILENAME:      KickMath.h
 AUTHOR:        Orlando S. Hoilett, Alyson S. Pickering, and Akio K. Fujita
 EMAIL:     	orlandohoilett@gmail.com
 VERSION:		4.0.0
 
 
 DESCRIPTION
 A library for performing a few simple mathematical calculations and for use
 with arrays. Functions include max and min detection, square root, centroid,
 derivatives, etc.
 
 This is a static class. Function calls must be preceded with the class name
 and scope resolution operator as follows KickMath<variable_type>:: where
 variable_type can be int16_t, uint32_t, float, etc.
 
 
 UPDATES
 2019/12/03:2157>
 			Initiated.
 2020/03/06:2300>
 			Added t-test function for matached paired, two-tailed test at
 			alpha = 0.05.
 VERSION 2.0.0
 2020/07/04:1018> (UTC-5)
 			- Added a lower processing power square root function.
 			- Added getMaxIndex and a getMinIndex functions.
 			- Moved Akio's peak detection functions to the KickPeaks class.
 2020/07/12:0658> (UTC-5)
 			- Updated comments.
 Version 3.0.0
 2020/08/18:1143> (UTC-5)
 			- moved to a templated class
 Version 3.1.0
 2020/08/22:1650>
 			- added a calculate median function.
 Version 4.0.0
 2020/08/23:0423> (UTC-5)
 			- changed magnitude types to uint32_t to match isqrt function except
 			in centroid function since the magnitude parameter is the only
 			parameter that needs to be templatized.
 Version 4.1.0
 2020/08/24:1733> (UTC-5)
 			- added a find function to find a specific value in an array as well
			as a getMax and getMin function that searches within certain bounds.
 2020/08/29:1819> (UTC-5)
			- added a cross-correlation function
 
 *******************************************************************************
 *******************************************************************************
 CHANGED VERSIONING back to 4.0.0 since all edits after v3.0.0 (2020/08/18:1143> (UTC-5))
 have been pre-release, having been added to a side branch and not the main branch
 *******************************************************************************
 *******************************************************************************
 
 Version 4.0.0
 2020/08/22:1650> (UTC-5)
 			- added a calculate median function.
 2020/08/23:0423> (UTC-5)
 			- changed magnitude types to uint32_t to match isqrt function except
 			in centroid function since the magnitude parameter is the only
 			parameter that needs to be templatized.
 2020/08/24:1733> (UTC-5)
 			- added a find function to find a specific value in an array as well
 			as a getMax and getMin function that searches within certain bounds.
 2020/08/29:1819> (UTC-5)
 			- added a cross-correlation function, xcorr.
 			- added getAbsMax and getAbsMin functions that return the largest
 			and smallest value in an array regardless of sign
 2020/08/30:0758> (UTC-5)
 			- Chnaged xcorr function name to corr, making it equivalent to
 			MATLAB's corr function with input arrays with one column each. No
			p-value calculation as of yet.
 2020/09/11:0825> (UTC-5)
 			- changed SE to standard_error in ttest function for compatibility.
 			SE might be a numeric constant for Arduino Uno, so the code wouldn't
 			compile for an Uno.
 2020/09/28:2240> (UTC-5)
 			- Updated comments.

 
 DISCLAIMER
 Linnes Lab code, firmware, and software is released under the
 MIT License (http://opensource.org/licenses/MIT).
 
 The MIT License (MIT)
 
 Copyright (c) 2019 Linnes Lab, Purdue University
 
 Permission is hereby granted, free of charge, to any person obtaining a copy of
 this software and associated documentation files (the "Software"), to deal in
 the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 of the Software, and to permit persons to whom the Software is furnished to do
 so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 
 */


#ifndef KickMath_h
#define KickMath_h


//Standard Arduino libraries
#include <Arduino.h>

//Custom Kick Libraries
#include "KickSort.h"
#include "ttestTable.h"


template<typename Type>


class KickMath
{

public:
	
	//static int32_t calcSqrt(int32_t num);
	static uint32_t calcSqrt(uint32_t num);
	static uint32_t calcMagnitude(Type x, Type y);
	static uint32_t calcMagnitude(Type x, Type y, Type z);
	
	
	static uint16_t find(const uint16_t samples, const Type data[], const Type num);
	static uint16_t find(const uint16_t samples, const Type data[], const Type num, const uint16_t i1, const uint16_t i2);
	
	
	static Type getAbsMax(uint16_t samples, const Type data[]);
	static Type getMax(uint16_t samples, const Type data[]);
	static Type getMax(uint16_t samples, const Type data[], const uint16_t i1, const uint16_t i2);
	static Type getAbsMin(uint16_t samples, const Type data[]);
	static Type getMin(uint16_t samples, const Type data[]);
	static Type getMin(uint16_t samples, const Type data[], const uint16_t i1, const uint16_t i2);
	static float getSum(uint16_t samples, const Type data[]);
	
	
	static uint16_t getMaxIndex(uint16_t samples, const Type data[]);
	static void getMaxIndex(uint16_t samples, const Type data[], uint16_t &max1, uint16_t &max2);
	static void getMaxIndex(uint16_t samples, const Type data[], uint16_t maxes[], const uint16_t num_maxes);
	
	static uint16_t getMinIndex(uint16_t samples, const Type data[]);

	
	static Type calcAverage(uint16_t samples, const Type data[]);
	static Type calcMedian(uint16_t samples, Type data[]);
	static Type calcMedian(uint16_t samples, const Type data[], Type tmpArray[]);
	
	static float calcStDev(uint16_t samples, const Type data[]);
	
	static Type calcPeaktoPeak(uint16_t samples, const Type data[]);
	static Type calcPeaktoPeak(uint16_t samples, const Type data[], const uint16_t i1, const uint16_t i2);
	static Type calcRMS(uint16_t samples, const Type data[]);
	
	
	static void calcDerivative(uint16_t samples, const Type data[], Type ddx[]);
	static void calcDerivative(uint16_t samples, uint16_t dt, const Type data[], float ddx[]);
	static void calcDerivative(uint16_t samples, const Type data[], float ddx[], const uint32_t t[]);

	
	static float calcCentroid(float fs, uint16_t samples, const Type mag[], uint16_t fcenter, uint16_t width);
	
	
	static bool tTest(const Type data1[], const Type data2[], const uint16_t samples, const float alpha);
	
	
	static float corr(const Type signalX[], const Type signalY[], uint16_t n);
	//static bool iscorr(const Type signalX[], const Type signalY[], uint16_t n, float r);
	//static float corr(const Type signalX[], const Type signalY[], uint16_t n, Type lags[]);
	
};



//int32_t KickMath::isqrt(int32_t num)
//num to calculate square root of
//
//A PDF of the sources are also included in the "extras/references/" folder
//
//Source: https://en.wikipedia.org/wiki/Methods_of_computing_square_roots
//https://web.archive.org/web/20120306040058/http://medialab.freaknet.org/martin/src/sqrt/sqrt.c
//template<typename Type>
template<typename Type>
uint32_t KickMath<Type>::calcSqrt(uint32_t val)
{
	//if(val < 0) return 0;
	
	uint32_t res = 0;
	uint32_t bit = 1 << 30; // The second-to-top bit is set.
	// Same as ((unsigned) INT32_MAX + 1) / 2.
	
	// "bit" starts at the highest power of four <= the argument.
	while (bit > val)
		bit >>= 2;
	
	while (bit != 0) {
		if (val >= res + bit) {
			val -= res + bit;
			res = (res >> 1) + bit;
		} else
			res >>= 1;
		bit >>= 2;
	}
	return res;
}


//uint16_t KickMath::calcMagnitude(int16_t x, int16_t y)
//x				x component of the 2D vector
//y				y component of the 2D vector
//
//Calculates the magnitude of a 2D vector
template<typename Type>
uint32_t KickMath<Type>::calcMagnitude(Type x, Type y)
{
	return calcSqrt(x*x + y*y);
}


//uint16_t KickMath::calcMagnitude(int16_t x, int16_t y, int16_t z)
//x				x component of the 3D vector
//y				y component of the 3D vector
//z				z component of the 3D vector
//
//Calculates the magnitude of a 3D vector
template<typename Type>
uint32_t KickMath<Type>::calcMagnitude(Type x, Type y, Type z)
{
	return calcSqrt(x*x + y*y + z*z);
}


//uint16_t KickMath<Type>::find(const uint16_t samples, const Type data[], const Type num)
//samples		number of samples within the array
//data			input array containing signal
//num			number we're searching for
//
//Finds a specific number within the given input array and returns the index of
//that number. If the number is not found, return 0.
template<typename Type>
uint16_t KickMath<Type>::find(const uint16_t samples, const Type data[], const Type num)
{
	for(uint16_t i = 0; i < samples; i++)
	{
		if(data[i] == num) return i;
	}
	
	return 0;
}


//uint16_t KickMath<Type>::find(const uint16_t samples, const Type data[], const Type num)
//samples		number of samples within the array
//data			input array containing signal
//num			number we're searching for
//i1			lower bound of the search index
//i2			upper bound of the search index
//
//Finds a specific number within the given input array within the i1 and i2
//indeces. Then returns the index of that number. If the number is not found,
//return 0.
template<typename Type>
uint16_t KickMath<Type>::find(const uint16_t samples, const Type data[], const Type num, const uint16_t i1, const uint16_t i2)
{
	for(uint16_t i = i1; i < i2; i++)
	{
		if(data[i] == num) return i;
	}
	
	return 0;
}


//Type KickMath<Type>::getAbsMax(uint16_t samples, const Type data[])
//samples		number of samples within the array
//data			input array containing signal
//
//Returns the value in the array with the largest magnitude (regardless of
//sign). The function maintains the sign of the value when it is returned to the
//function caller. For example, -200 has a larger magnitude than 28. The function
//will return -200, not 200.
template<typename Type>
Type KickMath<Type>::getAbsMax(uint16_t samples, const Type data[])
{
	//Fence post solution: assume the first value
	//is the max then compare and update from there
	Type absMax = data[0];
	
	for(uint16_t i = 1; i < samples; i++)
	{
		if (abs(data[i]) > abs(absMax)) absMax = data[i];
	}
	
	return absMax;
}


//int16_t KickMath::getMax(uint16_t samples, const int16_t data[])
//samples		number of samples within the array
//data			input array containing signal
//
//Finds the max value within an input array.
template<typename Type>
Type KickMath<Type>::getMax(uint16_t samples, const Type data[])
{
	//Fence post solution: assume the first value
	//is the max then compare and update from there
	Type max = data[0];
	
	for(uint16_t i = 1; i < samples; i++)
	{
		if (data[i] > max) max = data[i];
	}
	
	return max;
}


//Type KickMath<Type>::getMax(uint16_t samples, const Type data[], const uint16_t i1, const uint16_t i2)
//samples		number of samples within the array
//data			input array containing signal
//i1			lower bound of the search index
//i2			upper bound of the search index
//
//Finds the index of the max value in an input array within the bounds, i1 and i2.
template<typename Type>
Type KickMath<Type>::getMax(uint16_t samples, const Type data[], const uint16_t i1, const uint16_t i2)
{
	//Fence post solution: assume the first value
	//is the max then compare and update from there
	Type max = data[i1];
	
	for(uint16_t i = i1+1; i < i2; i++)
	{
		if (data[i] > max) max = data[i];
	}
	
	return max;
}


//uint16_t KickMath::getMaxIndex(uint16_t samples, const int32_t data[])
//samples		number of samples within the array
//data			input array containing signal
//
//Finds the index of the max value within an input array.
template<typename Type>
uint16_t KickMath<Type>::getMaxIndex(uint16_t samples, const Type data[])
{
	//Fence post solution: assume the first value
	//is the max then compare and update from there
	uint16_t imax = 0;
	
	for(uint16_t i = 1; i < samples; i++)
	{
		if (data[i] > data[imax]) imax = i;
	}
	
	return imax;
}


//void KickMath::getMaxIndex(uint16_t samples, const int32_t data[], uint16_t &max1, uint16_t &max2)
//samples		number of samples within the array
//data			input array containing signal
//max1			variable to store the index of the first highest value
//max2			variable to store the index of the second highest value
//
//Finds the index of the two highest values within an input array and stores
//them in the max1 and max2 variables.
//
//Could be used as a general function and return an array of up to 10 or so maxes (do hardcode a limit)
template<typename Type>
void KickMath<Type>::getMaxIndex(uint16_t samples, const Type data[], uint16_t &max1, uint16_t &max2)
{
	//Fence post solution: assume the first value
	//is the max then compare and update from there
	max1 = 0;
	max2 = 0;
	
	for(uint16_t i = 1; i < samples; i++)
	{
		if (data[i] > data[max1])
		{
			max2 = max1;
			max1 = i;
		}
	}
}


//void KickMath::getMaxIndex(uint16_t samples, const int32_t data[], uint16_t maxes[], const uint16_t num_maxes)
//samples		number of samples within the array
//data			input array containing signal
//maxes			array to store max values. C++ allows us to modify arrays used
//					as function parameters
//num_maxes		desired number of the top values requested by function caller
//
//Finds the indices of the max values in a function and returns the indices in an
//array. Indices are ordered from highest max at the 0th index to lowest max in
//the last index.
//
//C++ allows us to modify arrays used as function parameters so the maxes
//are stored in the "maxes" array.
template<typename Type>
void KickMath<Type>::getMaxIndex(uint16_t samples, const Type data[], uint16_t maxes[], const uint16_t num_maxes)
{
	//Fence post solution: assume the first value
	//is the max then compare and update from there
	for(uint16_t i = 0; i < num_maxes; i++)
	{
		maxes[i] = 0;
	}
	
	
	for(uint16_t i = 1; i < samples; i++)
	{
		if (data[i] > data[maxes[0]])
		{
			//shifting the maxes in the array if a new max is found
			for(uint16_t j = 1; j < num_maxes; j++)
			{
				maxes[num_maxes-j] = maxes[num_maxes-j-1];
			}
			
			//update index 0 which contains the index of the highest max.
			maxes[0] = i;
		}
	}
}


//Type KickMath<Type>::getAbsMin(uint16_t samples, const Type data[])
//samples		number of samples within the array
//data			input array containing signal
//
//Returns the value in the array with the smallest magnitude (regardless of
//sign). The function maintains the sign of the value when it is returned to the
//function caller. 0 is the number with the smallest magnitude that this function
//will return. -10, for example, has a larger magnitude than 0.
//
//For example, 15 has a smaller magnitude than -28. The function will return 15.
//For example, -29 has a smaller magnitude than 75. The function will return
//-29, not 29.
template<typename Type>
Type KickMath<Type>::getAbsMin(uint16_t samples, const Type data[])
{
	//Fence post solution: assume the first value
	//is the min then compare and update from there
	Type absMin = data[0];
	
	for(uint16_t i = 1; i < samples; i++)
	{
		if (abs(data[i]) < abs(absMin)) absMin = data[i];
	}
	
	return absMin;
}


//int16_t KickMath::getMin(uint16_t samples, const int16_t data[])
//samples		number of samples within the array
//data			input array containing signal
//
//Finds the minimum value within an input array.
template<typename Type>
Type KickMath<Type>::getMin(uint16_t samples, const Type data[])
{
	//Fence post solution: assume the first value
	//is the min then compare and update from there
	Type min = data[0];
	
	for(uint16_t i = 1; i < samples; i++)
	{
		if (data[i] < min) min = data[i];
	}
	
	return min;
}


//Type KickMath<Type>::getMin(uint16_t samples, const Type data[], const uint16_t i1, const uint16_t i2)
//samples		number of samples within the array
//data			input array containing signal
//i1			lower bound of the search index
//i2			upper bound of the search index
//
//Finds the index of the min value in an input array within the bounds, i1 and i2.
template<typename Type>
Type KickMath<Type>::getMin(uint16_t samples, const Type data[], const uint16_t i1, const uint16_t i2)
{
	//Fence post solution: assume the first value
	//is the min then compare and update from there
	Type min = data[i1];
	
	for(uint16_t i = i1+1; i < i2; i++)
	{
		if (data[i] < min) min = data[i];
	}
	
	return min;
}


//uint16_t KickMath::getMinIndex(uint16_t samples, const int32_t data[])
//samples		number of samples within the array
//data			input array containing signal
//
//Finds the index of the min value within an input array.
template<typename Type>
uint16_t KickMath<Type>::getMinIndex(uint16_t samples, const Type data[])
{
	//Fence post solution: assume the first value
	//is the min then compare and update from there
	uint16_t imin = 0;
	
	for(uint16_t i = 1; i < samples; i++)
	{
		if (data[i] < data[imin]) imin = i;
	}
	
	return imin;
}


//int32_t KickMath::getSum(uint16_t samples, const int16_t data[])
//samples		number of samples within the array
//data			input array containing signal
//
//Returns the sum of values in a given input array.
template<typename Type>
float KickMath<Type>::getSum(uint16_t samples, const Type data[])
{
	float sum = 0;
	
	for(uint16_t i = 0; i < samples; i++) sum += data[i];
	
	return sum;
}


//int16_t KickMath::calcAverage(uint16_t samples, const int16_t data[])
//samples		number of samples within the array
//data			input array containing signal
//
//Calculates the average of the values in the given input array.
template<typename Type>
Type KickMath<Type>::calcAverage(uint16_t samples, const Type data[])
{
	return getSum(samples,data)/(float)samples;
}


//Type KickMath<Type>::calcMedian(uint16_t samples, const Type data[], Type tmpArray[])
//samples		number of samples within the array
//data			input array containing signal (read-only)
//tmpArray		to prevent the sorting algorithm from attempting to modify the
//					read-only "data" array, give the sorting algorithm a dummy
//					array to work with instead.
//
//return		the median of the input array
//
//Calculates the median of the given input data
template<typename Type>
Type KickMath<Type>::calcMedian(uint16_t samples, const Type data[], Type tmpArray[])
{
	//copy array
	for(uint16_t i = 0; i < samples; i++)
	{
		tmpArray[i] = data[i];
	}
	
	//sort array
	KickSort<Type>::quickSort(tmpArray, samples);
	
	
	//calculate median
	uint16_t middleIndex = samples/2; //integer division so it truncates decimals in the event that samples is odd
	if(samples%2 == 0) return (tmpArray[middleIndex] + tmpArray[(middleIndex)-1])/2.0; //if only two values, find the average
	else return tmpArray[middleIndex];
}


//Type KickMath<Type>::calcMedian(uint16_t samples, Type data[])
//samples		number of samples within the array
//data			input array containing signal (read and write)
//
//return		the median of the input array
//
//Calculates the median of the given input data. The "data" arra is read and write.
template<typename Type>
Type KickMath<Type>::calcMedian(uint16_t samples, Type data[])
{
	//sort array
	KickSort<Type>::quickSort(data, samples);
	
	
	//calculate median
	uint16_t middleIndex = samples/2; //integer division so it truncates decimals in the event that samples is odd
	if(samples%2 == 0) return (data[middleIndex] + data[(middleIndex)-1])/2.0;
	else return data[middleIndex];
}


//float KickMath::calcStDev(uint16_t samples, const int32_t data[])
//samples		number of samples within the array
//data			input array containing signal
//
//return		standard deviation of signal stored in data array
//
//Calculates the standard deviation of the values in the given input array.
template<typename Type>
float KickMath<Type>::calcStDev(uint16_t samples, const Type data[])
{
	//float avg = calcAverage(samples, data);
	float avg = 0;
	for(uint16_t i = 0; i < samples; i++)
	{
		avg += data[i];
	}
	avg = avg/samples;
	
	
	float num = 0;
	for(uint16_t i = 0; i < samples; i++)
	{
		num += pow(data[i]-avg, 2); //or just num*num? which is faster?
	}
	
	return sqrt(num/(samples-1)); 
}


//int16_t KickMath::calcPeaktoPeak(uint16_t samples, const int16_t data[])
//samples		number of samples within the array
//data			input array containing signal
//
//return		peak-to-peak value of signal stored in data array
//
//Calculates the peak-to-peak amplitude of the values in the given input array.
//It would be faster to avoid calling the getMax and getMin functions separately
//to avoid scanning through the array twice. Will update in a future version.
template<typename Type>
Type KickMath<Type>::calcPeaktoPeak(uint16_t samples, const Type data[])
{
	return getMax(samples, data) - getMin(samples, data);
}


//Type KickMath<Type>::calcPeaktoPeak(uint16_t samples, const Type data[], const uint16_t i1, const uint16_t i2)
//samples		number of samples within the array
//data			input array containing signal
//i1			lower bound of the array to search through
//i2			upper bound of the array to search through
//
//Calculates and returns peak-to-peak amplitude of the values in the given input
//array, that is the maximum value minus the minimum value of the array.
template<typename Type>
Type KickMath<Type>::calcPeaktoPeak(uint16_t samples, const Type data[], const uint16_t i1, const uint16_t i2)
{
	return KickMath<Type>::getMax(samples, data, i1, i2) - KickMath<Type>::getMin(samples, data, i1, i2);
}


//float KickMath::calcRMS(uint16_t samples, const int16_t data[])
//samples		number of samples within the array
//data			input array containing signal
//
//return		root-mean-squared of signal stored in data array
//
//Calculates the root mean square of the signal across a given number of
//samples in accordance to the equation as defined here:
//<https://en.wikipedia.org/wiki/Root_mean_square>
//This method first calculates the average and removes the mean from each data
//point, then calculates RMS.
template<typename Type>
Type KickMath<Type>::calcRMS(uint16_t samples, const Type data[])
{
	Type avg = calcAverage(samples, data);
	
	
	float sum_of_squares = 0;
	for(uint16_t i = 0; i < samples; i++)
	{
		sum_of_squares += pow(data[i]-avg, 2);
	}
	
	//maybe switch to faster square root method
	return sqrt(sum_of_squares/samples);
}


//void KickMath::calcDerivative(uint16_t samples, const int16_t data[], int16_t ddx[])
//samples		number of samples within the array
//data			input array containing signal
//ddx			the array for storing the derivatives. C++ allows us to modify
//					arrays used as function parameters
//
//Calculates the derivative of the given input signal assuming the change in time
//is 1. This simply becomes the difference between subsequent values in the
//given input array.
//
//C++ allows us to modify arrays used as function parameters so the derivatives
//are stored in the "ddx" array.
template<typename Type>
void KickMath<Type>::calcDerivative(uint16_t samples, const Type data[], Type ddx[])
{
	//The first value in the ddx array is zero since the derivative needs two
	//values before a valid derivaitve is can be determined. As a result, the
	//derivative array has samples-1 valid answers. The first value is not useful.
	ddx[0] = 0;
	for (uint16_t i = 1; i < samples; i++)
	{
		ddx[i] = (data[i]-data[i-1]);
	}
}


//void KickMath::calcDerivative(uint16_t samples, uint16_t dt, const int16_t data[], float ddx[])
//samples		number of samples within the array
//dt			the difference in time between samples. Assumes constant
//					sampling rate for the entire array.
//data			input array containing signal
//ddx			the array for storing the derivatives. C++ allows us to modify
//					arrays used as function parameters
//
//Calculates the derivative of the given input signal assuming the change in time
//is dt.
//
//C++ allows us to modify arrays used as function parameters so the derivatives
//are stored in the "ddx" array.
template<typename Type>
void KickMath<Type>::calcDerivative(uint16_t samples, uint16_t dt, const Type data[], float ddx[])
{
	//The first value in the ddx array is zero since the derivative needs two
	//values before a valid derivaitve is can be determined. As a result, the
	//derivative array has samples-1 valid answers. The first value is not useful.
	ddx[0] = 0;
	for (uint16_t i = 1; i < samples; i++)
	{
		ddx[i] = (data[i]-data[i-1]) / (float)dt;
	}
}


//void KickMath::calcDerivative(uint16_t samples, const int16_t data[], float ddx[], const uint32_t t[])
//samples		number of samples within the array
//data			input array containing signal
//ddx			the array for storing the derivatives. C++ allows us to modify
//					arrays used as function parameters
//t				time array. holds the time each sample in data was taken. t[1]
//					corresponds to the time at which data[1] was samples. t[10]
//					corresponds to data[10] and so on.
//
//Calculates the derivative of the given input signal assuming the change in time
//is dt.
//
//C++ allows us to modify arrays used as function parameters so the derivatives
//are stored in the "ddx" array.
template<typename Type>
void KickMath<Type>::calcDerivative(uint16_t samples, const Type data[], float ddx[], const uint32_t t[])
{
	//The first value in the ddx array is zero since the derivative needs two
	//values before a valid derivaitve is can be determined. As a result, the
	//derivative array has samples-1 valid answers. The first value is not useful.
	ddx[0] = 0;
	for (uint16_t i = 1; i < samples; i++)
	{
		ddx[i] = (data[i]-data[i-1]) / (float)(t[i]-t[i-1]);
	}
}


//float KickMath::calcCentroid(float fs, uint16_t samples, const int32_t mag[], uint16_t fcenter, uint16_t width)
//fs			sampling frequency for data array
//samples		number of samples within the array
//mag			frequency weights
//fcenter		the index of the mag array to calculate the centroid around.
//width			the number of indices around the index for determinig centroid
//
//This metods calculates the centroid of a certain section of the frequency spectrum.
//
//The centroid is calculated by perfoming a weigthed average.
//https://en.wikipedia.org/wiki/Spectral_centroid
template<typename Type>
float KickMath<Type>::calcCentroid(float fs, uint16_t samples, const Type mag[], uint16_t fcenter, uint16_t width)
{
	float denominator = 0;
	float numerator = 0;
	float fs_scale = fs/samples; //frequency increments
	
	
	//if width exceeds the bounds of the array, limit width
	if(fcenter+width > samples-1) width = samples-fcenter;
	if(fcenter-width < 0) width = fcenter-0;
	
	
	for(uint16_t i = fcenter-width; i <= fcenter+width; i++)
	{
		denominator += mag[i];
		numerator += mag[i]*i*fs_scale;
	}
	
	
	return numerator/denominator;
}


//bool KickMath::tTest(const int16_t data1[], const int16_t data2[], const uint8_t samples, const float alpha)
//data1			pre-data
//data2			post-data
//samples		number of observations
//alpha			alpha (significance) level...currently only for alpha = 0.05
//
//returns true if reject null hypothesis
//returns false if fail to reject null hypothesis
//
//Runs a matched paired t-test and returns true if the the null hypothesis is rejected
//Reference saved here: /extras/references/paired-t-test.pdf
template<typename Type>
bool KickMath<Type>::tTest(const Type data1[], const Type data2[], const uint16_t samples, const float alpha)
{
	//if (samples > 30) return -1;
	float diff[30] = {};
	const uint16_t df = samples-1; //degress of freedom
	
	
	float sum = 0;
	for (uint16_t i = 0; i < samples; i++)
	{
		diff[i]= data2[i] - data1[i];
		sum += diff[i];
	}
	
	
	float meanDiff = sum/samples;
	float SdNum = 0;
	for (int i = 0; i < samples; i++)
	{
		SdNum += sq(diff[i] - meanDiff);
	}
	float std = sqrt(SdNum/df); //should this be samples or degress of freedom??
	float standard_error = std/sqrt(samples); //standard error
	
	
	//t-statistic = meanDiff/SEfloat
	//a95 is look up table for up to 35 degrees of freedom at
	//alpha = 0.05 for two-tailed test
	return abs(meanDiff/standard_error) > a95[df];
}


//float KickMath<Type>::corr(const Type signalX[], const Type signalY[], uint16_t n)
//signalX			input data 1
//signalY			input data 2
//n					number of samples in each array
//
//Calculates the Pearson's correlation coefficient between two signals which is
//a measure of the relatedness between the signals. This is equivalent to
//MATLAB's corr function.
//Source: https://www.statisticshowto.com/cross-correlation/
//https://www.statisticshowto.com/probability-and-statistics/
template<typename Type>
float KickMath<Type>::corr(const Type signalX[], const Type signalY[], uint16_t n)
{
	//this step adds roughly 70-90 us to the function with n = 128 or 366
	Type abs_max_1 = abs(KickMath<Type>::getAbsMax(n, signalX));
	Type abs_max_2 = abs(KickMath<Type>::getAbsMax(n, signalY));
	

	int64_t sumX = 0; //declare variable for sum of X
	int64_t sumY = 0; //declare variable for sum of Y
	uint64_t sumXsqr = 0; //declare variable for sum of X^2
	uint64_t sumYsqr = 0; //declare variable for sum of Y^2
	int64_t sumXY = 0; //declare variable for sum of X*Y
	
	
	if(abs_max_1 > 300 || abs_max_2 > 300)
	{
		//divide numbers by 10 to prevent data storage overflow
		for(uint16_t i = 0; i < n; i++)
		{
			sumX += signalX[i]/10; //add current X data point to sum
			sumY += signalY[i]/10; //add current Y data point to sum
			sumXsqr += sq(signalX[i]/10); //add current X^2 to sum
			sumYsqr += sq(signalY[i]/10); //add current Y^2 to sum
			sumXY += (signalX[i]/10) * (signalY[i]/10); //add current X*Y to sum
		}
	}
	else
	{
		for(uint16_t i = 0; i < n; i++)
		{
			sumX += signalX[i]; //add current X data point to sum
			sumY += signalY[i]; //add current Y data point to sum
			sumXsqr += sq(signalX[i]); //add current X^2 to sum
			sumYsqr += sq(signalY[i]); //add current Y^2 to sum
			sumXY += (signalX[i]) * (signalY[i]); //add current X*Y to sum
		}
	}

	
	//calculate and return r value
	return (n*sumXY - sumX*sumY)/sqrt((n*sumXsqr - sumX*sumX)*(n*sumYsqr - sumY*sumY));
}


////Source: https://www.statisticshowto.com/cross-correlation/
////https://www.statisticshowto.com/probability-and-statistics/
//template<typename Type>
//bool KickMath<Type>::iscorr(const Type signalX[], const Type signalY[], uint16_t n, float r)
//{
//	//this step adds roughly 70-90 us to the function with n = 128 or 366
//	Type abs_max_1 = abs(KickMath<Type>::getAbsMax(n, signalX));
//	Type abs_max_2 = abs(KickMath<Type>::getAbsMax(n, signalY));
//
//
//	int64_t sumX = 0; //declare variable for sum of X
//	int64_t sumY = 0; //declare variable for sum of Y
//	uint64_t sumXsqr = 0; //declare variable for sum of X^2
//	uint64_t sumYsqr = 0; //declare variable for sum of Y^2
//	int64_t sumXY = 0; //declare variable for sum of X*Y
//
//
//	if(abs_max_1 > 300 || abs_max_2 > 300)
//	{
//		//divide numbers by 10 to prevent data storage overflow
//		for(uint16_t i = 0; i < n; i++)
//		{
//			sumX += signalX[i]/10; //add current X data point to sum
//			sumY += signalY[i]/10; //add current Y data point to sum
//			sumXsqr += sq(signalX[i]/10); //add current X^2 to sum
//			sumYsqr += sq(signalY[i]/10); //add current Y^2 to sum
//			sumXY += (signalX[i]/10) * (signalY[i]/10); //add current X*Y to sum
//		}
//	}
//	else
//	{
//		for(uint16_t i = 0; i < n; i++)
//		{
//			sumX += signalX[i]; //add current X data point to sum
//			sumY += signalY[i]; //add current Y data point to sum
//			sumXsqr += sq(signalX[i]); //add current X^2 to sum
//			sumYsqr += sq(signalY[i]); //add current Y^2 to sum
//			sumXY += (signalX[i]) * (signalY[i]); //add current X*Y to sum
//		}
//	}
//
//
//	r = (n*sumXY - sumX*sumY)/sqrt((n*sumXsqr - sumX*sumX)*(n*sumYsqr - sumY*sumY)); //correlation coefficient
//	return false;
//}



#endif /* KickMath_h */


