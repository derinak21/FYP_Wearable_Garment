/*
 * FILENAME: EXAMPLE01_basic.ino
 * AUTHOR:   Orlando S. Hoilett and Alyson S. Pickering
 * CONTACT:  orlandohoilett@gmail.com
 * VERSION:  2.0.0
 * 
 * 
 * AFFILIATIONS
 * Linnes Lab, Weldon School of Biomedical Engineering,
 * Purdue University, West Lafayette, IN 47907
 * 
 * 
 * DESCRIPTION
 * Basic test the KickMath class's t-test function.
 * This is a static, templated class. Therefore, function calls must be
 * preceded with KickMath<variable_type>:: where variable_type should
 * be replaced with int16_t, int, float, etc.
 * 
 * The input signal values are randomly generated numbers.
 * 
 * 
 * UPDATES
 * Version 1.0.0
 * 2020/03/06:0603> (UTC-5)
 *            Simple example to test t-test function.
 * Version 2.0.0
 * 2020/08/18:1221> (UTC-5)
 *           - Moved to a templated class.
 *           - Added printout of basic other statistics like
 *             average and standard deviation.
 * 
 * 
 * DISCLAIMER
 * Linnes Lab code, firmware, and software is released under the
 * MIT License (http://opensource.org/licenses/MIT).
 * 
 * The MIT License (MIT)
 * 
 * Copyright (c) 2020 Linnes Lab, Purdue University
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */


#include "KickMath.h"
#include "ttestTable.h"


//const uint16_t samples = 4;
//int16_t num1[samples] = {1, 2, 3, 4};
//int16_t num2[samples] = {5, 3, 2, 3};
//int16_t num3[samples] = {15, 12, 13, 11};
//int16_t num4[samples] = {-1, -10, -4, -2};


const uint16_t samples = 20;
int16_t num1[samples] = { 18,21,16,22,19,24,17,21,23,18,14,
16,16,19,18,20,12,22,15,17};

int16_t num2[samples] = {22,25,17,24,16,29,20,23,19,20,15,15,
18,26,18,24,18,25,19,16};


#if defined(ARDUINO_ARCH_SAMD)
  #define SerialDebugger SerialUSB
#else
  #define SerialDebugger Serial
#endif


void setup()
{
  SerialDebugger.begin(9600);
  while(!SerialDebugger); //holds the program here until the serial monitor or plotter is opened


  //avg and standard deviation
  SerialDebugger.print("avg: ");
  SerialDebugger.print(KickMath<int16_t>::calcAverage(samples, num1));
  SerialDebugger.print(", ");
  SerialDebugger.print("st dev: ");
  SerialDebugger.println(KickMath<int16_t>::calcStDev(samples, num1));

  //avg and standard deviation
  SerialDebugger.print("avg: ");
  SerialDebugger.print(KickMath<int16_t>::calcAverage(samples, num2));
  SerialDebugger.print(", ");
  SerialDebugger.print("st dev: ");
  SerialDebugger.println(KickMath<int16_t>::calcStDev(samples, num2));

  //t-test
  SerialDebugger.println();
  SerialDebugger.print("t-test result: ");
  SerialDebugger.println(KickMath<int16_t>::tTest(num1, num2, 20, 0.05));
  SerialDebugger.println("returns true (1) if reject null hypothesis");
  SerialDebugger.println("returns false (0) if fail to reject null hypothesis");
  
}

void loop()
{
}
