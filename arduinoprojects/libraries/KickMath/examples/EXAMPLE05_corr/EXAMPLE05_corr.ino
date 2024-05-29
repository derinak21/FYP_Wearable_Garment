/*
 * FILENAME: EXAMPLE05_corr.ino
 * AUTHOR:   Orlando S. Hoilett
 * CONTACT:  orlandohoilett@gmail.com
 * VERSION:  1.0.0
 * 
 * 
 * AFFILIATIONS
 * Linnes Lab, Weldon School of Biomedical Engineering,
 * Purdue University, West Lafayette, IN 47907
 * 
 * 
 * DESCRIPTION
 * Basic test of the KickMath class to demonstrate the correlation
 * coefficient function. This is a templated, static class, so
 * function calls must be preceded with KickMath<variable_type>:: where
 * variable_type should be replaced with int16_t, int, float, etc.
 * 
 * The input signal is a simulated photoplethysmogram used to
 * measure heart rate. Each major peak is a new heart beat. The
 * smaller peak is a dicrotic notch which is generated when
 * the aortic valve closes.
 * 
 * 
 * UPDATES
 * Version 1.0.0
 * 2020/08/29:2019>
 *           - Initiated
 * 2020/08/30:0758> (UTC-5)
 *           - Chnaged xcorr function name to corr, making it equivalent to
 *           MATLAB's corr function with input arrays with one column each. No
 *           p-value calculation as of yet.
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


const uint16_t samples = 256;
//data is sampled every 42 milliseconds
//the input signal is a simulated photoplethysmogram used to measure
//heart rate. Each major peak is a new heart beat. The smaller peak is a
//dicrotic notch which is generated when the aortic valve closes.
const uint16_t input[] = {773, 702, 515, 389, 370, 447, 554, 601, 561, 492, 455, 458, 480, 497, 545, 724, 801, 651, 460, 370, 396, 494, 579, 570, 507, 450, 433, 449, 480, 499, 500, 591, 756, 736, 548, 396, 356, 416, 522, 577, 549, 479, 436, 430, 455, 485, 502, 504, 496, 570, 737, 731, 540, 391, 362, 436, 539, 580, 543, 471, 428, 427, 450, 473, 486, 492, 485, 471, 486, 635, 748, 639, 455, 354, 363, 461, 560, 577, 518, 463, 443, 458, 482, 503, 521, 515, 499, 585, 735, 710, 536, 402, 368, 422, 520, 584, 551, 487, 444, 433, 448, 478, 496, 502, 505, 595, 753, 695, 514, 383, 366, 433, 529, 584, 562, 498, 447, 437, 454, 479, 494, 500, 530, 681, 752, 622, 450, 366, 384, 468, 557, 584, 534, 469, 435, 438, 464, 495, 517, 526, 634, 777, 730, 544, 402, 366, 424, 528, 602, 574, 499, 450, 440, 457, 481, 500, 520, 658, 781, 678, 485, 374, 374, 447, 547, 599, 563, 491, 442, 439, 459, 480, 495, 567, 742, 768, 599, 421, 350, 377, 481, 574, 590, 528, 458, 430, 438, 464, 488, 510, 640, 776, 689, 502, 379, 363, 437, 546, 605, 581, 507, 452, 438, 455, 478, 504, 633, 790, 740, 541, 390, 356, 418, 520, 588, 573, 506, 450, 430, 444, 472, 581, 770, 764, 558, 383, 327, 377, 477, 564, 573, 514, 452, 425, 428, 449, 556, 752, 762, 575, 398, 337, 385, 496, 595, 590, 522, 460, 440, 451, 484, 650, 810, 723, 521, 389};
const float Fs = 23.8; //Hz


const uint16_t samples2 = 366;
const uint16_t input2Filtered[] = {1451,
1549,
1631,
2115,
2184,
1863,
1768,
1678,
1480,
1596,
2083,
2055,
1878,
1889,
1839,
1890,
2254,
2352,
1817,
1665,
1637,
1699,
1823,
2219,
2121,
1786,
1576,
1430,
1579,
1946,
2263,
1931,
1776,
1694,
1430,
1893,
2372,
2053,
1682,
1575,
1622,
1810,
2175,
1887,
1547,
1617,
1490,
1521,
2029,
2133,
1832,
1746,
1791,
1823,
1945,
2133,
1898,
1793,
1576,
1422,
1680,
1980,
1813,
1851,
1660,
1658,
2008,
2108,
1948,
1840,
1695,
1633,
1833,
2341,
2220,
1815,
1563,
1628,
1670,
2236,
2050,
1753,
1557,
1335,
1304,
1921,
2401,
1947,
1787,
1765,
1633,
1574,
1667,
2316,
2096,
1709,
1559,
1657,
1487,
1320,
1743,
2247,
2020,
1636,
1540,
1691,
1692,
1732,
2009,
1908,
1820,
1734,
1726,
1718,
1773,
2107,
2052,
1892,
1773,
1486,
1637,
1716,
1993,
2119,
1822,
1584,
1726,
1781,
1497,
1774,
2227,
2017,
1742,
1703,
1593,
1704,
1966,
2127,
2034,
1752,
1769,
1781,
1698,
2149,
2243,
2066,
1665,
1493,
1599,
1542,
2211,
2304,
1677,
1703,
1479,
1577,
1847,
2367,
1978,
1502,
1582,
1525,
1526,
2078,
2200,
1779,
1436,
1433,
1508,
2111,
2129,
1800,
1730,
1636,
1418,
1705,
2229,
2087,
1718,
1721,
1445,
1524,
1765,
2341,
2198,
1654,
1619,
1688,
1625,
1457,
2099,
2283,
1935,
1898,
1799,
1596,
1717,
2062,
2388,
1994,
1619,
1516,
1666,
1650,
1468,
2137,
2344,
2081,
1750,
1498,
1624,
1571,
1835,
2329,
2111,
1798,
1670,
1478,
1419,
1709,
2031,
2252,
1789,
1823,
1629,
1429,
1411,
1725,
2304,
1865,
1669,
1672,
1652,
1426,
1825,
2071,
1917,
1791,
1564,
1548,
1487,
1879,
2143,
2096,
1968,
1747,
1606,
1555,
1785,
2294,
1978,
1842,
1535,
1647,
1410,
1898,
2148,
2030,
1885,
1691,
1627,
1474,
2033,
2300,
1954,
1849,
1726,
1508,
1692,
2071,
2335,
1811,
1772,
1444,
1394,
1630,
2188,
2370,
1976,
1657,
1613,
1477,
1420,
1976,
2073,
1900,
1651,
1544,
1322,
1669,
2047,
2275,
1931,
1760,
1484,
1597,
1742,
2068,
1974,
1775,
1548,
1462,
1475,
1569,
2114,
2220,
1842,
1646,
1455,
1394,
1768,
2428,
2418,
2122,
2028,
1837,
1837,
2240,
2431,
2341,
2102,
1901,
1925,
1952,
2630,
2394,
2147,
1797,
1765,
1723,
1867,
2414,
2194,
1780,
1586,
1423,
1506,
2029,
2229,
1875,
1721,
1585,
1631,
1878,
2127,
2198,
2157,
1959,
1829,
1891,
2191,
2431,
2144,
1948,
2036,
1838,
1901,
2262,
2345,
2063,
1917,
2054,
2052,
1900,
2161,
2417};



const uint16_t input2Raw[] = {1454,
1601,
1674,
2366,
2220,
1698,
1720,
1632,
1378,
1657,
2335,
2041,
1787,
1895,
1814,
1917,
2442,
2403,
1542,
1588,
1623,
1732,
1887,
2424,
2071,
1614,
1469,
1356,
1657,
2136,
2427,
1761,
1697,
1653,
1295,
2132,
2620,
1889,
1492,
1521,
1647,
1907,
2364,
1739,
1373,
1654,
1425,
1537,
2292,
2188,
1678,
1702,
1815,
1841,
2009,
2231,
1778,
1739,
1465,
1344,
1813,
2136,
1728,
1872,
1562,
1657,
2189,
2161,
1866,
1785,
1621,
1602,
1937,
2603,
2159,
1607,
1434,
1662,
1692,
2528,
1955,
1600,
1456,
1222,
1289,
2240,
2649,
1714,
1706,
1755,
1565,
1545,
1715,
2651,
1984,
1510,
1483,
1709,
1400,
1235,
1962,
2507,
1904,
1439,
1492,
1769,
1693,
1753,
2152,
1856,
1775,
1691,
1723,
1715,
1802,
2280,
2024,
1810,
1713,
1339,
1715,
1758,
2137,
2184,
1669,
1462,
1800,
1810,
1351,
1918,
2462,
1909,
1601,
1684,
1537,
1762,
2102,
2210,
1987,
1608,
1779,
1788,
1656,
2382,
2292,
1975,
1459,
1405,
1654,
1513,
2557,
2353,
1354,
1717,
1365,
1628,
1987,
2636,
1778,
1257,
1624,
1496,
1527,
2364,
2264,
1562,
1260,
1432,
1548,
2423,
2139,
1631,
1694,
1588,
1306,
1854,
2500,
2014,
1528,
1724,
1303,
1566,
1890,
2639,
2125,
1374,
1602,
1724,
1593,
1371,
2430,
2378,
1756,
1879,
1749,
1492,
1780,
2240,
2557,
1791,
1426,
1464,
1744,
1642,
1375,
2482,
2452,
1946,
1580,
1369,
1690,
1545,
1972,
2584,
2000,
1637,
1605,
1380,
1389,
1859,
2198,
2367,
1551,
1841,
1530,
1326,
1403,
1887,
2603,
1640,
1568,
1674,
1642,
1311,
2032,
2198,
1838,
1727,
1447,
1541,
1456,
2082,
2280,
2073,
1903,
1634,
1534,
1529,
1905,
2557,
1816,
1772,
1377,
1706,
1289,
2150,
2277,
1970,
1811,
1591,
1595,
1396,
2322,
2439,
1776,
1795,
1663,
1396,
1788,
2267,
2472,
1541,
1753,
1275,
1369,
1753,
2477,
2464,
1774,
1493,
1591,
1407,
1391,
2263,
2124,
1811,
1524,
1490,
1208,
1848,
2242,
2393,
1754,
1672,
1342,
1656,
1817,
2237,
1927,
1673,
1431,
1419,
1483,
1618,
2395,
2275,
1648,
1546,
1358,
1363,
1961,
2769,
2413,
1970,
1980,
1740,
1838,
2449,
2530,
2296,
1979,
1798,
1938,
1967,
2980,
2273,
2020,
1618,
1750,
1702,
1942,
2697,
2082,
1568,
1486,
1339,
1549,
2300,
2333,
1694,
1643,
1516,
1655,
2006,
2256,
2236,
2137,
1858,
1762,
1923,
2346,
2555,
1997,
1847,
2082,
1736,
1934,
2449,
2389,
1919,
1843,
2125,
2052,
1822,
2297,
2549 };

void setup()
{
  Serial.begin(9600);

  //calculate and print correlation coefficient
  Serial.print("r: ");
  Serial.println(KickMath<uint16_t>::corr(input2Raw, input2Filtered, samples2));
}


void loop()
{
}