const unsigned int sampleRate = 22050;
const unsigned int sampleCount = 3327;
const signed char samples[] = {//0
0, 4, 6, 9, 12, 13, 15, 16, 19, 22, 21, 19, 23, 25, 25, 25, //16
28, 31, 29, 27, 27, 29, 30, 27, 31, 33, 35, 37, 38, 41, 41, 39, //32
38, 38, 35, 34, 33, 35, 34, 32, 31, 31, 28, 25, 21, 17, 12, 8, //48
3, 0, -2, -1, -3, -5, -6, -6, -7, -11, -11, -13, -16, -21, -27, -33, //64
-37, -39, -40, -40, -42, -44, -47, -49, -51, -54, -56, -58, -61, -67, -73, -76, //80
-78, -80, -84, -85, -85, -88, -91, -90, -90, -93, -94, -97, -95, -94, -95, -93, //96
-94, -96, -97, -96, -95, -97, -96, -95, -94, -93, -94, -94, -93, -90, -85, -82, //112
-82, -80, -74, -75, -73, -70, -69, -68, -69, -67, -64, -64, -64, -58, -54, -50, //128
-47, -42, -38, -35, -33, -28, -22, -17, -15, -12, -8, -6, -3, 2, 9, 13, //144
16, 18, 23, 30, 31, 33, 35, 39, 41, 46, 48, 51, 56, 59, 62, 67, //160
72, 79, 85, 88, 96, 99, 101, 104, 106, 111, 112, 109, 109, 108, 108, 108, //176
108, 110, 112, 112, 113, 114, 112, 111, 110, 108, 106, 109, 111, 108, 104, 101, //192
98, 96, 97, 96, 95, 96, 92, 90, 90, 91, 90, 88, 86, 84, 80, 78, //208
74, 70, 67, 64, 60, 58, 55, 51, 50, 49, 46, 42, 36, 29, 27, 27, //224
24, 18, 13, 8, 5, 3, 1, -1, -2, -5, -6, -5, -6, -9, -15, -20, //240
-25, -28, -29, -29, -26, -27, -32, -36, -40, -42, -43, -47, -48, -46, -45, -43, //256
-45, -45, -46, -46, -47, -47, -49, -51, -51, -51, -50, -49, -46, -43, -41, -39, //272
-41, -42, -41, -37, -36, -37, -39, -41, -41, -40, -36, -32, -31, -30, -29, -28, //288
-26, -22, -19, -16, -14, -14, -15, -16, -16, -15, -14, -9, -5, -3, 0, 0, //304
1, 4, 4, 4, 6, 7, 7, 9, 11, 12, 12, 13, 13, 12, 13, 9, //320
8, 6, 5, 5, 4, 4, 4, 2, 1, 4, 3, 1, 0, 0, 1, 2, //336
2, 1, 1, -1, -4, -7, -9, -8, -4, -1, -2, -5, -5, -3, -3, -6, //352
-10, -10, -8, -7, -8, -10, -9, -10, -12, -14, -16, -15, -16, -17, -19, -20, //368
-17, -18, -15, -14, -16, -19, -19, -19, -26, -35, -41, -41, -40, -42, -41, -38, //384
-39, -41, -42, -41, -39, -37, -34, -33, -33, -32, -31, -27, -24, -24, -24, -23, //400
-24, -25, -25, -24, -22, -22, -19, -16, -15, -12, -9, -8, -7, -6, -3, 0, //416
6, 9, 10, 12, 14, 18, 19, 22, 25, 28, 31, 34, 38, 42, 46, 49, //432
52, 53, 55, 57, 57, 56, 54, 53, 55, 58, 57, 55, 56, 59, 60, 63, //448
68, 70, 70, 68, 65, 66, 67, 68, 68, 66, 63, 65, 63, 59, 59, 60, //464
57, 54, 53, 52, 48, 45, 41, 42, 41, 38, 37, 37, 36, 33, 31, 31, //480
29, 20, 16, 15, 14, 13, 13, 13, 11, 11, 11, 6, -3, -4, -6, -7, //496
-9, -12, -12, -15, -18, -20, -22, -22, -25, -26, -26, -27, -29, -32, -33, -37, //512
-43, -47, -50, -49, -49, -51, -52, -54, -58, -57, -56, -56, -55, -55, -58, -63, //528
-68, -69, -68, -65, -62, -63, -65, -64, -63, -64, -61, -64, -65, -63, -61, -61, //544
-60, -59, -61, -60, -59, -60, -57, -53, -55, -59, -61, -57, -56, -56, -55, -51, //560
-49, -48, -48, -49, -49, -47, -43, -41, -41, -40, -39, -41, -37, -34, -33, -30, //576
-26, -20, -20, -19, -18, -19, -17, -15, -13, -10, -9, -7, -1, 0, 1, 4, //592
6, 6, 0, -3, -5, -3, -1, -4, -9, -5, -1, 2, 3, 0, -1, -1, //608
-1, -3, -4, -6, -10, -11, -12, -14, -16, -18, -19, -18, -20, -22, -21, -18, //624
-14, -14, -17, -19, -23, -28, -31, -30, -30, -32, -31, -31, -32, -31, -29, -31, //640
-31, -32, -33, -35, -37, -40, -37, -36, -34, -29, -28, -27, -28, -30, -31, -35, //656
-37, -37, -39, -39, -36, -33, -30, -28, -24, -19, -17, -15, -12, -13, -11, -9, //672
-11, -10, -9, -7, -4, -2, 1, 5, 5, 5, 7, 12, 15, 17, 17, 19, //688
23, 28, 37, 39, 39, 40, 45, 50, 58, 62, 62, 64, 67, 71, 72, 73, //704
75, 77, 78, 81, 84, 89, 91, 90, 90, 92, 94, 96, 96, 97, 97, 99, //720
102, 105, 108, 109, 112, 113, 113, 114, 114, 115, 118, 119, 118, 116, 115, 114, //736
115, 117, 117, 115, 113, 111, 109, 110, 109, 103, 98, 94, 92, 90, 87, 86, //752
84, 82, 82, 80, 79, 79, 73, 71, 68, 65, 62, 59, 56, 55, 53, 50, //768
46, 41, 38, 36, 34, 34, 35, 33, 30, 28, 29, 26, 23, 19, 17, 15, //784
8, 3, 2, -1, -2, -4, -6, -4, -6, -9, -11, -12, -13, -12, -12, -14, //800
-15, -15, -17, -20, -20, -20, -21, -22, -25, -26, -26, -29, -29, -28, -30, -30, //816
-31, -29, -26, -22, -20, -19, -19, -22, -22, -22, -21, -22, -23, -22, -22, -20, //832
-21, -23, -24, -25, -27, -26, -27, -26, -28, -33, -33, -29, -25, -25, -23, -21, //848
-21, -20, -19, -18, -17, -20, -24, -22, -23, -24, -23, -21, -20, -19, -18, -17, //864
-15, -15, -21, -23, -23, -25, -28, -29, -25, -23, -24, -26, -29, -30, -29, -28, //880
-31, -36, -46, -50, -51, -51, -53, -56, -56, -58, -59, -61, -63, -63, -66, -66, //896
-68, -76, -82, -88, -93, -96, -97, -98, -99, -102, -104, -106, -105, -105, -110, -116, //912
-117, -116, -116, -117, -117, -116, -115, -119, -121, -120, -122, -122, -124, -126, -127, -125, //928
-123, -124, -124, -123, -122, -118, -118, -118, -118, -118, -118, -117, -115, -112, -110, -108, //944
-106, -103, -100, -96, -90, -85, -81, -77, -75, -72, -69, -69, -68, -67, -65, -62, //960
-58, -52, -45, -40, -38, -36, -33, -32, -31, -27, -24, -21, -16, -11, -8, -3, //976
5, 12, 15, 16, 16, 18, 21, 23, 25, 28, 32, 35, 39, 41, 45, 45, //992
45, 46, 50, 55, 59, 62, 64, 66, 68, 68, 70, 73, 74, 77, 79, 79, //1008
82, 85, 86, 85, 85, 87, 89, 93, 96, 94, 93, 92, 90, 88, 90, 92, //1024
93, 95, 97, 96, 95, 94, 93, 94, 96, 96, 96, 96, 97, 97, 94, 91, //1040
91, 90, 89, 90, 92, 91, 86, 82, 80, 78, 76, 74, 70, 67, 64, 61, //1056
59, 57, 56, 54, 51, 49, 47, 46, 45, 40, 34, 31, 29, 31, 32, 31, //1072
33, 34, 33, 34, 31, 29, 27, 25, 21, 18, 17, 18, 16, 16, 20, 20, //1088
19, 20, 23, 25, 25, 24, 22, 20, 21, 20, 19, 17, 15, 15, 15, 19, //1104
22, 23, 21, 20, 20, 18, 16, 15, 14, 13, 10, 8, 7, 9, 11, 13, //1120
13, 11, 13, 9, 4, 3, 0, -1, -2, -2, 0, 0, 4, 7, 4, 3, //1136
3, 1, -1, -3, -5, -6, -7, -9, -11, -11, -11, -13, -17, -19, -20, -21, //1152
-22, -22, -22, -24, -25, -25, -27, -30, -33, -36, -34, -34, -36, -37, -40, -43, //1168
-45, -46, -49, -53, -56, -57, -56, -55, -58, -58, -57, -56, -55, -55, -56, -57, //1184
-59, -63, -67, -64, -62, -62, -60, -60, -59, -57, -56, -55, -53, -51, -50, -48, //1200
-48, -50, -52, -52, -52, -50, -49, -47, -43, -41, -41, -38, -33, -29, -26, -23, //1216
-19, -15, -11, -9, -5, 2, 4, 7, 9, 14, 20, 23, 25, 26, 26, 26, //1232
27, 31, 34, 37, 40, 43, 47, 52, 54, 57, 57, 57, 61, 60, 59, 61, //1248
64, 65, 62, 62, 63, 62, 63, 64, 61, 60, 60, 59, 61, 59, 56, 55, //1264
56, 60, 63, 63, 64, 64, 61, 59, 57, 52, 52, 51, 51, 51, 50, 51, //1280
54, 57, 58, 59, 56, 56, 57, 56, 56, 58, 58, 55, 52, 49, 49, 51, //1296
50, 46, 41, 39, 38, 38, 38, 35, 34, 32, 29, 23, 21, 18, 16, 14, //1312
7, 3, 0, -2, -4, -6, -10, -11, -12, -15, -20, -26, -30, -34, -38, -43, //1328
-45, -46, -47, -49, -52, -57, -61, -64, -70, -74, -76, -78, -81, -84, -88, -93, //1344
-96, -97, -98, -98, -98, -98, -97, -97, -99, -99, -101, -104, -105, -108, -110, -112, //1360
-112, -111, -110, -108, -107, -108, -109, -108, -106, -103, -102, -101, -100, -98, -96, -92, //1376
-88, -85, -84, -82, -81, -78, -75, -71, -67, -67, -65, -65, -61, -54, -50, -47, //1392
-48, -48, -45, -45, -43, -36, -32, -29, -26, -22, -19, -16, -15, -12, -7, -2, //1408
0, 2, 5, 5, 6, 7, 12, 15, 19, 23, 24, 23, 25, 27, 29, 32, //1424
36, 38, 40, 40, 38, 41, 44, 48, 51, 53, 55, 54, 54, 55, 55, 54, //1440
52, 50, 50, 52, 55, 53, 49, 48, 45, 42, 42, 44, 44, 41, 41, 39, //1456
35, 35, 34, 32, 31, 32, 33, 31, 29, 30, 29, 27, 26, 27, 30, 27, //1472
23, 19, 16, 16, 15, 16, 17, 18, 15, 12, 9, 8, 6, 7, 9, 6, //1488
4, 5, 7, 8, 10, 13, 12, 12, 10, 9, 10, 13, 10, 8, 8, 11, //1504
15, 15, 16, 16, 17, 17, 17, 15, 15, 15, 15, 16, 18, 19, 20, 20, //1520
20, 20, 20, 18, 15, 14, 14, 14, 15, 16, 16, 16, 17, 19, 17, 15, //1536
15, 16, 16, 15, 15, 17, 17, 16, 21, 21, 20, 18, 17, 17, 18, 21, //1552
23, 24, 26, 29, 28, 29, 32, 33, 32, 30, 31, 31, 30, 30, 32, 30, //1568
31, 34, 34, 33, 33, 34, 33, 32, 33, 32, 29, 27, 28, 25, 23, 21, //1584
17, 16, 12, 8, 5, 0, 0, -1, -2, -3, -5, -9, -12, -15, -16, -16, //1600
-17, -17, -18, -20, -23, -28, -32, -36, -38, -39, -41, -42, -43, -44, -42, -40, //1616
-40, -37, -35, -35, -34, -32, -33, -34, -31, -30, -29, -27, -25, -23, -21, -20, //1632
-18, -15, -10, -9, -7, -4, -3, -2, 0, 2, 6, 8, 10, 13, 16, 18, //1648
18, 15, 14, 17, 18, 17, 17, 17, 17, 20, 21, 19, 18, 19, 18, 20, //1664
20, 19, 20, 23, 27, 29, 30, 29, 24, 22, 21, 20, 19, 20, 23, 24, //1680
22, 20, 21, 22, 23, 27, 30, 29, 25, 22, 23, 21, 23, 27, 31, 35, //1696
37, 38, 39, 38, 35, 34, 36, 36, 36, 36, 36, 36, 31, 29, 27, 28, //1712
28, 25, 22, 20, 18, 16, 14, 15, 14, 12, 8, 2, -3, -11, -16, -20, //1728
-23, -26, -30, -37, -43, -46, -47, -51, -56, -60, -64, -70, -76, -82, -88, -88, //1744
-91, -94, -97, -100, -102, -103, -102, -101, -104, -110, -113, -116, -118, -119, -119, -118, //1760
-117, -117, -119, -119, -118, -114, -110, -107, -103, -100, -93, -89, -86, -85, -82, -79, //1776
-82, -80, -75, -70, -66, -58, -55, -54, -52, -49, -44, -37, -33, -28, -26, -25, //1792
-22, -20, -18, -13, -7, -4, 0, 4, 8, 9, 8, 9, 12, 15, 15, 15, //1808
14, 13, 14, 14, 14, 17, 18, 19, 20, 21, 21, 23, 27, 27, 27, 29, //1824
31, 30, 29, 27, 27, 28, 28, 26, 26, 26, 26, 25, 25, 26, 29, 32, //1840
33, 35, 34, 31, 32, 33, 34, 32, 30, 31, 30, 29, 28, 27, 29, 34, //1856
35, 33, 29, 27, 25, 20, 20, 19, 18, 17, 15, 17, 17, 16, 17, 17, //1872
19, 19, 16, 10, 7, 5, -2, -5, -7, -9, -11, -11, -12, -14, -16, -16, //1888
-13, -12, -13, -13, -14, -13, -12, -11, -10, -12, -13, -12, -9, -6, -3, -2, //1904
-2, 0, 4, 6, 6, 7, 10, 15, 14, 12, 13, 15, 15, 21, 23, 26, //1920
30, 32, 37, 43, 47, 49, 50, 52, 56, 56, 57, 59, 61, 64, 67, 70, //1936
72, 72, 72, 71, 71, 73, 73, 74, 74, 68, 69, 68, 66, 69, 70, 71, //1952
68, 69, 72, 70, 67, 72, 74, 75, 74, 75, 72, 71, 72, 73, 73, 73, //1968
72, 71, 72, 72, 71, 66, 64, 63, 60, 59, 58, 56, 55, 55, 56, 55, //1984
54, 52, 51, 48, 44, 41, 38, 35, 33, 30, 28, 25, 21, 18, 16, 14, //2000
11, 8, 5, 4, 3, 0, -5, -8, -12, -14, -14, -17, -22, -28, -32, -33, //2016
-35, -39, -40, -41, -41, -40, -40, -42, -43, -44, -44, -45, -44, -44, -47, -48, //2032
-47, -47, -49, -50, -52, -51, -51, -53, -54, -53, -53, -55, -54, -52, -54, -53, //2048
-53, -55, -58, -59, -53, -47, -44, -41, -42, -43, -45, -45, -43, -41, -40, -36, //2064
-31, -29, -28, -29, -28, -29, -27, -27, -26, -21, -18, -15, -14, -12, -8, -6, //2080
-7, -6, -8, -9, -6, -1, 5, 9, 7, 4, 4, 7, 8, 10, 11, 11, //2096
11, 12, 13, 13, 14, 13, 15, 17, 15, 13, 11, 12, 11, 7, 6, 6, //2112
6, 5, 2, -1, -2, -5, -7, -9, -13, -17, -18, -19, -18, -20, -26, -28, //2128
-32, -34, -33, -33, -37, -41, -42, -45, -49, -52, -58, -62, -64, -67, -71, -76, //2144
-76, -75, -74, -74, -76, -79, -84, -87, -88, -92, -93, -93, -92, -93, -93, -94, //2160
-95, -93, -93, -93, -93, -93, -89, -89, -90, -88, -87, -89, -90, -91, -91, -89, //2176
-86, -83, -82, -80, -77, -73, -71, -68, -66, -65, -64, -66, -67, -67, -67, -63, //2192
-62, -63, -59, -55, -53, -51, -49, -44, -39, -36, -34, -32, -29, -27, -24, -21, //2208
-18, -12, -10, -6, -5, -1, 4, 8, 13, 16, 18, 22, 24, 26, 31, 33, //2224
37, 38, 40, 43, 47, 51, 53, 57, 61, 64, 72, 73, 75, 78, 79, 81, //2240
82, 85, 87, 88, 88, 92, 94, 94, 94, 96, 98, 98, 99, 99, 98, 99, //2256
101, 103, 103, 103, 102, 101, 103, 101, 99, 96, 94, 96, 96, 94, 94, 96, //2272
97, 92, 87, 86, 89, 91, 89, 87, 84, 80, 80, 80, 79, 78, 77, 77, //2288
76, 73, 65, 62, 61, 60, 60, 58, 51, 48, 45, 42, 38, 37, 38, 37, //2304
36, 36, 34, 32, 33, 32, 30, 28, 29, 26, 25, 23, 21, 18, 17, 16, //2320
14, 13, 14, 17, 15, 11, 7, 6, 7, 8, 6, 5, 5, 6, 8, 11, //2336
15, 16, 16, 17, 17, 19, 19, 18, 16, 13, 9, 4, 2, 3, 2, 2, //2352
4, 5, 5, 5, 5, 5, 5, 5, 6, 6, 1, -6, -11, -11, -10, -9, //2368
-8, -7, -10, -11, -9, -8, -11, -17, -20, -22, -24, -27, -30, -31, -31, -31, //2384
-31, -32, -36, -37, -37, -39, -41, -42, -45, -48, -53, -57, -60, -62, -62, -62, //2400
-61, -61, -63, -61, -61, -62, -62, -62, -63, -70, -73, -74, -75, -77, -77, -76, //2416
-75, -76, -79, -80, -78, -77, -78, -77, -78, -82, -88, -88, -86, -86, -89, -91, //2432
-87, -84, -83, -82, -80, -81, -81, -81, -81, -80, -80, -84, -82, -81, -79, -77, //2448
-77, -77, -75, -71, -70, -68, -65, -65, -63, -59, -56, -53, -51, -50, -46, -41, //2464
-38, -34, -26, -23, -20, -16, -10, -2, 5, 9, 9, 9, 10, 18, 23, 27, //2480
32, 38, 41, 42, 46, 52, 57, 62, 68, 69, 70, 73, 78, 81, 82, 84, //2496
87, 91, 93, 97, 97, 98, 100, 101, 101, 100, 100, 102, 102, 100, 98, 97, //2512
96, 94, 90, 88, 88, 88, 88, 87, 84, 82, 81, 81, 78, 76, 76, 75, //2528
73, 71, 66, 60, 56, 56, 55, 53, 54, 54, 53, 49, 47, 46, 42, 41, //2544
41, 43, 44, 41, 40, 38, 35, 31, 31, 30, 29, 28, 25, 25, 25, 23, //2560
19, 13, 11, 10, 11, 11, 11, 13, 12, 11, 10, 8, 7, 9, 12, 14, //2576
13, 10, 7, 1, -2, -4, -4, -3, -2, -1, -1, -2, -2, 0, 3, 1, //2592
-1, -3, -6, -6, -7, -7, -6, -8, -9, -9, -9, -10, -14, -15, -14, -14, //2608
-13, -12, -13, -16, -22, -26, -27, -27, -28, -29, -30, -28, -25, -22, -19, -17, //2624
-14, -15, -14, -15, -18, -20, -26, -27, -29, -30, -29, -32, -33, -35, -34, -32, //2640
-33, -35, -37, -41, -45, -47, -50, -52, -52, -52, -54, -55, -58, -60, -58, -60, //2656
-63, -64, -67, -71, -76, -80, -83, -88, -92, -96, -98, -97, -97, -97, -96, -95, //2672
-94, -95, -96, -99, -103, -108, -111, -110, -113, -114, -115, -115, -113, -111, -112, -113, //2688
-112, -111, -108, -107, -109, -110, -111, -110, -109, -106, -102, -99, -94, -90, -87, -82, //2704
-80, -78, -75, -72, -70, -67, -66, -65, -65, -66, -61, -56, -55, -54, -50, -46, //2720
-41, -37, -34, -32, -30, -30, -31, -29, -26, -24, -25, -21, -17, -16, -15, -14, //2736
-12, -11, -10, -7, -5, -1, 4, 10, 12, 12, 14, 17, 20, 22, 26, 31, //2752
33, 34, 41, 44, 47, 49, 51, 54, 59, 62, 65, 68, 68, 68, 69, 71, //2768
75, 78, 79, 82, 82, 83, 85, 88, 91, 93, 94, 94, 93, 94, 96, 96, //2784
95, 96, 98, 99, 102, 104, 105, 105, 104, 102, 103, 102, 101, 102, 102, 104, //2800
106, 105, 107, 106, 105, 106, 103, 101, 100, 101, 99, 98, 96, 94, 95, 94, //2816
94, 94, 93, 96, 96, 96, 98, 97, 95, 94, 95, 91, 90, 89, 89, 91, //2832
89, 86, 88, 92, 95, 93, 91, 87, 85, 80, 74, 74, 75, 70, 70, 72, //2848
72, 72, 70, 70, 68, 67, 68, 68, 64, 59, 54, 51, 51, 51, 49, 46, //2864
45, 43, 39, 35, 31, 29, 28, 27, 25, 24, 24, 22, 20, 21, 22, 24, //2880
23, 20, 19, 18, 11, 8, 6, 6, 5, 1, -6, -9, -10, -10, -9, -9, //2896
-12, -14, -14, -16, -18, -19, -21, -23, -24, -24, -26, -26, -27, -29, -28, -27, //2912
-31, -33, -36, -36, -38, -42, -42, -44, -48, -49, -51, -53, -55, -54, -54, -57, //2928
-57, -60, -61, -62, -64, -65, -63, -58, -58, -60, -62, -66, -71, -74, -75, -73, //2944
-70, -69, -70, -73, -77, -77, -76, -75, -71, -70, -68, -67, -67, -66, -65, -64, //2960
-61, -58, -55, -52, -54, -57, -56, -55, -55, -58, -60, -58, -58, -57, -53, -53, //2976
-53, -54, -54, -55, -57, -58, -58, -57, -57, -54, -52, -51, -50, -50, -52, -58, //2992
-63, -66, -66, -66, -69, -71, -71, -73, -73, -72, -75, -77, -78, -78, -79, -81, //3008
-83, -85, -87, -86, -86, -88, -89, -93, -97, -99, -99, -100, -101, -99, -99, -99, //3024
-98, -101, -104, -106, -104, -103, -105, -104, -102, -100, -99, -100, -101, -97, -95, -93, //3040
-92, -90, -88, -87, -82, -78, -76, -72, -67, -63, -59, -55, -51, -48, -45, -44, //3056
-41, -37, -32, -26, -20, -13, -10, -5, -2, 5, 9, 15, 20, 24, 29, 35, //3072
34, 38, 45, 50, 56, 57, 59, 60, 61, 65, 69, 72, 74, 77, 80, 82, //3088
84, 90, 93, 94, 95, 94, 94, 95, 97, 99, 101, 109, 115, 117, 116, 114, //3104
113, 113, 114, 114, 111, 111, 111, 109, 108, 107, 105, 103, 103, 103, 101, 100, //3120
101, 103, 100, 97, 95, 93, 89, 83, 82, 81, 79, 73, 66, 64, 61, 58, //3136
56, 53, 48, 46, 44, 43, 42, 40, 37, 36, 34, 30, 27, 25, 22, 21, //3152
19, 18, 16, 11, 9, 7, 5, 6, 6, 6, 4, 2, 0, -2, -2, 0, //3168
1, 3, 1, -1, 2, 4, 5, 3, 2, 3, 4, 5, 6, 7, 9, 11, //3184
11, 12, 12, 11, 12, 15, 19, 22, 23, 23, 25, 30, 32, 36, 42, 45, //3200
47, 52, 54, 54, 54, 56, 58, 60, 60, 60, 60, 62, 65, 65, 66, 69, //3216
71, 74, 75, 73, 73, 73, 70, 64, 62, 62, 62, 62, 62, 61, 59, 55, //3232
52, 48, 43, 41, 38, 35, 33, 32, 30, 25, 21, 15, 10, 6, -1, -3, //3248
-6, -10, -12, -15, -21, -25, -29, -35, -43, -49, -55, -60, -63, -69, -73, -71, //3264
-71, -74, -77, -79, -82, -87, -90, -94, -96, -95, -98, -97, -95, -93, -94, -97, //3280
-100, -102, -102, -104, -109, -112, -110, -107, -103, -96, -91, -89, -91, -92, -90, -89, //3296
-87, -84, -84, -84, -81, -76, -69, -66, -63, -58, -55, -55, -54, -50, -47, -43, //3312
-39, -36, -31, -27, -23, -18, -12, -6, -6, -5, -7, -9, -6, -3, 0, };
