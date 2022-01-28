
	volatile int32_t testInt = -3200000;
	volatile int32_t testShort = 0;
	uint32_t startTime, stopTime;

	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 10000; i++){
		
	}
	stopTime = OSA_TimeGetMsec();
	uint32_t loopTime = stopTime-startTime;
	warpPrint("\nDonothing: %u", loopTime);

	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 10000; i++){
		testShort = (testInt << 15);
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nLShift 15: %u", stopTime-startTime-loopTime);

	double tes = 1000;
	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 100; i++){
		testShort = (int32_t) (tes*sin(i));
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nSin(i): %u", stopTime-startTime-loopTime);
	
	static volatile int16_t tftest [3] = {1,2,3};
	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 100000; i++){
		testShort = (int32_t) tftest[2];
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nROM: %u", stopTime-startTime-loopTime);

	volatile int16_t tf2test [3] = {1,2,3};
	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 100000; i++){
		testShort = (int32_t) tf2test[2];
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nRAM: %u", stopTime-startTime-loopTime);

	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 10000; i++){
		memcpy(&testShort, &testInt, 4);
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nmemcpy %u", stopTime-startTime-loopTime);
	
	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 10000; i++){
		testShort = (testInt << 16);
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nLShift 16: %u", stopTime-startTime-loopTime);
	
	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 10000; i++){
		testShort = (testInt >> 15);
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nRShift 15: %u", stopTime-startTime-loopTime);
	
	
	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 10000; i++){
		testShort = (testInt >> 16);
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nRShift 16: %u", stopTime-startTime-loopTime);
	
	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 10000; i++){
		testShort = (testInt / 1000);
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nDiv 1000: %u", stopTime-startTime-loopTime);

	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 10000; i++){
		testShort = (testInt / 32768);
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nDiv 32768: %u", stopTime-startTime-loopTime);

	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 10000; i++){
		testShort = (testInt / 32767);
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nDiv 32767: %u", stopTime-startTime-loopTime);

	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 10000; i++){
		testShort = (testInt / 65536);
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nDiv 65536: %u", stopTime-startTime-loopTime);

	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 10000; i++){
		testShort = (testInt * 1000);
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nMult 1000: %u", stopTime-startTime-loopTime);

	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 10000; i++){
		testShort = (testInt *65536);
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nMult 65536: %u", stopTime-startTime-loopTime);

	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 10000; i++){
		testShort = (testInt *32768);
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nMult 32768: %u", stopTime-startTime-loopTime);

	startTime = OSA_TimeGetMsec();
	for(volatile int i = 0; i < 10000; i++){
		testShort = (testInt * (-32768));
	}
	stopTime = OSA_TimeGetMsec();
	warpPrint("\nMult -32768: %u", stopTime-startTime-loopTime);
