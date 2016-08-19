//Interrupt Service Routines
//This is the basic ISR for a generic propeller
//Takes an int i where i is the propeller number.
void rpm(int i){
  if (halfRotations[i] == 0) {
    startTimePulse[i] = micros();
    halfRotations[i]++;
  }
  else if (halfRotations[i] == numPulsesPerRevolution / 2) {
    endTimePulse[i] = micros();
    pulseDurations[i][placeData[i]] = endTimePulse[i] - startTimePulse[i];
    placeData[i] = (placeData[i] + 1) % numRPMPulsesToAverage;
    halfRotations[i] = 0; //1; //if the last end time pulse is used for the next start time.
    //startTimePulse[i] = endTimePulse[i];
  }
  else {
    halfRotations[i]++;
  }
}
//These wrap the generic function for the individual propellers.
//as ISR's can't take parameters.
void rpm0() {rpm(0);}
void rpm1() {rpm(1);}
void rpm2() {rpm(2);}
void rpm3() {rpm(3);}
//End of Interrupt Service Routines

// get truncated (i.e. robust) mean of the pulse durations
float averageNumMicrosPerPulse(long *rpmPulseDuration) {
  long longestPulse = -999, secondLongestPulse = -999, shortestPulse = -999, secondShortestPulse = -999;
  long truncatedSumOfPulseDurations, sumOfPulseDurations = 0;
  byte uncollectedDataCount = 0;
  for (int i = 0; i < numRPMPulsesToAverage; ++i) {
    if (rpmPulseDuration[i] == -999) {
      uncollectedDataCount++;
      continue;
    }
    sumOfPulseDurations += abs(rpmPulseDuration[i]);
    if (abs(rpmPulseDuration[i]) > longestPulse) {
      secondLongestPulse = longestPulse;
      longestPulse = abs(rpmPulseDuration[i]);
    }
    else if (abs(rpmPulseDuration[i]) > secondLongestPulse) secondLongestPulse = abs(rpmPulseDuration[i]);
    if (abs(rpmPulseDuration[i]) < shortestPulse || shortestPulse == -999) {
      secondShortestPulse = shortestPulse;
      shortestPulse = abs(rpmPulseDuration[i]);
    }
    else if (abs(rpmPulseDuration[i]) < secondShortestPulse) secondShortestPulse = abs(rpmPulseDuration[i]);
  }
  if (uncollectedDataCount >= numRPMPulsesToAverage - 4) return 0.0; //If not enough data, prevent a zero divide error that would occur 3 lines down.
                                                                     //This is an error return value indicating prop not spinning, handle output seperately.
  truncatedSumOfPulseDurations = sumOfPulseDurations - longestPulse - secondLongestPulse - shortestPulse - secondShortestPulse;
  return truncatedSumOfPulseDurations / (numRPMPulsesToAverage - 4. - uncollectedDataCount);
}

//Checks to see if the difference between the rotor's start time and the current time is greater than the time out.
//If not spinning, reset the row of pulseDurations
//Takes a rotor number as a parameter
boolean isSpinning(int rotorNum){
  unsigned long presentTime = micros();
  noInterrupts();
  unsigned long startTime = startTimePulse[rotorNum];
  interrupts();
  if (startTime == -999) return false;
  if (abs(presentTime - startTime) > numMicrosBeforeRPMTimeout) {
    noInterrupts();
    halfRotations[rotorNum] = 0;                     //This way the pulse time won't get incredibly long if stopped and started later.
    for (int i = 0; i < numRPMPulsesToAverage; i++){ //Uninitialize the rpm data for that rotor.
      pulseDurations[rotorNum][i] = -999;            //So when it spins up again, it will only use new data
    } interrupts();
    return false;
  }
  return true;
}

//Takes a rotor number.
//Returns the RPM for that rotor.
float getRPM(int rotorNum) {
  long rpmPulseDuration[numRPMPulsesToAverage];
  noInterrupts(); //being carefully to copy needed data out of volatile memory
  for (int i = 0; i < numRPMPulsesToAverage; i++) {
    rpmPulseDuration[i] = pulseDurations[rotorNum][i];
  } interrupts();
  
  float averagePulseLen = averageNumMicrosPerPulse(rpmPulseDuration);
  if (averagePulseLen == 0.0) return 0.0; //The prop may have recently started spinning but there isn't enough data to calc rpm.
  return numMicrosPerMinute / (numPulsesPerRevolution * averagePulseLen);
}

void initializeRPMsensors() {
  // initialize digital pins 18, 19, A14, A15 as digital input.
  for (int i = 0; i < 4; ++i) {
    pinMode(rpmTimerPin[i], INPUT);
  }
  
  // initialize the pulseDurations array at -999 to indicate data not yet collected.
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 20; j++) {
      pulseDurations[i][j] = -999;
    }
  }
  //attachInterrupt(digitalPinToInterrupt(rpmTimerPin[0]), rpm0, RISING);
  PcInt::attachInterrupt(               rpmTimerPin[0] , rpm0, RISING);
  PcInt::attachInterrupt(               rpmTimerPin[1] , rpm1, RISING);
  PcInt::attachInterrupt(               rpmTimerPin[2] , rpm2, RISING);
  attachInterrupt(digitalPinToInterrupt(rpmTimerPin[3]), rpm3, RISING);
}

/* //Pin change interrupts without a library (not been tested; should work in theory, but for an arduino mega 2560 only)
void initializeInterrupts(){
  cli() //turns off interrupts
  
  PCICR = 0x04 //turns on PCIE2 which will trigger an interrupt on any change
               //of PCINT23..16 using PCI2 vector
  PCMSK2 = 0b11000000 //enables interrupts only on PCINT23..22
  
  sei() //turns interrupts back on
}
ISR(PCINT2_vect){
  Static int pinState_A15_prev = digitalRead(A15);
  Static int pinState_A14_prev = digitalRead(A14);
  
  Int pinState_A15 = digitalRead(A15);
  Int pinState_A14 = digitalRead(A14);
  
  If ( pinState_A15_prev == LO &&  pinState_A15 == HI){
    rpm2();
  }
  If ( pinState_A14_prev == LO &&  pinState_A14 == HI){
    rpm1();
  }
  
  pinState_A15_prev = pinState_A15;
  pinState_A14_prev = pinState_A14;
}*/
