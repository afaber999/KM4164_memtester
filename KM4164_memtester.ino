// Simple DRAM memory tester for KM4164 DRAM chips
// A.L. Faber (c) 2020

const byte PR_A0 = A0;
const byte PR_A1 = A1;
const byte PR_A2 = A2;
const byte PR_A3 = A3;
const byte PR_A4 = A4;
const byte PR_A5 = A5;
const byte PR_A6 = 6;
const byte PR_A7 = 7;


const byte PR_DI = 9;
const byte PR_DQ = 8;

const byte PR_CAS = 12;
const byte PR_RAS = 11;
const byte PR_W   = 10;


void SetAddressByte( uint8_t ab) {
  digitalWrite(PR_A0, ab & 0b00000001);
  digitalWrite(PR_A1, ab & 0b00000010);
  digitalWrite(PR_A2, ab & 0b00000100);
  digitalWrite(PR_A3, ab & 0b00001000);
  digitalWrite(PR_A4, ab & 0b00010000);
  digitalWrite(PR_A5, ab & 0b00100000);
  digitalWrite(PR_A6, ab & 0b01000000);
  digitalWrite(PR_A7, ab & 0b10000000);
}

void setup() {

  Serial.begin(115200);
  Serial.println("Start 4164 mem tester (c) A.L. Faber 2020 v1.1");

  pinMode(PR_A0, OUTPUT);
  pinMode(PR_A1, OUTPUT);
  pinMode(PR_A2, OUTPUT);
  pinMode(PR_A3, OUTPUT);
  pinMode(PR_A4, OUTPUT);
  pinMode(PR_A5, OUTPUT);
  pinMode(PR_A6, OUTPUT);
  pinMode(PR_A7, OUTPUT);

  pinMode(PR_CAS, OUTPUT);
  pinMode(PR_RAS, OUTPUT);
  pinMode(PR_W, OUTPUT);

  pinMode(PR_DI, OUTPUT);
  pinMode(PR_DQ, INPUT_PULLUP);

  digitalWrite(PR_CAS,HIGH);
  digitalWrite(PR_RAS,HIGH);
  digitalWrite(PR_W,  HIGH);

  digitalWrite(PR_DI, LOW);
  Serial.flush();
  
  SetAddressByte(0);
}



uint8_t ReadBit( uint16_t address ) {

  byte ca  = (byte)(address >> 8 );
  byte ra  = (byte)(address & 0xFF );

  SetAddressByte(ra);
  digitalWrite(PR_RAS,LOW);
  
  SetAddressByte(ca);
  digitalWrite(PR_CAS,LOW);

  //delayMicroseconds(1);
    
  auto ret = digitalRead( PR_DQ );  

  digitalWrite(PR_CAS,HIGH);
  digitalWrite(PR_RAS,HIGH);

  return ret;
}

uint8_t WriteBit( uint32_t address, uint8_t bitVal ) {

  // swap around to force refresh every 2 ms
  byte ca  = (byte)(address >> 8 );
  byte ra  = (byte)(address & 0xFF );

  SetAddressByte(ra);
  digitalWrite(PR_RAS, LOW);
  digitalWrite(PR_W,   LOW);
  digitalWrite(PR_DI,  bitVal);

  SetAddressByte(ca);
  digitalWrite(PR_CAS,LOW);

  //delayMicroseconds(1);
  
  digitalWrite(PR_W,   HIGH);
  digitalWrite(PR_CAS, HIGH);
  digitalWrite(PR_RAS, HIGH);
}


void ReportTestResult( String test, uint32_t errs) {

  Serial.print("Executed test: ");
  Serial.print(test);
  if ( errs ) {
    Serial.print(" Failed : ");
    Serial.print(errs);
    Serial.println(" errors ");
  } else {
    Serial.println(" OK ");    
  }
  Serial.flush();
}


static void Fill(byte bt, uint32_t first, uint32_t last, uint32_t skip) {

  for ( uint32_t addr=first; addr < last; addr+= skip ) {
    WriteBit( addr, bt );
  }
}

static uint32_t Check(byte bt, uint32_t first, uint32_t last, uint32_t skip) {
  
  uint32_t errs = 0;
  for ( uint32_t addr=first; addr < last; addr+= skip ) {
    if ( ReadBit(addr) != bt ) {
      errs++;
    }
  }
  return errs;
}


static char Pattern[] = { 'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','0','1','2','3','4','5','6','7','8','9','!','@','#','$','%','^'};

static void FillPat( uint32_t first, uint32_t last) {

  byte bt = 0;
  int pi = 0;
  
  for ( uint32_t addr=first; addr < last; addr++ ) {
    WriteBit( addr, bt & 0x01 );
    if ( (addr % 8) == 0 ) {
      bt = Pattern[ pi % sizeof(Pattern) ];
      pi++;   
    } else {
      bt = bt >> 1;
    }
  }
}

static uint32_t CheckPat(uint32_t first, uint32_t last) {
  
  uint32_t errs = 0;
  byte bt = 0;
  int pi = 0;

  for ( uint32_t addr=first; addr < last; addr++ ) {
    if ( ReadBit(addr) != (bt & 0x01) ) {
      errs++;
    }
    if ( (addr % 8) == 0 ) {
      bt = Pattern[ pi % sizeof(Pattern) ];
      pi++;   
    } else {
      bt = bt >> 1;
    }
  }
  return errs;
}


void loop() {

  uint32_t last_addr =0xFFFF;
  
  uint32_t errs1;
  uint32_t errs2;
  uint32_t errs3;
  uint32_t errs4;
  uint32_t errs5;
  uint32_t errs6;

  noInterrupts(); 
  Fill(0,0,last_addr,1); 
  errs1 = Check(0,0,last_addr,1);

  Fill(1,0,last_addr,1); 
  errs2 = Check(1,0,last_addr,1);

  Fill(0,0,last_addr, 2); 
  errs3 = Check(0,0,last_addr,2) + Check(1,1,last_addr,2);  

  Fill(1,0,last_addr, 2); 
  errs4 = Check(1,0,last_addr,1);  

  Fill(0,0,last_addr,1); 
  Fill(1,0,last_addr, 11); 
  errs5 = Check(1,0,last_addr,11);  

  FillPat(0,0xFFFF); 
  errs6 = CheckPat(0,0xFFFF);  

  interrupts();
  
  ReportTestResult(" 1", errs1); 
  ReportTestResult(" 2", errs2); 
  ReportTestResult(" 3", errs3); 
  ReportTestResult(" 4", errs4); 
  ReportTestResult(" 5", errs4); 
  ReportTestResult(" 6", errs6); 

  Fill(1,0,last_addr, 1); 
  Fill(0,0,last_addr, 2);
   
  // check retention time
  long totdelay = 5;
  for ( auto l = 0; l< 100; l++ ) {
	  
    delay(totdelay);
	
    auto errs7 = Check(0,0,last_addr,2) + Check(1,1,last_addr,2);
	
    Serial.print("Check DRAM retention errors after a delay of " );
    Serial.print(totdelay);
    Serial.print(" ms: " );

	if ( (errs7 > 0)  && (totdelay <= 20) ) {
		Serial.print(" failed fot : ");
		Serial.print(errs7);
		Serial.println(" errors ");
	} else {
		Serial.println(" OK ");    
	}
	  Serial.flush();

	
    totdelay *= 2;
  }
}
