
void setup() { 
//Initialise serial port
Serial.begin(9600);
} 
void loop() { 
//Iterate over one cycle of the sine wave 
for (double x=-PI; x<PI; x += 0.025)
 //Write the value of the sin function to the serial port
Serial.println(sin(x));

}
