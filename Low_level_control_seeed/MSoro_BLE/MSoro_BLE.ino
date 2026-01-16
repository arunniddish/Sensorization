/* MSoRo_BLE.ino */ 
/* Last updated: 9/2/22
 * This code enables gait-based control of a multi-limbed robot with
 * with bang-bang (i.e., on/off) actuation. The user will use the
 * user-defined parameter block to define
 *   1. Arduino pin numbers
 *   2. time constants
 * and can control gait definition and gait execution via Serial communication.
 *
 * Gait definitions are performed via the serial monitor (SM) command as follows:
 *   Example gait: A = [2,3,5,9]
 *   SM: "define A 2 3 5 9 end " - Leave a space after the 'end'
 *  
 * Gait executions are performed as follows:
 *   Example: run gait A for 12 cycles
 *   SM: "start A 12 end " - Leave a space after the 'end'
 *  
 */

 /* Include Libraries*/
#include <ArduinoBLE.h>

/* User-defined parameters */
//const int8_t motor[] ={2,3,5,6,9,10,11,12};      // motor pins on Arduino
const int8_t motor[] ={1,0,3,2,5,6,10,9};      // motor pins on Arduino Xiao changing motor polarity from default above for some motors
const int T_transition = 450;    // total transition time constant in ms.
const int T_unspool = 50;        // motor unspooling time constant in ms.
bool display_states = false;     // flag to print all robot states


/* Initializations (should not change) */
const int8_t number_of_motors = sizeof(motor)/ 2;    // bang-bang control
/* If more robot states are desired (e.g., including intermediate actuation
 * states), the exponent base here can be adjusted.
 */
const int number_of_states = pow(2, number_of_motors);
/* Initialize matrix to store all actuation permutations (robot states). */
int state_matrix[number_of_motors][number_of_states] = { };  
/* Use booleans to avoid excessive / unwanted motor unspooling. */
bool just_curled[number_of_motors] = { };
bool just_relaxed[number_of_motors] = { };
/* Serial communication variables */
String cmd;
String ser_read_string;
String BLE_read_string;
/* Gait information variables */
int8_t cycle[20][20];     // matrix to store all gait definitions (max 20 gaits of length 20)
int8_t cycle_gait[20];
String gait_name;
String gait_value;
String Msoro_cmd = "";
int gait_value_int;
int size_gait[20];
String gaits = "ABCDEFGHIJKLMNOP";
int gait_index;
int space_index;

/* Initializing Bluetooth Low Energy*/
#define BLE_UUID                  "2C2FC88C4-F244-5A80-21F1-EE0224E80658"

BLEService MsoroService( BLE_UUID );
BLEStringCharacteristic MsoroCharacteristic( BLE_UUID, BLERead | BLEWrite, 50 );

/* Setup function for communication and state definition (runs once). */
void setup() 
{
  Serial.begin(9600); // baud rate for serial monitor communication (bps).
  define_states();
  delay(3000);
   // while ( !Serial );

  BLE.begin();
  
  // set advertised local name and service UUID:
  BLE.setDeviceName( "Seeeduino Sense C" );
  BLE.setLocalName( "Seeeduino Sense C" );
  BLE.setAdvertisedService( MsoroService );

  // BLE add characteristics
  MsoroService.addCharacteristic( MsoroCharacteristic );

  // add service
  BLE.addService( MsoroService );

  // set the initial value for the characeristic:
  MsoroCharacteristic.writeValue( Msoro_cmd );

  // start advertising
  BLE.advertise();
}

void loop()
{
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  if ( central )
  {
    Serial.print( "Connected to central: " );
    Serial.println( central.address() );

    while ( central.connected() )
    {
      if ( MsoroCharacteristic.written() )
      {
        Msoro_cmd = MsoroCharacteristic.value();
        cmd = BLE_read();
        if (cmd == "define")
        {
          define_cycle();
        }
        if (cmd == "start")
        {
          start_cycle();
        }

      }
    }
  }
}

/* This  function defines a matrix of robot states. Basically, it labels each
 * state with a binary number that corresponds to which motors are on or off.
 * 1 = on, 0 = off 
 */
void define_states() {
  for (int k=0; k<= number_of_states-1; k++) {
    int spacing=1;
    for (int j=number_of_motors-1; j>=0; j--) {
      if (state_matrix[j][k]==0 && k+spacing<=number_of_states-1){
        state_matrix[j][k+spacing]=1;
      }
      spacing = spacing*2;
    }
  }
  if (display_states == true){
    for (int m=0; m<=number_of_states-1; m++) {
      /* This part is optional as it just prints the binary value of each 
       *  state number (helpful for debugging).
       */
      Serial.print("State ");
      Serial.print(m+1);
      Serial.print(" = ");
      for (int n=0; n<=number_of_motors-1;n++){
        Serial.print(state_matrix[n][m]);
      }
      Serial.println(" ");
    }
  }
}

/* This  function controls the motor based on the gait cycle (i.e., array of
 * state numbers) provided.
 */
void cycle_through_states (int8_t *cycle, int8_t cycle_size) {
  for (int i=0; i<cycle_size;i++) {
    unsigned long transition_start = millis();
    Serial.print("State ");
    Serial.print(cycle[i]);
    Serial.print(": ");
    for (int j=0; j<=3; j++) {
      Serial.print(state_matrix[j][cycle[i]-1]);
      if (state_matrix[j][cycle[i]-1] == 0 && just_relaxed[j]==false) {
        digitalWrite(motor[2*j], LOW);
        digitalWrite(motor[2*j+1], HIGH);
        just_relaxed[j] = true;
        just_curled[j] = false;
      }
      else if (state_matrix[j][cycle[i]-1] == 1)  {
        digitalWrite(motor[2*j],HIGH);
        digitalWrite(motor[2*j+1], LOW);
        just_relaxed[j] = false;
        just_curled[j] = true;
      }
      if (j==3) {
        delay(T_unspool);
        for (int k=0; k<=3; k++) {
          if (state_matrix[k][cycle[i]-1] == 0) {
            digitalWrite(motor[2*k+1], LOW);
          }
          if (k==3){
            while (millis() -transition_start<= T_transition-1) {
              delay(1);
            }
            Serial.println("");
          }
        }
      }
    }
  }
  for (int j=0; j<=3; j++) {
    digitalWrite(motor[2*j],LOW);
    digitalWrite(motor[2*j+1],LOW);
  }
}


String BLE_read()
{
  space_index = Msoro_cmd.indexOf(' ');
  BLE_read_string = Msoro_cmd.substring(0,space_index);
  Msoro_cmd = Msoro_cmd.substring(space_index+1);
  return BLE_read_string;
}

/* This function takes input for defining the gait types. 
 *  Example A = [2,3,5,9];
 *  "define A 2 3 5 9 end " - Leave a space after the 'end'
 */
void define_cycle(){
  gait_name = BLE_read();
  gait_index = gaits.indexOf(gait_name);
  Serial.print("The index is ");
  Serial.println(gait_index);
  gait_value = "exist"; /* Initially the gait value exists which is sent from MATLAB*/
  int i = 0;
  while(space_index != -1) {
    gait_value = BLE_read();
    gait_value_int = gait_value.toInt();
    if(gait_value != "end" && gait_value_int != 0 ) {
      cycle[gait_index][i] = (int8_t)gait_value_int;
      Serial.println(cycle[gait_index][i]);
      i = i+1;  
    }  
    size_gait[gait_index] = i;
  }
       
  Serial.println("#Defined"); /* This is printed so that Matlab can read it from Serial port for acknowledging that gait defining process 
                                     has been completed*/   
}


/*
 * For the given definition of gaits type, this function performs the execution of the gait and number of times it needs to be performed.
 * Example: "start A 12" - Performs gait A for 12 times. Do not leave a space after the '12'.
 */
void start_cycle()
{
  gait_name = BLE_read();
  gait_index = gaits.indexOf(gait_name);
  for(int j = 0;j<size_gait[gait_index];j++){
     cycle_gait[j] = cycle[gait_index][j];
  } 
    
  int num_cycles = BLE_read().toInt();
  //while (Serial.available()>0){
  //  cmd = serial_read();
  //  if (cmd == "end"){}
  //}
  for (int k=0; k<num_cycles ; k++){
    //if (Serial.available() > 4) {
      if (MsoroCharacteristic.written()) {
      cmd = BLE_read();
      cmd.trim();
      if (cmd == "stop" ){
        Serial.print ("#Stopped Gait ");
        Serial.println(gaits.charAt(gait_index));
        break;
      }
    }
    cycle_through_states(cycle_gait, size_gait[gait_index]);
    Serial.print("#Completed ");
    Serial.print(k+1);
    Serial.print("/");
    Serial.print(num_cycles);
    Serial.println(" times");
  } 
  if (cmd != "stop" ) {
    Serial.print("#Completed Gait ");
    Serial.print(gaits.charAt(gait_index));
    Serial.print(" ");
    Serial.print(num_cycles);
    Serial.println(" times");
  }
}


