/* BasicMSoRo.ino */
/* Author: Caitlin Freeman
 * Last updated: 1/27/22
 * This code enables gait-based control of a multi-limbed robot with
 * with bang-bang (i.e., on/off) actuation. The user will use the 
 * user-defined parameter block to define
 *   1. Arduino pin numbers
 *   2. time constants
 *   3. fundamental locomotion gaits
 * and can control gait switching by manipulating the loop block. 
 * Future work:
 *   1. More advanced gait switching by recognizing common nodes, 
 *      including automatic gait switching handling
 *   2. Communication with MATLAB for feedback control
 *   3. Automatic gait permutation handling
 */

 /* Include Libraries*/
#include <ArduinoBLE.h>

/* User-defined parameters */
//const int8_t motor[] ={2,3,5,6,9,10,11,12};      // motor pins on Arduino
const int8_t motor[] ={1,0,3,2,5,6,10,9};      // motor pins on Arduino Xiao changing motor polarity from default above for some motors
const int T_transition = 450;    // total transition time constant in ms.
const int T_unspool = 50;        // motor unspooling time constant in ms.

// Euler cycle 1 
int8_t cycle1[] = {16,4,6,4,14,4,12,4,1,4,13,4,5,4,9,4,15,4,11,4,10,4,3,4,7,4,2,4,8,4,16,8,6,8,14,8,12,8,1,8,13,8,5,8,9,8,15,8,11,8,10,8,3,8,7,8,2,8,16,2,6,2,14,2,12,2,1,2,13,2,5,2,9,2,15,2,11,2,10,2,3,2,7,2};
int8_t cycle2[] = {16,7,6,7,14,7,12,7,1,7,13,7,5,7,9,7,15,7,11,7,10,7,3,7,16,3,6,3,14,3,12,3,1,3,13,3,5,3,9,3,15,3,11,3,10,3,16,10,6,10,14,10,12,10,1,10,13,10,5,10,9,10,15,10,11,10,16,11,6,11,14,11,12,11,1,11,13,11,5,11,9};
int8_t cycle3[] = {11,15,11,16,15,6,15,14,15,12,15,1,15,13,15,5,15,9,15,16,9,6,9,14,9,12,9,1,9,13,9,5,9,16,5,6,5,14,5,12,5,1,5,13,5,16,13,6,13,14,13,12,13,1,13,16,1,6,1,14,1,12,1,16,12,6,12,14,12,16,14,6,14,16,6,16}; 

// Euler cycle 2
int8_t cycle4[] = {8,7,2,7,3,7,13,7,1,7,6,7,10,7,9,7,15,7,14,7,16,7,5,7,11,7,12,7,4,7,8,4,2,4,3,4,13,4,1,4,6,4,10,4,9,4,15,4,14,4,16,4,5,4,11,4,12,4,8,12,2,12,3,12,13,12,1,12,6,12,10,12,9,12,15,12,14,12,16,12,5,12,11};
int8_t cycle5[] = {12,8,11,2,11,3,11,13,11,1,11,6,11,10,11,9,11,15,11,14,11,16,11,5,11,8,5,2,5,3,5,13,5,1,5,6,5,10,5,9,5,15,5,14,5,16,5,8,16,2,16,3,16,13,16,1,16,6,16,10,16,9,16,15,16,14,16,8,14,2,14,3,14,13,14,1,14};
int8_t cycle6[] = {6,14,10,14,9,14,15,14,8,15,2,15,3,15,13,15,1,15,6,15,10,15,9,15,8,9,2,9,3,9,13,9,1,9,6,9,10,9,8,10,2,10,3,10,13,10,1,10,6,10,8,6,2,6,3,6,13,6,1,6,8,1,2,1,3,1,13,1,8,13,2,13,3,13,8,3,2,3,8,2,8};

// Euler cycle 3
int8_t cycle7[] = {7,9,6,9,12,9,14,9,3,9,10,9,8,9,15,9,11,9,5,9,4,9,16,9,1,9,13,9,2,9,7,2,6,2,12,2,14,2,3,2,10,2,8,2,15,2,11,2,5,2,4,2,16,2,1,2,13,2,7,13,6,13,12,13,14,13,3,13,10,13,8,13,15,13,11,13,5,13,4,13,16,13};
int8_t cycle8[] = {1,13,7,1,6,1,12,1,14,1,3,1,10,1,8,1,15,1,11,1,5,1,4,1,16,1,7,16,6,16,12,16,14,16,3,16,10,16,8,16,15,16,11,16,5,16,4,16,7,4,6,4,12,4,14,4,3,4,10,4,8,4,15,4,11,4,5,4,7,5,6,5,12,5,14,5,3,5,10,5,8,5,15};
int8_t cycle9[] = {5,11,5,7,11,6,11,12,11,14,11,3,11,10,11,8,11,15,11,7,15,6,15,12,15,14,15,3,15,10,15,8,15,7,8,6,8,12,8,14,8,3,8,10,8,7,10,6,10,12,10,14,10,3,10,7,3,6,3,12,3,14,3,7,14,6,14,12,14,7,12,6,12,7,6,7};

// Euler cycle 4
int8_t cycle10[] = {4,2,5,2,13,2,10,2,14,2,8,2,6,2,12,2,16,2,3,2,15,2,9,2,7,2,1,2,11,2,4,11,5,11,13,11,10,11,14,11,8,11,6,11,12,11,16,11,3,11,15,11,9,11,7,11,1,11,4,1,5,1,13,1,10,1,14,1,8,1,6,1,12,1,16,1,3,1,15,1,9,1,7,1,4,7,5};
int8_t cycle11[] = {7,13,7,10,7,14,7,8,7,6,7,12,7,16,7,3,7,15,7,9,7,4,9,5,9,13,9,10,9,14,9,8,9,6,9,12,9,16,9,3,9,15,9,4,15,5,15,13,15,10,15,14,15,8,15,6,15,12,15,16,15,3,15,4,3,5,3,13,3,10,3,14,3,8,3,6,3,12,3,16,3};
int8_t cycle12[] = {4,16,5,16,13,16,10,16,14,16,8,16,6,16,12,16,4,12,5,12,13,12,10,12,14,12,8,12,6,12,4,6,5,6,13,6,10,6,14,6,8,6,4,8,5,8,13,8,10,8,14,8,4,14,5,14,13,14,10,14,4,10,5,10,13,10,4,13,5,13,4,5,4};

// Euler cycle 5
int8_t cycle13[] = {12,15,11,15,16,15,6,15,1,15,9,15,13,15,4,15,10,15,3,15,8,15,7,15,14,15,2,15,5,15,12,5,11,5,16,5,6,5,1,5,9,5,13,5,4,5,10,5,3,5,8,5,7,5,14,5,2,5,12,2,11,2,16,2,6,2,1,2,9,2,13,2,4,2,10,2,3,2,8,2};
int8_t cycle14[] = {7,2,14,2,12,14,11,14,16,14,6,14,1,14,9,14,13,14,4,14,10,14,3,14,8,14,7,14,12,7,11,7,16,7,6,7,1,7,9,7,13,7,4,7,10,7,3,7,8,7,12,8,11,8,16,8,6,8,1,8,9,8,13,8,4,8,10,8,3,8,12,3,11,3,16,3,6,3,1,3,9,3,13};
int8_t cycle15[] = {3,4,3,10,3,12,10,11,10,16,10,6,10,1,10,9,10,13,10,4,10,12,4,11,4,16,4,6,4,1,4,9,4,13,4,12,13,11,13,16,13,6,13,1,13,9,13,12,9,11,9,16,9,6,9,1,9,12,1,11,1,16,1,6,1,12,6,11,6,16,6,12,16,11,16,12,11,12};

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

const int8_t cycle1_size = sizeof(cycle1);
const int8_t cycle2_size = sizeof(cycle2);
const int8_t cycle3_size = sizeof(cycle3);

const int8_t cycle4_size = sizeof(cycle4);
const int8_t cycle5_size = sizeof(cycle5);
const int8_t cycle6_size = sizeof(cycle6);

const int8_t cycle7_size = sizeof(cycle7);
const int8_t cycle8_size = sizeof(cycle8);
const int8_t cycle9_size = sizeof(cycle9);

const int8_t cycle10_size = sizeof(cycle10);
const int8_t cycle11_size = sizeof(cycle11);
const int8_t cycle12_size = sizeof(cycle12);

const int8_t cycle13_size = sizeof(cycle13);
const int8_t cycle14_size = sizeof(cycle14);
const int8_t cycle15_size = sizeof(cycle15);

/* Initializing Bluetooth Low Energy*/
#define BLE_UUID                  "2C2FC88C4-F244-5A80-21F1-TE0224E80658"

BLEService MsoroService( BLE_UUID );
BLEStringCharacteristic MsoroCharacteristic( BLE_UUID, BLERead | BLEWrite, 50 );


/* Setup function for communication and state definition (runs once). */
void setup() {
  Serial.begin(9600); // baud rate for serial monitor communication (bps).
  define_states();
  delay(3000);

  BLE.begin();
  
  // set advertised local name and service UUID:
  BLE.setDeviceName( "Seeeduino Sense Sensor" );
  BLE.setLocalName( "Seeeduino Sense Sensor" );
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


/* Anything placed inside the loop function will cycle continously forever
 * unless interrupts are added. 
 */
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

        if (cmd == "Euler1")
        {
          cycle_through_states(cycle1, cycle1_size);
          cycle_through_states(cycle2, cycle2_size);    
          cycle_through_states(cycle3, cycle3_size);
        }
        if (cmd == "Euler2")
        {
          cycle_through_states(cycle4, cycle4_size);
          cycle_through_states(cycle5, cycle5_size);    
          cycle_through_states(cycle6, cycle6_size);
        }
        if (cmd == "Euler3")
        {
          cycle_through_states(cycle7, cycle7_size);
          cycle_through_states(cycle8, cycle8_size);    
          cycle_through_states(cycle9, cycle9_size);
        }
        if (cmd == "Euler4")
        {
          cycle_through_states(cycle10, cycle10_size);
          cycle_through_states(cycle11, cycle11_size);    
          cycle_through_states(cycle12, cycle12_size);
        }
        if (cmd == "Euler5")
        {
          cycle_through_states(cycle13, cycle13_size);
          cycle_through_states(cycle14, cycle14_size);    
          cycle_through_states(cycle15, cycle15_size);
        }
      }
    }
  }


//cycle_through_states(cycleB, cycleB_size);
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

