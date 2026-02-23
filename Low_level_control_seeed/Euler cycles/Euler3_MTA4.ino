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


/* User-defined parameters */
const int8_t motor[] ={2,3,5,6,9,10,11,12};      // motor pins on Arduino
const int T_transition = 450;    // total transition time constant in ms.
const int T_unspool = 50;        // motor unspooling time constant in ms.
int8_t cycle1[] = {7,9,6,9,12,9,14,9,3,9,10,9,8,9,15,9,11,9,5,9,4,9,16,9,1,9,13,9,2,9,7,2,6,2,12,2,14,2,3,2,10,2,8,2,15,2,11,2,5,2,4,2,16,2,1,2,13,2,7,13,6,13,12,13,14,13,3,13,10,13,8,13,15,13,11,13,5,13,4,13,16,13};
int8_t cycle2[] = {1,13,7,1,6,1,12,1,14,1,3,1,10,1,8,1,15,1,11,1,5,1,4,1,16,1,7,16,6,16,12,16,14,16,3,16,10,16,8,16,15,16,11,16,5,16,4,16,7,4,6,4,12,4,14,4,3,4,10,4,8,4,15,4,11,4,5,4,7,5,6,5,12,5,14,5,3,5,10,5,8,5,15};
int8_t cycle3[] = {5,11,5,7,11,6,11,12,11,14,11,3,11,10,11,8,11,15,11,7,15,6,15,12,15,14,15,3,15,10,15,8,15,7,8,6,8,12,8,14,8,3,8,10,8,7,10,6,10,12,10,14,10,3,10,7,3,6,3,12,3,14,3,7,14,6,14,12,14,7,12,6,12,7,6,7};
*/


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

/* Setup function for communication and state definition (runs once). */
void setup() {
  Serial.begin(9600); // baud rate for serial monitor communication (bps).
  define_states();
  delay(3000);
      cycle_through_states(cycle1, cycle1_size);
        cycle_through_states(cycle2, cycle2_size);    
      cycle_through_states(cycle3, cycle3_size);
}


/* Anything placed inside the loop function will cycle continously forever
 * unless interrupts are added. 
 */
void loop() {

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
