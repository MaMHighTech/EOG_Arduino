int fs=100;     // Sampling rate (Hz) Don't change!
float Ts = (float)1000/fs;   //Sampling Period (ms)
//ADC COnfiguration
const byte adc_in = A2; // ADC input pin.(For example A2 for Arduino Uno, 27 for ESP32)
const byte adc_bits = 10; // The resolution of your MCU's ADC
const byte default_bits = 10; // Don't change!
const float vref = 5; // Reference voltage of your MCU's ADC (V)
const float default_vref = 5 ;// Default reference voltage of the Arduino Uno (V) Don't Change
const float adc_scale = pow(2,default_bits-adc_bits)*vref/default_vref; // Scales the input signal
const float eog_offset = 1.40; // DC offset of the Mam Sense Board EOG output. (V)
const float sig_offset = round(pow(2,default_bits)*eog_offset/default_vref);

unsigned long counter = 0;
short data_buff[200];

//Templates for the eye movements.
short look_left[60] = {-6,-3,4,16,32,55,80,107,131,153,171,180,187,184,181,172,164,155,148,141,136,132,130,126,124,122,120,118,114,111,107,104,101,98,95,91,89,87,84,81,78,76,74,72,70,68,66,63,62,60,58,58,55,53,52,52,50,49,47,46};
short look_right[60] = {7,4,-3,-14,-29,-49,-69,-93,-115,-132,-144,-154,-156,-158,-156,-152,-146,-141,-137,-133,-129,-126,-123,-120,-117,-113,-110,-105,-103,-101,-98,-94,-93,-90,-88,-86,-84,-80,-79,-77,-75,-72,-71,-67,-66,-64,-62,-60,-57,-56,-54,-52,-51,-50,-49,-47,-46,-45,-43};
short left_right[60] = {2,5,10,18,30,46,62,78,90,102,107,113,110,109,104,97,92,86,84,78,75,71,67,62,58,50,41,30,15,-1,-15,-29,-37,-45,-48,-49,-48,-47,-43,-41,-37,-36,-34,-34,-30,-33,-30,-31,-29,-29,-28,-28,-26,-27,-26,-25,-24,-23,-23,-23};
short right_left[60] = {3,-2,-9,-19,-32,-49,-61,-77,-89,-99,-105,-108,-108,-107,-103,-98,-94,-90,-85,-81,-76,-69,-61,-49,-34,-19,-4,11,24,34,40,45,46,47,46,45,44,43,42,43,41,42,39,38,36,36,35,34,33,32,32,31,30,28,28,28,26,26,24,24};
short blink_left[60] = {-12,-6,16,32,53,69,79,82,79,66,52,38,23,13,5,1,-2,-3,-2,-3,-3,-3,-3,-2,-2,-2,-2,-2,-2,-2,-5,-6,-5,-4,-4,-4,-4,-3,-4,-4,-4,-3,-4,-4,-4,-4,-4,-4,-3,0,0,0,0,0,0,0,0,0};
short blink_right[60] = {6,2,-3,-13,-25,-40,-52,-65,-71,-74,-71,-65,-56,-44,-34,-26,-17,-12,-7,-4,-3,-2,-2,-2,-1,-1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,2,1,1,0,0,0,0,0,0,0,0};

float pos_pk_thrs=50;               // If your horizontal eye movements are not detected 
float pos_pk = pos_pk_thrs;         // lower these thresholds.
float neg_pk_thrs=-50;              //
float neg_pk = neg_pk_thrs;
int pos_pk_n=0;             // Designate the indices where peaks are detected
int neg_pk_n=0;             //

float left_blink_thrs = 200;        //  If horizontal movements are detected as blink increase
float right_blink_thrs = -200;      //  these thresholds.

boolean pos_pk_reached;
boolean neg_pk_reached;

float error=0;
float blink_err=0;
float quick_mov_err=0;
float err_left_thrs=6;         // If your horizontal eye movements are not detected increase the error thresholds.
float err_right_thrs=6;        // If other movements are detected as horizontal movements decrease the error thresholds.
float err_coef=2;              // If fast eye movements are detected as blink increase the coefficient(Must be in range 1-3).
  void setup() {
    Serial.begin(9600);
  }
  
  void loop(){
             
      delay(Ts);    
      data_buff[counter%200] = round( analogRead(adc_in)*adc_scale-sig_offset);
    // Serial.println(data_buff[counter%200]);
      
// This block of code checks if the peak threshold is reached and records the value and its index
     if(data_buff[counter%200]>pos_pk){
        pos_pk_reached=true;
        pos_pk_n = counter%200;
        pos_pk = data_buff[counter%200];
      }

// Similarly this code checks if the negative peak threshold is reached and records the value and its index
      if(data_buff[counter%200]<neg_pk){
        neg_pk_reached=true;
        neg_pk_n = counter%200;
        neg_pk = data_buff[counter%200];
      }

      // If positive peak is reached wait for 60 more samples then start applying least mean square method.
      if(counter%200==(60+pos_pk_n)%200 &&  pos_pk_reached ){
         pos_pk_reached =false;
         error=0;
         blink_err=0;
         quick_mov_err=0;

       if((pos_pk - data_buff[(pos_pk_n+185)%200]) >= left_blink_thrs){  // Since blink signals are stronger than other movements if the blink threshold is reached
        Serial.println("Left Blink");                                    // there's no need to compare the signal with the templates.
        //Related drone command here...
       }
         
       else if((pos_pk - data_buff[(pos_pk_n+188)%200]) >= pos_pk_thrs){
          
          for(int i=0;i<60;i++){ 
            error += sq(look_left[i] - data_buff[(i+pos_pk_n+188)%200]*187/pos_pk);       // Compute the similarity between recorded signal and 'look left' movement.
          }
          
          for(int i=7;i<60;i++){ 
           blink_err += sq(blink_left[i] - data_buff[(i+pos_pk_n+193)%200]*82/pos_pk);    // Compute the similarity between recorded signal and 'blink' movement.
          }

          for(int i=11;i<60;i++){ 
            quick_mov_err += sq(left_right[i] - (data_buff[(i+pos_pk_n+189)%200])*113/pos_pk); // Compute the similarity between recorded signal and 'quick left' movement.
          } 
          
          blink_err = sqrt(blink_err)/53;             // Normalize the error rates.
          error = sqrt(error)/60;                     //
          quick_mov_err = sqrt(quick_mov_err)/49;
          
          if(blink_err < error && blink_err*err_coef<quick_mov_err && blink_err<err_left_thrs){    // Compare the error rates and send the corresponding command.
            Serial.println("Left Blink");                                                          // Blink error is multiplied by error coefficient for error protection.
            //Related drone command here...
          } else if(error<blink_err && error<quick_mov_err && error<err_left_thrs){
            Serial.println("Looked Left");
            //Related drone command here...
          } else if(quick_mov_err<error && quick_mov_err<blink_err*err_coef*0.75 && quick_mov_err<err_left_thrs){
            Serial.println("Left then middle");
            //Related drone command here...
          }
       
       }
      neg_pk=neg_pk_thrs;
      neg_pk_reached=false;
      pos_pk=pos_pk_thrs;
      }

      // This part of the code is very similar to the left eye movements only for the right eye.
      if(counter%200==(60+neg_pk_n)%200 && neg_pk_reached){
        blink_err=0;
        neg_pk_reached = false;
        error=0;
        quick_mov_err=0;
        if( (neg_pk - data_buff[(neg_pk_n+185)%200]) <= right_blink_thrs){
          Serial.println("Right Blink");
          //Related drone command here...
        }
        
        else if( (neg_pk - data_buff[(neg_pk_n+188)%200]) <= neg_pk_thrs){

          for(int i=0;i<60;i++){
            error += sq(look_right[i] + data_buff[(i+neg_pk_n+187)%200]*158/neg_pk);
          }
          for(int i=8;i<60;i++){ 
            blink_err += sq(blink_right[i] + data_buff[(i+neg_pk_n+190)%200]*75/neg_pk);
          }
          for(int i=12;i<60;i++){ 
            quick_mov_err += sq(right_left[i] + (data_buff[(i+neg_pk_n+188)%200])*108/neg_pk);
          } 
          
           error = sqrt(error)/60;
           blink_err = sqrt(blink_err)/60;
           quick_mov_err = sqrt(quick_mov_err)/60;


           if(blink_err < error && blink_err*err_coef<quick_mov_err && blink_err<err_right_thrs){
             Serial.println("Right Blink");
             //Related drone command here...
           } else if(error<blink_err && error<quick_mov_err && error<err_right_thrs){
             Serial.println("Looked Right");
             //Related drone command here...
           } else if (quick_mov_err<error && quick_mov_err<blink_err*err_coef*0.75 && quick_mov_err<err_right_thrs){
             Serial.println("Right then middle");
             //Related drone command here...
           }
        }
        pos_pk=pos_pk_thrs;
        neg_pk=neg_pk_thrs;
        pos_pk_reached=false;
      }
    counter++;
  }
