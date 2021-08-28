/* This is the controller for a line-following robot */

//------ Output Pin numbers -------

    //Left motor:
constexpr int INPUT_PIN_L   = 6;
constexpr int PIN_A_L       = 7;
constexpr int PIN_B_L       = 8;
    //Right motor:
constexpr int INPUT_PIN_R   = 3;
constexpr int PIN_A_R       = 4;
constexpr int PIN_B_R       = 5;
    //Analog inputs: [sensor]
constexpr int PIN_SENSOR_A  = A0;
constexpr int PIN_SENSOR_B  = A1;


//-- Motor control: 
constexpr int PWM_MAX           = 255;
constexpr uint32_t ON_INTERVAL  = 7000; //9000;
constexpr float INPUT_MIN       = 5.0;
constexpr int MOD_TRANSITION    = 120.0;


//-- Proportional - Derivative parameters:
constexpr float EXP_SMOOTH  = 0.4;
     //Derivative filter:
constexpr float COEFF_DEV[] = {5, 8, 27, 48, 42, 0, -42, -48, -27, -8, -5}; 
constexpr float DENOM_DEV   =  512.0; 
constexpr int   N_STORED    = sizeof(COEFF_DEV)/sizeof( COEFF_DEV[0] );
    //2nd derivative filter:
constexpr float COEFF_DEV_2[N_STORED] = {15, 6, -1, -6, -9, -10, -9, -6, -1, 6, 15};
constexpr float DENOM_DEV_2 = 429.0 ;
     //Proportional filter
constexpr float COEFF_PRO[N_STORED] = {-4, -20, -20, 80, 280, 392, 280, 80, -20, -20, 4};
constexpr float DENOM_PRO   = 1024.0;
     //Integration of sensor [Quadrature]
constexpr float COEFF_INT[] = { 7, 12, 15, 16, 15, 12, 7 };
constexpr float DENOM_INT   = 14.0;
constexpr int   N_INT       = sizeof(COEFF_INT)/sizeof( COEFF_INT[0] ); 

     //Control coefficients:
constexpr float K_PROP      = 0.25; // 0.25;
constexpr float K_DEV       = 5.0; // 6.0;
constexpr float K_DEV_2     = 20.0; // 20.0;
constexpr float K_INT       = 0.0005;
// Anti wind-up limit:
constexpr float INT_LIMIT   = 100.0;
// Threshold to enable integration
constexpr float INT_ENABLE  = 0.1;


//-- Foward throttle parameters:
constexpr int   N_INITIAL       = 1000;
constexpr int   FOR_THROTTLE    = 150;
constexpr float DECAY           = 0.01;
constexpr float PERCENT_INITIAL = 0.9;
constexpr float MOTOR_BIAS      = 0.01;
// Threshold when throttle is reduced
const float     TURN_LIMIT      = 20000;


//-- Global variables:
float   past_average[N_STORED] = {0};
int     input_initial   = 0;
int     diff_initial    = 0;
float   forward         = 0;


//----- Motor control: --

//-- Class for pulse-frequency modulation:
class pulse {
  public:
    int freq_mod;

    //Constructor
    pulse( void ) {
     last_time = 0;
     freq_mod = 0;   
     off_interval = 0;
    }
    //Destructor
    ~pulse ( void ) {}

    void set_period( float input ) {
      if( input < INPUT_MIN ) {
        off_interval = ON_INTERVAL*( PWM_MAX/INPUT_MIN - 1 );
      } else {
        off_interval = ON_INTERVAL*( PWM_MAX/input - 1 );
      }      
    }

    void check_time( void ) {      
      float interval = micros() - last_time;
      
      if( interval <= (ON_INTERVAL + off_interval) ) {
        if( interval <= ON_INTERVAL ) {
          freq_mod = PWM_MAX;
        } else {
          freq_mod = 0;
        }  
      } else {
        last_time = micros();
      }
    }
    
  private:
    uint32_t last_time;  
    uint32_t off_interval;
};


//-- Function to set motor throttle and direction:
void set_motor( int input, int pin_input, int pin_a, int pin_b ) {
 
  analogWrite( pin_input, abs(input) );

  if( input > 0 ) {
    digitalWrite( pin_b, HIGH );
    digitalWrite( pin_a, LOW );
  } else { 
    digitalWrite( pin_b, LOW );
    digitalWrite( pin_a, HIGH );
  }
}

//-- Fuction to identity sign:
int sign( int input ) {
  if( input < 0 ) {
    return -1;
  } else {
    return 1;
  }
}

//------> Global pulse classes <------
pulse wave_L, wave_R;

//Note: Turn > 0 implies right turn (positive X axis)

//-- Function to combine forward and directional inputs:
void set_speed( int forward, int turn ) {
  
  int input_L = constrain(  (1.0 + MOTOR_BIAS)*(forward + turn ), -PWM_MAX, PWM_MAX );
  int input_R = constrain(  (1.0 - MOTOR_BIAS)*(forward - turn), -PWM_MAX, PWM_MAX );

  wave_L.set_period( abs(input_L) );
  wave_R.set_period( abs(input_R) );

  if( abs(input_L) < INPUT_MIN ) { 
    input_L = 0;
  } 
  else if( abs(input_L) < MOD_TRANSITION ) { 
    input_L = sign(input_L)*wave_L.freq_mod;
  }

  if( abs(input_R) < INPUT_MIN ) {
    input_R = 0;  
  } else if( abs(input_R) < MOD_TRANSITION ) {
    input_R = sign(input_R)*wave_R.freq_mod;
  }

  set_motor( input_L, INPUT_PIN_L, PIN_A_L, PIN_B_L );  
  set_motor( input_R, INPUT_PIN_R, PIN_A_R, PIN_B_R ); 
}

//----- Proportional-Derivative components: --

//-- Function to output difference of sensor inputs:
int sensor_diff( void ) {
  return ( analogRead(PIN_SENSOR_A) - analogRead(PIN_SENSOR_B) );
}

//-- Function to get exponential average of sensor difference:
float exp_average_diff( void ) {
  static float val = 0;
  val += ( (sensor_diff() - diff_initial) - val )*EXP_SMOOTH; 
  return val;
}

//-- Function to store averaged values in array:
void store_average( void ) {
  for( int index = (N_STORED-1) ; index > 0 ; index -= 1 ) {
    past_average[index] = past_average[index - 1];
  }
  past_average[0] = exp_average_diff();
}

//-- Function to calculate time derivative of difference [averaged values]:
float smooth_deriv( void ) {
  float deriv = 0;
  for( int index = 0; index < N_STORED ; index += 1 ) {
      deriv += past_average[index]*COEFF_DEV[index];
  }
  return deriv/DENOM_DEV;
}

//-- Function to calculate second time derivative of difference [averaged values]:
float smooth_deriv_2( void ) {
  float deriv_2 = 0;
  for( int index = 0; index < N_STORED ; index += 1 ) {
      deriv_2 += past_average[index]*COEFF_DEV_2[index];
  }
  return deriv_2/DENOM_DEV_2;
}

//-- Filter to remove noise from sensor difference [averaged values]:
float smooth_prop( void ) {
  float prop = 0; 
  for( int index = 0; index < N_STORED ; index += 1 ) {
      prop += past_average[index]*COEFF_PRO[index];
  }
  return prop/DENOM_PRO;
}

//-- Function to get quadrature (time integral) of the average sensor difference: [Note: can be dissabled through the enabled input] 
float quadrature( float enable ) {
    
  static float short_term_int  = 0;
  static float long_term_int = 0;
  static int count = 0;

  //- Checking if integration enabled
  if( enable > INT_ENABLE ) { 

      // Trapezoidal integration for immediate samples
    short_term_int += K_INT*( past_average[0] + past_average[1] )/2.0 ;
    count += 1;
          // Newton-cotes quadrature for previous samples [long term integral] 
    if( count == (N_INT-1) ) {
        count = 0;
        short_term_int = 0;
    
        float accum = 0;
        for( int index = 0; index < N_INT ; index += 1 ) {
          accum += past_average[index]*COEFF_INT[index];
        }
        long_term_int += K_INT*accum/DENOM_INT;
      }

        //- Anti Wind-up code: Limits integral from increasing beyond bounds 
      float integral = (short_term_int + long_term_int);

      if( abs(integral) > INT_LIMIT ) {
        long_term_int = sign(integral)*INT_LIMIT;
        return long_term_int;
      } 
      else {
        return integral;
      }
        //- Default if integration is dissabled
    } else {
      short_term_int = 0;
      long_term_int = 0;
      count = 0;
      return 0;
  }
}

//----- Sensor calibration and determining forward motion: --

//-- Average of sensor inputs:
float av_input( void ) {
  return  ( analogRead(PIN_SENSOR_A) + analogRead(PIN_SENSOR_B) )/2.0; 
}

//-- Blinking senquence for Pin 13 LED:
void blink_led( int count ) {
    for( int index = 0; index < count ; index += 1 ) {
    digitalWrite( LED_BUILTIN, HIGH );
    delay( 100 );
    digitalWrite( LED_BUILTIN, LOW );
    delay( 100 );
  }
}

//-- Function to generate calibration value for sensor average:
void initialize( void ) {

  blink_led( 10 );

 //- Calibrating sensor average:
  int64_t average = 0; 
  for( int index = 0; index < N_INITIAL ; index += 1 ) {
     average += av_input();
  }
  input_initial = average/N_INITIAL;

 //- Calibrating sensor difference: 
  average = 0; 
  for( int index = 0; index < N_INITIAL ; index += 1 ) {
     average += sensor_diff();
  }
  diff_initial = average/N_INITIAL ;//N_INITIAL;

  blink_led( 5 );
}

//-- Function to smoothly enable or dissable forward speed:
void set_forward( void ) {
  if( av_input() < input_initial*PERCENT_INITIAL ) {
    forward = DECAY + forward*(1 - DECAY);
  } else {
    forward = forward*(1 - DECAY);
  }
}

//- Functiont to reduce forward speed during sharp turns
float modulate_forward( void ) {
  float modulation;
  float var = abs( past_average[0] )*FOR_THROTTLE ;
  
  if( var > TURN_LIMIT ) {
    modulation = TURN_LIMIT/var;
  } else {
    modulation = 1;
  }
  return (forward*FOR_THROTTLE)*modulation ;
}

//------ Main functions: 

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  
  pinMode( PIN_SENSOR_A, INPUT );
  pinMode( PIN_SENSOR_B, INPUT );

  for( int index = 2 ; index <= 8 ; index += 1 ) {
    pinMode( index, OUTPUT );
  }
 initialize();
}

void loop() {
 wave_L.check_time();
 wave_R.check_time();

 store_average();
 
 set_forward();
 set_speed( modulate_forward(), -smooth_deriv()*K_DEV - smooth_prop()*K_PROP - smooth_deriv_2()*K_DEV_2 - quadrature(forward) );
}

//Note: 2nd derivative seems to adversely affect power consumption. LED on arduino visibly darkens if 2nd derivative gain is increased.
//Note 2: add static variables to functions to avoid global variables [allows for recursive calls, accumulation, etc ]
