/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION    1.1.2

enum Mode {MANUAL=0,AUTOMATIC=1};
enum Direction {DIRECT=0,REVERSE=1};

template <typename T, const int MINLIMIT=0, const int MAXLIMIT=255, const unsigned long SAMPLETIME=100>
class PIDT
{
    // prevent strange errors due to implicit conversions etc.
    PIDT(const PIDT<T>& other) = delete; // non construction-copyable
    PIDT& operator=(const PIDT<T>&) = delete; // non copyable
    PIDT() = delete;

  public:

    //commonly used functions **************************************************************************

    /*Constructor (...)*********************************************************
     *    The parameters specified here are those for for which we can't set up
     *    reliable defaults, so we need to have the user set them.
     ***************************************************************************/
    PIDT(T* Input, T* Output, T* Setpoint,
         T Kp, T Ki, T Kd, Direction ControllerDirection):
            myOutput(Output), myInput(Input), mySetpoint(Setpoint), inAuto(false)
    {

        SetOutputLimits(MINLIMIT, MAXLIMIT);   //default output limit corresponds to
                                               //the arduino pwm limits

		mapOutput = false;
		
        SampleTime = SAMPLETIME;               //default Controller Sample Time is 0.1 seconds

        SetControllerDirection(ControllerDirection);
        SetTunings(Kp, Ki, Kd);

        lastTime = millis()-SampleTime;
        Initialize();
		
		//turn on by default
		SetMode(AUTOMATIC);
    }


    /* Compute() **********************************************************************
     *     This, as they say, is where the magic happens.  this function should be called
     *   every time "void loop()" executes.  the function will decide for itself whether a new
     *   pid Output needs to be computed.  returns true when the output is computed,
     *   false when nothing has been done.
     **********************************************************************************/
    bool Compute()
    {
       if(!inAuto) return false;
       unsigned long now = millis();
       unsigned long timeChange = (now - lastTime);
       if(timeChange>=SampleTime)
       {
          /*Compute all the working error variables*/
          T input = *myInput;
          T error = *mySetpoint - input;
          ITerm+= (ki * error);
          if(ITerm > outMax) ITerm= outMax;
          else if(ITerm < outMin) ITerm= outMin;
          T dInput = (input - lastInput);

          /*Compute PIDT Output*/
          T output = kp * error + ITerm- kd * dInput;

          if(output > outMax) output = outMax;
          else if(output < outMin) output = outMin;
		  
		  if(mapOutput) {
			 output = map(output, outMin, outMax, mapMin, mapMax);
		  }
		  
          *myOutput = output;

          /*Remember some variables for next time*/
          lastInput = input;
          lastTime = now;
          return true;
       }
       else return false;
    }


    /* SetTunings(...)*************************************************************
     * This function allows the controller's dynamic performance to be adjusted.
     * it's called automatically from the constructor, but tunings can also
     * be adjusted on the fly during normal operation
     ******************************************************************************/

    void SetTunings(T Kp, T Ki, T Kd)
    {
       if (Kp<0 || Ki<0 || Kd<0) return;

       dispKp = Kp; dispKi = Ki; dispKd = Kd;

       T SampleTimeInSec = ((T)SampleTime)/1000;
       kp = Kp;
       ki = Ki * SampleTimeInSec;
       kd = Kd / SampleTimeInSec;

      if(controllerDirection ==REVERSE)
       {
          kp = (0 - kp);
          ki = (0 - ki);
          kd = (0 - kd);
       }
    }

    /* SetSampleTime(...) *********************************************************
     * sets the period, in Milliseconds, at which the calculation is performed
     ******************************************************************************/

    void SetSampleTime(int NewSampleTime)
    {
       if (NewSampleTime > 0)
       {
          T ratio  = (T)NewSampleTime
                          / (T)SampleTime;
          ki *= ratio;
          kd /= ratio;
          SampleTime = (unsigned long)NewSampleTime;
       }
    }

    /* SetOutputLimits(...)****************************************************
     *  This function will be used far more often than SetInputLimits. While
     *  the input to the controller will generally be in the 0-1023 range,
	 *	the output will be a little different (0-255 by default). Maybe they'll
     *  be doing a time window and will need 0-8000 or something.  or maybe they'll
     *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
     *  here.
     **************************************************************************/

    void SetOutputLimits(T Min, T Max)
    {
       if(Min >= Max) return;
       outMin = Min;
       outMax = Max;

       if(inAuto)
       {
           if(*myOutput > outMax) *myOutput = outMax;
           else if(*myOutput < outMin) *myOutput = outMin;

           if(ITerm > outMax) ITerm= outMax;
           else if(ITerm < outMin) ITerm= outMin;
       }
    }
	
   /* SetOutputMapping(...)****************************************************
    * 	This function will alter the output to be mapped to selected range. For example, if user wants
	*	to have output mapped to (0-1500) or other range using map(x, lowLimit, highLimit, newLow, newHigh)
	*	which is useful for relay switching. map will be applied at the end of the Compute() method.
    **************************************************************************/

   void SetOutputMapping(T newLow, T newHigh)
   {
      if(newLow >= newHigh) return;
      mapMin = newLow;
      mapMax = newHigh;
	  mapOutput = true;
   }

    /* SetMode(...)****************************************************************
     * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
     * when the transition from manual to auto occurs, the controller is
     * automatically initialized
     ******************************************************************************/

    void SetMode(Mode mode)
    {
        bool newAuto = (mode == AUTOMATIC);
        if(newAuto && !inAuto)
        {  /*we just went from manual to auto*/
            Initialize();
        }
        inAuto = newAuto;
    }

    /* SetControllerDirection(...)*************************************************
     * The PIDT will either be connected to a DIRECT acting process (+Output leads
     * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
     * know which one, because otherwise we may increase the output when we should
     * be decreasing.  This is called from the constructor.
     ******************************************************************************/

    void SetControllerDirection(Direction direction)
    {
       if(inAuto && direction !=controllerDirection)
       {
          kp = (0 - kp);
          ki = (0 - ki);
          kd = (0 - kd);
       }
       controllerDirection = direction;
    }


    /* Status Funcions*************************************************************
     * Just because you set the Kp=-1 doesn't mean it actually happened.  these
     * functions query the internal state of the PID.  they're here for display
     * purposes.  this are the functions the PID Front-end uses for example
     ******************************************************************************/

    T GetKp(){ return  dispKp; }
    T GetKi(){ return  dispKi;}
    T GetKd(){ return  dispKd;}
    int GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
    int GetDirection(){ return controllerDirection;}

  private:
    /* Initialize()****************************************************************
     *  does all the things that need to happen to ensure a bumpless transfer
     *  from manual to automatic mode.
     ******************************************************************************/

    void Initialize()
    {
       ITerm = *myOutput;
       lastInput = *myInput;
       if(ITerm > outMax) ITerm = outMax;
       else if(ITerm < outMin) ITerm = outMin;
    }

    T dispKp;               // * we'll hold on to the tuning parameters in user-entered
    T dispKi;               //   format for display purposes
    T dispKd;               //

    T kp;                   // * (P)roportional Tuning Parameter
    T ki;                   // * (I)ntegral Tuning Parameter
    T kd;                   // * (D)erivative Tuning Parameter

    Direction controllerDirection;

    T *myInput;             // * Pointers to the Input, Output, and Setpoint variables
    T *myOutput;            //   This creates a hard link between the variables and the
    T *mySetpoint;          //   PID, freeing the user from having to constantly tell us
                            //   what these values are.  with pointers we'll just know.

    unsigned long lastTime;
    T ITerm, lastInput;

    unsigned long SampleTime;
    T outMin, outMax, mapMin, mapMax;
    bool inAuto, mapOutput;
};

typedef PIDT<double> PID;
#endif

