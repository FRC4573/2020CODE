/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorSensorV3;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
///import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;


 //**************************************************************************\\
 // The VM is configured to automatically run this class, and to call the    \\
 // functions corresponding to each mode, as described in the TimedRobot     \\
 // documentation. If you change the name of this class or the package after \\
 // creating this project, you must also update the build.gradle file in the \\
 // project.                                                                 \\
 //**************************************************************************\\
public class Robot extends TimedRobot {
  Joystick drive_stick;
  Joystick control_stick;
  //DoubleSolenoid liftPiston;
  //DoubleSolenoid hatchPiston;
  DifferentialDrive m_drive;

  DigitalInput limitSwitchLower;
  DigitalInput limitSwitch1;
  DigitalInput limitSwitch2;
  DigitalInput limitSwitchUpper;

  double m_deadZone;
  double m_driveMotorSpeed;
  double m_driveTurnSpeed;
  double m_intakeSpeed;
  double m_shooterSpeed;
  double ballMotorSpeed;
  double displayCtr;
  double m_elevatorSpeed;
  double manElevSpeed;
  double autoElevSpeed;
  double Y;
  double m_Intake; 
  //double m_RIntake;
  double m_LOutakeLow;
  double m_ROutakeLow;
  double m_LOutakeHigh;
  double m_ROutakeHigh;
  double m_Belt;

  int currentStage;
  int destinationStage;
  int elevatorDir;
  int lastStage;
  boolean autoMode;
  boolean lowering;
  static final int IMG_WIDTH = 320;
  static final int IMG_HEIGHT = 240;

  //Constants for the four colors
  static final int BTNColorRed = 7;
  static final int BTNColorGreen = 6;
  static final int BTNColorBlue = 11;
  static final int BTNColorYellow = 10;
  //buttons for intake and shooter
  static final int BTNBELTREVERSE = 5;
  static final int BTNINTAKE = 4; //use trigger for intake
  static final int BTNSHOOTERHIGH = 3; //button 3 for shooter
  static final int BTNSHOOTERLOW = 2; //button 2 for shooter
  static final int NONE = -1;
  static final int BTNBELT = 1; //button for Belt
  //static final boolean ACTIVE = false; // limit switches are active low (false)
  //static final boolean INACTIVE = true;
  //constants for each color count
  static final int greenCount = 1;
  static final int redCount = 1;
  static final int yellowCount = 1;
  static final int blueCount = 1;
  //temporary variables for colors and rotations
  static int total = 0;
  static int rotation = 0;
  static int greenTemp= 0;
  static int redTemp = 0;
  static int blueTemp = 0;
  static int yellowTemp = 0;
  //Commands for the climbing piston
  static final int BTNCLIMB = 8;
  static final int BTNUNCLIMB = 9;
  //startTotal is used to reset any variable
  static final int startTotal = 0;
  //dont touch anything with endTotal it works somehow
  static final int endtotal = 6;
  //used to increment rotations because ++ was not working at the time of test
  static final int rotationAdd = 1;
  static boolean commandRan = false;

  PWMVictorSPX m_frontRight = new PWMVictorSPX(6);
  PWMVictorSPX m_frontLeft = new PWMVictorSPX(7);
  PWMVictorSPX m_rearRight = new PWMVictorSPX(8);
  PWMVictorSPX m_rearLeft = new PWMVictorSPX(9);

  //possibly use this constructor instead of what is above//seen in other teams code //kacper
  WPI_VictorSPX m_frontLight = new WPI_VictorSPX(10);

  SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
  SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
  
  CANSparkMax m_IntakeMotor = new CANSparkMax(8, MotorType.kBrushless);
  Talon m_LOutakeMotor = new Talon(1);
  Talon m_ROutakeMotor = new Talon(0);
  Talon m_BeltMotor = new Talon(2);
  Talon m_ColorSpinner  = new  Talon(4);
  Talon m_ClimbMotor = new Talon(3);
  int retries;
  
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  Timer timer = new Timer();
  static boolean stage1 = false;
  static boolean stage2 = false;
  static boolean stage3 = false;
  static final int BTNDEPLOYHATCH = 8;
  static final int BTNUNDEPLOYHATCH = 9;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  
  //****************************************************************************\\
  // This function is run when the robot is first started up and should be used \\
  // for any initialization code.                                               \\
  //****************************************************************************\\
  
  
  @Override
  public void robotInit() {

    System.out.println("Robot Init: ");
 //liftPiston = new DoubleSolenoid(0, 1); // Cylinder solenoid ch. 0/1 for Lift Piston
   // hatchPiston = new DoubleSolenoid(2, 3); // Cylinder solenoid ch. 2/3 for Climbing Piston

    m_deadZone = 0.1;
    m_driveMotorSpeed = 1;
    m_driveTurnSpeed = 0.75;
    //m_shooterSpeed = 0.9;
    //m_intakeSpeed = -0.4;
    displayCtr = 0;
    Y = 0; //don't delete the sacred Y
    //autoMode = false;
    //lowering = false;
    //Intake and Outake motor speeds
    m_Intake = 0.30;
    m_ROutakeLow = -0.3;
    m_LOutakeLow = 0.3;
    m_ROutakeHigh = -.7;
    m_LOutakeHigh = .7;
    m_Belt = .9;
    
    
    

   drive_stick = new Joystick(0);
    control_stick = new Joystick(1);


    m_drive = new DifferentialDrive(m_left, m_right);
    m_drive.setExpiration(0.50);
    m_drive.arcadeDrive(0, 0, true);
    m_drive.setSafetyEnabled(false);
    limitSwitchLower = new DigitalInput(0);
    limitSwitch1 = new DigitalInput(1);
    limitSwitch2 = new DigitalInput(2);
    limitSwitchUpper = new DigitalInput(3);

    //liftPiston = new DoubleSolenoid(0, 1); // Cylinder solenoid ch. 0/1 for Lift Piston

    UsbCamera camera0 = CameraServer.getInstance().startAutomaticCapture("SecondCam", 0);
    camera0.setResolution(IMG_WIDTH / 2, IMG_HEIGHT / 2);
    camera0.setFPS(15);

    UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture("DriveCam", 1);
    camera1.setResolution(IMG_WIDTH / 2, IMG_HEIGHT / 2);
    camera1.setFPS(15);
    System.out.println("END Robot Init: ");

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);    
  }

  //*********************************************************************************\\
  // This function is called every robot packet, no matter the mode. Use this for    \\
  // items like diagnostics that you want ran during disabled, autonomous,           \\
  // teleoperated and test.                                                          \\
  //                                                                                 \\                                                                            \\
  // This runs after the mode specific periodic functions, but before LiveWindow     \\
  // and SmartDashboard integrated updating.                                         \\
  //*********************************************************************************\\
  

  @Override
  public void autonomousInit() {
    System.out.println("StartAutoInit");
    timer.start();
    //retries = 50; // wait up to 2 secs for game data
    System.out.println("EndAutoInit");
  }

  //************************************************************\\
  // This function is called periodically during autonomous.    \\
  //Using timer we have a different function for every interval\\
  //from 0-2 we set the drive motors to .15 and -.15 to drive \\
  //from 2-5 we turn right right motor is set to .15 to drive the turn\\
  //from 5-9 we turn left by putting power to left motor\\
  //for the rest of autonomous we run the belt and shooter\\
  //have to be calibrated to the right distancees\\
  //have to have code for every case              \\
  //look up how to use java switch cases look at the code commented out line 277\\
  //Keller is good at math he knows how to calculate distances and speeds\\
  //you can change the time intervals and anything in the code below\\
  //you must use while loops because you want to run the function while\\
  //it is in that time interval. 
  //************************************************************\\
  @Override
  public void autonomousPeriodic() {
    gyro.reset();
    int roboLocation = DriverStation.getInstance().getLocation();//gets driver station location

    while(timer.get()<2){
      m_left.set(0.15);
      m_right.set(-0.15);
      
    }
    while(timer.get()>=2 && timer.get()<5){
      m_left.set(0.02);
      m_right.set(0.15);
    }
    while(timer.get()>=5 && timer.get()<9){
      m_left.set(0.15);
      m_right.set(0.02);
    }
    while(timer.get()<15){
      m_BeltMotor.set(0.80);
      m_LOutakeMotor.set(0.30);
      m_ROutakeMotor.set(-0.30);
      m_left.set(0.0);
      m_left.set(0.0);
    }
    m_LOutakeMotor.set(0.0);
    m_ROutakeMotor.set(0.0);
    m_BeltMotor.set(0.0);
    



    /*
    switch(roboLocation){
      case 1:
      //code for the robot starting on the left
      
      break;
      case 2:
      //code for the robot startinf in the middle

      break;

      case 3:
      //code for the robot starting on the right

      break;
  
    }
    */
    }

  //********************************************************\\
  // This function is called at the start of Teloeop.       \\
  //********************************************************\\
  @Override
  public void teleopInit() {
    m_drive.arcadeDrive(0.0,0.0);
    //System.out.println("TeleOpInit");
    //updateDisplays();
  }

  //*******************************************************************\\
  // This function is called periodically during operator control.     \\
  //*******************************************************************\\
  @Override
  public void teleopPeriodic() {

    // Get Drive Joystick input for arcade driving

    double X = getJoystickValue(drive_stick, 1) * m_driveMotorSpeed;
    double Z = getJoystickValue(drive_stick, 2) * m_driveTurnSpeed;


    
    if (lastStage > 1){
      X = X *0.75; //reduce to 75% speed if lift is up beyond level 1
    }
    m_drive.arcadeDrive(-X, Z, true); // Drive the robot
    //ballBelt(); //Controlls ball intake method
    ballControl(); // call ball control routine(for intake and shooting)
    ballControlSensor();
    beltControl();
   // updateDisplays(); // call Dashboard debug display 
    pistonControl(); //Calls the piston extend/retract routine
  }

  @Override
  public void robotPeriodic() {
    //**************************************************************************************\\
    // The method GetColor() returns a normalized color value from the sensor and can be    \\
    // useful if outputting the color to an RGB LED or similar. To                          \\
    // read the raw color, use GetRawColor().                                               \\
    //                                                                                      \\
    // The color sensor works best when within a few inches from an object in               \\
    // well lit conditions (the built in LED is a big help here!). The farther              \\
    // an object is the more light from the surroundings will bleed into the                \\
    // measurements and make it difficult to accurately determine its color.                \\
    //**************************************************************************************\\
    Color detectedColor = m_colorSensor.getColor();

    //*****************************************************\\
    // Run the color match algorithm on our detected color \\
    //*****************************************************\\
        String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
  
    //*******************************************************************************\\
    // Open Smart Dashboard or Shuffleboard to see the color detected by the sensor. \\
    //*******************************************************************************\\
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }
  

  @Override
  public void testPeriodic() {
  }

  //*******************************************************************\\
  //This function is used to read joystick & eliminate deadzone issues \\
  //*******************************************************************\\

  public double getJoystickValue(Joystick joystick, int iKey) {
    double dVal = joystick.getRawAxis(iKey);
    if (Math.abs(dVal) < m_deadZone) {
      return 0;
    } else {
      return dVal;
    }
  }

  //***************************************\\
  // This function controls the ball motor \\
  //***************************************\\  
  public void ballControl() {

    if (control_stick.getRawButtonPressed(BTNINTAKE) == true) {
      m_IntakeMotor.set(m_Intake);
    }
    if (control_stick.getRawButtonReleased(BTNINTAKE) == true) {
      m_IntakeMotor.set(0);

    }
    if (control_stick.getRawButtonPressed(BTNSHOOTERHIGH) == true) {
      m_LOutakeMotor.set(m_LOutakeHigh); //This section defines the high speed shot
      m_ROutakeMotor.set(m_ROutakeHigh);
    }
    if (control_stick.getRawButtonReleased(BTNSHOOTERHIGH) == true) {
      m_LOutakeMotor.set(0);
      m_ROutakeMotor.set(0);
    }
    if (control_stick.getRawButtonPressed(BTNSHOOTERLOW) == true) {
      m_LOutakeMotor.set(m_LOutakeLow); //This section defines the low speed shot
      m_ROutakeMotor.set(m_ROutakeLow);
    }
    if (control_stick.getRawButtonReleased(BTNSHOOTERLOW) == true) {
      m_LOutakeMotor.set(0);
      m_ROutakeMotor.set(0);
    }
  }

 //******************************************\\
 //This function controls they belt system \\
 //******************************************\\
  public void beltControl() {

     if (control_stick.getRawButtonPressed(BTNBELT) == true) {
     m_BeltMotor.set(m_Belt); 
     }
     if (control_stick.getRawButtonReleased(BTNBELT) == true) {
      m_BeltMotor.set(0.0); 
    }
    if(control_stick.getRawButtonPressed(BTNBELTREVERSE)==true){
      m_BeltMotor.set(-0.50);
    }
    if(control_stick.getRawButtonReleased(BTNBELTREVERSE)==true){
      m_BeltMotor.set(0.0);
    }
  }

  //*************************************************************************\\
  // This function runs motor for 3 rotations and stops on specified color   \\
  //*************************************************************************\\
  public int getColorCmd() {
      if (control_stick.getRawButton(BTNColorRed) == true)
        return 0;
      if (control_stick.getRawButton(BTNColorGreen) == true)
        return 1;
      if (control_stick.getRawButton(BTNColorBlue) == true)
        return 2;
      if (control_stick.getRawButton(BTNColorYellow) == true)
       return 3;
      return NONE;
    }
  
  public void ballControlSensor() {
  //gets the color from te sensor and then matches it with its "most likely" color
  //use detected.color for comparisons because that's what is needed to compare colors
  Color match = m_colorSensor.getColor();
  ColorMatchResult detected = m_colorMatcher.matchClosestColor(match);
  int button = getColorCmd();
//while button 0 is held the color sensor will do the following lines of code
  if (button>=0){
    //following if statements compare the detected colors to blue,green,red,yellow
    //if color is detected the temporary variable is set to 1 
    if(detected.color == kBlueTarget){
      blueTemp = blueCount;
      m_ColorSpinner.set(.2);
    }
    if(detected.color== kGreenTarget){
        greenTemp = greenCount;
        m_ColorSpinner.set(.2);
    }  
    if(detected.color == kRedTarget){
        redTemp = redCount;
        m_ColorSpinner.set(.2);
    } 
    //only made the temporary variable 1 if the confidnce was over .95 because white was getting identified as yellow with low confidence

    if(detected.color== kYellowTarget && detected.confidence>0.95 ){
        yellowTemp = yellowCount;
        m_ColorSpinner.set(.2);
      }
    // creates a total of temporary variables
    int total = yellowTemp+redTemp+blueTemp+greenTemp;
    //if the total is 4 that means we ran through all the colors and we reset the temporary variables to 0
    //and add 1 to the rotation count
      if(total==4){
        rotation += rotationAdd;
        total = startTotal;
        yellowTemp = startTotal;
        redTemp= startTotal;
        blueTemp= startTotal;
        greenTemp = startTotal;
        
      }
      //if the rotation surpasses 3 and the color matches with the color we want to stop at
      //the motor speed is set to 0 to stop running
  if(rotation>6 && detected.color == kRedTarget && button==0){
      m_ColorSpinner.set(0.0);
      total = endtotal;
        }

  if(rotation>6 && detected.color == kGreenTarget && button==1){
    m_ColorSpinner.set(0.0);
    total = endtotal;
    }
  if(rotation>6 && detected.color == kBlueTarget && button==2){
    m_ColorSpinner.set(0.0);
    total = endtotal;
      }
  if(rotation>6 && detected.color == kYellowTarget && detected.confidence>0.95 && button==3){
    m_ColorSpinner.set(0.0);
    total = endtotal;
        }
  }
  //if button 0 is not held anymore it resets the rotations to 0
  if (button<0){
    rotation = startTotal;
    }
  }

  //*************************************************************************\\
  //This function displays info on the Labview Smartdashboard periodically   \\
  //*************************************************************************\\
  public void updateDisplays() {
    if (displayCtr % 25 == 0) { // Update displays on Dashboard every ~500msec
      displayCtr = 0; // reset display CTR
      SmartDashboard.putString("DB/String 0", Double.toString(Y));
      SmartDashboard.putString("DB/String 1", "Next Stage: " + Double.toString(destinationStage));
      SmartDashboard.putString("DB/String 2", "Current Stage: " + Double.toString(currentStage));
      SmartDashboard.putString("DB/String 3", "Elev Speed: " + Double.toString(m_elevatorSpeed));
      SmartDashboard.putString("DB/String 3", "Elev Dir: " + Double.toString(elevatorDir));  
    }
  }
  //****************************************************\\
  // This function checks for user control of pistons   \\
  //****************************************************\\
  public void pistonControl() {

    // Logic for lift (climb) piston control
   /* if (control_stick.getRawButton(BTNCLIMB) == true) {
      extendLift();
      System.out.println("hi");
    }
    */
    //Retract lift pistons if button pressed
    /*if (control_stick.getRawButton(BTNUNCLIMB) == true){
      retractLift();
    }
*/
  //}
  //*************************************************************\\
  // These functions extend/retract the climbing (lift) pistons  \\
  //*************************************************************\\
  /*public void extendLift() {
    liftPiston.set(DoubleSolenoid.Value.kReverse);
    System.out.println("bye");
  }
  public void retractLift() {
   // liftPiston.set(DoubleSolenoid.Value.kForward);
  
*/
    displayCtr++;
  }
} //End all
