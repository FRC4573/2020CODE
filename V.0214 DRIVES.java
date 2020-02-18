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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PWMVictorSPX;

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
  double m_LOutake;
  double m_ROutake;
  double m_Pulley;

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
  static final int BTNINTAKE = 1; //use trigger for intake
  static final int BTNSHOOTER = 3; //button 3 for shooter
  static final int NONE = -1;
  static final int BTNPULLEY = 2; //button for pulley
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

  PWMVictorSPX m_frontRight = new PWMVictorSPX(1);
  PWMVictorSPX m_frontLeft = new PWMVictorSPX(2);
  PWMVictorSPX m_rearRight = new PWMVictorSPX(3);
  PWMVictorSPX m_rearLeft = new PWMVictorSPX(4);
  

  CANSparkMax m_IntakeMotor = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax m_LOutakeMotor = new CANSparkMax(6, MotorType.kBrushed);
  CANSparkMax m_ROutakeMotor = new CANSparkMax(7, MotorType.kBrushed);
  CANSparkMax m_PulleyMotor = new CANSparkMax(8, MotorType.kBrushed);
  Talon m_ColorSpinner  = new  Talon(9);
  

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
    m_Intake = 0.425;
    //m_RIntake = 0.425;
    m_ROutake = -0.425;
    m_LOutake = -0.425;
    m_Pulley = 0.425;
    

   drive_stick = new Joystick(0);
    control_stick = new Joystick(1);
    SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
    SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);

    m_drive = new DifferentialDrive(m_left, m_right);
    m_drive.setExpiration(0.50);
    m_drive.arcadeDrive(0, 0, true);
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
    System.out.println("AutoInit");
    System.out.println("EndAutoInit");
  }

  //************************************************************\\
  // This function is called periodically during autonomous.    \\
  //************************************************************\\
  @Override
  public void autonomousPeriodic() {
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
    pulleyControl();
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
      //m_RIntakeMotor.set(m_RIntake); //the - makes it spin opposite of LIntake
    }
    if (control_stick.getRawButtonReleased(BTNINTAKE) == true) {
      m_IntakeMotor.set(0);
      //m_RIntakeMotor.set(0);
    }
    if (control_stick.getRawButtonPressed(BTNSHOOTER) == true) {
      m_LOutakeMotor.set(m_LOutake); //This sectoin reverses the spin to shoot the balls
      m_ROutakeMotor.set(m_ROutake);
    }
    if (control_stick.getRawButtonReleased(BTNSHOOTER) == true) {
      m_LOutakeMotor.set(0);
      m_ROutakeMotor.set(0);
    }
  }

 //******************************************\\
 //This function controls they pulley system \\
 //******************************************\\
  public void pulleyControl() {

     if (control_stick.getRawButtonPressed(BTNPULLEY) == true) {
     m_PulleyMotor.set(m_Pulley); 
  }
}/*
  // ***************************************************************************
  // These functions extend/retract the Hatch pistons
  // ***************************************************************************
  public void extendHatchPiston() {
    hatchPiston.set(DoubleSolenoid.Value.kForward);
  }
  public void retractHatchPiston() {
    hatchPiston.set(DoubleSolenoid.Value.kReverse);
  } */
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
