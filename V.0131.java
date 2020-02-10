/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Define the rintake and lintake motors JC\\
package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
//import sun.awt.www.content.image.jpeg;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


//**************************Vision Processing Imports******************************** */
import org.opencv.core.*;
import org.opencv.videoio.VideoCapture;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
import java.awt.image.BufferedImage;
import javax.swing.JFrame;
import java.awt.FlowLayout;
import javax.swing.ImageIcon;
import javax.swing.JLabel;
import java.awt.image.*;
import java.awt.Image;
//****************************End Vision Imports*************************************** */


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Joystick drive_stick;
  Joystick control_stick;
  DoubleSolenoid liftPiston;
  DoubleSolenoid hatchPiston;
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
  double m_LIntake; 
  double m_RIntake;
  double m_LOutake;
  double m_ROutake;


  int currentStage;
  int destinationStage;
  int elevatorDir;
  int lastStage;
  boolean autoMode;
  boolean lowering;
  static final int IMG_WIDTH = 320;
  static final int IMG_HEIGHT = 240;

  // Define control stick buttons
  // elevator lift controls
  static final int UP = 4;
  static final int DOWN = 5;
  // Buttons for levels on elevatorlift
  static final int BTNSTAGE0 = 7;
  static final int BTNSTAGE1 = 6;
  static final int BTNSTAGE2 = 11;
  static final int BTNSTAGE3 = 10;
  // button for climbing piston
  static final int BTNCLIMB = 8;
  static final int BTNUNCLIMB = 9;
  // constants for each color
  static final int STAGE0 = 0;
  static final int STAGE1 = 1;
  static final int STAGE2 = 2;
  static final int STAGE3 = 3;
  //buttons for intake and shooter
  static final int BTNINTAKE = 1; //use trigger for intake
  static final int BTNSHOOTER = 3;
  static final int NONE = -1;
  static final boolean ACTIVE = false; // limit switches are active low (false)
  static final boolean INACTIVE = true;
  //constants for each color
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
  //startTotal is used to reset any variable
  static final int startTotal = 0;
  //dont touch anything with endTotal it works somehow
  static final int endtotal = 6;
  //used to increment rotations because ++ was not working at the time of test
  static final int rotationAdd = 1;

  Talon m_frontRight = new Talon(0);
  Talon m_frontLeft = new Talon(1);
  Talon m_rearLeft = new Talon(2);
  Talon m_rearRight = new Talon(3);

  Talon m_LIntakeMotor = new Talon(4);
  Talon m_RIntakeMotor = new  Talon(5);
  Talon m_ColorSpinner  = new  Talon(6);

  VideoCapture camera0;
  UsbCamera camera1;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  
  //********************************************************************************
  // This function is run when the robot is first started up and should be used
  // for any initialization code.
  //********************************************************************************
  
  
  @Override
  public void robotInit() {

    
    System.out.println("Robot Init: ");

    m_deadZone = 0.1;
    m_driveMotorSpeed = 1.0;
    m_driveTurnSpeed = 0.75;
    m_shooterSpeed = 0.90;
    m_intakeSpeed = -0.40;
    displayCtr = 0;
    Y = 0; //don't delete the sacred Y
    autoMode = false;
    lowering = false;
    m_LIntake = 0.4573;
    m_RIntake = 0.4573;
    m_ROutake = -0.4573;
    m_LOutake = -0.4573;




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

    liftPiston = new DoubleSolenoid(0, 1); // Cylinder solenoid ch. 0/1 for Lift Piston
    hatchPiston = new DoubleSolenoid(2, 3); // Cylinder solenoid ch. 2/3 for Climbing Piston
    /*KM
    camera0 = CameraServer.getInstance().startAutomaticCapture("SecondCam", 0);
    camera0.setResolution(IMG_WIDTH / 2, IMG_HEIGHT / 2);
    camera0.setFPS(15);
    KM */

    camera1 = CameraServer.getInstance().startAutomaticCapture("DriveCam", 1);
    camera1.setResolution(IMG_WIDTH / 2, IMG_HEIGHT / 2);
    camera1.setFPS(15);
    System.out.println("END Robot Init: ");

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);    
  }

  //*********************************************************************************
  // This function is called every robot packet, no matter the mode. Use this for
  // items like diagnostics that you want ran during disabled, autonomous,
  // teleoperated and test.
  //
  // <p>
  // This runs after the mode specific periodic functions, but before LiveWindow
  // and SmartDashboard integrated updating.
  //*********************************************************************************
  

  @Override
  public void autonomousInit() {
    System.out.println("AutoInit");
    System.out.println("EndAutoInit");
  }

  //************************************************************
  // This function is called periodically during autonomous.
  //************************************************************ 
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic(); // for 2019 game only: call teleop in Autonomous mode

  }

  //********************************************************
  // This function is called at the start of Teloeop.
  //******************************************************** 
  @Override
  public void teleopInit() {
    m_drive.arcadeDrive(0.0,0.0);
    //System.out.println("TeleOpInit");
    //updateDisplays();
  }

  //*******************************************************************
  //  This function is called periodically during operator control.
  //*******************************************************************
  @Override
  public void teleopPeriodic() {

    // Get Drive Joystick input for arcade driving

    double X = getJoystickValue(drive_stick, 1) * m_driveMotorSpeed;
    double Z = getJoystickValue(drive_stick, 2) * m_driveTurnSpeed;

    if (lastStage > 1){
      X = X *0.75; //reduce to 75% speed if lift is up beyond level 1
    }
    m_drive.arcadeDrive(-X, Z, true); // Drive the robot
    

    ballControl(); // call ball control routine(for intake and shooting)
    ballControlSensor();
    visionTracking();
   // updateDisplays(); // call Dashboard debug display 
  }

  /**
   * This function is called periodically during test mode.
   */

//*******************************************************************
  //  Testing 
  //*******************************************************************


  public BufferedImage Mat2BufferedImage(Mat m){
    int bufferSize = m.channels()*m.cols()*m.rows();
    byte [] b = new byte[bufferSize];
    m.get(0,0,b); // get all the pixels
    BufferedImage image = new BufferedImage(m.cols(),m.rows(), 255);
    final byte[] targetPixels = ((DataBufferByte) image.getRaster().getDataBuffer()).getData();
    System.arraycopy(b, 0, targetPixels, 0, b.length);
    return image;
}

public void displayImage(Image img2) {
    ImageIcon icon=new ImageIcon(img2);
    JFrame frame=new JFrame();
    frame.setLayout(new FlowLayout());
    frame.setSize(img2.getWidth(null)+50, img2.getHeight(null)+50);
    JLabel lbl=new JLabel();
    lbl.setIcon(icon);
    frame.add(lbl);
    frame.setVisible(true);
    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
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

    /**
     * Run the color match algorithm on our detected color
     */
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

      m_LIntakeMotor.set(m_LIntake);
      m_RIntakeMotor.set(m_RIntake); //the - makes it spin opposite of LIntake
    }
    if (control_stick.getRawButtonReleased(BTNINTAKE) == true) {
      m_LIntakeMotor.set(0);
      m_RIntakeMotor.set(0);
    }
    if (control_stick.getRawButtonPressed(BTNSHOOTER) == true) {
      m_LIntakeMotor.set(m_LOutake); //This sectoin reverses the spin to shoot the balls
      m_RIntakeMotor.set(m_ROutake);
    }
    if (control_stick.getRawButtonReleased(BTNSHOOTER) == true) {
      m_LIntakeMotor.set(0);
      m_RIntakeMotor.set(0);
    }
  }
  //*************************************************************************\\
  // This function runs motor for 3 rotations and stops on specified color   \\
  //*************************************************************************\\
  public int getColorCmd() {
      if (control_stick.getRawButton(BTNSTAGE0) == true)
        return 0;
      if (control_stick.getRawButton(BTNSTAGE1) == true)
        return 1;
      if (control_stick.getRawButton(BTNSTAGE2) == true)
        return 2;
      if (control_stick.getRawButton(BTNSTAGE3) == true)
       return 3;
      return NONE;
    }
  
  public void ballControlSensor() {
  Color match = m_colorSensor.getColor(); //gets the color from te sensor and then matches it with its "most likely" color
  ColorMatchResult detected = m_colorMatcher.matchClosestColor(match);  //use detected.color for comparisons because that's what is needed to compare colors
  int button = getColorCmd();

  if (button>=0){     //while button 0 is held the color sensor will do the following lines of code
    //following if statements compare the detected colors to blue,green,red,yellow if color is detected the temporary variable is set to 1 
    if(detected.color == kBlueTarget){
      blueTemp = blueCount;
      m_ColorSpinner.set(.75);
    }
    if(detected.color== kGreenTarget){
        greenTemp = greenCount;
        m_ColorSpinner.set(.75);
    }  
    if(detected.color == kRedTarget){
        redTemp = redCount;
        m_ColorSpinner.set(.75);
    } 

    if(detected.color== kYellowTarget && detected.confidence>0.95 ){ //only made the temporary variable 1 if the confidnce was over .95 because white was getting identified as yellow with low confidence
        yellowTemp = yellowCount;
        m_ColorSpinner.set(.75);
      }
   
    int total = yellowTemp+redTemp+blueTemp+greenTemp; // creates a total of temporary variables
      if(total==4){ //if the total is 4 that means we ran through all the colors and we reset the temporary variables to 0//and add 1 to the rotation count
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
  if (button<0){    //if button 0 is not held anymore it resets the rotations to 0
    rotation = startTotal;
    }
  }
  }

  //**************************************************************************
  //function is used to test vision tracking
  //************************************************************************** 
  public void visionTracking(){
    Mat frame = new Mat();
    camera0.read(frame); //reads the frame as a Mat which is a matrix of pixels that is used with opencdv
    Image matTransition = Mat2BufferedImage(frame); //converts Mat to an image
    displayImage(matTransition); //should open the window to test vision
  }

  // ***************************************************************************
  // This function displays info on the Labview Smartdashboard periodically
  // ***************************************************************************
  public void updateDisplays() {
    if (displayCtr % 25 == 0) { // Update displays on Dashboard every ~500msec
      displayCtr = 0; // reset display CTR
      SmartDashboard.putString("DB/String 0", Double.toString(Y));
      SmartDashboard.putString("DB/String 1", "Next Stage: " + Double.toString(destinationStage));
      SmartDashboard.putString("DB/String 2", "Current Stage: " + Double.toString(currentStage));
      SmartDashboard.putString("DB/String 3", "Elev Speed: " + Double.toString(m_elevatorSpeed));
      SmartDashboard.putString("DB/String 3", "Elev Dir: " + Double.toString(elevatorDir));  
    }
    displayCtr++;
  }
} // End all 
