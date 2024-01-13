// Copyright (c) FIRST and other WPILib contributors. hi...
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//    =================================================================================================================
//        _____    ________________    ______                      ____.           ____ ___.             __           
//       /  |  |  /  _____/\_____  \  /  __  \                    |    |_____     / ___\\_ |__    ____ _/  |_  ______ 
//      /   |  |_/   __  \   _(__  <  >      <      ______        |    |\__  \   / /_/  >| __ \  /  _ \\   __\/  ___/ 
//     /    ^   /\  |__\  \ /       \/   --   \    /_____/    /\__|    | / __ \_ \___  / | \_\ \(  <_> )|  |  \___ \  
//     \____   |  \_____  //______  /\______  /               \________|(____  //_____/  |___  / \____/ |__| /____  > 
//          |__|        \/        \/        \/                               \/              \/                   \/  
//    =================================================================================================================
                                                                                                               

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() { 
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
