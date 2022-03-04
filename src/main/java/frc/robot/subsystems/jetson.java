/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.net.ServerSocket;
import java.net.Socket;
import java.lang.System;
import java.io.IOException;
import java.net.UnknownHostException;
import java.nio.charset.StandardCharsets;

import com.fasterxml.jackson.databind.deser.impl.BeanAsArrayBuilderDeserializer;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.BufferedInputStream;
import java.lang.String;
public class jetson extends SubsystemBase {
  private Socket socket;
  private DataInputStream input;
  private DataOutputStream output;
  String address = "10.71.53.103";
  int port = 7000;



  public jetson() {
    try
        { 
          System.out.println("Connecting to Jetson");
            socket = new Socket(address, port);
            System.out.println(socket.getRemoteSocketAddress());
            input = new DataInputStream(socket.getInputStream());
            // sends output to the socket 
            output = new DataOutputStream(socket.getOutputStream());
            System.out.println("Jetson Connected at" + address); 
            input.close(); 
            output.close(); 
            socket.close(); 
        } 
        catch(UnknownHostException u) 
        { 
            System.out.println(u); 
        } 
        catch(IOException i) 
        { 
            System.out.println(i); 
        } 
      
  }
  public void read(){
            // string to read message from input 

  
        // keep reading until "Over" is input 
         
            try
            { 
              this.socket = new Socket(address, port);
              this.input = new DataInputStream(socket.getInputStream());
              String s = new String(input.readNBytes(9), StandardCharsets.UTF_8);
              System.out.println(s );
                
            } 
            catch(IOException i) 
            { 
                System.out.println(i); 
            } catch(NullPointerException i){
              System.out.println("Jetson Not connected");
            }
        } 
      
    public void send(){
      try{
        String line = "hello";
        output.writeUTF(line);
      } catch(IOException i){
        System.out.println("not getting anything");
      }
    }
 
  @Override
  public void periodic() {
    read();
  }
}
*/