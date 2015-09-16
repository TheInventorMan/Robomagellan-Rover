CON
    _clkmode = xtal1 + pll16x
    _xinfreq = 5_000_000

    RateTurnP = 2
    NumPylons = 3 {NOT including the starting pylon}

    AutoOutput = 2
    RelayInput = 3

    ThrottleOut = 1
    SteerOut = 0

    CameraTx = 9
    CameraRx = 8

    HitSwitch = 18

    RED_HIGH_Value = 255
    GREEN_HIGH_Value = 255
    BLUE_HIGH_Value = 255

    RED_LOW_Value = 255
    GREEN_LOW_Value = 255
    BLUE_LOW_Value = 255
    

VAR

    long PylonStack[128] 
    long DebugStack[128]
    long ServoStack[128]
   
    byte AtWaypoint
    byte AutoPinState
    byte pylonFound
    long workVal
    byte WptNum
    byte failCnt
    
    byte ACKstr[8]
    byte x
    byte StrStack[64]
    long indexCtr
    long tsPosition
    long confidence

    byte trackingActive
    byte TRegLock
    byte TPosLock
    
    long DriveRegister
    long TurnRegister

    long startTime
    long endTime
    long Kus
    long pos[5]
    byte index
   
    
OBJ
    serial[2] : "Extended_FDSerial.spin"
    PWM : "PWM_32_v4.spin" 

PUB Start

    serial[0].start(31, 30, %0000, 9600) {Propeller to debug}
    serial[1].start(8, 9, %0000, 19200)  {Camera to Propeller}

    PWM.Start
    
    AtWaypoint := 0
    AutoPinState := 1
    pylonFound := 0
    tsPosition := 1500
    confidence := 0
    DriveRegister := 1500
    TurnRegister := 1500
    WptNum := 1
    failCnt := 0

    Kus := clkfreq / 1_000_000

    cognew (Debugging, @DebugStack)
    cognew (PylonFind, @PylonStack)
    cognew (ServoReader, @ServoStack)      

    TRegLock := locknew
    TPosLock := locknew
    
    dira[RelayInput]~
    
    repeat
      if ina[RelayInput] == 1
        AtWaypoint := 1
        
      if AutoPinState == 0        
        PWM.Servo(AutoOutput, 1000)
      elseif AutoPinState == 1
        PWM.Servo(AutoOutput, 1550)

      PWM.Servo(ThrottleOut, DriveRegister)
      
      repeat until not lockset(TRegLock)
      PWM.Servo(SteerOut, TurnRegister)
      lockclr(TRegLock)
      
      waitcnt((50 * (clkfreq / 1_000)-2300 #> 400) + cnt)

PUB PylonFind
    
    repeat
      if AtWaypoint == 1
        AutoPinState := 0

         trackingActive := 0

         {serial[1].str(string("SS 1 1 1500", 13))
         waitcnt((750 * (clkfreq / 1_000)-2300 #> 400) + cnt)  }

         if WptNum == 1
           setPylon1
         elseif WptNum == 2
           setPylon2
         elseif WptNum == 3
           setPylon3
         elseif WptNum == 4
           setPylon4
         else
           setTrackingParameters
           getAcknowledge
            
         {setTrackingParameters
         getAcknowledge
         waitcnt((500 * (clkfreq / 1_000)-2300 #> 400) + cnt) }        

         setAutomaticTilt
         getAcknowledge
         waitcnt((500 * (clkfreq / 1_000)-2300 #> 400) + cnt)
                        
         trackColor
         getAcknowledge
         waitcnt((500 * (clkfreq / 1_000)-2300 #> 400) + cnt)
        
         repeat

           if trackingActive == 0
             trackColor
             getAcknowledge
           waitcnt((100 * (clkfreq / 1_000)-2300 #> 400) + cnt)
           
           getTrackingPacket
           waitcnt((10 * (clkfreq / 1_000)-2300 #> 400) + cnt)
            
           if confidence > 40
              waitcnt((100 * (clkfreq / 1_000)-2300 #> 400) + cnt)

              getTrackingPacket
              waitcnt((10 * (clkfreq / 1_000)-2300 #> 400) + cnt)
              
              if confidence > 40
                pylonFound := 1
                repeat until not lockset(TRegLock)
                TurnRegister := 1500   
                lockclr(TRegLock)
                
           else

             failCnt++
             if (failCnt // 10) == 0                         {ensure camera coverage area is maximized}
               if tsPosition > 1750
                  setTiltServoCenter
               elseif tsPosition < 1250
                  setTiltServoLeft
               else
                  setTiltServoRight      
               trackingActive := 0
               
         until pylonFound == 1

         if confidence > 40
           repeat 
              
              waitcnt((10 * (clkfreq / 1_000)-2300 #> 400) + cnt)

              repeat until not lockset(TPosLock)
              if (tsPosition > 1800) OR (tsPosition < 1300)
                workVal := 1500 -((tsPosition - 1500) * RateTurnP)         
                workVal #>= 950
                workVal <#= 2050
                repeat until not lockset(TRegLock)
                TurnRegister := workVal
                lockclr(TRegLock)
             lockclr(TPosLock)
              
              DriveRegister :=  1850
       
           until (tsPosition > 1300) AND (tsPosition < 1800)
          
        repeat
            
            waitcnt((10 * (clkfreq / 1_000)-2300 #> 400) + cnt) 

            repeat until not lockset(TPosLock)
            if (tsPosition > 1520) OR (tsPosition < 1480)          
              workVal := 1500 -((tsPosition - 1500) * RateTurnP)          
              workVal #>= 950
              workVal <#= 2050
              repeat until not lockset(TRegLock)
              TurnRegister := workVal
              lockclr(TRegLock)   
            lockclr(TPosLock)
            
            DriveRegister := 1850
            
        until ina[hitSwitch] == 1

          serial[1].tx(13) 
          DriveRegister := 1500
          
          repeat until not lockset(TRegLock)
          TurnRegister := 1500
          lockclr(TRegLock)
          
          waitcnt((1000 * (clkfreq / 1_000)-2300 #> 400) + cnt)

          if WptNum <> NumPylons
            DriveRegister := 1000

            waitcnt((4000 * (clkfreq / 1_000)-2300 #> 400) + cnt)

            DriveRegister := 1500

            AutoPinState := 1
            WptNum++
            {set waypoint reset pin input here}
          pylonFound := 0
          AtWaypoint := 0

PUB setTrackingParameters

    serial[1].str(string("ST 82 115 36 52 32 65", 13))
    serial[0].str(string("ST 172 205 80 133 82 164"))
    serial[0].tx(13)
    trackingActive := 0
    return
    
PUB setPylon1

    serial[1].str(string("ST 156 172 80 93 65 90", 13))
    getAcknowledge
    serial[0].str(string("ST 172 205 80 133 82 164"))
    serial[0].tx(13)

    waitcnt((500 * (clkfreq / 1_000)-2300 #> 400) + cnt)

    serial[1].str(string("CB 0", 13))
    getAcknowledge
    waitcnt((50 * (clkfreq / 1_000)-2300 #> 400) + cnt)
    
    serial[1].str(string("CC -11", 13))
    getAcknowledge
    waitcnt((50 * (clkfreq / 1_000)-2300 #> 400) + cnt)
    
    trackingActive := 0
    return

PUB setPylon2

    serial[1].str(string("ST 156 172 80 93 65 90", 13))
    getAcknowledge
    serial[0].str(string("ST 172 205 80 133 82 164"))
    serial[0].tx(13)

    waitcnt((500 * (clkfreq / 1_000)-2300 #> 400) + cnt)

    serial[1].str(string("CB 0", 13))
    getAcknowledge
    waitcnt((50 * (clkfreq / 1_000)-2300 #> 400) + cnt)
    
    serial[1].str(string("CC -11", 13))
    getAcknowledge
    waitcnt((50 * (clkfreq / 1_000)-2300 #> 400) + cnt)
    
    trackingActive := 0
    return

PUB setPylon3

    serial[1].str(string("ST 156 172 80 93 65 90", 13))
    getAcknowledge
    serial[0].str(string("ST 172 205 80 133 82 164"))
    serial[0].tx(13)

    waitcnt((500 * (clkfreq / 1_000)-2300 #> 400) + cnt)

    serial[1].str(string("CB 0", 13))
    getAcknowledge
    waitcnt((50 * (clkfreq / 1_000)-2300 #> 400) + cnt)
    
    serial[1].str(string("CC -11", 13))
    getAcknowledge
    waitcnt((50 * (clkfreq / 1_000)-2300 #> 400) + cnt)
    
    trackingActive := 0
    return

PUB setPylon4

    serial[1].str(string("ST 82 115 36 52 32 65", 13))
    getAcknowledge
    serial[0].str(string("ST 172 205 80 133 82 164"))
    serial[0].tx(13)

    waitcnt((500 * (clkfreq / 1_000)-2300 #> 400) + cnt)

    serial[1].str(string("CB 0", 13))
    getAcknowledge
    waitcnt((50 * (clkfreq / 1_000)-2300 #> 400) + cnt)
    
    serial[1].str(string("CC 0", 13))
    getAcknowledge
    waitcnt((50 * (clkfreq / 1_000)-2300 #> 400) + cnt)
    
    trackingActive := 0
    return

PUB setAutomaticTilt

    serial[1].str(string("AT 1 0", 13))
    serial[0].str(string("AT 1 0", 13))
    trackingActive := 0        
    return
    
PUB trackColor

    serial[1].str(string("TC", 13))
    serial[0].str(string("TC", 13))
    trackingActive := 1
    return

PUB setTiltServoLeft

    serial[1].tx(13)
    serial[1].str(string("SS 1 1 1850", 13))
    waitcnt((750 * (clkfreq / 1_000)-2300 #> 400) + cnt)
    setAutomaticTilt
    getAcknowledge
    return

PUB setTiltServoRight

    serial[1].tx(13)
    serial[1].str(string("SS 1 1 1150", 13))
    waitcnt((750 * (clkfreq / 1_000)-2300 #> 400) + cnt)
    setAutomaticTilt
    getAcknowledge
    return

PUB setTiltServoCenter

    serial[1].tx(13)
    serial[1].str(string("SS 1 1 1500", 13))
    waitcnt((750 * (clkfreq / 1_000)-2300 #> 400) + cnt)
    setAutomaticTilt
    getAcknowledge
    return    
                                
PUB getTrackingPacket

     repeat
        x := serial[1].rx
     until x == "T"

    { serial[1].Rxflush  }
     indexCtr := 0
           
     repeat      
        StrStack[indexCtr] := serial[1].rx
        {serial[0].tx(StrStack[indexCtr])}
        indexCtr += 1
     until StrStack[indexCtr-1] == 13

     indexCtr -= 1
     confidence := 0
     
     if StrStack[indexCtr-2] == 32
       confidence := (StrStack[indexCtr - 1] - 48)
     elseif StrStack[indexCtr-3] == 32
       confidence := ((StrStack[indexCtr - 2] - 48) * 10) + (StrStack[indexCtr - 1] - 48)
     elseif StrStack[indexCtr-4] == 32
       confidence := ((StrStack[indexCtr - 3] - 48) * 100) + ((StrStack[indexCtr - 2] - 48) * 10) + (StrStack[indexCtr - 1] - 48)
     return

PUB getAcknowledge

    serial[1].rxstr(@ACKstr)
    {serial[0].str(@ACKstr) }
    return
    
PUB Debugging

    repeat

      serial[0].str(string("Auto Pin State:  "))
      serial[0].dec(AutoPinState)
      serial[0].tx(13)

      serial[0].str(string("Throttle Output:  "))
      serial[0].dec(DriveRegister)
      serial[0].tx(13)

      serial[0].str(string("Steering Output:  "))
      serial[0].dec(TurnRegister)
      serial[0].tx(13)

      serial[0].str(string("At Waypoint:  "))
      serial[0].dec(AtWaypoint)
      serial[0].tx(13)

      serial[0].str(string("Pylon Found:  "))
      serial[0].dec(pylonFound)
      serial[0].tx(13)

      serial[0].str(string("Confidence:  "))
      serial[0].dec(confidence)
      serial[0].tx(13)

      serial[0].str(string("Wpt Number:  "))
      serial[0].dec(WptNum)
      serial[0].tx(13)

      serial[0].str(string("Tilt Servo:  "))
      serial[0].dec(tsPosition)
      serial[0].tx(13)
      serial[0].tx(13)
      
      waitcnt((500 * (clkfreq / 1_000)-2300 #> 400) + cnt)

PUB ServoReader

    index := 0
    pos[0] := 1500
    pos[1] := 1500
    pos[2] := 1500
    pos[3] := 1500
    pos[4] := 1500

    repeat
      dira[20]~
      waitpeq(|<20, |<20, 0)
      startTime := cnt
      waitpeq(%00000000_00000000_00000000_00000000, |<20, 0)
      endTime := cnt
      
      if (startTime < endTime)
        pos[index] := (endTime - startTime)/ Kus
        index++
        if index == 5
          index := 0
        repeat until not lockset(TPosLock)
        tsPosition := (pos[0] + pos[1] + pos[2] + pos[3] + pos[4]) / 5
        lockclr(TPosLock)
      
                 