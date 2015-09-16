CON
    _clkmode = xtal1 + pll16x
    _xinfreq = 5_000_000


VAR
    byte camData
    byte sentData
    long stack[64]

    byte buffer[64]
    long index, i
    
OBJ

    BS2 : "BS2_Functions"
    serial[2] : "FullDuplexSerial"

PUB Main

    serial[0].start(31, 30, %0000, 9600) {Propeller to Basic}
    serial[1].start(8, 9, %0000, 19200)  {Camera to Propeller}

    index := 0
    
    cognew(Bypass, @stack)
    
    repeat  
      camData := serial[1].rx
      if camData == 13
        repeat
          buffer[index] := serial[1].rx
          index += 1
        until buffer[index-1] == 13 or buffer[index-1] == 65
        if buffer[index-1] <> 65
          waitcnt((50 * (clkfreq / 1_000)-2300 #> 400) + cnt)
          repeat i from 0 to index
            if buffer[i] == 32
              serial[0].str(string("X"))
            else  
              serial[0].tx(buffer[i])
            waitcnt((1 * (clkfreq / 1_000)-2300 #> 400) + cnt)
          serial[1].rxFlush 
        index := 0
        
      

PUB Bypass

    repeat
      sentData := serial[0].rx
      serial[1].tx(sentData)
    