void runZMove( void )
{
float ZPos;  
  // compute Z new position
  ZPos = (float)(  g_Parameters.ZToFocus - (g_Parameters.Thickness/2) ) / 10.0;

  // set mm unit
  g_String = "G21";
  Serial.println( g_String.c_str() );

  // set absolute coord
  g_String = "G90";
  Serial.println( g_String.c_str() );
  
  g_String = "G0 Z" + String( ZPos, 3 );
  Serial.println( g_String.c_str() );
}


void runTest( void )
{
  switch( g_Parameters.Mode )
  {
      case 0 : // empty square
        // set mm unit
        g_String = "G21";
        Serial.println( g_String.c_str() );
      
        // set relative coord
        g_String = "G91";
        Serial.println( g_String.c_str() );

        // compute G Code speed
        short Speed = g_Parameters.Speed * 60;

        // compute G Code power
        short Power = (g_Parameters.PWM * 256) / 100;

        g_String = "M03 S" + String( Power );
        Serial.println( g_String.c_str() );

        // generate moves pos
        g_String = "G01 X" + String( g_Parameters.Size ) + " F" + String( Speed );
        Serial.println( g_String.c_str() );
        g_String = "G01 Y" + String( g_Parameters.Size ) + " F" + String( Speed );
        Serial.println( g_String.c_str() );
        g_String = "G01 X-" + String( g_Parameters.Size ) + " F" + String( Speed );
        Serial.println( g_String.c_str() );
        g_String = "G01 Y-" + String( g_Parameters.Size ) + " F" + String( Speed );
        Serial.println( g_String.c_str() );
        
        g_String = "M05";
        Serial.println( g_String.c_str() );
        
      break;
      
      case 1 : // line filled square
      //g_Parameters.Step = 20;     // 1/10 mm
      break;

      case 2 : // dot filled square
  //    g_Parameters.Duration = 100;  // ms
      break;

  }


  
}
