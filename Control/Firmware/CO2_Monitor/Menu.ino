void UpdateMonitorMode( void )
{
  lcd.clear();
  
  lcd.setCursor(2, 0);
  displayMessages("Laser Monitoring");

  // display PWM value
  lcd.setCursor(0, 2);
  displayMessages("PWM:");
  if(g_IsSignal)
  {
    g_String = (String)(g_Cycle);  
    displayMessages( g_String.c_str() );
    displayMessages( "%" );
  }
  else
  {
    displayMessages("--NA--");
  }
  
  // display FREQ value
  lcd.setCursor(0, 3);
  displayMessages("Freq:");
  if(g_IsSignal)
  {
    g_String = (String)(g_Freq);  
    displayMessages( g_String.c_str() );
    displayMessages( "Hz" );
  }
  else
  {
    displayMessages("--NA--");
  }
}

/****************************************************************/

short UpdateSetupZPosition( short Item )
{
short CurrentItem;  

  if( Item < 0 )
    Item = 1;
  if( Item > 1 )
    Item = 0;

  if(!g_Edit)
    Item = 99;
  
  lcd.clear();
  
  lcd.setCursor(3, 0);
  displayMessages("Set Z position");

/**********************************/
  lcd.setCursor(0, 1);
  CurrentItem = 0;
  if( Item  == CurrentItem )
    displayMessages(">");
  displayMessages("PWM:");
  if( ( g_EditMode ) && ( Item  == CurrentItem ) )
  {
    displayMessages( Tmp );
  }
  else
  {
    g_String = (String)(g_Parameters.PWM);  
    displayMessages( g_String.c_str() );
    displayMessages(" %");
  }
  if( Item  == CurrentItem )
    displayMessages("<");


/**********************************/
  lcd.setCursor(0, 2);
  CurrentItem = 1;
  if( Item  == CurrentItem )
    displayMessages(">");
  displayMessages("Thick:");
  if( ( g_EditMode ) && ( Item  == CurrentItem ) )
  {
    displayMessages( Tmp );
  }
  else
  {
    g_String = (String)(g_Parameters.Thickness);  
    displayMessages( g_String.c_str() );
    displayMessages(" 1/10mm");
  }
  if( Item  == CurrentItem )
    displayMessages("<");

/**********************************/
  /*
  CurrentItem = 2;
  if( Item  == CurrentItem )
  {
    lcd.setCursor(8, 3);
    displayMessages(">>Move<<");
  }
  else
  {
    lcd.setCursor(8, 3);
    displayMessages("Move");
  }
  */
  return(Item);
}


/****************************************************************/
/*    
    g_Parameters.Size = 10;     // mm
    g_Parameters.Speed = 10;     // mm/s
    g_Parameters.Mode = 0;
    g_Parameters.Step = 20;     // 1/10 mm
    g_Parameters.Duration = 100;  // ms
*/
short UpdateSetupTest( short Item )
{
short CurrentItem;  

  if( Item < 0 )
    Item = 4;
  if( Item > 4 )
    Item = 0;

  if(!g_Edit)
    Item = 99;

  lcd.clear();
  
  lcd.setCursor(7, 0);
  displayMessages("Testing");

/**********************************/
  lcd.setCursor(0, 1);
  CurrentItem = 0;
  if( Item  == CurrentItem )
    displayMessages(">");
  displayMessages("SZ:");
  if( ( g_EditMode ) && ( Item  == CurrentItem ) )
  {
    displayMessages( Tmp );
  }
  else
  {
    g_String = (String)(g_Parameters.Size);  
    displayMessages( g_String.c_str() );
    displayMessages("mm");
  }
  if( Item  == CurrentItem )
    displayMessages("<");

/**********************************/
  lcd.setCursor(10, 1);
  CurrentItem = 1;
  if( Item  == CurrentItem )
    displayMessages(">");
  displayMessages("S:");
  if( ( g_EditMode ) && ( Item  == CurrentItem ) )
  {
    displayMessages( Tmp );
  }
  else
  {
    g_String = (String)(g_Parameters.Speed);  
    displayMessages( g_String.c_str() );
    displayMessages("mm/s");
  }
  if( Item  == CurrentItem )
    displayMessages("<");

/**********************************/
  lcd.setCursor(0, 2);
  CurrentItem = 2;
  if( Item  == CurrentItem )
    displayMessages(">");
  displayMessages("Mode:");
  if( ( g_EditMode ) && ( Item  == CurrentItem ) )
  {
    displayMessages( Tmp );
  }
  else
  {
    g_String = (String)(g_Parameters.Mode);  
    displayMessages( g_String.c_str() );
  }
  if( Item  == CurrentItem )
    displayMessages("<");

/**********************************/
  lcd.setCursor(10, 2);
  CurrentItem = 3;
  if( Item  == CurrentItem )
    displayMessages(">");
  displayMessages("St:");
  if( ( g_EditMode ) && ( Item  == CurrentItem ) )
  {
    displayMessages( Tmp );
  }
  else
  {
    g_String = (String)(g_Parameters.Step);  
    displayMessages( g_String.c_str() );
    displayMessages("dm");
  }
  if( Item  == CurrentItem )
    displayMessages("<");

/**********************************/
  lcd.setCursor(0, 3);
  CurrentItem = 4;
  if( Item  == CurrentItem )
    displayMessages(">");
  displayMessages("Dur:");
  if( ( g_EditMode ) && ( Item  == CurrentItem ) )
  {
    displayMessages( Tmp );
  }
  else
  {
    g_String = (String)(g_Parameters.Duration);  
    displayMessages( g_String.c_str() );
    displayMessages("ms");
  }
  if( Item  == CurrentItem )
    displayMessages("<");

/**********************************/
/*
  CurrentItem = 5;
  if( Item  == CurrentItem )
  {
    lcd.setCursor(12, 3);
    displayMessages(">>Run<<");
  }
  else
  {
    lcd.setCursor(14, 3);
    displayMessages("Run");
  }
*/
  return(Item);
}

/****************************************************************/

short UpdateSetupConfig( short Item )
{
short CurrentItem;  

  if( Item < 0 )
    Item = 1;
  if( Item > 1 )
    Item = 0;

  if(!g_Edit)
    Item = 99;
  
  lcd.clear();
  
  lcd.setCursor(0, 0);
  displayMessages("Setup configuration");

/**********************************/
  lcd.setCursor(0, 1);
  CurrentItem = 0;
  if( Item  == CurrentItem )
    displayMessages(">");
  displayMessages("Max PWM:");
  if( ( g_EditMode ) && ( Item  == CurrentItem ) )
  {
    displayMessages( Tmp );
  }
  else
  {
    g_String = (String)(g_Parameters.MaxPWM);  
    displayMessages( g_String.c_str() );
    displayMessages(" %");
  }
  if( Item  == CurrentItem )
    displayMessages("<");


/**********************************/
  lcd.setCursor(0, 2);
  CurrentItem = 1;
  if( Item  == CurrentItem )
    displayMessages(">");
  displayMessages("Z Space:");
  if( ( g_EditMode ) && ( Item  == CurrentItem ) )
  {
    displayMessages( Tmp );
  }
  else
  {
    g_String = (String)(g_Parameters.ZToFocus);  
    displayMessages( g_String.c_str() );
    displayMessages(" 1/10 mm");
  }
  if( Item  == CurrentItem )
    displayMessages("<");
   

  return(Item);
}
