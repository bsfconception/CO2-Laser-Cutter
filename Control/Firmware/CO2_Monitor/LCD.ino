void displaySetup() 
{
    lcd.init();                      // initialize the lcd 
    // Print a message to the LCD.
    lcd.backlight();
 
}


void displayMessages( char *str )
{
  for(int i=0; i<strlen(str); i++)
  {
    lcd.write( *(str+i) );    
  }
}

    

void DisplayClear( void )
{
    lcd.clear();

}



void  displaySendText( char *str, int x, int y, int color )
{
  if(x>=0)
    lcd.setCursor(x, y);
  displayMessages(str);
}


void StartupScreen( void )
{
    lcd.clear();
    lcd.setCursor(6, 0);
    displayMessages("Starting");
    lcd.setCursor(2, 1);
    displayMessages("CO2 Laser Control");
    lcd.setCursor(2, 3);
    displayMessages("(c)BSf Conception");
}
