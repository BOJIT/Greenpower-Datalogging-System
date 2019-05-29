#ifndef main_h
#define main_h

void ParseData();
void Report_Vals();

#ifdef Module1
    void Voltage_Sample();
    void Current_Sample();
#endif

#ifdef Module2
    void Temp_Sample();
    void RPM_Calc();
#endif

#ifdef Module3
    void LCD_Print();
    void GPS_Sample();
#endif


#endif