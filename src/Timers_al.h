/* 2019-07-20 ANdrzej Łęgosz
// plik Timers_al.h:
//    ma być w katalogu src
//    definiuje:
//      typedef struct STRUCT_TIMER;
//      STRUCT_TIMER TIMER[amount_timers];
//      void timer_al();
// w programie głównym main.cpp należy:
//      w nagłówku:
            const uint8_t amount_timers = 9;
            #include <Timers_al.h>
        w setup(), zadeklarować startowe parametry wszystkich instancji timerów, np:
          TIMER[0].handler = SD_CARD;           TIMER[0].time_const = dlyNextSave;        TIMER[0].active = true;
          TIMER[1].handler = SENSORS_PRINT;     TIMER[1].time_const = dlyNextSensorRead;  TIMER[1].active = true;
          .....
        dla timerów wyłączonych:
          w funkcji, w której timer jest aktywowany:
            TIMER[5].active = true;//timer cwu
          w funkccji, w której timer jest wyłączany:
            TIMER[5].active = false;//timer cwu
          w funkcji, w której timer jest podtrzymywany przed osiągnięciem zadanego czasu:
            if(TIMER[8].active)
              {
                TIMER[8].last_state = false;
              }
              else
              {
                TIMER[8].active = true;
              }
//      na początku loop():
            timer_al();

  dla każdego timera można odczytać aktulny czas [ms]: TIMER[5].time_current

 */
struct STRUCT_TIMER
{
  void (*handler)();
  boolean active;
  boolean last_state;
  uint32_t time_const;
  uint32_t time_start;
  uint32_t time_current;
} __attribute__((packed));

STRUCT_TIMER TIMER[amount_timers];

void timer_al()
{
  static uint8_t i;
  
  i++;
  if(i >= amount_timers)
  {
    i = 0;
  }

  if(TIMER[i].active)
  {
    if(!TIMER[i].last_state)
    {
      TIMER[i].time_start = millis();
      TIMER[i].time_current = 0UL;
      TIMER[i].last_state = true;
    }
    else
    {
      TIMER[i].time_current = millis() - TIMER[i].time_start;
      if(TIMER[i].time_current >= TIMER[i].time_const)//timer osiągnął zadany czas
      {
        (*(TIMER[i].handler))();//wywołanie funkcji przypisanej do timera[i]
        TIMER[i].last_state = false;//ponowny start timera: (TIMER[i].time_start =  millis())
      }
    }
  }
  else//timer[i] not active
  {
    TIMER[i].last_state = false;
  }
  
}

//**************************************************
// Timers_al.h
