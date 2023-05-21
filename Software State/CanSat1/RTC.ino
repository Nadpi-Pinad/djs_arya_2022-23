
/********************************RTC SETUP AND LOOP FUNCTION()**********************************/
void rtc_setup_function() {
    setSyncProvider(getTeensy3Time);
    if (timeStatus() != timeSet)
    {}
  }
  
  time_t getTeensy3Time() {
    return Teensy3Clock.get();
  }

void rtc_loop_function() {
    s = second();
    if (s >= 100)
    {
      s = 0;
    }
    if (hour() < 5 && minute() < 30) {
      h = (24 + hour() - 6);
      m = (60 + minute() - 30);
    }
    else if (hour() >= 5 && minute() >= 30) {
      h = hour() - 5;
      m = minute() - 30;
    }
    else if (hour() < 5 && minute() >= 30) {
      h = (24 + hour() - 5);
      m = minute() - 30;
    }
    else if (hour() >= 5 && minute() < 30) {
      h = hour() - 6;
      m = (60 + minute() - 30);
    }
  }
