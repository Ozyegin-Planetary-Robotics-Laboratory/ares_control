#ifndef LED_CONTROLLER_HPP
#define LED_CONTROLLER_HPP

#include <deque>
#include <ares_control/GoalSite.h>


namespace ares_control
{
  class LedController
  {
    public:
    enum LEDColor
    {
      RED,
      BLUE,
      GREEN,
      YELLOW
    }; // enum LEDColor

    /**
     * @brief Turns off all the lights.
     * 
     */
    void setOff();

    /**
     * @brief Sets the color of the light, immediately turns on if off.
     * 
     * @param color 
     */
    void setColor(const LEDColor color);
    
    /**
     * @brief Flash the current color on and off, turns on colours if off.
     * 
     */
    void setFlashing(const LEDColor color);

    private:

  }; //class LedController
} // namespace ares_control




#endif // LED_CONTROLLER_HPP